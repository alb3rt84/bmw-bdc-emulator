#!/usr/bin/env python3
"""
BMW BDC/ZGM Bench Companion — improved GUI

Tabs:
  1) Live Control  — JSON Serial/UDP :13401, or Robotell USB-CAN (CH340)
  2) DoIP UDS      — factory-style diagnostics over Ethernet :13400 (same as ISTA path)
  3) Log

Build Windows .exe:
  build_exe.bat
  or: pyinstaller --noconfirm --onefile --windowed --name BmwBdcCompanion gui_app.py
"""

from __future__ import annotations

import json
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

from bmw_cyclic import Signals, due, encode_ignition
from robotell_can import CanFrame, RobotellCan


APP_TITLE = "BMW BDC/ZGM Bench Companion"
UDP_CTRL_PORT = 13401
DOIP_PORT = 13400
SERIAL_BAUD = 115200
SEND_HZ = 20

DOIP_VER = 0x02
DOIP_INV = 0xFD
PT_VEHICLE_IDENT_REQ = 0x0001
PT_VEHICLE_ANNOUNCE = 0x0004
PT_ROUTING_ACT_REQ = 0x0005
PT_ROUTING_ACT_RES = 0x0006
PT_DIAG_MSG = 0x8001
PT_DIAG_ACK = 0x8002
PT_DIAG_NACK = 0x8003

LA_TESTER = 0x0E00
LA_BDC = 0x0010


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_hex_bytes(text: str) -> bytes:
    cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
    parts = [p for p in cleaned.replace("-", " ").split() if p]
    return bytes(int(p, 16) for p in parts)


# ---------------------------------------------------------------------------
# JSON companion transport (live signals)
# ---------------------------------------------------------------------------

class SerialTransport:
    def __init__(self, port: str, baud: int = SERIAL_BAUD) -> None:
        if serial is None:
            raise RuntimeError("pyserial not installed — pip install pyserial")
        self._ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.3)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    def send_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        self._ser.write(line.encode("utf-8"))

    def read_line(self, timeout: float = 0.05) -> str | None:
        self._ser.timeout = timeout
        raw = self._ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace").strip()


class UdpTransport:
    def __init__(self, host: str, port: int = UDP_CTRL_PORT) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.05)
        self._sock.bind(("", 0))

    def close(self) -> None:
        try:
            self._sock.close()
        except Exception:
            pass

    def send_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        self._sock.sendto(line.encode("utf-8"), self._addr)

    def read_line(self, timeout: float = 0.05) -> str | None:
        self._sock.settimeout(timeout)
        try:
            data, _ = self._sock.recvfrom(1024)
        except (socket.timeout, OSError):
            return None
        return data.decode("utf-8", errors="replace").strip()


# ---------------------------------------------------------------------------
# DoIP client (factory Ethernet path to BDC)
# ---------------------------------------------------------------------------

class DoipClient:
    def __init__(self) -> None:
        self.sock: socket.socket | None = None
        self.host = ""
        self.la_gateway = LA_BDC
        self.la_tester = LA_TESTER
        self._lock = threading.Lock()
        self._buf = bytearray()

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def close(self) -> None:
        with self._lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            self._buf.clear()

    @staticmethod
    def _hdr(payload_type: int, payload: bytes) -> bytes:
        return struct.pack("!BBHI", DOIP_VER, DOIP_INV, payload_type, len(payload)) + payload

    def discover(self, host: str, timeout: float = 1.5) -> dict | None:
        """UDP vehicle identification → parse announcement."""
        payload = b""
        pkt = self._hdr(PT_VEHICLE_IDENT_REQ, payload)
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp.settimeout(timeout)
        try:
            udp.sendto(pkt, (host, DOIP_PORT))
            data, addr = udp.recvfrom(512)
        except (socket.timeout, OSError):
            return None
        finally:
            udp.close()

        if len(data) < 8 + 32:
            return None
        ptype = struct.unpack("!H", data[2:4])[0]
        if ptype != PT_VEHICLE_ANNOUNCE:
            return None
        body = data[8:]
        vin = body[0:17].decode("ascii", errors="replace")
        la = struct.unpack("!H", body[17:19])[0]
        self.la_gateway = la
        return {"addr": addr[0], "vin": vin, "la": la}

    def connect(self, host: str, timeout: float = 3.0) -> None:
        self.close()
        self.host = host
        s = socket.create_connection((host, DOIP_PORT), timeout=timeout)
        s.settimeout(0.2)
        self.sock = s
        self._buf.clear()
        self.routing_activation()

    def routing_activation(self) -> bytes:
        # source LA (2) + activation type (1) + reserved (4) = 7 bytes min; many stacks use 11
        payload = struct.pack("!HBI", self.la_tester, 0x00, 0) + b"\x00\x00\x00\x00"
        # ISO 13400 routing activation request payload: SA(2)+type(1)+reserved(4) = 7
        payload = struct.pack("!HB", self.la_tester, 0x00) + b"\x00\x00\x00\x00"
        self._send_raw(self._hdr(PT_ROUTING_ACT_REQ, payload))
        return self._recv_payload_type(PT_ROUTING_ACT_RES, timeout=2.0)

    def diagnostic(self, uds: bytes, target_la: int | None = None) -> bytes:
        if not self.sock:
            raise RuntimeError("DoIP not connected")
        ta = self.la_gateway if target_la is None else target_la
        payload = struct.pack("!HH", self.la_tester, ta) + uds
        self._send_raw(self._hdr(PT_DIAG_MSG, payload))

        # Expect ACK then diagnostic response (may be interleaved)
        deadline = time.monotonic() + 3.0
        uds_resp = b""
        while time.monotonic() < deadline:
            ptype, body = self._recv_one(timeout=max(0.05, deadline - time.monotonic()))
            if ptype is None:
                continue
            if ptype == PT_DIAG_ACK:
                continue
            if ptype == PT_DIAG_NACK:
                code = body[4] if len(body) >= 5 else -1
                raise RuntimeError(f"DoIP diagnostic NACK code=0x{code:02X}")
            if ptype == PT_DIAG_MSG and len(body) >= 4:
                # SA/TA + UDS
                uds_resp = body[4:]
                break
        return uds_resp

    def _send_raw(self, data: bytes) -> None:
        with self._lock:
            if not self.sock:
                raise RuntimeError("DoIP not connected")
            self.sock.sendall(data)

    def _recv_one(self, timeout: float) -> tuple[int | None, bytes]:
        """Read one DoIP packet from stream buffer."""
        if not self.sock:
            return None, b""
        end = time.monotonic() + timeout
        while True:
            if len(self._buf) >= 8:
                plen = struct.unpack("!I", self._buf[4:8])[0]
                total = 8 + plen
                if len(self._buf) >= total:
                    pkt = bytes(self._buf[:total])
                    del self._buf[:total]
                    ptype = struct.unpack("!H", pkt[2:4])[0]
                    return ptype, pkt[8:]

            remaining = end - time.monotonic()
            if remaining <= 0:
                return None, b""
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return None, b""
            except OSError:
                return None, b""
            if not chunk:
                raise RuntimeError("DoIP TCP closed")
            self._buf.extend(chunk)

    def _recv_payload_type(self, want: int, timeout: float) -> bytes:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            ptype, body = self._recv_one(timeout=end - time.monotonic())
            if ptype == want:
                return body
        raise TimeoutError(f"DoIP timeout waiting for payload type 0x{want:04X}")


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class CompanionApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)

        self._ctrl: SerialTransport | UdpTransport | None = None
        self._robotell: RobotellCan | None = None
        self._rx_q: queue.Queue = queue.Queue()
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._can_stop = threading.Event()
        self._can_thread: threading.Thread | None = None
        self._connect_cancel = threading.Event()
        self._connecting = False
        self._pending_sig = False
        self._last_sig_sent = 0.0
        self._doip = DoipClient()
        self._signals = Signals()
        self._cyclic_enabled = True
        self._tx_count = 0
        self._rx_count = 0
        self._rx_seen: dict[tuple, int] = {}
        self._port_map: dict[str, str] = {}
        self._tx_jobs: list[dict] = []
        self._tx_job_lock = threading.Lock()
        self._tx_seq = 0
        self.minsize(960, 500)
        self._build_ui()
        self.update_idletasks()
        screen_h = self.winfo_screenheight()
        screen_w = self.winfo_screenwidth()
        height = min(620, max(520, screen_h - 96))
        width = min(1180, max(1000, screen_w - 48))
        self.geometry(f"{width}x{height}")

        self._refresh_ports()
        self.after(50, self._poll_rx)
        self.after(50, self._stream_sig_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=2)
        ttk.Label(top, text=APP_TITLE, font=("Segoe UI", 10, "bold")).pack(side="left")
        self.global_status = tk.StringVar(value="Ready")
        ttk.Label(top, textvariable=self.global_status).pack(side="right")

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=10, pady=4)

        self.tab_live = ttk.Frame(nb)
        self.tab_doip = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_live, text="Live Control")
        nb.add(self.tab_doip, text="DoIP UDS (BDC)")
        nb.add(self.tab_log, text="Log")

        self._build_live_tab()
        self._build_doip_tab()
        self._build_log_tab()

    # ----------------------------- Live Control -----------------------------
    def _build_live_tab(self) -> None:
        panes = ttk.Panedwindow(self.tab_live, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=4, pady=2)
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=1)
        panes.add(right, weight=1)

        self._build_link_panel(left)
        self._build_wake_panel(left)
        self._build_signal_panel(left)
        self._build_can_tx_panel(right)
        self._build_can_rx_panel(right)
        self._install_wake_jobs()
        self._mode_changed()
        self._on_dlc_changed()

    def _build_link_panel(self, parent: ttk.Frame) -> None:
        conn = ttk.LabelFrame(parent, text="Połączenie")
        conn.pack(fill="x", padx=4, pady=2)

        self.mode = tk.StringVar(value="robotell")
        ttk.Radiobutton(conn, text="Robotell", variable=self.mode, value="robotell",
                        command=self._mode_changed).grid(row=0, column=0, sticky="w", padx=6, pady=2)
        ttk.Radiobutton(conn, text="ESP32 COM", variable=self.mode, value="serial",
                        command=self._mode_changed).grid(row=0, column=1, sticky="w", padx=6, pady=2)
        ttk.Radiobutton(conn, text="UDP", variable=self.mode, value="udp",
                        command=self._mode_changed).grid(row=0, column=2, sticky="w", padx=6, pady=2)

        ttk.Label(conn, text="COM").grid(row=1, column=0, sticky="w", padx=6)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="Odśwież", command=self._refresh_ports).grid(row=1, column=2, padx=4)

        ttk.Label(conn, text="IP").grid(row=2, column=0, sticky="w", padx=6)
        self.ip_var = tk.StringVar(value="192.168.0.10")
        self.ip_entry = ttk.Entry(conn, textvariable=self.ip_var, width=16)
        self.ip_entry.grid(row=2, column=1, sticky="w", padx=4)

        self.robotell_row = ttk.Frame(conn)
        self.robotell_row.grid(row=3, column=0, columnspan=3, sticky="we", padx=4, pady=2)
        ttk.Label(self.robotell_row, text="USB").pack(side="left")
        self.usb_baud_var = tk.StringVar(value="115200")
        ttk.Combobox(
            self.robotell_row, textvariable=self.usb_baud_var, width=9,
            values=("115200", "2000000", "1000000", "921600", "460800", "230400", "57600", "38400", "19200", "9600"),
        ).pack(side="left", padx=4)
        ttk.Label(self.robotell_row, text="CAN").pack(side="left", padx=(6, 0))
        self.can_bitrate_var = tk.StringVar(value="500000")
        ttk.Combobox(
            self.robotell_row, textvariable=self.can_bitrate_var, width=9,
            values=("500000", "250000", "125000", "100000", "1000000"),
        ).pack(side="left", padx=4)
        self.auto_baud_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.robotell_row, text="Auto baud", variable=self.auto_baud_var).pack(side="left", padx=4)

        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_ctrl)
        self.conn_btn.grid(row=4, column=0, columnspan=2, sticky="we", padx=6, pady=4)
        self.ctrl_status = tk.StringVar(value="Rozłączony")
        ttk.Label(conn, textvariable=self.ctrl_status).grid(row=4, column=2, sticky="w")
        conn.columnconfigure(1, weight=1)

    def _build_wake_panel(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Wybudzenie BMW")
        box.pack(fill="both", expand=True, padx=4, pady=2)
        bar = ttk.Frame(box)
        bar.pack(fill="x", padx=4, pady=2)
        self.cyclic_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            bar, text="Automaty", variable=self.cyclic_var, command=self._on_cyclic_toggle,
        ).pack(side="left")
        ttk.Button(bar, text="Włącz / wyłącz", command=self._toggle_tx_job).pack(side="left", padx=4)
        ttk.Button(bar, text="Usuń", command=self._delete_tx_job).pack(side="left")

        cols = ("on", "id", "dlc", "data", "period")
        self.wake_tree = ttk.Treeview(box, columns=cols, show="headings", height=4)
        for key, title, width, stretch in (
            ("on", "", 36, False),
            ("id", "ID", 70, False),
            ("dlc", "DLC", 40, False),
            ("data", "Dane", 180, True),
            ("period", "Period", 70, False),
        ):
            self.wake_tree.heading(key, text=title)
            self.wake_tree.column(key, width=width, stretch=stretch, anchor="center")
        self.wake_tree.column("data", anchor="w")
        self.wake_tree.pack(fill="both", expand=True, padx=4, pady=2)
        self.wake_tree.bind("<Double-1>", self._load_tx_job)
        self.wake_tree.bind("<<TreeviewSelect>>", lambda _e: self._remember_tree(self.wake_tree))

    def _build_signal_panel(self, parent: ttk.Frame) -> None:
        dash = ttk.LabelFrame(parent, text="Sygnały")
        dash.pack(fill="x", padx=4, pady=2)
        self.ign_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            dash, text="KL15", variable=self.ign_var, command=self._on_ignition,
        ).pack(anchor="w", padx=8, pady=1)
        self.rpm_var = tk.DoubleVar(value=0)
        self.spd_var = tk.DoubleVar(value=0)
        self.fuel_var = tk.DoubleVar(value=50)
        self.clt_var = tk.DoubleVar(value=90)
        self._add_slider(dash, "RPM", self.rpm_var, 0, 8000, "{:.0f}")
        self._add_slider(dash, "km/h", self.spd_var, 0, 300, "{:.0f}")
        self._add_slider(dash, "Paliwo", self.fuel_var, 0, 100, "{:.0f}%")
        self._add_slider(dash, "Temp", self.clt_var, -40, 140, "{:.0f}°")
        btns = ttk.Frame(dash)
        btns.pack(fill="x", padx=8, pady=2)
        ttk.Button(btns, text="Idle", command=self._preset_idle).pack(side="left", padx=2)
        ttk.Button(btns, text="50 km/h", command=self._preset_drive).pack(side="left", padx=2)

    def _build_can_tx_panel(self, parent: ttk.Frame) -> None:
        tx = ttk.LabelFrame(parent, text="Nadawanie")
        tx.pack(fill="both", expand=True, padx=4, pady=2)

        top = ttk.Frame(tx)
        top.pack(fill="x", padx=4, pady=2)
        ttk.Label(top, text="ID").pack(side="left")
        self.tx_id_var = tk.StringVar(value="123")
        ttk.Entry(top, textvariable=self.tx_id_var, width=8, font=("Consolas", 11), justify="center").pack(side="left", padx=3)
        ttk.Label(top, text="DLC").pack(side="left", padx=(6, 0))
        self.tx_dlc_var = tk.StringVar(value="8")
        dlc = ttk.Combobox(
            top, textvariable=self.tx_dlc_var, width=3, state="readonly",
            values=[str(i) for i in range(9)],
        )
        dlc.pack(side="left", padx=3)
        dlc.bind("<<ComboboxSelected>>", self._on_dlc_changed)

        self.tx_format = tk.StringVar(value="std")
        ttk.Radiobutton(top, text="STD", variable=self.tx_format, value="std",
                        command=self._on_dlc_changed).pack(side="left", padx=2)
        ttk.Radiobutton(top, text="EXT", variable=self.tx_format, value="ext",
                        command=self._on_dlc_changed).pack(side="left", padx=2)
        self.tx_kind = tk.StringVar(value="data")
        ttk.Radiobutton(top, text="DATA", variable=self.tx_kind, value="data",
                        command=self._on_dlc_changed).pack(side="left", padx=2)
        ttk.Radiobutton(top, text="RTR", variable=self.tx_kind, value="remote",
                        command=self._on_dlc_changed).pack(side="left", padx=2)

        ttk.Label(top, text="Period").pack(side="left", padx=(8, 0))
        self.tx_period_var = tk.StringVar(value="100")
        ttk.Entry(top, textvariable=self.tx_period_var, width=6, font=("Consolas", 11), justify="center").pack(side="left", padx=3)
        ttk.Label(top, text="ms").pack(side="left")

        byte_row = ttk.Frame(tx)
        byte_row.pack(fill="x", padx=4, pady=2)
        self.tx_byte_vars: list[tk.StringVar] = []
        self.tx_byte_entries: list[ttk.Entry] = []
        for i in range(8):
            cell = ttk.Frame(byte_row)
            cell.pack(side="left", padx=2)
            ttk.Label(cell, text=f"D{i}").pack()
            var = tk.StringVar(value="00")
            ent = ttk.Entry(cell, textvariable=var, width=4, justify="center", font=("Consolas", 11))
            ent.pack()
            self.tx_byte_vars.append(var)
            self.tx_byte_entries.append(ent)
            var.trace_add("write", self._on_dlc_changed)

        bar = ttk.Frame(tx)
        bar.pack(fill="x", padx=4, pady=2)
        ttk.Button(bar, text="Wyślij raz", command=self._manual_can_send).pack(side="left", padx=2)
        ttk.Button(bar, text="Cyklicznie", command=self._add_tx_job).pack(side="left", padx=2)
        ttk.Button(bar, text="Włącz / wyłącz", command=self._toggle_tx_job).pack(side="left", padx=2)
        ttk.Button(bar, text="Usuń", command=self._delete_tx_job).pack(side="left", padx=2)

        cols = ("on", "id", "dlc", "data", "period")
        self.tx_tree = ttk.Treeview(tx, columns=cols, show="headings", height=4)
        for key, title, width, stretch in (
            ("on", "", 36, False),
            ("id", "ID", 80, False),
            ("dlc", "DLC", 40, False),
            ("data", "Dane", 200, True),
            ("period", "Period", 70, False),
        ):
            self.tx_tree.heading(key, text=title)
            self.tx_tree.column(key, width=width, stretch=stretch, anchor="center")
        self.tx_tree.column("data", anchor="w")
        self.tx_tree.pack(fill="both", expand=True, padx=4, pady=2)
        self.tx_tree.bind("<Double-1>", self._load_tx_job)
        self.tx_tree.bind("<<TreeviewSelect>>", lambda _e: self._remember_tree(self.tx_tree))
        self._job_tree = self.tx_tree
        self.tx_id_var.trace_add("write", self._on_dlc_changed)
        self.tx_period_var.trace_add("write", self._on_dlc_changed)

    def _build_can_rx_panel(self, parent: ttk.Frame) -> None:
        mon = ttk.LabelFrame(parent, text="Odbiór")
        mon.pack(fill="both", expand=True, padx=4, pady=2)
        head = ttk.Frame(mon)
        head.pack(fill="x", padx=4, pady=1)
        self.can_counter = tk.StringVar(value="TX 0   RX 0")
        ttk.Label(head, textvariable=self.can_counter).pack(side="left")
        ttk.Button(head, text="Wyczyść", command=self._clear_can_monitor).pack(side="right")
        cols = ("dir", "id", "dlc", "data", "count")
        self.can_tree = ttk.Treeview(mon, columns=cols, show="headings", height=6)
        for key, title, width, stretch in (
            ("dir", "", 42, False),
            ("id", "ID", 90, False),
            ("dlc", "DLC", 40, False),
            ("data", "Dane", 200, True),
            ("count", "Ile", 50, False),
        ):
            self.can_tree.heading(key, text=title)
            self.can_tree.column(key, width=width, stretch=stretch, anchor="center")
        self.can_tree.column("data", anchor="w")
        self.can_tree.pack(fill="both", expand=True, padx=4, pady=2)

    def _remember_tree(self, tree: ttk.Treeview) -> None:
        self._job_tree = tree

    def _install_wake_jobs(self) -> None:
        frames = (
            (0x510, 100, bytes([0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
            (0x12F, 100, encode_ignition(True)),
            (0x34A, 20, bytes([0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
            (0x2F8, 1000, bytes([0x24, 0x0C, 0x0F, 0x0E, 0x00, 0x00, 0x00, 0xFF])),
        )
        for can_id, period_ms, data in frames:
            self._tx_seq += 1
            job = {
                "iid": f"w{self._tx_seq}",
                "can_id": can_id,
                "dlc": 8,
                "data": data,
                "extended": False,
                "remote": False,
                "period_s": period_ms / 1000.0,
                "enabled": True,
                "last": 0.0,
                "wake": True,
            }
            self._tx_jobs.append(job)
            self.wake_tree.insert("", "end", iid=job["iid"], values=self._job_row(job))

    def _add_slider(self, parent, title, var, amin, amax, fmt):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=8, pady=1)
        ttk.Label(frame, text=title, width=16).pack(side="left")
        val_lbl = ttk.Label(frame, width=10)
        val_lbl.pack(side="right")

        def _update(_=None):
            val_lbl.configure(text=fmt.format(var.get()))
            self._pending_sig = True

        ttk.Scale(frame, from_=amin, to=amax, variable=var, command=_update).pack(
            side="left", fill="x", expand=True, padx=8
        )
        _update()

    def _preset_idle(self) -> None:
        self.ign_var.set(True)
        self.rpm_var.set(800)
        self.spd_var.set(0)
        self._on_ignition()
        self._pending_sig = True

    def _preset_drive(self) -> None:
        self.ign_var.set(True)
        self.rpm_var.set(2200)
        self.spd_var.set(50)
        self.fuel_var.set(60)
        self.clt_var.set(90)
        self._on_ignition()
        self._pending_sig = True

    # ----------------------------- DoIP UDS --------------------------------
    def _build_doip_tab(self) -> None:
        pad = {"padx": 10, "pady": 6}
        conn = ttk.LabelFrame(self.tab_doip, text="DoIP connection (factory Ethernet path)")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="ESP32 IP").grid(row=0, column=0, sticky="w", padx=8, pady=4)
        self.doip_ip = tk.StringVar(value="192.168.0.10")
        ttk.Entry(conn, textvariable=self.doip_ip, width=18).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(conn, text=f":{DOIP_PORT}  LA BDC=0x{LA_BDC:04X}").grid(row=0, column=2, sticky="w")

        bf = ttk.Frame(conn)
        bf.grid(row=1, column=0, columnspan=3, sticky="we", padx=8, pady=6)
        ttk.Button(bf, text="Discover (UDP)", command=self._doip_discover).pack(side="left", padx=4)
        self.doip_btn = ttk.Button(bf, text="Connect + Routing", command=self._doip_toggle)
        self.doip_btn.pack(side="left", padx=4)
        self.doip_status = tk.StringVar(value="DoIP disconnected")
        ttk.Label(bf, textvariable=self.doip_status).pack(side="left", padx=8)

        self.doip_info = tk.StringVar(value="VIN: —")
        ttk.Label(conn, textvariable=self.doip_info).grid(row=2, column=0, columnspan=3, sticky="w", padx=8, pady=4)

        svc = ttk.LabelFrame(self.tab_doip, text="UDS services (same handler as CAN OBD)")
        svc.pack(fill="x", **pad)

        row = ttk.Frame(svc)
        row.pack(fill="x", padx=8, pady=6)
        ttk.Button(row, text="Default Session", command=lambda: self._uds_send(bytes([0x10, 0x01]))).pack(side="left", padx=3)
        ttk.Button(row, text="Extended Session", command=lambda: self._uds_send(bytes([0x10, 0x03]))).pack(side="left", padx=3)
        ttk.Button(row, text="Tester Present", command=lambda: self._uds_send(bytes([0x3E, 0x00]))).pack(side="left", padx=3)

        row2 = ttk.Frame(svc)
        row2.pack(fill="x", padx=8, pady=6)
        ttk.Button(row2, text="Read VIN (F190)", command=lambda: self._uds_send(bytes([0x22, 0xF1, 0x90]))).pack(side="left", padx=3)
        ttk.Button(row2, text="Read Session (F186)", command=lambda: self._uds_send(bytes([0x22, 0xF1, 0x86]))).pack(side="left", padx=3)
        ttk.Button(row2, text="Read Live (0100)", command=lambda: self._uds_send(bytes([0x22, 0x01, 0x00]))).pack(side="left", padx=3)
        ttk.Button(row2, text="Read SN (F18C)", command=lambda: self._uds_send(bytes([0x22, 0xF1, 0x8C]))).pack(side="left", padx=3)

        row3 = ttk.Frame(svc)
        row3.pack(fill="x", padx=8, pady=6)
        ttk.Button(row3, text="DTC count (19 01 FF)", command=lambda: self._uds_send(bytes([0x19, 0x01, 0xFF]))).pack(side="left", padx=3)
        ttk.Button(row3, text="Clear DTC (14 FF FF FF)", command=lambda: self._uds_send(bytes([0x14, 0xFF, 0xFF, 0xFF]))).pack(side="left", padx=3)

        raw = ttk.LabelFrame(self.tab_doip, text="Raw UDS (hex)")
        raw.pack(fill="x", **pad)
        self.raw_uds = tk.StringVar(value="22 F1 90")
        ttk.Entry(raw, textvariable=self.raw_uds).pack(side="left", fill="x", expand=True, padx=8, pady=8)
        ttk.Button(raw, text="Send", command=self._uds_send_raw).pack(side="left", padx=8)

        out = ttk.LabelFrame(self.tab_doip, text="Last UDS response")
        out.pack(fill="both", expand=True, **pad)
        self.uds_out = scrolledtext.ScrolledText(out, height=10, wrap="word", state="disabled")
        self.uds_out.pack(fill="both", expand=True, padx=6, pady=6)

    # ----------------------------- Log -------------------------------------
    def _build_log_tab(self) -> None:
        self.log = scrolledtext.ScrolledText(self.tab_log, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, padx=8, pady=8)
        ttk.Button(self.tab_log, text="Clear log", command=self._clear_log).pack(anchor="e", padx=8, pady=4)

    def _append_log(self, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {text}"
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _uds_show(self, text: str) -> None:
        self.uds_out.configure(state="normal")
        self.uds_out.insert("end", text + "\n")
        self.uds_out.see("end")
        self.uds_out.configure(state="disabled")
        self._append_log(text)

    # ----------------------------- Live conn -------------------------------
    def _mode_changed(self) -> None:
        mode = self.mode.get()
        uses_com = mode in ("serial", "robotell")
        self.port_combo.configure(state="readonly" if uses_com else "disabled")
        self.ip_entry.configure(state="normal" if mode == "udp" else "disabled")
        if mode == "robotell":
            self.robotell_row.grid()
        else:
            self.robotell_row.grid_remove()

    def _selected_port(self) -> str:
        label = self.port_var.get().strip()
        if label in self._port_map:
            return self._port_map[label]
        return label.split(" — ")[0].strip()

    def _refresh_ports(self) -> None:
        self._port_map = {}
        labels: list[str] = []
        preferred = ""
        if list_ports:
            for p in list_ports.comports():
                bits = []
                if p.description and p.description not in ("n/a", p.device):
                    bits.append(p.description)
                if p.vid is not None and p.pid is not None:
                    bits.append(f"{p.vid:04X}:{p.pid:04X}")
                label = p.device if not bits else f"{p.device} — {' '.join(bits)}"
                self._port_map[label] = p.device
                labels.append(label)
                if p.vid == 0x1A86 and not preferred:
                    preferred = label
        self.port_combo["values"] = labels
        if self.port_var.get() not in labels:
            self.port_var.set(preferred or (labels[0] if labels else ""))

    def _toggle_ctrl(self) -> None:
        if self._ctrl is not None or self._robotell is not None or self._connecting:
            self._ctrl_disconnect()
            return
        if self.mode.get() == "robotell":
            self._connect_robotell()
            return
        try:
            if self.mode.get() == "serial":
                port = self._selected_port()
                if not port:
                    raise RuntimeError("Select a COM port")
                self._ctrl = SerialTransport(port)
                self.ctrl_status.set(f"Serial {port}")
            else:
                ip = self.ip_var.get().strip()
                if not ip:
                    raise RuntimeError("Enter ESP32 IP")
                self._ctrl = UdpTransport(ip, UDP_CTRL_PORT)
                self.ctrl_status.set(f"UDP {ip}:{UDP_CTRL_PORT}")
        except Exception as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            self._ctrl = None
            return

        self.conn_btn.configure(text="Disconnect")
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._ctrl_send({"cmd": "ping"})
        self._pending_sig = True
        self.global_status.set("Live control connected")

    def _connect_robotell(self) -> None:
        port = self._selected_port()
        if not port:
            messagebox.showerror(
                APP_TITLE,
                "Brak portu COM.\nWepnij adapter Robotell i zainstaluj sterownik CH340, potem Refresh.",
            )
            return
        try:
            usb_baud = int(self.usb_baud_var.get().strip())
            can_bitrate = int(self.can_bitrate_var.get().strip())
        except ValueError:
            messagebox.showerror(APP_TITLE, "USB baud i CAN bit/s muszą być liczbami.")
            return

        self._connecting = True
        self._connect_cancel.clear()
        self._cyclic_enabled = bool(self.cyclic_var.get())
        self.conn_btn.configure(text="Anuluj")
        self.ctrl_status.set("Szukam adaptera…")
        auto = bool(self.auto_baud_var.get())

        def work() -> None:
            can = RobotellCan()
            try:
                def status(text: str) -> None:
                    if self._connect_cancel.is_set():
                        raise RuntimeError("Anulowano")
                    self._rx_q.put(("status", text))

                result = can.open(
                    port,
                    usb_baud,
                    can_bitrate,
                    auto_usb_baud=auto,
                    on_status=status,
                )
                if self._connect_cancel.is_set():
                    can.close()
                    self._rx_q.put(("cancel",))
                    return
                self._rx_q.put(("connected", can, result))
            except Exception as exc:
                can.close()
                if self._connect_cancel.is_set():
                    self._rx_q.put(("cancel",))
                else:
                    self._rx_q.put(("fail", str(exc)))

        threading.Thread(target=work, daemon=True).start()

    def _on_robotell_connected(self, can: RobotellCan, result) -> None:
        if self._connect_cancel.is_set():
            can.close()
            self._on_connect_cancelled()
            return
        self._connecting = False
        self._robotell = can
        self._tx_count = 0
        self._rx_count = 0
        sn = f"  S/N {result.serial_number}" if result.serial_number else ""
        text = f"Robotell {result.port}  USB {result.usb_baud}  CAN {result.can_bitrate}{sn}"
        self.ctrl_status.set(text)
        self.conn_btn.configure(text="Disconnect")
        self.global_status.set("Robotell USB-CAN połączony")
        self._append_log(text)
        self._publish_signals()
        self._can_stop.clear()
        self._can_thread = threading.Thread(target=self._can_worker, daemon=True)
        self._can_thread.start()

    def _on_robotell_fail(self, err: str) -> None:
        self._connecting = False
        self._robotell = None
        self.conn_btn.configure(text="Connect")
        self.ctrl_status.set("Brak połączenia")
        self.global_status.set("Robotell: brak odpowiedzi")
        self._append_log("Robotell: " + err)
        messagebox.showerror(APP_TITLE, err)

    def _on_can_link_lost(self, err: str) -> None:
        if self._robotell is not None:
            self._robotell.close()
            self._robotell = None
        self._connecting = False
        self.conn_btn.configure(text="Connect")
        self.ctrl_status.set("Połączenie USB zerwane")
        self.global_status.set("Robotell rozłączony")
        self._append_log("Robotell link lost: " + err)

    def _on_connect_cancelled(self) -> None:
        self._connecting = False
        if self._robotell is None:
            self.conn_btn.configure(text="Connect")
            self.ctrl_status.set("Anulowano")

    def _on_cyclic_toggle(self) -> None:
        self._cyclic_enabled = bool(self.cyclic_var.get())
        with self._tx_job_lock:
            for job in self._tx_jobs:
                if job.get("wake"):
                    job["enabled"] = self._cyclic_enabled
                    job["last"] = 0.0
                    self._paint_job(job)

    def _ctrl_disconnect(self) -> None:
        self._connect_cancel.set()
        self._can_stop.set()
        self._rx_stop.set()
        if self._can_thread and self._can_thread.is_alive():
            self._can_thread.join(timeout=0.8)
        self._can_thread = None
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        if self._ctrl:
            self._ctrl.close()
        self._ctrl = None
        if self._robotell:
            self._robotell.close()
        self._robotell = None
        self._connecting = False
        self.conn_btn.configure(text="Connect")
        self.ctrl_status.set("Rozłączony")
        self.global_status.set("Rozłączony")

    def _can_worker(self) -> None:
        last: dict[int, float] = {}
        while not self._can_stop.is_set():
            can = self._robotell
            if can is None:
                break
            if self._cyclic_enabled:
                now = time.monotonic()
                with self._tx_job_lock:
                    covered = {job["can_id"] for job in self._tx_jobs}
                for can_id, data in due(self._signals, last, now):
                    if can_id in covered:
                        continue
                    try:
                        can.send(can_id, data)
                        self._tx_count += 1
                    except Exception as exc:
                        self._rx_q.put(("lost", str(exc)))
                        self._can_stop.set()
                        break
            if self._can_stop.is_set():
                break
            due_jobs: list[dict] = []
            with self._tx_job_lock:
                now = time.monotonic()
                for job in self._tx_jobs:
                    if not job["enabled"] or job["period_s"] <= 0:
                        continue
                    if (now - job["last"]) >= job["period_s"]:
                        job["last"] = now
                        due_jobs.append(job)
            for job in due_jobs:
                if self._can_stop.is_set():
                    break
                try:
                    can.send(
                        job["can_id"], job["data"],
                        extended=job["extended"], remote=job["remote"], dlc=job["dlc"],
                    )
                    self._tx_count += 1
                    self._rx_q.put(("can", self._frame_from_job(job), "TX"))
                except Exception as exc:
                    self._rx_q.put(("lost", str(exc)))
                    self._can_stop.set()
                    break
            try:
                while not self._can_stop.is_set():
                    frame = can.recv(0)
                    if frame is None:
                        break
                    if frame.arbitration_id >= 0x01FFFE00:
                        continue
                    self._rx_count += 1
                    self._rx_q.put(("can", frame))
            except Exception as exc:
                self._rx_q.put(("lost", str(exc)))
                self._can_stop.set()
                break
            time.sleep(0.002)

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            ctrl = self._ctrl
            if ctrl is None:
                break
            line = ctrl.read_line(0.1)
            if line:
                self._rx_q.put(line)

    def _poll_rx(self) -> None:
        try:
            while True:
                item = self._rx_q.get_nowait()
                if isinstance(item, tuple) and item:
                    kind = item[0]
                    if kind == "can":
                        direction = item[2] if len(item) > 2 else "RX"
                        self._note_can_frame(item[1], direction)
                    elif kind == "status":
                        self.ctrl_status.set(item[1])
                    elif kind == "connected":
                        self._on_robotell_connected(item[1], item[2])
                    elif kind == "fail":
                        self._on_robotell_fail(item[1])
                    elif kind == "cancel":
                        self._on_connect_cancelled()
                    elif kind == "lost":
                        self._on_can_link_lost(item[1])
                    else:
                        self._append_log("CTRL << " + str(item))
                else:
                    self._append_log("CTRL << " + str(item))
        except queue.Empty:
            pass
        self._update_can_counters()
        self.after(50, self._poll_rx)

    def _note_can_frame(self, frame, direction: str = "RX") -> None:
        can_id = frame.arbitration_id
        extended = bool(frame.is_extended)
        key = (direction, can_id, extended)
        count = self._rx_seen.get(key, 0) + 1
        self._rx_seen[key] = count
        if frame.is_remote:
            data = "RTR"
        else:
            data = hex_bytes(frame.data)
        suffix = " EXT" if extended else " STD"
        iid = f"{direction}:{can_id:X}:{suffix}"
        values = (direction, f"0x{can_id:X}{suffix}", frame.dlc, data, count)
        if self.can_tree.exists(iid):
            self.can_tree.item(iid, values=values)
        else:
            self.can_tree.insert("", "end", iid=iid, values=values)
            self.can_tree.see(iid)

    def _update_can_counters(self) -> None:
        self.can_counter.set(f"TX {self._tx_count}    RX {self._rx_count}")

    def _clear_can_monitor(self) -> None:
        kids = self.can_tree.get_children()
        if kids:
            self.can_tree.delete(*kids)
        self._rx_seen.clear()
        self._rx_count = 0

    def _on_dlc_changed(self, *_args) -> None:
        try:
            dlc = int(self.tx_dlc_var.get())
        except ValueError:
            dlc = 0
        dlc = max(0, min(8, dlc))
        remote = self.tx_kind.get() == "remote"
        for i, entry in enumerate(self.tx_byte_entries):
            entry.configure(state="disabled" if remote or i >= dlc else "normal")

    def _parse_tx_frame(self) -> tuple[int, bytes, int, bool, bool]:
        raw_id = self.tx_id_var.get().strip().lower().replace("0x", "")
        if not raw_id:
            raise ValueError("wpisz ID, np. 12F")
        can_id = int(raw_id, 16)
        dlc = int(self.tx_dlc_var.get())
        if not 0 <= dlc <= 8:
            raise ValueError("DLC musi być 0–8")
        extended = self.tx_format.get() == "ext"
        if not extended and can_id > 0x7FF:
            raise ValueError("ID standard max 7FF — wybierz rozszerzony albo mniejsze ID")
        if can_id < 0 or can_id > 0x1FFFFFFF:
            raise ValueError("ID poza zakresem")
        remote = self.tx_kind.get() == "remote"
        data = bytearray()
        if not remote:
            for i in range(dlc):
                text = self.tx_byte_vars[i].get().strip()
                if not text:
                    text = "00"
                if len(text) > 2:
                    raise ValueError(f"D{i} to jeden bajt, np. A5")
                value = int(text, 16)
                if not 0 <= value <= 0xFF:
                    raise ValueError(f"D{i} poza 00–FF")
                data.append(value)
        return can_id, bytes(data), dlc, extended, remote

    def _frame_from_job(self, job: dict) -> CanFrame:
        return CanFrame(
            job["can_id"], job["data"], job["extended"], job["remote"], job["dlc"],
        )

    def _job_row(self, job: dict) -> tuple[str, str, str, str, str]:
        if job["remote"]:
            data = "RTR"
        else:
            data = hex_bytes(job["data"]) if job["data"] else "(puste)"
        if job["period_s"] <= 0:
            period = "—"
        else:
            period = f"{job['period_s'] * 1000:.0f} ms"
        ident = f"0x{job['can_id']:X}"
        if job["extended"]:
            ident += " EXT"
        return ("TAK" if job["enabled"] else "nie", ident, str(job["dlc"]), data, period)

    def _manual_can_send(self) -> None:
        if self._robotell is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz adapter Robotell (tryb Robotell USB-CAN).")
            return
        try:
            can_id, data, dlc, extended, remote = self._parse_tx_frame()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        try:
            self._robotell.send(can_id, data, extended=extended, remote=remote, dlc=dlc)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        self._tx_count += 1
        frame = CanFrame(can_id, data, extended, remote, dlc)
        self._note_can_frame(frame, "TX")
        kind = "RTR" if remote else hex_bytes(data)
        fmt = "EXT" if extended else "STD"
        self._append_log(f"TX  ID 0x{can_id:X}  {fmt}  DLC {dlc}  {kind}")

    def _read_period_s(self) -> float:
        text = self.tx_period_var.get().strip().replace(",", ".")
        if not text:
            return 0.0
        period_ms = float(text)
        if period_ms < 0:
            raise ValueError("Period nie może być ujemny")
        return period_ms / 1000.0

    def _add_tx_job(self) -> None:
        try:
            can_id, data, dlc, extended, remote = self._parse_tx_frame()
            period_s = self._read_period_s()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        if period_s <= 0:
            messagebox.showerror(APP_TITLE, "Wpisz Period w ms, np. 100.")
            return
        with self._tx_job_lock:
            for job in self._tx_jobs:
                if job["can_id"] == can_id and job["extended"] == extended and job["remote"] == remote:
                    job["data"] = data
                    job["dlc"] = dlc
                    job["period_s"] = period_s
                    job["enabled"] = True
                    job["last"] = 0.0
                    self._paint_job(job)
                    return
        self._tx_seq += 1
        job = {
            "iid": str(self._tx_seq),
            "can_id": can_id,
            "dlc": dlc,
            "data": data,
            "extended": extended,
            "remote": remote,
            "period_s": period_s,
            "enabled": True,
            "last": 0.0,
            "wake": False,
        }
        with self._tx_job_lock:
            self._tx_jobs.append(job)
        self.tx_tree.insert("", "end", iid=job["iid"], values=self._job_row(job))
        self.tx_tree.selection_set(job["iid"])
        self.tx_tree.see(job["iid"])
        self._job_tree = self.tx_tree

    def _paint_job(self, job: dict) -> None:
        tree = self.wake_tree if job.get("wake") else self.tx_tree
        if tree.exists(job["iid"]):
            tree.item(job["iid"], values=self._job_row(job))

    def _selected_job(self) -> dict | None:
        tree = getattr(self, "_job_tree", None)
        if tree is None:
            return None
        sel = tree.selection()
        if not sel:
            return None
        iid = sel[0]
        with self._tx_job_lock:
            for job in self._tx_jobs:
                if job["iid"] == iid:
                    return job
        return None

    def _toggle_tx_job(self) -> None:
        job = self._selected_job()
        if job is None:
            messagebox.showinfo(APP_TITLE, "Zaznacz wiersz.")
            return
        job["enabled"] = not job["enabled"]
        job["last"] = 0.0
        self._paint_job(job)

    def _delete_tx_job(self) -> None:
        job = self._selected_job()
        if job is None:
            return
        tree = self.wake_tree if job.get("wake") else self.tx_tree
        with self._tx_job_lock:
            self._tx_jobs = [item for item in self._tx_jobs if item["iid"] != job["iid"]]
        if tree.exists(job["iid"]):
            tree.delete(job["iid"])

    def _load_tx_job(self, _event=None) -> None:
        job = self._selected_job()
        if job is None:
            return
        self.tx_id_var.set(f"{job['can_id']:X}")
        self.tx_dlc_var.set(str(job["dlc"]))
        self.tx_format.set("ext" if job["extended"] else "std")
        self.tx_kind.set("remote" if job["remote"] else "data")
        for i in range(8):
            if i < len(job["data"]):
                self.tx_byte_vars[i].set(f"{job['data'][i]:02X}")
        if job["period_s"] > 0:
            self.tx_period_var.set(f"{job['period_s'] * 1000:.0f}")
        else:
            self.tx_period_var.set("0")
        self._on_dlc_changed()

    def _ping(self) -> None:
        if self._robotell is not None:
            self._append_log(
                f"{self.ctrl_status.get()}  TX {self._tx_count}  RX {self._rx_count}"
            )
            return
        self._ctrl_send({"cmd": "ping"})

    def _publish_signals(self) -> None:
        self._signals = Signals(
            ignition=bool(self.ign_var.get()),
            rpm=int(self.rpm_var.get()),
            speed_kmh=float(self.spd_var.get()),
            fuel_pct=float(self.fuel_var.get()),
            coolant_c=int(self.clt_var.get()),
        )
        payload = encode_ignition(self._signals.ignition)
        with self._tx_job_lock:
            for job in self._tx_jobs:
                if job.get("wake") and job["can_id"] == 0x12F and job["data"] != payload:
                    job["data"] = payload
                    self._paint_job(job)

    def _ctrl_send(self, obj: dict) -> None:
        if self._ctrl is None:
            return
        line = json.dumps(obj, separators=(",", ":"))
        try:
            self._ctrl.send_line(line)
            self._append_log("CTRL >> " + line)
        except Exception as exc:
            self._append_log(f"CTRL send error: {exc}")

    def _on_ignition(self) -> None:
        self._ctrl_send({"cmd": "ign", "on": 1 if self.ign_var.get() else 0})

    def _stream_sig_tick(self) -> None:
        self._publish_signals()
        now = time.monotonic()
        if self._pending_sig and self._ctrl is not None and (now - self._last_sig_sent) >= (1.0 / SEND_HZ):
            self._pending_sig = False
            self._last_sig_sent = now
            self._ctrl_send({
                "cmd": "sig",
                "rpm": int(self.rpm_var.get()),
                "spd": float(self.spd_var.get()),
                "fuel": float(self.fuel_var.get()),
                "clt": int(self.clt_var.get()),
            })
        self.after(20, self._stream_sig_tick)

    # ----------------------------- DoIP actions ----------------------------
    def _doip_discover(self) -> None:
        ip = self.doip_ip.get().strip()

        def work():
            try:
                info = self._doip.discover(ip)
                self.after(0, lambda: self._on_discover_done(info))
            except Exception as exc:
                self.after(0, lambda: messagebox.showerror(APP_TITLE, str(exc)))

        threading.Thread(target=work, daemon=True).start()
        self._append_log(f"DoIP discover → {ip}:{DOIP_PORT}")

    def _on_discover_done(self, info: dict | None) -> None:
        if not info:
            messagebox.showwarning(APP_TITLE, "No DoIP Vehicle Announcement.\nCheck Ethernet / IP / firmware.")
            self.doip_status.set("Discover failed")
            return
        self.doip_ip.set(info["addr"])
        self.doip_info.set(f"VIN: {info['vin']}   LA: 0x{info['la']:04X}   from {info['addr']}")
        self.doip_status.set("Discovered")
        self._append_log(f"DoIP announce VIN={info['vin']} LA=0x{info['la']:04X}")

    def _doip_toggle(self) -> None:
        if self._doip.connected:
            self._doip.close()
            self.doip_btn.configure(text="Connect + Routing")
            self.doip_status.set("DoIP disconnected")
            self.global_status.set("DoIP disconnected")
            return

        ip = self.doip_ip.get().strip()

        def work():
            try:
                self._doip.connect(ip)
                self.after(0, self._on_doip_connected)
            except Exception as exc:
                self.after(0, lambda: self._on_doip_fail(str(exc)))

        self.doip_status.set("Connecting…")
        threading.Thread(target=work, daemon=True).start()

    def _on_doip_connected(self) -> None:
        self.doip_btn.configure(text="Disconnect DoIP")
        self.doip_status.set(f"Connected {self.doip_ip.get()}:{DOIP_PORT}")
        self.global_status.set("DoIP routing active")
        self._append_log("DoIP routing activation OK")

    def _on_doip_fail(self, err: str) -> None:
        self.doip_status.set("Connect failed")
        messagebox.showerror(APP_TITLE, err)
        self._append_log("DoIP error: " + err)

    def _uds_send_raw(self) -> None:
        try:
            data = parse_hex_bytes(self.raw_uds.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, "Invalid hex")
            return
        self._uds_send(data)

    def _uds_send(self, uds: bytes) -> None:
        if not self._doip.connected:
            messagebox.showinfo(APP_TITLE, "Connect DoIP first (tab DoIP UDS).")
            return

        def work():
            try:
                resp = self._doip.diagnostic(uds)
                decoded = self._decode_uds(uds, resp)
                self.after(0, lambda: self._uds_show(
                    f">> {hex_bytes(uds)}\n<< {hex_bytes(resp)}\n{decoded}\n"
                ))
            except Exception as exc:
                self.after(0, lambda: self._uds_show(f"UDS error: {exc}\n"))

        threading.Thread(target=work, daemon=True).start()

    @staticmethod
    def _decode_uds(req: bytes, resp: bytes) -> str:
        if not resp:
            return "(empty response)"
        if resp[0] == 0x7F and len(resp) >= 3:
            return f"NegativeResponse SID=0x{resp[1]:02X} NRC=0x{resp[2]:02X}"
        if req and req[0] == 0x22 and resp[0] == 0x62 and len(req) >= 3:
            did = (req[1] << 8) | req[2]
            data = resp[3:]
            if did == 0xF190:
                return f"VIN = {data.decode('ascii', errors='replace')}"
            if did == 0xF186 and data:
                return f"Active session = 0x{data[0]:02X}"
            if did == 0xF18C:
                return f"Serial = {data.decode('ascii', errors='replace')}"
            if did == 0x0100 and len(data) >= 7:
                ign = data[0]
                rpm = (data[1] << 8) | data[2]
                spd = ((data[3] << 8) | data[4]) / 10.0
                fuel = data[5]
                clt = data[6] if data[6] < 128 else data[6] - 256
                return f"Live: ign={ign} rpm={rpm} spd={spd:.1f}km/h fuel={fuel}% clt={clt}°C"
        if resp[0] == 0x50:
            return f"Session changed, type=0x{resp[1]:02X}" if len(resp) > 1 else "Session OK"
        if resp[0] == 0x7E:
            return "TesterPresent OK"
        return "OK"

    def _on_close(self) -> None:
        self._ctrl_disconnect()
        self._doip.close()
        self.destroy()


def main() -> None:
    app = CompanionApp()
    app.mainloop()


if __name__ == "__main__":
    main()
