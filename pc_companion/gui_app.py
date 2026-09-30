#!/usr/bin/env python3
"""
BMW BDC/ZGM Bench Companion — improved GUI

Tabs:
  1) Live Control  — JSON Serial/UDP :13401 (KL15, RPM, speed, fuel, coolant)
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
        self.geometry("720x680")
        self.minsize(640, 600)

        self._ctrl: SerialTransport | UdpTransport | None = None
        self._rx_q: queue.Queue[str] = queue.Queue()
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._pending_sig = False
        self._last_sig_sent = 0.0
        self._doip = DoipClient()

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_rx)
        self.after(50, self._stream_sig_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text=APP_TITLE, font=("Segoe UI", 12, "bold")).pack(side="left")
        self.global_status = tk.StringVar(value="Ready")
        ttk.Label(top, textvariable=self.global_status).pack(side="right")

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=10, pady=4)

        self.tab_live = ttk.Frame(nb)
        self.tab_cfg = ttk.Frame(nb)
        self.tab_doip = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_live, text="Live Control")
        nb.add(self.tab_cfg, text="BDC Identity")
        nb.add(self.tab_doip, text="DoIP UDS (BDC)")
        nb.add(self.tab_log, text="Log")

        self._build_live_tab()
        self._build_cfg_tab()
        self._build_doip_tab()
        self._build_log_tab()

    # ----------------------------- Live Control -----------------------------
    def _build_live_tab(self) -> None:
        pad = {"padx": 10, "pady": 6}
        conn = ttk.LabelFrame(self.tab_live, text="JSON companion link (Serial / UDP :13401)")
        conn.pack(fill="x", **pad)

        self.mode = tk.StringVar(value="serial")
        ttk.Radiobutton(conn, text="Serial COM", variable=self.mode, value="serial",
                        command=self._mode_changed).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        ttk.Radiobutton(conn, text="UDP :13401", variable=self.mode, value="udp",
                        command=self._mode_changed).grid(row=0, column=1, sticky="w", padx=8, pady=4)

        ttk.Label(conn, text="COM").grid(row=1, column=0, sticky="w", padx=8)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=1, column=2, padx=4)

        ttk.Label(conn, text="ESP32 IP").grid(row=2, column=0, sticky="w", padx=8)
        self.ip_var = tk.StringVar(value="192.168.0.10")
        self.ip_entry = ttk.Entry(conn, textvariable=self.ip_var, width=18)
        self.ip_entry.grid(row=2, column=1, sticky="we", padx=4)

        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_ctrl)
        self.conn_btn.grid(row=3, column=0, columnspan=2, sticky="we", padx=8, pady=8)
        self.ctrl_status = tk.StringVar(value="Disconnected")
        ttk.Label(conn, textvariable=self.ctrl_status).grid(row=3, column=2, sticky="w")
        conn.columnconfigure(1, weight=1)

        dash = ttk.LabelFrame(self.tab_live, text="Bench signals → cyclic CAN")
        dash.pack(fill="both", expand=True, **pad)

        self.ign_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            dash, text="Terminal 15 (Ignition) ON", variable=self.ign_var,
            command=self._on_ignition,
        ).pack(anchor="w", padx=12, pady=8)

        self.rpm_var = tk.DoubleVar(value=0)
        self.spd_var = tk.DoubleVar(value=0)
        self.fuel_var = tk.DoubleVar(value=50)
        self.clt_var = tk.DoubleVar(value=90)
        self._add_slider(dash, "Engine RPM", self.rpm_var, 0, 8000, "{:.0f} rpm")
        self._add_slider(dash, "Vehicle Speed", self.spd_var, 0, 300, "{:.0f} km/h")
        self._add_slider(dash, "Fuel Level", self.fuel_var, 0, 100, "{:.0f} %")
        self._add_slider(dash, "Coolant Temp", self.clt_var, -40, 140, "{:.0f} °C")

        btns = ttk.Frame(dash)
        btns.pack(fill="x", padx=12, pady=8)
        ttk.Button(btns, text="Idle preset", command=self._preset_idle).pack(side="left", padx=4)
        ttk.Button(btns, text="Drive 50 km/h", command=self._preset_drive).pack(side="left", padx=4)
        ttk.Button(btns, text="Ping ECU JSON", command=lambda: self._ctrl_send({"cmd": "ping"})).pack(side="left", padx=4)

        self._mode_changed()

    def _add_slider(self, parent, title, var, amin, amax, fmt):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=12, pady=4)
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

    # ----------------------------- BDC Identity ----------------------------
    def _build_cfg_tab(self) -> None:
        pad = {"padx": 10, "pady": 6}
        info = ttk.LabelFrame(self.tab_cfg, text="VIN / FA / I-Stufe (zapis w NVS na ESP)")
        info.pack(fill="both", expand=True, **pad)

        form = ttk.Frame(info)
        form.pack(fill="x", padx=8, pady=8)

        self.cfg_vin = tk.StringVar(value="WBADEMOGCHASSIS01")
        self.cfg_fa = tk.StringVar(value="")
        self.cfg_istufe = tk.StringVar(value="")
        self.cfg_serial = tk.StringVar(value="")
        self.cfg_model = tk.StringVar(value="G30")
        self.cfg_la = tk.StringVar(value="16")

        rows = [
            ("VIN (17 znaków)", self.cfg_vin),
            ("FA (ASCII / paste)", self.cfg_fa),
            ("I-Stufe", self.cfg_istufe),
            ("Serial (F18C)", self.cfg_serial),
            ("Model / Baureihe", self.cfg_model),
            ("DoIP LA (dec, 16=0x0010)", self.cfg_la),
        ]
        for i, (label, var) in enumerate(rows):
            ttk.Label(form, text=label).grid(row=i, column=0, sticky="w", pady=3)
            ttk.Entry(form, textvariable=var, width=56).grid(
                row=i, column=1, sticky="we", padx=6, pady=3
            )
        form.columnconfigure(1, weight=1)

        btns = ttk.Frame(info)
        btns.pack(fill="x", padx=8, pady=8)
        ttk.Button(btns, text="Load from ESP", command=self._cfg_load).pack(side="left", padx=4)
        ttk.Button(btns, text="Save to ESP (NVS)", command=self._cfg_save).pack(side="left", padx=4)
        ttk.Button(btns, text="Reset defaults", command=self._cfg_reset).pack(side="left", padx=4)

        ttk.Label(
            info,
            text="UDS DIDs: F190=VIN, F18C=Serial, 0101=FA, 0102=I-Stufe, 0103=Model\n"
                 "DoIP Vehicle Announcement używa VIN + LA z tej konfiguracji.",
            justify="left",
        ).pack(anchor="w", padx=8, pady=6)

    def _cfg_load(self) -> None:
        if self._ctrl is None:
            messagebox.showinfo(APP_TITLE, "Najpierw Connect w zakładce Live Control (COM/UDP).")
            return
        self._ctrl_send({"cmd": "cfg"})

    def _cfg_save(self) -> None:
        if self._ctrl is None:
            messagebox.showinfo(APP_TITLE, "Najpierw Connect w zakładce Live Control.")
            return
        vin = self.cfg_vin.get().strip().upper()
        if len(vin) != 17:
            messagebox.showerror(APP_TITLE, "VIN must be exactly 17 characters")
            return
        try:
            la = int(self.cfg_la.get().strip(), 0)
        except ValueError:
            messagebox.showerror(APP_TITLE, "Invalid LA")
            return
        self._ctrl_send({
            "cmd": "cfg",
            "vin": vin,
            "fa": self.cfg_fa.get().strip(),
            "istufe": self.cfg_istufe.get().strip(),
            "serial": self.cfg_serial.get().strip(),
            "model": self.cfg_model.get().strip(),
            "la": la,
            "save": 1,
        })

    def _cfg_reset(self) -> None:
        if self._ctrl is None:
            messagebox.showinfo(APP_TITLE, "Najpierw Connect w zakładce Live Control.")
            return
        self._ctrl_send({"cmd": "cfg", "reset": 1, "save": 1})

    def _apply_cfg_reply(self, obj: dict) -> None:
        if "vin" in obj:
            self.cfg_vin.set(str(obj.get("vin", "")))
        if "fa" in obj:
            self.cfg_fa.set(str(obj.get("fa", "")))
        if "istufe" in obj:
            self.cfg_istufe.set(str(obj.get("istufe", "")))
        if "serial" in obj:
            self.cfg_serial.set(str(obj.get("serial", "")))
        if "model" in obj:
            self.cfg_model.set(str(obj.get("model", "")))
        if "la" in obj:
            self.cfg_la.set(str(obj.get("la", "")))

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
        ttk.Button(row2, text="Read FA (0101)", command=lambda: self._uds_send(bytes([0x22, 0x01, 0x01]))).pack(side="left", padx=3)
        ttk.Button(row2, text="Read I-Stufe (0102)", command=lambda: self._uds_send(bytes([0x22, 0x01, 0x02]))).pack(side="left", padx=3)

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
        serial_mode = self.mode.get() == "serial"
        self.port_combo.configure(state="readonly" if serial_mode else "disabled")
        self.ip_entry.configure(state="disabled" if serial_mode else "normal")

    def _refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()] if list_ports else []
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_ctrl(self) -> None:
        if self._ctrl is not None:
            self._ctrl_disconnect()
            return
        try:
            if self.mode.get() == "serial":
                port = self.port_var.get().strip()
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

    def _ctrl_disconnect(self) -> None:
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        if self._ctrl:
            self._ctrl.close()
        self._ctrl = None
        self.conn_btn.configure(text="Connect")
        self.ctrl_status.set("Disconnected")

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
                line = self._rx_q.get_nowait()
                self._append_log("CTRL << " + line)
                if line.startswith("{") and "\"cfg\"" in line:
                    try:
                        obj = json.loads(line)
                        if obj.get("ok") and (obj.get("cfg") == 1 or "vin" in obj):
                            self._apply_cfg_reply(obj)
                    except json.JSONDecodeError:
                        pass
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

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
            if did == 0x0101:
                return f"FA = {data.decode('ascii', errors='replace')}"
            if did == 0x0102:
                return f"I-Stufe = {data.decode('ascii', errors='replace')}"
            if did == 0x0103:
                return f"Model = {data.decode('ascii', errors='replace')}"
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
