#!/usr/bin/env python3
"""
BMW BDC/ZGM Bench Emulator — PC Companion GUI

Connect via USB-Serial (COM port) or UDP (ESP32 Ethernet :13401).
Sends JSON-line commands that the ESP32 injects into cyclic CAN frames.

Build Windows .exe (from this folder):
  pip install -r requirements.txt
  pyinstaller --noconfirm --onefile --windowed --name BmwBdcCompanion gui_app.py
"""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


APP_TITLE = "BMW BDC/ZGM Bench Companion"
UDP_PORT_DEFAULT = 13401
SERIAL_BAUD = 115200
SEND_HZ = 20  # slider stream rate while dragging


class Transport:
    def close(self) -> None: ...
    def send_line(self, line: str) -> None: ...
    def read_line(self, timeout: float = 0.05) -> str | None: ...


class SerialTransport(Transport):
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


class UdpTransport(Transport):
    def __init__(self, host: str, port: int = UDP_PORT_DEFAULT) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.05)
        self._sock.bind(("", 0))  # ephemeral local port for replies

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
        except socket.timeout:
            return None
        except OSError:
            return None
        return data.decode("utf-8", errors="replace").strip()


class CompanionApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("520x560")
        self.minsize(480, 520)

        self._tx: Transport | None = None
        self._rx_q: queue.Queue[str] = queue.Queue()
        self._rx_stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._pending_sig = False
        self._last_sig_sent = 0.0

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_rx)
        self.after(50, self._stream_sig_tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        pad = {"padx": 10, "pady": 6}

        conn = ttk.LabelFrame(self, text="Connection")
        conn.pack(fill="x", **pad)

        self.mode = tk.StringVar(value="serial")
        ttk.Radiobutton(conn, text="Serial COM", variable=self.mode, value="serial",
                        command=self._mode_changed).grid(row=0, column=0, sticky="w", padx=8, pady=4)
        ttk.Radiobutton(conn, text="UDP Ethernet", variable=self.mode, value="udp",
                        command=self._mode_changed).grid(row=0, column=1, sticky="w", padx=8, pady=4)

        ttk.Label(conn, text="COM port").grid(row=1, column=0, sticky="w", padx=8)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=18, state="readonly")
        self.port_combo.grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=1, column=2, padx=4)

        ttk.Label(conn, text="ESP32 IP").grid(row=2, column=0, sticky="w", padx=8)
        self.ip_var = tk.StringVar(value="192.168.0.10")
        self.ip_entry = ttk.Entry(conn, textvariable=self.ip_var, width=18)
        self.ip_entry.grid(row=2, column=1, sticky="we", padx=4)
        ttk.Label(conn, text=f"UDP :{UDP_PORT_DEFAULT}").grid(row=2, column=2, sticky="w")

        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_conn)
        self.conn_btn.grid(row=3, column=0, columnspan=2, sticky="we", padx=8, pady=8)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(conn, textvariable=self.status_var).grid(row=3, column=2, sticky="w")
        conn.columnconfigure(1, weight=1)

        dash = ttk.LabelFrame(self, text="Control Dashboard")
        dash.pack(fill="both", expand=True, **pad)

        self.ign_var = tk.BooleanVar(value=True)
        self.ign_btn = ttk.Checkbutton(
            dash, text="Terminal 15 (Ignition) ON", variable=self.ign_var,
            command=self._on_ignition, style="Switch.TCheckbutton"
        )
        self.ign_btn.pack(anchor="w", padx=12, pady=8)

        self.rpm_var = tk.DoubleVar(value=0)
        self.spd_var = tk.DoubleVar(value=0)
        self.fuel_var = tk.DoubleVar(value=50)
        self.clt_var = tk.DoubleVar(value=90)

        self._add_slider(dash, "Engine RPM", self.rpm_var, 0, 8000, 50, "{:.0f} rpm")
        self._add_slider(dash, "Vehicle Speed", self.spd_var, 0, 300, 1, "{:.0f} km/h")
        self._add_slider(dash, "Fuel Level", self.fuel_var, 0, 100, 1, "{:.0f} %")
        self._add_slider(dash, "Coolant Temp", self.clt_var, -40, 140, 1, "{:.0f} °C")

        logf = ttk.LabelFrame(self, text="Device replies")
        logf.pack(fill="both", expand=False, **pad)
        self.log = tk.Text(logf, height=8, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, padx=6, pady=6)

        self._mode_changed()

    def _add_slider(self, parent, title, var, amin, amax, res, fmt):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=12, pady=4)
        ttk.Label(frame, text=title, width=16).pack(side="left")
        val_lbl = ttk.Label(frame, width=10)
        val_lbl.pack(side="right")

        def _update(_=None):
            val_lbl.configure(text=fmt.format(var.get()))
            self._pending_sig = True

        scale = ttk.Scale(frame, from_=amin, to=amax, variable=var, command=_update)
        scale.pack(side="left", fill="x", expand=True, padx=8)
        _update()

    def _mode_changed(self) -> None:
        serial_mode = self.mode.get() == "serial"
        state_s = "readonly" if serial_mode else "disabled"
        state_u = "normal" if not serial_mode else "disabled"
        self.port_combo.configure(state=state_s)
        self.ip_entry.configure(state=state_u)

    def _refresh_ports(self) -> None:
        ports = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # ----------------------------------------------------------- connection
    def _toggle_conn(self) -> None:
        if self._tx is not None:
            self._disconnect()
            return
        try:
            if self.mode.get() == "serial":
                port = self.port_var.get().strip()
                if not port:
                    raise RuntimeError("Select a COM port")
                self._tx = SerialTransport(port)
                self.status_var.set(f"Serial {port}")
            else:
                ip = self.ip_var.get().strip()
                if not ip:
                    raise RuntimeError("Enter ESP32 IP")
                self._tx = UdpTransport(ip, UDP_PORT_DEFAULT)
                self.status_var.set(f"UDP {ip}:{UDP_PORT_DEFAULT}")
        except Exception as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            self._tx = None
            return

        self.conn_btn.configure(text="Disconnect")
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._send({"cmd": "ping"})
        self._pending_sig = True

    def _disconnect(self) -> None:
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        if self._tx:
            self._tx.close()
        self._tx = None
        self.conn_btn.configure(text="Connect")
        self.status_var.set("Disconnected")

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            tx = self._tx
            if tx is None:
                break
            line = tx.read_line(0.1)
            if line:
                self._rx_q.put(line)

    def _poll_rx(self) -> None:
        try:
            while True:
                line = self._rx_q.get_nowait()
                self._append_log(line)
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    def _send(self, obj: dict) -> None:
        if self._tx is None:
            return
        line = json.dumps(obj, separators=(",", ":"))
        try:
            self._tx.send_line(line)
            self._append_log(">> " + line)
        except Exception as exc:
            self._append_log(f"send error: {exc}")

    def _on_ignition(self) -> None:
        self._send({"cmd": "ign", "on": 1 if self.ign_var.get() else 0})

    def _stream_sig_tick(self) -> None:
        now = time.monotonic()
        if self._pending_sig and self._tx is not None and (now - self._last_sig_sent) >= (1.0 / SEND_HZ):
            self._pending_sig = False
            self._last_sig_sent = now
            self._send({
                "cmd": "sig",
                "rpm": int(self.rpm_var.get()),
                "spd": float(self.spd_var.get()),
                "fuel": float(self.fuel_var.get()),
                "clt": int(self.clt_var.get()),
            })
        self.after(20, self._stream_sig_tick)

    def _on_close(self) -> None:
        self._disconnect()
        self.destroy()


def main() -> None:
    app = CompanionApp()
    app.mainloop()


if __name__ == "__main__":
    main()
