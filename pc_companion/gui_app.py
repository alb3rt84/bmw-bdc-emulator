#!/usr/bin/env python3
"""ZGW emulator window: KL30 / KL15 switches and a log of Ethernet plus MCP2515 RX."""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk


APP_TITLE = "ZGW Emulator"
UDP_PORT = 13401
DEFAULT_IP = "169.254.1.20"


class UdpLink:
    def __init__(self, host: str, port: int = UDP_PORT) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.2)
        self._sock.bind(("", 0))

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def send_line(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        self._sock.sendto(line.encode("utf-8"), self._addr)

    def read_line(self) -> str | None:
        try:
            data, _ = self._sock.recvfrom(512)
        except (socket.timeout, OSError):
            return None
        return data.decode("utf-8", errors="replace").strip()


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.minsize(640, 420)
        self.geometry("720x480")

        self._link: UdpLink | None = None
        self._rx: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ignore_switch = False

        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="IP ZGW").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ttk.Entry(top, textvariable=self.ip_var, width=18).pack(side="left", padx=6)
        self.conn_btn = ttk.Button(top, text="Połącz", command=self._toggle)
        self.conn_btn.pack(side="left")
        self.status = tk.StringVar(value="Rozłączony")
        ttk.Label(top, textvariable=self.status).pack(side="left", padx=10)

        clamps = ttk.LabelFrame(self, text="Ramka 0x12F na MCP2515")
        clamps.pack(fill="x", padx=10, pady=4)
        self.kl30_var = tk.BooleanVar(value=True)
        self.kl15_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            clamps, text="KL30", variable=self.kl30_var, command=self._send_clamps,
        ).pack(side="left", padx=16, pady=8)
        ttk.Checkbutton(
            clamps, text="KL15", variable=self.kl15_var, command=self._send_clamps,
        ).pack(side="left", padx=16, pady=8)
        ttk.Label(
            clamps,
            text="Oba włączone: 45 FF 45 FF FF FF FF FF",
        ).pack(side="left", padx=8)

        log_frame = ttk.LabelFrame(self, text="Log Ethernet i ramki MCP2515")
        log_frame.pack(fill="both", expand=True, padx=10, pady=6)
        self.log = scrolledtext.ScrolledText(log_frame, wrap="word", state="disabled", font=("Consolas", 10))
        self.log.pack(fill="both", expand=True, padx=6, pady=6)
        ttk.Button(log_frame, text="Wyczyść", command=self._clear_log).pack(anchor="e", padx=6, pady=4)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._poll)
        self.after(5000, self._heartbeat)

    def _append(self, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{ts}] {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _toggle(self) -> None:
        if self._link is not None:
            self._disconnect()
            return
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showerror(APP_TITLE, "Wpisz adres ZGW.")
            return
        try:
            self._link = UdpLink(ip)
        except OSError as exc:
            self._link = None
            messagebox.showerror(APP_TITLE, str(exc))
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        self.conn_btn.configure(text="Rozłącz")
        self.status.set(f"UDP {ip}:{UDP_PORT}")
        self._append(f"Połączono z {ip}:{UDP_PORT}")
        self._send_raw({"cmd": "ping"})
        self._send_clamps()

    def _disconnect(self) -> None:
        self._stop.set()
        link = self._link
        self._link = None
        if link is not None:
            link.close()
        self.conn_btn.configure(text="Połącz")
        self.status.set("Rozłączony")
        self._append("Rozłączono")

    def _send_raw(self, obj: dict) -> None:
        link = self._link
        if link is None:
            return
        try:
            link.send_line(json.dumps(obj, separators=(",", ":")))
        except OSError as exc:
            self._append(f"Błąd wysyłki: {exc}")

    def _send_clamps(self) -> None:
        if self._ignore_switch:
            return
        self._send_raw({
            "cmd": "kl",
            "kl30": 1 if self.kl30_var.get() else 0,
            "kl15": 1 if self.kl15_var.get() else 0,
        })

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            link = self._link
            if link is None:
                break
            line = link.read_line()
            if line:
                self._rx.put(line)

    def _poll(self) -> None:
        while True:
            try:
                line = self._rx.get_nowait()
            except queue.Empty:
                break
            self._show_line(line)
        self.after(50, self._poll)

    def _show_line(self, line: str) -> None:
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            self._append(line)
            return
        if not isinstance(obj, dict):
            self._append(line)
            return
        ev = obj.get("ev")
        if ev == "eth":
            self._append(str(obj.get("msg", "")))
            return
        if ev == "can":
            self._append("MCP  " + str(obj.get("msg", "")))
            return
        if obj.get("ok") == 1 and "kl30" in obj and "kl15" in obj:
            self.status.set(f"KL30={'ON' if obj['kl30'] else 'OFF'}  KL15={'ON' if obj['kl15'] else 'OFF'}")
            return
        if obj.get("ok") == 0:
            self._append("ZGW  " + str(obj.get("err", line)))

    def _heartbeat(self) -> None:
        if self._link is not None:
            self._send_raw({"cmd": "ping"})
        self.after(5000, self._heartbeat)

    def _on_close(self) -> None:
        self._disconnect()
        self.destroy()


def main() -> None:
    App().mainloop()


if __name__ == "__main__":
    main()
