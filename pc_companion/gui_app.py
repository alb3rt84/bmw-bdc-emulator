#!/usr/bin/env python3
"""ZGW emulator window: KL30 / KL15 switches and a log of Ethernet plus MCP2515 RX."""

from __future__ import annotations

import json
import queue
import socket
import sys
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

from editor_window import VehicleEditor
from vehicle_xml import Fa, SvtEcu, fa_bytes, fa_from_bytes, load_fa, load_svt


APP_TITLE = "ZGW Emulator"
UDP_PORT = 13401
DEFAULT_IP = "169.254.1.20"

_VIN_VAL = {str(i): i for i in range(10)}
_VIN_VAL.update(zip("ABCDEFGH", range(1, 9)))
_VIN_VAL.update(zip("JKLMN", (1, 2, 3, 4, 5)))
_VIN_VAL.update({"P": 7, "R": 9})
_VIN_VAL.update(zip("STUVWXYZ", (2, 3, 4, 5, 6, 7, 8, 9)))
_VIN_WEIGHT = (8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2)


def bundled_data_dir() -> Path:
    if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
        return Path(sys._MEIPASS) / "data"
    return Path(__file__).resolve().parent / "data"


def vin_check_digit_ok(vin: str) -> bool:
    if len(vin) != 17 or any(c not in _VIN_VAL for c in vin):
        return False
    total = sum(_VIN_VAL[c] * w for c, w in zip(vin, _VIN_WEIGHT))
    rem = total % 11
    expect = "X" if rem == 10 else str(rem)
    return vin[8] == expect


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
            data, _ = self._sock.recvfrom(1536)
        except (socket.timeout, OSError):
            return None
        return data.decode("utf-8", errors="replace").strip()


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.minsize(640, 520)
        self.geometry("760x620")

        self._link: UdpLink | None = None
        self._rx: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._ignore_switch = False
        self._vin_armed = False
        self._fa: Fa | None = None
        self._svt: list[SvtEcu] = []
        self._fa_from = ""
        self._template = True
        self._accept_pull = False
        self._km_armed = False
        self._editor: VehicleEditor | None = None

        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="IP ZGW").pack(side="left")
        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        ttk.Entry(top, textvariable=self.ip_var, width=18).pack(side="left", padx=6)
        self.conn_btn = ttk.Button(top, text="Połącz", command=self._toggle)
        self.conn_btn.pack(side="left")
        self.status = tk.StringVar(value="Rozłączony")
        ttk.Label(top, textvariable=self.status).pack(side="left", padx=10)

        vin_box = ttk.LabelFrame(self, text="VIN nadawany przy identyfikacji")
        vin_box.pack(fill="x", padx=10, pady=4)
        row = ttk.Frame(vin_box)
        row.pack(fill="x", padx=8, pady=8)
        ttk.Label(row, text="VIN").pack(side="left")
        self.vin_var = tk.StringVar(value="WBA5V510X0FJ28775")
        ttk.Entry(row, textvariable=self.vin_var, width=22, font=("Consolas", 12)).pack(side="left", padx=8)
        ttk.Button(row, text="Nadawaj", command=self._send_vin).pack(side="left")
        self.vin_status = tk.StringVar(
            value="Pełne 17 znaków VIN auta. Nagłówek ISTA pokazuje ostatnie 7."
        )
        ttk.Label(vin_box, textvariable=self.vin_status).pack(anchor="w", padx=8, pady=(0, 6))

        km_box = ttk.LabelFrame(self, text="Przebieg")
        km_box.pack(fill="x", padx=10, pady=4)
        km_row = ttk.Frame(km_box)
        km_row.pack(fill="x", padx=8, pady=8)
        ttk.Label(km_row, text="Kilometry").pack(side="left")
        self.km_var = tk.StringVar(value="0")
        ttk.Entry(km_row, textvariable=self.km_var, width=10, font=("Consolas", 12)).pack(side="left", padx=8)
        ttk.Button(km_row, text="Ustaw", command=self._send_km).pack(side="left")
        ttk.Label(
            km_row,
            text="CAN 0x330, bajty 0–2, little-endian, co 1 s.",
        ).pack(side="left", padx=8)

        order = ttk.LabelFrame(self, text="FA (zapis VCM w BDC) i SVT")
        order.pack(fill="x", padx=10, pady=4)
        buttons = ttk.Frame(order)
        buttons.pack(fill="x", padx=8, pady=(8, 4))
        ttk.Button(buttons, text="Edytuj FA i SVT", command=self._open_editor).pack(side="left")
        ttk.Button(buttons, text="Pobierz z emulatora", command=self._pull_vehicle).pack(side="left", padx=8)
        self.fa_status = tk.StringVar(value="Ładowanie FA i SVT…")
        ttk.Label(order, textvariable=self.fa_status, wraplength=700).pack(anchor="w", padx=8, pady=(0, 4))
        self.svt_status = tk.StringVar(value="")
        ttk.Label(order, textvariable=self.svt_status, wraplength=700).pack(anchor="w", padx=8, pady=(0, 8))

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
            text="Ten sam bajt idzie w CAN 0x12F i w ENET TCP 6811. Oba włączone: 45.",
        ).pack(side="left", padx=8)

        log_frame = ttk.LabelFrame(self, text="Log Ethernet i ramki MCP2515")
        log_frame.pack(fill="both", expand=True, padx=10, pady=6)
        self.log = scrolledtext.ScrolledText(log_frame, wrap="word", state="disabled", font=("Consolas", 10))
        self.log.pack(fill="both", expand=True, padx=6, pady=6)
        ttk.Button(log_frame, text="Wyczyść", command=self._clear_log).pack(anchor="e", padx=6, pady=4)

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._load_bundled()
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
        self._accept_pull = True
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

    def _send_vin(self) -> None:
        if self._link is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz z emulatorem.")
            return
        vin = "".join(self.vin_var.get().split()).upper()
        self.vin_var.set(vin)
        if len(vin) != 17 or any(c in "IOQ" or not c.isalnum() for c in vin):
            messagebox.showerror(APP_TITLE, "VIN ma mieć 17 znaków, bez liter I, O i Q.")
            return
        if not vin_check_digit_ok(vin):
            messagebox.showwarning(
                APP_TITLE,
                "9. znak VIN nie jest poprawną cyfrą kontrolną. ISTA odrzuci ten numer i poprosi o wpisanie go jeszcze raz.",
            )
        self._vin_armed = True
        self._send_raw({"cmd": "vin", "vin": vin})

    def _send_km(self) -> None:
        if self._link is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz z emulatorem.")
            return
        text = self.km_var.get().strip().replace(" ", "")
        if not text.isdigit():
            messagebox.showerror(APP_TITLE, "Przebieg ma być liczbą kilometrów.")
            return
        km = int(text)
        if km > 0xFFFFFF:
            messagebox.showerror(APP_TITLE, "Przebieg mieści się w 16777215 km.")
            return
        self._km_armed = True
        self._send_raw({"cmd": "km", "km": km})

    def _refresh_fa_status(self) -> None:
        self.svt_status.set(f"SVT: {len(self._svt)} sterowników. Edycja jest w oknie FA i SVT.")
        order = self._fa.summary() if self._fa is not None else "brak FA"
        vin = self._fa.vin if self._fa is not None and self._fa.vin else ""
        if self._template:
            self.fa_status.set(
                f"G20 na sztywno: {order}"
                + (f", VIN {vin}" if vin else "")
                + ". To FA jest zamówieniem zapisanym w BDC."
            )
            return
        name = self._fa_from or "plik"
        self.fa_status.set(f"{name}: {order}" + (f", VIN {vin}" if vin else "") + ".")

    def _apply_fa(self, fa: Fa, name: str) -> None:
        self._fa = fa
        if fa.vin:
            self.vin_var.set(fa.vin)
        self._fa_from = name

    def _load_xml(self, path: str) -> None:
        try:
            self._apply_fa(load_fa(path), Path(path).name)
        except (OSError, ValueError, ET.ParseError) as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        self._template = False
        self._refresh_fa_status()
        loaded = self._fa.summary() if self._fa is not None else "bez FA"
        self._append(f"Wczytano {Path(path).name}: {loaded}")
        if self._link is not None:
            self._send_vehicle()

    def _pick_fa(self) -> None:
        path = filedialog.askopenfilename(
            title="FA.xml",
            filetypes=[("XML", "*.xml"), ("Wszystkie", "*.*")],
        )
        if path:
            self._load_xml(path)

    def _load_bundled(self) -> None:
        data = bundled_data_dir()
        try:
            fa_path = data / "FA.xml"
            if fa_path.is_file():
                self._apply_fa(load_fa(fa_path), fa_path.name)
            svt_path = data / "SVT.xml"
            if svt_path.is_file():
                self._svt = load_svt(svt_path)
        except (OSError, ValueError, ET.ParseError) as exc:
            self.fa_status.set(str(exc))
            return
        self._template = True
        self._refresh_fa_status()

    def _send_vehicle(self) -> None:
        if self._link is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz z emulatorem.")
            return
        vin = "".join(self.vin_var.get().split()).upper()
        self.vin_var.set(vin)
        if len(vin) != 17 or any(c in "IOQ" or not c.isalnum() for c in vin):
            messagebox.showerror(APP_TITLE, "VIN ma mieć 17 znaków, bez liter I, O i Q.")
        else:
            if not vin_check_digit_ok(vin):
                messagebox.showwarning(
                    APP_TITLE,
                    "9. znak VIN nie jest poprawną cyfrą kontrolną. ISTA odrzuci ten numer i poprosi o wpisanie go jeszcze raz.",
                )
            self._vin_armed = True
            self._send_raw({"cmd": "vin", "vin": vin})
        if self._fa is None:
            self._append("Brak FA. Wczytaj je w edycji.")
        else:
            self._send_raw({"cmd": "fa", "hex": fa_bytes(self._fa).hex()})
        self._send_svt()

    def _send_svt(self) -> None:
        if not self._svt:
            self._append("Brak SVT.")
            return
        self._send_raw({"cmd": "svt_begin"})
        for ecu in self._svt:
            self._send_raw({
                "cmd": "svt_ecu",
                "addr": f"{ecu.addr:02X}",
                "name": ecu.name,
                "ver": str(ecu.svk_version),
                "dep": str(ecu.prog_dep),
                "hex": ecu.wire().hex(),
            })
        self._send_raw({"cmd": "svt_commit"})

    def note(self, text: str) -> None:
        self._append(text)

    def push_vehicle(self, fa: Fa, svt: list[SvtEcu]) -> None:
        self._fa = fa
        self._svt = list(svt)
        if fa.vin:
            self.vin_var.set(fa.vin)
        self._template = False
        self._refresh_fa_status()
        self._append(f"FA {fa.summary()}, SVT {len(svt)} sterowników")
        if self._link is None:
            messagebox.showinfo(APP_TITLE, "Zapisane w oknie. Połącz z emulatorem, żeby to wysłać.")
            return
        self._send_vehicle()

    def _open_editor(self) -> None:
        editor = self._editor
        if editor is not None and editor.winfo_exists():
            editor.lift()
            return
        self._editor = VehicleEditor(self, self._fa, self._svt)

    def _pull_vehicle(self) -> None:
        if self._link is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz z emulatorem.")
            return
        self._accept_pull = True
        self._send_raw({"cmd": "get"})

    def _editor_if_open(self) -> VehicleEditor | None:
        editor = self._editor
        if editor is not None and editor.winfo_exists():
            return editor
        return None

    def _take_board_fa(self, obj: dict) -> None:
        raw = str(obj.get("fahex") or obj.get("hex") or "")
        if not raw:
            return
        try:
            fa = fa_from_bytes(bytes.fromhex(raw))
        except ValueError as exc:
            self._append(str(exc))
            return
        vin = str(obj.get("vin") or "")
        if not vin and self._fa is not None:
            vin = self._fa.vin
        if not vin:
            vin = self.vin_var.get().strip()
        if vin:
            fa.vin = vin
        self._apply_fa(fa, "emulator")
        self._template = False
        self._refresh_fa_status()
        editor = self._editor_if_open()
        if editor is not None:
            editor.show_fa(fa)

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
            msg = str(obj.get("msg", ""))
            self._append(msg)
            if msg.startswith("[VIN] announcing "):
                vin = msg.rsplit(" ", 1)[-1]
                if len(vin) == 17:
                    self.vin_var.set(vin)
                    if self._fa is not None:
                        self._fa.vin = vin
                    editor = self._editor_if_open()
                    if editor is not None:
                        editor.show_vin(vin)
            return
        if ev == "fa":
            self._take_board_fa(obj)
            summary = str(obj.get("sum", ""))
            if summary:
                self._append(f"FA zapisane w emulatorze: {summary}")
            return
        if ev == "can":
            self._append("MCP  " + str(obj.get("msg", "")))
            return
        if obj.get("ok") == 1 and obj.get("svt") == "commit":
            self._append(f"SVT w emulatorze: {len(self._svt)} sterowników")
            return
        if obj.get("ok") == 1 and obj.get("svt") in ("begin", "ecu"):
            return
        if obj.get("ok") == 1 and "fa" in obj and "kl30" not in obj:
            self._append(f"FA w emulatorze: {obj['fa']}")
            return
        if obj.get("ok") == 1 and obj.get("vcm") == 1:
            self._append("I-Stufe zapisane w emulatorze")
            return
        if obj.get("ok") == 1 and "kl30" in obj and "kl15" in obj:
            vin = str(obj.get("vin", ""))
            self.status.set(
                f"KL30={'ON' if obj['kl30'] else 'OFF'}  KL15={'ON' if obj['kl15'] else 'OFF'}"
            )
            if vin:
                self.vin_status.set(f"Emulator nadaje {vin}")
            if self._accept_pull:
                self._accept_pull = False
                if "km" in obj:
                    self.km_var.set(str(int(obj["km"])))
                self._take_board_fa(obj)
                self._append("Pobrano z emulatora przebieg i FA.")
            if self._km_armed and "km" in obj:
                self._km_armed = False
                self._append(f"Przebieg ustawiony: {int(obj['km'])} km")
            if self._vin_armed and vin:
                self._vin_armed = False
                self._append(f"VIN ustawiony: {vin}")
            return
        if obj.get("ok") == 0:
            err = str(obj.get("err", line))
            self._append("ZGW  " + err)
            if err == "bad_vin":
                self._vin_armed = False
                messagebox.showerror(APP_TITLE, "Emulator odrzucił VIN.")
            elif err == "bad_fa":
                messagebox.showerror(APP_TITLE, "Emulator odrzucił FA.")
            elif err == "bad_km":
                self._km_armed = False
                messagebox.showerror(APP_TITLE, "Emulator odrzucił przebieg.")
            elif err == "bad_svt":
                self._append("Emulator odrzucił SVT.")
            elif err == "bad_vcm":
                messagebox.showerror(APP_TITLE, "Emulator odrzucił I-Stufe.")
            elif err == "too_big":
                messagebox.showerror(APP_TITLE, "FA jest za duże na jedną ramkę UDP.")

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
