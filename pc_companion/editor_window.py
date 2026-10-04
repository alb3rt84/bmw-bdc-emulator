"""Window for loading and editing the vehicle order and the fitted controllers."""

from __future__ import annotations

import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import filedialog, messagebox, scrolledtext, ttk

from vehicle_xml import (
    Fa,
    Sgbm,
    SvtEcu,
    fa_from_fields,
    load_fa,
    load_svt,
    parse_part_text,
    save_fa,
    save_svt,
)

APP_TITLE = "ZGW Emulator"


class VehicleEditor(tk.Toplevel):
    def __init__(self, app: tk.Misc, fa: Fa | None, svt: list[SvtEcu]) -> None:
        super().__init__(app)
        self.app = app
        self.title("FA i SVT")
        self.geometry("920x680")
        self.minsize(760, 520)
        self._svt: list[SvtEcu] = [self._copy_ecu(ecu) for ecu in svt]
        self._selected: int | None = None
        self._ignore_select = False

        book = ttk.Notebook(self)
        book.pack(fill="both", expand=True, padx=8, pady=8)
        book.add(self._fa_tab(), text="FA / VCM")
        book.add(self._svt_tab(), text="SVT")

        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(bar, text="Zapisz w emulatorze", command=self._push).pack(side="right")
        ttk.Label(
            bar,
            text="Zapis FA z Rheingolda wraca do tej karty. SVT wysyła się stąd.",
        ).pack(side="left")

        if fa is not None:
            self.show_fa(fa)
        self._fill_ecu_list()

    def _fa_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self)
        form = ttk.Frame(tab)
        form.pack(fill="x", padx=8, pady=8)
        self._series = tk.StringVar()
        self._type = tk.StringVar()
        self._time = tk.StringVar()
        self._colour = tk.StringVar()
        self._fabric = tk.StringVar()
        self._vin = tk.StringVar()
        fields = (
            ("Seria", self._series),
            ("Typ", self._type),
            ("Data", self._time),
            ("Lakier", self._colour),
            ("Tapicerka", self._fabric),
            ("VIN", self._vin),
        )
        for col, (label, var) in enumerate(fields):
            ttk.Label(form, text=label).grid(row=0, column=col, sticky="w")
            width = 20 if label == "VIN" else 8
            ttk.Entry(form, textvariable=var, width=width).grid(row=1, column=col, padx=(0, 8), sticky="w")

        self._e_text = self._code_box(tab, "E-Worty, po jednym albo spacjami")
        self._sa_text = self._code_box(tab, "SA, po jednym albo spacjami")
        self._ho_text = self._code_box(tab, "HO, po jednym albo spacjami")

        row = ttk.Frame(tab)
        row.pack(fill="x", padx=8, pady=8)
        ttk.Button(row, text="Wczytaj FA.xml", command=self._load_fa).pack(side="left")
        ttk.Button(row, text="Zapisz FA.xml", command=self._save_fa).pack(side="left", padx=8)
        return tab

    def _code_box(self, parent: ttk.Frame, title: str) -> scrolledtext.ScrolledText:
        frame = ttk.LabelFrame(parent, text=title)
        frame.pack(fill="both", expand=True, padx=8, pady=4)
        box = scrolledtext.ScrolledText(frame, height=4, font=("Consolas", 10))
        box.pack(fill="both", expand=True, padx=4, pady=4)
        return box

    def _svt_tab(self) -> ttk.Frame:
        tab = ttk.Frame(self)
        panes = ttk.Panedwindow(tab, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=8)

        left = ttk.Frame(panes)
        self._ecu_list = tk.Listbox(left, exportselection=False, font=("Consolas", 10))
        self._ecu_list.pack(fill="both", expand=True)
        self._ecu_list.bind("<<ListboxSelect>>", self._on_select)
        buttons = ttk.Frame(left)
        buttons.pack(fill="x", pady=4)
        ttk.Button(buttons, text="Dodaj", command=self._add_ecu).pack(side="left")
        ttk.Button(buttons, text="Usuń", command=self._remove_ecu).pack(side="left", padx=6)
        panes.add(left, weight=1)

        right = ttk.Frame(panes)
        form = ttk.Frame(right)
        form.pack(fill="x")
        ttk.Label(form, text="Adres").pack(side="left")
        self._addr = tk.StringVar()
        ttk.Entry(form, textvariable=self._addr, width=6).pack(side="left", padx=6)
        ttk.Label(form, text="Nazwa").pack(side="left")
        self._ecu_name = tk.StringVar()
        ttk.Entry(form, textvariable=self._ecu_name, width=18).pack(side="left", padx=6)
        ttk.Label(
            right,
            text="Części, jedna na linię: HWEL 00004159 013.000.002",
        ).pack(anchor="w", pady=(8, 2))
        self._parts = scrolledtext.ScrolledText(right, font=("Consolas", 10))
        self._parts.pack(fill="both", expand=True)
        panes.add(right, weight=3)

        row = ttk.Frame(tab)
        row.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Button(row, text="Wczytaj SVT.xml", command=self._load_svt).pack(side="left")
        ttk.Button(row, text="Zapisz SVT.xml", command=self._save_svt).pack(side="left", padx=8)
        return tab

    def show_vin(self, vin: str) -> None:
        self._vin.set(vin)

    def show_fa(self, fa: Fa) -> None:
        self._series.set(fa.series)
        self._type.set(fa.type_key)
        self._time.set(fa.time_criteria)
        self._colour.set(fa.colour)
        self._fabric.set(fa.fabric)
        self._vin.set(fa.vin)
        self._set_box(self._e_text, "\n".join(fa.e_codes))
        self._set_box(self._sa_text, "\n".join(fa.sa_codes))
        self._set_box(self._ho_text, "\n".join(fa.ho_codes))

    def _set_box(self, box: scrolledtext.ScrolledText, text: str) -> None:
        box.delete("1.0", "end")
        box.insert("1.0", text)

    def _read_fa(self) -> Fa:
        return fa_from_fields(
            self._series.get(),
            self._type.get(),
            self._time.get(),
            self._colour.get(),
            self._fabric.get(),
            self._e_text.get("1.0", "end"),
            self._sa_text.get("1.0", "end"),
            self._ho_text.get("1.0", "end"),
            self._vin.get(),
        )

    def _copy_ecu(self, ecu: SvtEcu) -> SvtEcu:
        return SvtEcu(
            ecu.addr,
            ecu.name,
            [Sgbm(part.klass, part.ident, part.main, part.sub, part.patch) for part in ecu.parts],
            ecu.svk_version,
            ecu.prog_dep,
        )

    def _fill_ecu_list(self) -> None:
        self._ignore_select = True
        self._ecu_list.delete(0, "end")
        for ecu in self._svt:
            self._ecu_list.insert("end", f"{ecu.addr:02X}  {ecu.name}")
        self._ignore_select = False
        if self._svt:
            self._ecu_list.selection_set(0)
            self._load_ecu(0)

    def _remember_ecu(self) -> None:
        if self._selected is None:
            return
        addr = int(self._addr.get().strip(), 16)
        name = self._ecu_name.get().strip().upper()
        parts = parse_part_text(self._parts.get("1.0", "end"))
        if not 1 <= addr <= 0xFE or addr == 0xDF:
            raise ValueError("Adres sterownika ma być od 01 do FE, bez DF.")
        if not name or any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for c in name):
            raise ValueError("Nazwa sterownika: 1–19 znaków A–Z, 0–9 i _.")
        if len(name) > 19:
            raise ValueError("Nazwa sterownika ma mieć najwyżej 19 znaków.")
        others = {ecu.addr for i, ecu in enumerate(self._svt) if i != self._selected}
        if addr in others:
            raise ValueError(f"Adres 0x{addr:02X} jest już na liście.")
        current = self._svt[self._selected]
        current.addr = addr
        current.name = name
        current.parts = parts
        self._ecu_list.delete(self._selected)
        self._ecu_list.insert(self._selected, f"{addr:02X}  {name}")
        self._ecu_list.selection_set(self._selected)

    def _load_ecu(self, index: int) -> None:
        ecu = self._svt[index]
        self._selected = index
        self._addr.set(f"{ecu.addr:02X}")
        self._ecu_name.set(ecu.name)
        self._set_box(self._parts, "\n".join(part.line() for part in ecu.parts))

    def _on_select(self, _event: object) -> None:
        if self._ignore_select:
            return
        picked = self._ecu_list.curselection()
        if not picked or picked[0] == self._selected:
            return
        try:
            self._remember_ecu()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            self._ignore_select = True
            if self._selected is not None:
                self._ecu_list.selection_clear(0, "end")
                self._ecu_list.selection_set(self._selected)
            self._ignore_select = False
            return
        self._load_ecu(picked[0])

    def _add_ecu(self) -> None:
        try:
            if self._selected is not None:
                self._remember_ecu()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            return
        used = {ecu.addr for ecu in self._svt}
        addr = 1
        while addr in used or addr == 0xDF:
            addr += 1
            if addr > 0xFE:
                messagebox.showerror(APP_TITLE, "Brak wolnego adresu.", parent=self)
                return
        self._svt.append(SvtEcu(addr, "ECU", parse_part_text("HWEL 00000000 001.000.000")))
        self._fill_ecu_list()
        last = len(self._svt) - 1
        self._ecu_list.selection_clear(0, "end")
        self._ecu_list.selection_set(last)
        self._load_ecu(last)

    def _remove_ecu(self) -> None:
        if self._selected is None:
            return
        del self._svt[self._selected]
        self._selected = None
        self._fill_ecu_list()

    def _load_fa(self) -> None:
        path = filedialog.askopenfilename(
            parent=self, title="FA.xml", filetypes=[("XML", "*.xml"), ("Wszystkie", "*.*")],
        )
        if not path:
            return
        try:
            self.show_fa(load_fa(path))
        except (OSError, ValueError, ET.ParseError) as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            return
        self.app.note(f"Wczytano FA {path}")

    def _save_fa(self) -> None:
        try:
            fa = self._read_fa()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            return
        path = filedialog.asksaveasfilename(
            parent=self, title="Zapisz FA.xml", defaultextension=".xml",
            filetypes=[("XML", "*.xml")],
        )
        if not path:
            return
        save_fa(path, fa)
        self.app.note(f"Zapisano FA {path}")

    def _load_svt(self) -> None:
        path = filedialog.askopenfilename(
            parent=self, title="SVT.xml", filetypes=[("XML", "*.xml"), ("Wszystkie", "*.*")],
        )
        if not path:
            return
        try:
            self._svt = load_svt(path)
            self._selected = None
            self._fill_ecu_list()
        except (OSError, ValueError, ET.ParseError) as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            return
        self.app.note(f"Wczytano SVT {path}")

    def _save_svt(self) -> None:
        try:
            if self._selected is not None:
                self._remember_ecu()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            return
        path = filedialog.asksaveasfilename(
            parent=self, title="Zapisz SVT.xml", defaultextension=".xml",
            filetypes=[("XML", "*.xml")],
        )
        if not path:
            return
        save_svt(path, self._svt)
        self.app.note(f"Zapisano SVT {path}")

    def _push(self) -> None:
        try:
            fa = self._read_fa()
            if self._selected is not None:
                self._remember_ecu()
            if not self._svt:
                raise ValueError("SVT nie może być puste.")
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc), parent=self)
            return
        self.app.push_vehicle(fa, self._svt)

    def current_svt(self) -> list[SvtEcu]:
        return self._svt
