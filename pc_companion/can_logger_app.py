#!/usr/bin/env python3
"""
Robotell USB-CAN logger and recorder.

Separate from the ZGW window. The stick is a CH340 talking the Robotell
binary protocol. This program shows every CAN frame, writes a .canlog file,
and plays that file back onto the bus with the original spacing.
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

try:
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    list_ports = None

from can_log import (
    CanRecording,
    LoggedFrame,
    RecordingWriter,
    load_recording,
    parse_id_filter,
    playback_waits,
)
from robotell_can import CanFrame, RobotellCan


APP_TITLE = "Robotell CAN logger"
TRACE_LIMIT = 2000
CH340_VID = 0x1A86


class LoggerApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("980x740")
        self.minsize(820, 560)

        self._can: RobotellCan | None = None
        self._reader: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self._connecting = False
        self._queue: queue.Queue = queue.Queue()
        self._port_map: dict[str, str] = {}

        self._writer: RecordingWriter | None = None
        self._recording: CanRecording | None = None
        self._record_path = ""
        self._view_t0: float | None = None
        self._recent: deque[LoggedFrame] = deque(maxlen=5000)
        self._showing_file = False
        self._id_count: dict[tuple, int] = {}
        self._id_last: dict[tuple, float] = {}
        self._id_rows: dict[tuple, tuple] = {}
        self._row = 0
        self._rx = 0
        self._tx = 0
        self._filter: set[int] | None = None

        self._play_thread: threading.Thread | None = None
        self._stop_play = threading.Event()

        self._build()
        self._refresh_ports()
        self.after(30, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build(self) -> None:
        root = ttk.Frame(self, padding=8)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)

        self._build_link(root)
        self._build_view(root)
        self._build_record(root)
        self._build_send(root)

        self.status = tk.StringVar(value="Wybierz port CH340 i połącz adapter.")
        ttk.Label(root, textvariable=self.status).grid(row=4, column=0, sticky="we", pady=(6, 0))

    def _build_link(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Połączenie Robotell USB-CAN", padding=6)
        box.grid(row=0, column=0, sticky="we")
        ttk.Label(box, text="Port").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=42, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="we", padx=4)
        ttk.Button(box, text="Odśwież", command=self._refresh_ports).grid(row=0, column=2, padx=2)

        ttk.Label(box, text="USB").grid(row=0, column=3, padx=(8, 0))
        self.usb_var = tk.StringVar(value="115200")
        ttk.Combobox(
            box, textvariable=self.usb_var, width=10,
            values=("115200", "2000000", "1000000", "921600", "460800", "57600"),
        ).grid(row=0, column=4, padx=4)
        self.auto_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="Auto baud", variable=self.auto_var).grid(row=0, column=5)

        ttk.Label(box, text="CAN").grid(row=0, column=6, padx=(8, 0))
        self.can_var = tk.StringVar(value="500000")
        ttk.Combobox(
            box, textvariable=self.can_var, width=10,
            values=("500000", "250000", "125000", "1000000", "100000"),
        ).grid(row=0, column=7, padx=4)
        self.connect_btn = ttk.Button(box, text="Połącz", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=8, padx=(6, 0))
        box.columnconfigure(1, weight=1)

    def _build_view(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Podgląd", padding=6)
        box.grid(row=1, column=0, sticky="nsew", pady=6)
        box.columnconfigure(0, weight=1)
        box.rowconfigure(1, weight=1)

        bar = ttk.Frame(box)
        bar.grid(row=0, column=0, sticky="we")
        ttk.Label(bar, text="Filtr ID").pack(side="left")
        self.filter_var = tk.StringVar()
        entry = ttk.Entry(bar, textvariable=self.filter_var, width=28)
        entry.pack(side="left", padx=4)
        entry.bind("<Return>", lambda _e: self._apply_filter())
        ttk.Button(bar, text="Zastosuj", command=self._apply_filter).pack(side="left")
        ttk.Button(bar, text="Wyczyść filtr", command=self._clear_filter).pack(side="left", padx=4)
        self.counter_var = tk.StringVar(value="RX 0    TX 0")
        ttk.Label(bar, textvariable=self.counter_var).pack(side="left", padx=12)
        ttk.Button(bar, text="Wyczyść podgląd", command=self._clear_view).pack(side="right")

        book = ttk.Notebook(box)
        book.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
        trace_tab = ttk.Frame(book)
        id_tab = ttk.Frame(book)
        book.add(trace_tab, text="Ślad")
        book.add(id_tab, text="Identyfikatory")

        self.trace = self._tree(
            trace_tab,
            ("czas", "kier", "id", "dlc", "dane"),
            ("Czas s", "Kier.", "ID", "DLC", "Dane"),
            (110, 50, 110, 50, 420),
        )
        self.trace.bind("<Double-1>", lambda _e: self._take_id(self.trace, 2))
        self.ids = self._tree(
            id_tab,
            ("id", "dlc", "dane", "liczba", "odstep"),
            ("ID", "DLC", "Ostatnie dane", "Liczba", "Odstęp ms"),
            (110, 50, 420, 80, 90),
        )
        self.ids.bind("<Double-1>", lambda _e: self._take_id(self.ids, 0))

    def _tree(self, parent: ttk.Frame, names: tuple[str, ...], headings: tuple[str, ...], widths: tuple[int, ...]) -> ttk.Treeview:
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        tree = ttk.Treeview(parent, columns=names, show="headings")
        for name, heading, width in zip(names, headings, widths):
            tree.heading(name, text=heading)
            tree.column(name, width=width, anchor="w")
        scroll = ttk.Scrollbar(parent, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=scroll.set)
        tree.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        return tree

    def _build_record(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Nagranie i odtwarzanie", padding=6)
        box.grid(row=2, column=0, sticky="we")
        self.record_btn = ttk.Button(box, text="Nagrywaj", command=self._start_record)
        self.record_btn.pack(side="left")
        self.stop_rec_btn = ttk.Button(box, text="Stop nagrywania", command=self._stop_record, state="disabled")
        self.stop_rec_btn.pack(side="left", padx=4)
        ttk.Button(box, text="Otwórz nagranie", command=self._open_recording).pack(side="left", padx=(8, 0))
        self.play_btn = ttk.Button(box, text="Odtwórz", command=self._start_play, state="disabled")
        self.play_btn.pack(side="left", padx=(12, 0))
        self.stop_play_btn = ttk.Button(box, text="Zatrzymaj", command=self._stop_play_click, state="disabled")
        self.stop_play_btn.pack(side="left", padx=4)
        ttk.Label(box, text="prędkość").pack(side="left", padx=(8, 2))
        self.speed_var = tk.StringVar(value="1")
        ttk.Combobox(box, textvariable=self.speed_var, width=4, values=("1", "2", "5", "10")).pack(side="left")
        self.file_var = tk.StringVar(value="brak pliku")
        ttk.Label(box, textvariable=self.file_var).pack(side="left", padx=10)

    def _build_send(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="Wyślij ramkę (zapisze się jako TX)", padding=6)
        box.grid(row=3, column=0, sticky="we", pady=(6, 0))
        ttk.Label(box, text="ID").pack(side="left")
        self.tx_id = tk.StringVar(value="12F")
        ttk.Entry(box, textvariable=self.tx_id, width=10).pack(side="left", padx=4)
        self.tx_ext = tk.BooleanVar(value=False)
        ttk.Checkbutton(box, text="rozszerzone", variable=self.tx_ext).pack(side="left")
        self.tx_rtr = tk.BooleanVar(value=False)
        ttk.Checkbutton(box, text="RTR", variable=self.tx_rtr).pack(side="left", padx=4)
        ttk.Label(box, text="DLC").pack(side="left")
        self.tx_dlc = tk.StringVar(value="8")
        ttk.Spinbox(box, from_=0, to=8, textvariable=self.tx_dlc, width=3).pack(side="left", padx=4)
        ttk.Label(box, text="dane").pack(side="left")
        self.tx_data = tk.StringVar(value="45 FF 45 FF FF FF FF FF")
        ttk.Entry(box, textvariable=self.tx_data, width=36).pack(side="left", padx=4)
        ttk.Button(box, text="Wyślij", command=self._send_once).pack(side="left")

    def _refresh_ports(self) -> None:
        self._port_map = {}
        labels: list[str] = []
        preferred = ""
        if list_ports is not None:
            for port in list_ports.comports():
                bits = []
                if port.description and port.description not in ("n/a", port.device):
                    bits.append(port.description)
                if port.vid is not None and port.pid is not None:
                    bits.append(f"{port.vid:04X}:{port.pid:04X}")
                label = port.device if not bits else f"{port.device} — {' '.join(bits)}"
                self._port_map[label] = port.device
                labels.append(label)
                if port.vid == CH340_VID and not preferred:
                    preferred = label
        self.port_combo["values"] = labels
        if self.port_var.get() not in labels:
            self.port_var.set(preferred or (labels[0] if labels else ""))

    def _selected_port(self) -> str:
        label = self.port_var.get().strip()
        if label in self._port_map:
            return self._port_map[label]
        return label.split(" — ")[0].strip()

    def _toggle_connect(self) -> None:
        if self._connecting:
            return
        if self._can is not None:
            self._disconnect()
            return
        port = self._selected_port()
        if not port:
            messagebox.showinfo(APP_TITLE, "Wybierz port COM adaptera.")
            return
        try:
            usb_baud = int(self.usb_var.get())
            can_bitrate = int(self.can_var.get())
        except ValueError:
            messagebox.showerror(APP_TITLE, "USB i CAN muszą być liczbami, np. 115200 i 500000.")
            return
        self._connecting = True
        self.connect_btn.configure(state="disabled")
        self.status.set(f"Łączenie z {port}…")

        def work() -> None:
            can = RobotellCan()
            try:
                result = can.open(
                    port,
                    usb_baud=usb_baud,
                    can_bitrate=can_bitrate,
                    auto_usb_baud=bool(self.auto_var.get()),
                    on_status=lambda text: self._queue.put(("status", text)),
                )
            except Exception as exc:
                can.close()
                self._queue.put(("connect-fail", str(exc)))
                return
            self._queue.put(("connect-ok", can, result))

        threading.Thread(target=work, daemon=True).start()

    def _on_connected(self, can: RobotellCan, result) -> None:
        self._connecting = False
        self._can = can
        self._stop_reader.clear()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()
        self._view_t0 = time.monotonic()
        self._showing_file = False
        self._recent.clear()
        self._reset_tables()
        self._rx = 0
        self._tx = 0
        self._update_counters()
        self.connect_btn.configure(state="normal", text="Rozłącz")
        sn = f", S/N {result.serial_number}" if result.serial_number else ""
        text = f"Połączono {result.port}, USB {result.usb_baud}, CAN {result.can_bitrate}{sn}"
        if self._recording is not None and self._recording.events:
            text += f". Nagranie: {len(self._recording.events)} ramek."
        self.status.set(text)

    def _on_connect_fail(self, err: str) -> None:
        self._connecting = False
        self.connect_btn.configure(state="normal", text="Połącz")
        self.status.set(err)
        messagebox.showerror(APP_TITLE, err)

    def _disconnect(self) -> None:
        self._stop_play_click()
        was_recording = self._writer is not None
        self._stop_record()
        self._stop_reader.set()
        can = self._can
        self._can = None
        if can is not None:
            can.close()
        self.connect_btn.configure(state="normal", text="Połącz")
        if not was_recording:
            self.status.set("Rozłączono.")

    def _reader_loop(self) -> None:
        while not self._stop_reader.is_set():
            can = self._can
            if can is None:
                return
            try:
                frame = can.recv(0.05)
            except Exception as exc:
                self._queue.put(("lost", str(exc)))
                return
            if frame is not None:
                self._queue.put(("frame", "RX", frame))

    def _poll(self) -> None:
        processed = 0
        while processed < 500:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            processed += 1
            kind = item[0]
            if kind == "frame":
                self._handle_frame(item[1], item[2])
            elif kind == "status":
                self.status.set(item[1])
            elif kind == "connect-ok":
                self._on_connected(item[1], item[2])
            elif kind == "connect-fail":
                self._on_connect_fail(item[1])
            elif kind == "lost":
                self.status.set("Utracono adapter: " + item[1])
                self._disconnect()
            elif kind == "progress":
                self.status.set(f"Odtwarzanie {item[1]} / {item[2]}")
            elif kind == "play-done":
                self._play_finished(item[1:])
        self.after(30, self._poll)

    def _handle_frame(self, direction: str, frame: CanFrame) -> None:
        now = time.monotonic()
        if self._writer is not None:
            event = self._writer.add(direction, frame, now)
            self.file_var.set(f"{self._record_path}   {self._writer.count} ramek")
        else:
            if self._view_t0 is None:
                self._view_t0 = now
            event = LoggedFrame(max(0.0, now - self._view_t0), direction, frame)
        if direction == "RX":
            self._rx += 1
        else:
            self._tx += 1
        self._update_counters()
        if self._showing_file:
            return
        self._recent.append(event)
        visible = self._accepts(frame)
        self._touch_id(event, visible)
        if visible:
            self._append_trace(event)

    def _accepts(self, frame: CanFrame) -> bool:
        return self._filter is None or frame.arbitration_id in self._filter

    def _append_trace(self, event: LoggedFrame, *, follow: bool = True) -> None:
        self._row += 1
        iid = str(self._row)
        self.trace.insert("", "end", iid=iid, values=self._trace_row(event))
        children = self.trace.get_children()
        if len(children) > TRACE_LIMIT:
            self.trace.delete(children[0])
        if follow:
            self.trace.see(iid)

    def _id_key(self, frame: CanFrame) -> tuple:
        return (frame.arbitration_id, frame.is_extended, frame.is_remote)

    def _id_iid(self, key: tuple) -> str:
        return f"{key[0]:X}-{'x' if key[1] else 's'}-{'r' if key[2] else 'd'}"

    def _touch_id(self, event: LoggedFrame, visible: bool) -> None:
        frame = event.frame
        key = self._id_key(frame)
        prev = self._id_last.get(key)
        gap = "" if prev is None else f"{max(0.0, event.t_s - prev) * 1000:.0f}"
        self._id_last[key] = event.t_s
        count = self._id_count.get(key, 0) + 1
        self._id_count[key] = count
        values = (self._id_text(frame), frame.dlc, self._data_text(frame), count, gap)
        self._id_rows[key] = values
        iid = self._id_iid(key)
        if not visible:
            if self.ids.exists(iid):
                self.ids.delete(iid)
            return
        if self.ids.exists(iid):
            self.ids.item(iid, values=values)
        else:
            self.ids.insert("", "end", iid=iid, values=values)

    def _trace_row(self, event: LoggedFrame) -> tuple[str, str, str, int | None, str]:
        return (
            f"{event.t_s:.3f}",
            event.direction,
            self._id_text(event.frame),
            event.frame.dlc,
            self._data_text(event.frame),
        )

    def _id_text(self, frame: CanFrame) -> str:
        text = f"{frame.arbitration_id:X}"
        if frame.is_extended:
            text += " X"
        return text

    def _data_text(self, frame: CanFrame) -> str:
        if frame.is_remote:
            return "RTR"
        return " ".join(f"{byte:02X}" for byte in frame.data)

    def _update_counters(self) -> None:
        self.counter_var.set(f"RX {self._rx}    TX {self._tx}")

    def _clear_trees(self) -> None:
        for tree in (self.trace, self.ids):
            children = tree.get_children()
            if children:
                tree.delete(*children)

    def _reset_tables(self) -> None:
        self._clear_trees()
        self._id_count.clear()
        self._id_last.clear()
        self._id_rows.clear()

    def _clear_view(self) -> None:
        self._recent.clear()
        self._showing_file = False
        self._rx = 0
        self._tx = 0
        self._view_t0 = time.monotonic()
        self._reset_tables()
        self._update_counters()

    def _apply_filter(self) -> None:
        try:
            self._filter = parse_id_filter(self.filter_var.get())
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        self._rebuild_view()

    def _clear_filter(self) -> None:
        self.filter_var.set("")
        self._filter = None
        self._rebuild_view()

    def _take_id(self, tree: ttk.Treeview, column: int) -> None:
        selected = tree.selection()
        if not selected:
            return
        values = tree.item(selected[0], "values")
        if not values:
            return
        self.filter_var.set(str(values[column]).split()[0])
        self._apply_filter()

    def _rebuild_view(self) -> None:
        self._clear_trees()
        if self._showing_file and self._recording is not None:
            self._id_count.clear()
            self._id_last.clear()
            self._id_rows.clear()
            matching = []
            for event in self._recording.events:
                visible = self._accepts(event.frame)
                self._touch_id(event, visible)
                if visible:
                    matching.append(event)
            self._rx = sum(1 for event in matching if event.direction == "RX")
            self._tx = sum(1 for event in matching if event.direction == "TX")
            self._update_counters()
            for event in matching[-TRACE_LIMIT:]:
                self._append_trace(event, follow=False)
            children = self.trace.get_children()
            if children:
                self.trace.see(children[-1])
            return
        for event in self._recent:
            if self._accepts(event.frame):
                self._append_trace(event, follow=False)
        for key, values in self._id_rows.items():
            if self._filter is None or key[0] in self._filter:
                self.ids.insert("", "end", iid=self._id_iid(key), values=values)
        children = self.trace.get_children()
        if children:
            self.trace.see(children[-1])

    def _start_record(self) -> None:
        if self._can is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz adapter Robotell.")
            return
        if self._play_thread is not None and self._play_thread.is_alive():
            messagebox.showinfo(APP_TITLE, "Zatrzymaj odtwarzanie, zanim zaczniesz nagrywać.")
            return
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = filedialog.asksaveasfilename(
            title="Zapisz nagranie CAN",
            defaultextension=".canlog",
            initialfile=f"canlog-{stamp}.canlog",
            filetypes=[("Nagranie CAN", "*.canlog"), ("Wszystkie pliki", "*.*")],
        )
        if not path:
            return
        can = self._can
        meta = {
            "port": can.port,
            "usb_baud": str(can.usb_baud),
            "can_bitrate": str(can.can_bitrate),
            "started": datetime.now().isoformat(timespec="seconds"),
        }
        try:
            self._writer = RecordingWriter(path, meta)
        except OSError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        self._record_path = path
        self._showing_file = False
        self.record_btn.configure(state="disabled")
        self.stop_rec_btn.configure(state="normal")
        self.play_btn.configure(state="disabled")
        self.file_var.set(f"{path}   0 ramek")
        self.status.set("Nagrywanie. Każda ramka jest od razu zapisywana na dysk.")

    def _stop_record(self) -> None:
        writer = self._writer
        if writer is None:
            return
        path = writer.path
        count = writer.count
        writer.close()
        self._writer = None
        self.record_btn.configure(state="normal")
        self.stop_rec_btn.configure(state="disabled")
        try:
            self._recording = load_recording(path)
        except ValueError as exc:
            self.status.set(f"Nagranie zamknięte ({count}), ale nie da się go wczytać: {exc}")
            return
        self.file_var.set(f"{path}   {count} ramek")
        self.play_btn.configure(state="normal")
        self.status.set(f"Zapisano {count} ramek. Odtwórz wyśle je z powrotem na magistralę.")

    def _open_recording(self) -> None:
        path = filedialog.askopenfilename(
            title="Otwórz nagranie CAN",
            filetypes=[("Nagranie CAN", "*.canlog"), ("Wszystkie pliki", "*.*")],
        )
        if not path:
            return
        try:
            recording = load_recording(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        self._recording = recording
        self._record_path = path
        self.file_var.set(f"{path}   {len(recording.events)} ramek")
        self.play_btn.configure(state="normal")
        if self._can is None and self._writer is None:
            self._showing_file = True
            self._rebuild_view()
            self.status.set(f"Wczytano {len(recording.events)} ramek. Połącz adapter, żeby je odtworzyć.")
        else:
            self.status.set(f"Wczytano {len(recording.events)} ramek do odtworzenia.")

    def _start_play(self) -> None:
        if self._can is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz adapter Robotell.")
            return
        if self._writer is not None:
            messagebox.showinfo(APP_TITLE, "Zatrzymaj nagrywanie, zanim odtworzysz plik.")
            return
        if self._recording is None or not self._recording.events:
            messagebox.showinfo(APP_TITLE, "Otwórz nagranie, w którym są ramki.")
            return
        if self._play_thread is not None and self._play_thread.is_alive():
            return
        try:
            speed = float(self.speed_var.get().replace(",", "."))
            waits = playback_waits(self._recording.events, speed)
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        events = list(self._recording.events)
        self._stop_play.clear()
        self.play_btn.configure(state="disabled")
        self.stop_play_btn.configure(state="normal")
        self.record_btn.configure(state="disabled")

        def work() -> None:
            error = ""
            sent = 0
            for index, (event, wait) in enumerate(zip(events, waits), 1):
                if wait > 0 and self._stop_play.wait(wait):
                    break
                if self._stop_play.is_set():
                    break
                can = self._can
                if can is None:
                    error = "Adapter został rozłączony."
                    break
                frame = event.frame
                try:
                    can.send(
                        frame.arbitration_id,
                        frame.data,
                        extended=frame.is_extended,
                        remote=frame.is_remote,
                        dlc=frame.dlc,
                    )
                except Exception as exc:
                    error = str(exc)
                    break
                sent = index
                self._queue.put(("frame", "TX", frame))
                self._queue.put(("progress", index, len(events)))
            self._queue.put(("play-done", error, sent, len(events)))

        self._play_thread = threading.Thread(target=work, daemon=True)
        self._play_thread.start()

    def _stop_play_click(self) -> None:
        self._stop_play.set()

    def _play_finished(self, item: tuple) -> None:
        error = item[0] if item else ""
        sent = item[1] if len(item) > 1 else 0
        total = item[2] if len(item) > 2 else 0
        self.play_btn.configure(state="normal" if self._recording else "disabled")
        self.stop_play_btn.configure(state="disabled")
        if self._writer is None:
            self.record_btn.configure(state="normal")
        if error:
            self.status.set(f"Odtwarzanie przerwane po {sent} ramkach: {error}")
        elif sent < total:
            self.status.set(f"Odtwarzanie zatrzymane po {sent} z {total} ramek.")
        else:
            self.status.set(f"Odtworzono {sent} ramek.")

    def _send_once(self) -> None:
        if self._can is None:
            messagebox.showinfo(APP_TITLE, "Najpierw połącz adapter Robotell.")
            return
        try:
            can_id, data, dlc, extended, remote = self._parse_tx()
        except ValueError as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        try:
            self._can.send(can_id, data, extended=extended, remote=remote, dlc=dlc)
        except Exception as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        frame = CanFrame(can_id, b"" if remote else data, extended, remote, dlc)
        self._handle_frame("TX", frame)

    def _parse_tx(self) -> tuple[int, bytes, int, bool, bool]:
        raw_id = self.tx_id.get().strip().lower().removeprefix("0x")
        if not raw_id:
            raise ValueError("Wpisz ID, np. 12F")
        can_id = int(raw_id, 16)
        extended = bool(self.tx_ext.get())
        if not extended and can_id > 0x7FF:
            raise ValueError("ID standard max 7FF — zaznacz rozszerzone albo wpisz mniejsze ID")
        if can_id < 0 or can_id > 0x1FFFFFFF:
            raise ValueError("ID poza zakresem")
        dlc = int(self.tx_dlc.get())
        if not 0 <= dlc <= 8:
            raise ValueError("DLC musi być 0–8")
        remote = bool(self.tx_rtr.get())
        data = bytearray()
        if not remote:
            text = self.tx_data.get().replace(",", " ").replace("0x", " ").replace("0X", " ")
            for token in text.split():
                if len(token) > 2:
                    raise ValueError(f"{token} to więcej niż jeden bajt")
                value = int(token, 16)
                if not 0 <= value <= 0xFF:
                    raise ValueError(f"{token} poza 00–FF")
                data.append(value)
            if len(data) > dlc:
                raise ValueError("Za dużo bajtów względem DLC")
            while len(data) < dlc:
                data.append(0)
        return can_id, bytes(data), dlc, extended, remote

    def _on_close(self) -> None:
        self._stop_play.set()
        self._stop_record()
        self._stop_reader.set()
        if self._can is not None:
            self._can.close()
            self._can = None
        self.destroy()


def main() -> None:
    LoggerApp().mainloop()


if __name__ == "__main__":
    main()
