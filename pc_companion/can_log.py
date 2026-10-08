"""
Text log of CAN frames for the Robotell recorder.

One frame per line, time in seconds from the start of the recording:

    0.100000 RX 12F 0 0 8 45FF45FFFFFFFFFF
    1.000000 TX 330 0 0 3 010000

The first lines are comments. ``# robotell-canlog 1`` is the version.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from robotell_can import CanFrame


MAGIC = "robotell-canlog"
VERSION = 1


@dataclass(frozen=True)
class LoggedFrame:
    t_s: float
    direction: str
    frame: CanFrame


@dataclass
class CanRecording:
    events: list[LoggedFrame]
    meta: dict[str, str]


def format_event(event: LoggedFrame) -> str:
    frame = event.frame
    if event.direction not in ("RX", "TX"):
        raise ValueError("direction must be RX or TX")
    dlc = 0 if frame.dlc is None else int(frame.dlc)
    data = ""
    if not frame.is_remote:
        raw = bytes(frame.data[:dlc])
        if len(raw) < dlc:
            raw = raw + bytes(dlc - len(raw))
        data = raw.hex().upper()
    ext = 1 if frame.is_extended else 0
    rtr = 1 if frame.is_remote else 0
    return (
        f"{event.t_s:.6f} {event.direction} {frame.arbitration_id:X} "
        f"{ext} {rtr} {dlc} {data}".rstrip()
    )


def parse_event(line: str) -> LoggedFrame:
    parts = line.split()
    if len(parts) not in (6, 7):
        raise ValueError("oczekiwano: czas RX|TX id ext rtr dlc [dane]")
    try:
        t_s = float(parts[0])
    except ValueError as exc:
        raise ValueError("zły czas") from exc
    if t_s < 0:
        raise ValueError("czas nie może być ujemny")
    direction = parts[1].upper()
    if direction not in ("RX", "TX"):
        raise ValueError("kierunek musi być RX albo TX")
    try:
        can_id = int(parts[2], 16)
        extended = _bit(parts[3], "ext")
        remote = _bit(parts[4], "rtr")
        dlc = int(parts[5], 10)
    except ValueError as exc:
        raise ValueError("złe pola ramki") from exc
    if can_id < 0 or can_id > 0x1FFFFFFF:
        raise ValueError("ID poza zakresem")
    if not extended and can_id > 0x7FF:
        raise ValueError("ID standard max 7FF")
    if dlc < 0 or dlc > 8:
        raise ValueError("DLC musi być 0–8")
    hex_data = parts[6] if len(parts) == 7 else ""
    if remote and hex_data:
        raise ValueError("ramka RTR nie ma danych")
    if not remote and len(hex_data) != dlc * 2:
        raise ValueError("długość danych nie zgadza się z DLC")
    try:
        data = bytes.fromhex(hex_data) if hex_data else b""
    except ValueError as exc:
        raise ValueError("dane nie są hex") from exc
    frame = CanFrame(
        arbitration_id=can_id,
        data=data,
        is_extended=extended,
        is_remote=remote,
        dlc=dlc,
    )
    return LoggedFrame(t_s, direction, frame)


def load_recording(path: str) -> CanRecording:
    events: list[LoggedFrame] = []
    meta: dict[str, str] = {}
    seen_magic = False
    with open(path, encoding="utf-8") as handle:
        for lineno, raw in enumerate(handle, 1):
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                body = line[1:].strip()
                if body.startswith(MAGIC):
                    parts = body.split()
                    if len(parts) != 2 or parts[1] != str(VERSION):
                        raise ValueError(f"Linia {lineno}: nieznana wersja nagrania")
                    seen_magic = True
                    continue
                key, _, value = body.partition(" ")
                if key:
                    meta[key] = value
                continue
            try:
                events.append(parse_event(line))
            except ValueError as exc:
                raise ValueError(f"Linia {lineno}: {exc}") from exc
    if not seen_magic and events:
        raise ValueError("Brak nagłówka robotell-canlog")
    return CanRecording(events, meta)


def playback_waits(events: list[LoggedFrame], speed: float = 1.0) -> list[float]:
    """Seconds to wait before each frame. Speed 2 plays twice as fast."""
    if speed <= 0:
        raise ValueError("prędkość musi być większa od zera")
    waits: list[float] = []
    prev = 0.0
    for event in events:
        delta = event.t_s - prev
        if delta < 0:
            delta = 0.0
        waits.append(delta / speed)
        if event.t_s > prev:
            prev = event.t_s
    return waits


def parse_id_filter(text: str) -> set[int] | None:
    """None means every ID. Raises ValueError on a bad token."""
    cleaned = text.replace(",", " ").replace(";", " ").strip()
    if not cleaned:
        return None
    found: set[int] = set()
    for token in cleaned.split():
        item = token.lower().removeprefix("0x")
        try:
            value = int(item, 16)
        except ValueError as exc:
            raise ValueError(f"zły filtr ID: {token}") from exc
        if value < 0 or value > 0x1FFFFFFF:
            raise ValueError(f"zły filtr ID: {token}")
        found.add(value)
    return found


class RecordingWriter:
    """Append-only file. Each frame is flushed so a closed window keeps the log."""

    def __init__(self, path: str, meta: dict[str, str] | None = None) -> None:
        self.path = path
        self.count = 0
        self._t0 = time.monotonic()
        self._handle = open(path, "w", encoding="utf-8", newline="\n")
        self._handle.write(f"# {MAGIC} {VERSION}\n")
        for key, value in (meta or {}).items():
            safe_key = str(key).replace(" ", "_")
            safe_value = str(value).replace("\n", " ").strip()
            self._handle.write(f"# {safe_key} {safe_value}\n")
        self._handle.flush()

    def add(self, direction: str, frame: CanFrame, now: float | None = None) -> LoggedFrame:
        if now is None:
            now = time.monotonic()
        event = LoggedFrame(max(0.0, now - self._t0), direction, frame)
        self._handle.write(format_event(event) + "\n")
        self._handle.flush()
        self.count += 1
        return event

    def close(self) -> None:
        if self._handle.closed:
            return
        self._handle.close()


def _bit(text: str, name: str) -> bool:
    if text not in ("0", "1"):
        raise ValueError(f"{name} musi być 0 albo 1")
    return text == "1"
