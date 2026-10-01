"""BMW G-Chassis cyclic payloads — same layout as src/bmw_frames.cpp."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Signals:
    ignition: bool = True
    rpm: int = 0
    speed_kmh: float = 0.0
    fuel_pct: float = 50.0
    coolant_c: int = 90


def encode_rpm(rpm: int) -> bytes:
    rpm = max(0, min(8000, int(rpm)))
    raw = rpm * 4
    out = bytearray(8)
    out[2] = (raw >> 8) & 0xFF
    out[3] = raw & 0xFF
    return bytes(out)


def encode_speed(kmh: float) -> bytes:
    kmh = max(0.0, min(300.0, float(kmh)))
    raw = int(round(kmh * 10.0))
    out = bytearray(8)
    out[0] = raw & 0xFF
    out[1] = (raw >> 8) & 0xFF
    return bytes(out)


def encode_coolant(celsius: int) -> bytes:
    c = max(-40, min(140, int(celsius)))
    out = bytearray(8)
    out[0] = (c + 48) & 0xFF
    return bytes(out)


def encode_fuel(pct: float) -> bytes:
    pct = max(0.0, min(100.0, float(pct)))
    out = bytearray(8)
    out[0] = int(round(pct)) & 0xFF
    return bytes(out)


def encode_ignition(on: bool) -> bytes:
    if on:
        return bytes([0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    return bytes([0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])


# (name, can_id, period_seconds, payload builder)
CYCLIC = (
    ("NM_BDC_0x510", 0x510, 0.100, lambda _s: bytes([0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ("Terminal_0x12F", 0x12F, 0.100, lambda s: encode_ignition(s.ignition)),
    ("Fahrzustand_0x34A", 0x34A, 0.020, lambda _s: bytes([0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ("ZeitDatum_0x2F8", 0x2F8, 1.000, lambda _s: bytes([0x24, 0x0C, 0x0F, 0x0E, 0x00, 0x00, 0x00, 0xFF])),
    ("RPM_0x0A5", 0x0A5, 0.020, lambda s: encode_rpm(s.rpm)),
    ("Speed_0x1A1", 0x1A1, 0.020, lambda s: encode_speed(s.speed_kmh)),
    ("Coolant_0x1D0", 0x1D0, 0.100, lambda s: encode_coolant(s.coolant_c)),
    ("Fuel_0x349", 0x349, 0.200, lambda s: encode_fuel(s.fuel_pct)),
)


def due(signals: Signals, last_sent: dict[int, float], now: float) -> list[tuple[int, bytes]]:
    """Return frames whose period has elapsed. Updates last_sent."""
    ready: list[tuple[int, bytes]] = []
    for _name, can_id, period, build in CYCLIC:
        prev = last_sent.get(can_id, 0.0)
        if (now - prev) < period:
            continue
        ready.append((can_id, build(signals)))
        last_sent[can_id] = now
    return ready
