"""
Robotell USB-CAN adapter (CH340 + STM32, binary protocol — not SLCAN).

Wire format, matching python-can's robotell interface:
  AA AA | 17-byte body with 0xA5 escaping | 55 55
Body: CAN id (4, LE), data (8), dlc, channel, format, frame type, checksum.
Checksum is the low 8 bits of the sum of the first 16 body bytes.
Bytes 0xA5, 0xAA and 0x55 are escaped as A5 <byte>.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


PACKET_HEAD = 0xAA
PACKET_TAIL = 0x55
PACKET_ESC = 0xA5

CAN_CONFIG_CHANNEL = 0xFF
CAN_SERIALBPS_ID = 0x01FFFE90
CAN_ART_ID = 0x01FFFEA0
CAN_ABOM_ID = 0x01FFFEB0
CAN_RESET_ID = 0x01FFFEC0
CAN_BAUD_ID = 0x01FFFED0
CAN_READ_SERIAL1 = 0x01FFFFF0
CAN_READ_SERIAL2 = 0x01FFFFF1

CAN_STANDARD = 0
CAN_EXTENDED = 1
CAN_DATA = 0
CAN_REMOTE = 1

# Factory USB speed is 115200. Some sticks are shipped at 2 Mbaud.
USB_BAUD_CANDIDATES = (
    115200,
    2000000,
    1000000,
    921600,
    460800,
    230400,
    57600,
    38400,
    19200,
    9600,
)

MAX_CAN_BITRATE = 1_000_000


@dataclass
class CanFrame:
    arbitration_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False
    dlc: int | None = None

    def __post_init__(self) -> None:
        if self.dlc is None:
            self.dlc = len(self.data)
        else:
            self.dlc = int(self.dlc)


@dataclass
class ProbeResult:
    port: str
    usb_baud: int
    can_bitrate: int
    serial_number: str | None


def checksum(body16: bytes) -> int:
    return sum(body16[:16]) & 0xFF


def encode_body(
    can_id: int,
    data: bytes,
    *,
    channel: int = 0,
    extended: bool = False,
    remote: bool = False,
    dlc: int | None = None,
) -> bytes:
    payload = bytes(data[:8])
    length = len(payload) if dlc is None else dlc
    if length < 0 or length > 8:
        raise ValueError("DLC must be 0..8")
    if not remote and dlc is None and len(data) > 8:
        raise ValueError("CAN data longer than 8 bytes")
    body = bytearray(17)
    body[0] = can_id & 0xFF
    body[1] = (can_id >> 8) & 0xFF
    body[2] = (can_id >> 16) & 0xFF
    body[3] = (can_id >> 24) & 0xFF
    if not remote:
        count = min(len(payload), length)
        body[4 : 4 + count] = payload[:count]
    body[12] = length
    body[13] = channel & 0xFF
    body[14] = CAN_EXTENDED if extended else CAN_STANDARD
    body[15] = CAN_REMOTE if remote else CAN_DATA
    body[16] = checksum(body)
    return bytes(body)


def encode_packet(body: bytes) -> bytes:
    if len(body) != 17:
        raise ValueError("body must be 17 bytes")
    packet = bytearray((PACKET_HEAD, PACKET_HEAD))
    for byte in body:
        if byte in (PACKET_ESC, PACKET_HEAD, PACKET_TAIL):
            packet.append(PACKET_ESC)
        packet.append(byte)
    packet.append(PACKET_TAIL)
    packet.append(PACKET_TAIL)
    return bytes(packet)


def pop_frames(buf: bytearray) -> list[bytes]:
    """Pull complete, checksum-valid 17-byte bodies out of buf. Leaves a partial tail."""
    frames: list[bytes] = []
    while True:
        head = buf.find(bytes((PACKET_HEAD, PACKET_HEAD)))
        if head < 0:
            if buf and buf[-1] == PACKET_HEAD:
                del buf[:-1]
            else:
                buf.clear()
            break
        if head > 0:
            del buf[:head]

        raw = bytearray()
        i = 2
        consumed = False
        while i < len(buf):
            byte = buf[i]
            if byte == PACKET_ESC:
                if i + 1 >= len(buf):
                    return frames
                raw.append(buf[i + 1])
                i += 2
            elif byte == PACKET_TAIL and i + 1 < len(buf) and buf[i + 1] == PACKET_TAIL:
                del buf[: i + 2]
                if len(raw) == 17 and raw[16] == checksum(raw):
                    frames.append(bytes(raw))
                consumed = True
                break
            else:
                raw.append(byte)
                i += 1
            if len(raw) > 17:
                del buf[0]
                consumed = True
                break
        if not consumed:
            break
    return frames


def frame_from_body(body: bytes) -> CanFrame:
    can_id = body[0] | (body[1] << 8) | (body[2] << 16) | (body[3] << 24)
    dlc = body[12] & 0x0F
    if dlc > 8:
        dlc = 8
    remote = body[15] == CAN_REMOTE
    data = b"" if remote else bytes(body[4 : 4 + dlc])
    return CanFrame(
        arbitration_id=can_id,
        data=data,
        is_extended=body[14] == CAN_EXTENDED,
        is_remote=remote,
        dlc=dlc,
    )


def explain_open_error(exc: BaseException, port: str) -> str:
    text = str(exc).lower()
    busy = (
        "access is denied" in text
        or "permission" in text
        or "errno 13" in text
        or "resource busy" in text
    )
    if busy:
        return (
            f"Port {port} jest zajęty. Zamknij EmbededDebug albo inny program, "
            "który trzyma ten COM."
        )
    missing = (
        "filenotfound" in text
        or "file not found" in text
        or "could not open" in text
        or "errno 2" in text
        or "no such file" in text
    )
    if missing:
        return (
            f"Nie ma portu {port}. Wepnij adapter jeszcze raz i naciśnij Refresh. "
            "Jeśli COM w ogóle nie występuje, zainstaluj sterownik CH340."
        )
    return f"Nie udało się otworzyć {port}: {exc}"


class RobotellCan:
    def __init__(self) -> None:
        self._ser = None
        self._rxbuf = bytearray()
        self._can_q: list[bytes] = []
        self._cfg_q: list[bytes] = []
        self._lock = threading.RLock()
        self.port = ""
        self.usb_baud = 0
        self.can_bitrate = 0
        self.serial_number: str | None = None

    @property
    def connected(self) -> bool:
        return self._ser is not None

    def close(self) -> None:
        with self._lock:
            ser = self._ser
            self._ser = None
            self._rxbuf.clear()
            self._can_q.clear()
            self._cfg_q.clear()
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def open(
        self,
        port: str,
        usb_baud: int = 115200,
        can_bitrate: int = 500_000,
        *,
        auto_usb_baud: bool = True,
        on_status=None,
        serial_opener=None,
    ) -> ProbeResult:
        if serial_opener is None and serial is None:
            raise RuntimeError("Brak pyserial. Zainstaluj: pip install pyserial")
        if not port:
            raise RuntimeError("Wybierz port COM adaptera Robotell.")
        if can_bitrate <= 0 or can_bitrate > MAX_CAN_BITRATE:
            raise ValueError(f"CAN bitrate musi być 1..{MAX_CAN_BITRATE}")

        bauds: list[int] = []
        for baud in (usb_baud, *USB_BAUD_CANDIDATES):
            if baud not in bauds:
                bauds.append(baud)
        if not auto_usb_baud:
            bauds = [usb_baud]

        self.close()
        self._opener = serial_opener
        last_open_error: str | None = None
        opened_any = False

        for baud in bauds:
            if on_status:
                on_status(f"Próba {port} @ {baud} bod…")
            try:
                ser = self._open_serial(port, baud)
            except Exception as exc:
                last_open_error = explain_open_error(exc, port)
                # Wrong baud is worth another try; a missing or busy port is not.
                if _is_bad_baud(exc):
                    continue
                raise RuntimeError(last_open_error) from exc

            opened_any = True
            try:
                if self._handshake(ser, can_bitrate):
                    with self._lock:
                        self._ser = ser
                        self.port = port
                        self.usb_baud = baud
                        self.can_bitrate = can_bitrate
                    self.serial_number = self._read_serial_number(timeout=0.25)
                    self._best_effort_flags()
                    return ProbeResult(port, baud, can_bitrate, self.serial_number)
            except Exception:
                try:
                    ser.close()
                except Exception:
                    pass
                raise
            try:
                ser.close()
            except Exception:
                pass

        if not opened_any and last_open_error:
            raise RuntimeError(last_open_error)
        tried = ", ".join(str(b) for b in bauds)
        raise RuntimeError(
            "Port COM się otwiera, ale adapter Robotell nie odpowiada. "
            f"Sprawdzone prędkości USB: {tried}. "
            "Zamknij EmbededDebug, wybierz port z opisem CH340 / USB-SERIAL "
            "i zostaw włączone Auto USB baud. Fabrycznie jest to zwykle 115200."
        )

    def send(
        self,
        can_id: int,
        data: bytes,
        *,
        extended: bool = False,
        remote: bool = False,
        dlc: int | None = None,
    ) -> None:
        with self._lock:
            ser = self._require()
            body = encode_body(can_id, data, extended=extended, remote=remote, dlc=dlc)
            self._write_body(ser, body)

    def recv(self, timeout: float = 0.0) -> CanFrame | None:
        with self._lock:
            if self._ser is None:
                return None
            body = self._read_body(config=False, timeout=timeout)
            if body is None:
                return None
            return frame_from_body(body)

    def _require(self):
        if self._ser is None:
            raise RuntimeError("Adapter Robotell nie jest połączony")
        return self._ser

    def _open_serial(self, port: str, baud: int):
        opener = getattr(self, "_opener", None)
        if opener is not None:
            return opener(port, baud)
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        ser.timeout = 0.05
        ser.write_timeout = 1.0
        ser.xonxoff = False
        ser.rtscts = False
        ser.dsrdtr = False
        ser.open()
        # Opening CH340 often pulses DTR. Keep the stick out of reset.
        ser.dtr = False
        ser.rts = False
        time.sleep(0.2)
        ser.reset_input_buffer()
        return ser

    def _handshake(self, ser, can_bitrate: int) -> bool:
        self._rxbuf.clear()
        self._can_q.clear()
        self._cfg_q.clear()
        self._write_config(ser, CAN_RESET_ID, 0)
        self._read_body(config=True, timeout=0.2, ser=ser)
        self._write_config(ser, CAN_BAUD_ID, can_bitrate)
        ack = self._read_body(config=True, timeout=0.45, ser=ser)
        if ack is not None:
            return True
        # One reset pulse, then ask again. Some sticks ignore the first command.
        try:
            ser.dtr = True
            time.sleep(0.05)
            ser.dtr = False
        except Exception:
            pass
        time.sleep(0.35)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        self._rxbuf.clear()
        self._cfg_q.clear()
        self._write_config(ser, CAN_BAUD_ID, can_bitrate)
        return self._read_body(config=True, timeout=0.55, ser=ser) is not None

    def _best_effort_flags(self) -> None:
        ser = self._ser
        if ser is None:
            return
        for config_id, value in ((CAN_ART_ID, 1), (CAN_ABOM_ID, 1)):
            try:
                self._write_config(ser, config_id, value)
                self._read_body(config=True, timeout=0.15, ser=ser)
            except Exception:
                return

    def _read_serial_number(self, timeout: float) -> str | None:
        ser = self._ser
        if ser is None:
            return None
        sn1 = self._read_config(ser, CAN_READ_SERIAL1, timeout)
        if not sn1 or not any(sn1):
            return None
        sn2 = self._read_config(ser, CAN_READ_SERIAL2, timeout)
        if not sn2 or not any(sn2):
            return None
        parts = [f"{sn1[i]:02X}{sn1[i + 1]:02X}" for i in range(0, 8, 2)]
        parts += [f"{sn2[i]:02X}{sn2[i + 1]:02X}" for i in range(0, 4, 2)]
        return "-".join(parts)

    def _config_size(self, config_id: int) -> int:
        if config_id in (CAN_ART_ID, CAN_ABOM_ID):
            return 1
        if config_id in (CAN_BAUD_ID, CAN_SERIALBPS_ID):
            return 4
        if config_id in (CAN_READ_SERIAL1, CAN_READ_SERIAL2):
            return 8
        return 0

    def _write_config(self, ser, config_id: int, value: int) -> None:
        size = self._config_size(config_id)
        data = bytearray(size)
        for i in range(size):
            data[i] = (value >> (8 * i)) & 0xFF
        body = encode_body(
            config_id,
            bytes(data),
            channel=CAN_CONFIG_CHANNEL,
            extended=True,
            dlc=size,
        )
        self._write_body(ser, body)

    def _read_config(self, ser, config_id: int, timeout: float) -> bytes | None:
        size = self._config_size(config_id)
        body = encode_body(
            config_id,
            b"",
            channel=CAN_CONFIG_CHANNEL,
            extended=True,
            remote=True,
            dlc=size,
        )
        self._write_body(ser, body)
        reply = self._read_body(config=True, timeout=timeout, ser=ser)
        if reply is None:
            return None
        return reply[4:12]

    def _write_body(self, ser, body: bytes) -> None:
        ser.write(encode_packet(body))
        ser.flush()

    def _read_body(self, *, config: bool, timeout: float, ser=None):
        ser = self._ser if ser is None else ser
        if ser is None:
            return None
        queue = self._cfg_q if config else self._can_q
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            self._drain_buffer()
            if queue:
                return queue.pop(0)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                waiting = ser.in_waiting
            except Exception:
                waiting = 0
            ser.timeout = min(0.05, remaining) if timeout else 0
            try:
                chunk = ser.read(waiting or 1)
            except Exception:
                return None
            if chunk:
                self._rxbuf.extend(chunk)
            elif timeout == 0:
                return None

    def _drain_buffer(self) -> None:
        for body in pop_frames(self._rxbuf):
            if body[13] == CAN_CONFIG_CHANNEL:
                self._cfg_q.append(body)
            else:
                self._can_q.append(body)


def _is_bad_baud(exc: BaseException) -> bool:
    text = str(exc).lower()
    return "baudrate" in text or "baud rate" in text or isinstance(exc, ValueError)
