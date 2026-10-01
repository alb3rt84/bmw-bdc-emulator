"""Offline tests for the Robotell framing and BMW cyclic payloads."""

from __future__ import annotations

import unittest

from bmw_cyclic import Signals, due, encode_coolant, encode_fuel, encode_ignition, encode_rpm, encode_speed
from robotell_can import (
    CAN_BAUD_ID,
    CAN_CONFIG_CHANNEL,
    RobotellCan,
    checksum,
    encode_body,
    encode_packet,
    explain_open_error,
    frame_from_body,
    pop_frames,
)


class FramingTests(unittest.TestCase):
    def test_bitrate_500k_packet(self) -> None:
        # 500000 = 0x0007A120, little-endian, config channel.
        body = encode_body(
            CAN_BAUD_ID,
            bytes([0x20, 0xA1, 0x07, 0x00]),
            channel=CAN_CONFIG_CHANNEL,
            extended=True,
            dlc=4,
        )
        self.assertEqual(body[16], checksum(body))
        packet = encode_packet(body)
        self.assertEqual(
            packet,
            bytes([
                0xAA, 0xAA,
                0xD0, 0xFE, 0xFF, 0x01,
                0x20, 0xA1, 0x07, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x04, 0xFF, 0x01, 0x00,
                body[16],
                0x55, 0x55,
            ]),
        )
        self.assertEqual(body[16], 0x9A)

    def test_escape_aa_in_payload(self) -> None:
        body = encode_body(0x123, bytes([0xAA]))
        packet = encode_packet(body)
        self.assertIn(bytes([0xA5, 0xAA]), packet)
        buf = bytearray(b"\x00\x11" + packet + b"\x22")
        frames = pop_frames(buf)
        self.assertEqual(frames, [body])
        self.assertEqual(bytes(buf), b"")
        frame = frame_from_body(frames[0])
        self.assertEqual(frame.arbitration_id, 0x123)
        self.assertEqual(frame.data, bytes([0xAA]))
        self.assertFalse(frame.is_extended)

    def test_partial_packet_is_kept(self) -> None:
        body = encode_body(0x510, bytes([0x00, 0x01, 0, 0, 0, 0, 0, 0]))
        packet = encode_packet(body)
        buf = bytearray(packet[:8])
        self.assertEqual(pop_frames(buf), [])
        buf.extend(packet[8:])
        self.assertEqual(pop_frames(buf), [body])

    def test_explicit_dlc_truncates_and_pads(self) -> None:
        body = encode_body(0x12F, bytes([0x45, 0xFF, 0x11, 0x22]), dlc=2)
        self.assertEqual(body[12], 2)
        self.assertEqual(body[4:6], bytes([0x45, 0xFF]))
        self.assertEqual(body[6:12], bytes(6))
        frame = frame_from_body(body)
        self.assertEqual(frame.dlc, 2)
        self.assertEqual(frame.data, bytes([0x45, 0xFF]))

        padded = encode_body(0x12F, bytes([0x45, 0xFF]), dlc=4)
        self.assertEqual(padded[12], 4)
        self.assertEqual(padded[4:8], bytes([0x45, 0xFF, 0x00, 0x00]))

    def test_remote_frame_keeps_dlc(self) -> None:
        body = encode_body(0x100, b"", remote=True, dlc=4)
        self.assertEqual(body[12], 4)
        self.assertEqual(body[15], 1)
        self.assertEqual(body[4:12], bytes(8))
        frame = frame_from_body(body)
        self.assertTrue(frame.is_remote)
        self.assertEqual(frame.dlc, 4)
        self.assertEqual(frame.data, b"")

    def test_bad_checksum_dropped(self) -> None:
        body = bytearray(encode_body(0x1, b"\x02"))
        body[16] ^= 0xFF
        buf = bytearray(encode_packet(bytes(body)))
        self.assertEqual(pop_frames(buf), [])


class FakeSerial:
    def __init__(self, echo: bool) -> None:
        self.echo = echo
        self.written = bytearray()
        self._in = bytearray()
        self.timeout = 0
        self.is_open = True
        self.dtr = False
        self.rts = False

    @property
    def in_waiting(self) -> int:
        return len(self._in)

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        self._in.clear()

    def flush(self) -> None:
        return None

    def write(self, data: bytes) -> int:
        self.written.extend(data)
        if self.echo:
            pending = bytearray(data)
            for body in pop_frames(pending):
                if body[13] == CAN_CONFIG_CHANNEL:
                    self._in.extend(encode_packet(body))
        return len(data)

    def read(self, n: int = 1) -> bytes:
        if not self._in:
            return b""
        take = bytes(self._in[:n])
        del self._in[:n]
        return take


class OpenTests(unittest.TestCase):
    def test_handshake_confirms_adapter(self) -> None:
        opened: list[tuple[str, int]] = []

        def opener(port: str, baud: int) -> FakeSerial:
            opened.append((port, baud))
            return FakeSerial(echo=True)

        can = RobotellCan()
        result = can.open(
            "COM5",
            usb_baud=115200,
            can_bitrate=500000,
            auto_usb_baud=True,
            serial_opener=opener,
        )
        self.assertEqual(result.port, "COM5")
        self.assertEqual(result.usb_baud, 115200)
        self.assertEqual(result.can_bitrate, 500000)
        self.assertEqual(opened, [("COM5", 115200)])
        can.send(0x12F, bytes([0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]))
        self.assertTrue(can.connected)
        can.close()
        self.assertFalse(can.connected)

    def test_silence_is_not_a_connection(self) -> None:
        def opener(port: str, baud: int) -> FakeSerial:
            return FakeSerial(echo=False)

        can = RobotellCan()
        with self.assertRaises(RuntimeError) as ctx:
            can.open(
                "COM3",
                usb_baud=115200,
                can_bitrate=500000,
                auto_usb_baud=False,
                serial_opener=opener,
            )
        self.assertIn("nie odpowiada", str(ctx.exception))
        self.assertFalse(can.connected)

    def test_busy_port_message(self) -> None:
        text = explain_open_error(PermissionError(13, "Access is denied"), "COM4")
        self.assertIn("zajęty", text)
        self.assertIn("COM4", text)


class BmwPayloadTests(unittest.TestCase):
    def test_rpm_and_ignition_match_firmware(self) -> None:
        # 800 rpm * 4 = 3200 = 0x0C80 in bytes 2..3
        self.assertEqual(encode_rpm(800), bytes([0, 0, 0x0C, 0x80, 0, 0, 0, 0]))
        self.assertEqual(encode_speed(50), bytes([0xF4, 0x01, 0, 0, 0, 0, 0, 0]))
        self.assertEqual(encode_coolant(90)[0], 90 + 48)
        self.assertEqual(encode_fuel(50)[0], 50)
        self.assertEqual(encode_ignition(True)[0], 0x45)
        self.assertEqual(encode_ignition(False)[0], 0x00)

    def test_due_respects_period(self) -> None:
        sig = Signals(ignition=True, rpm=800, speed_kmh=50, fuel_pct=50, coolant_c=90)
        last: dict[int, float] = {}
        first = due(sig, last, now=10.0)
        ids = [can_id for can_id, _ in first]
        self.assertIn(0x510, ids)
        self.assertIn(0x0A5, ids)
        again = due(sig, last, now=10.005)
        self.assertEqual(again, [])
        later = due(sig, last, now=10.03)
        later_ids = {can_id for can_id, _ in later}
        self.assertIn(0x0A5, later_ids)
        self.assertNotIn(0x510, later_ids)


if __name__ == "__main__":
    unittest.main()
