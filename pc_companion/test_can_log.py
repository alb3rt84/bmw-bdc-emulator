"""Offline tests for the Robotell CAN recording format."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from can_log import (
    MAGIC,
    VERSION,
    LoggedFrame,
    RecordingWriter,
    format_event,
    load_recording,
    parse_event,
    parse_id_filter,
    playback_waits,
)
from robotell_can import CanFrame


def _frame(can_id: int, data: bytes, **kwargs) -> CanFrame:
    return CanFrame(can_id, data, **kwargs)


class FormatTests(unittest.TestCase):
    def test_roundtrip_standard_extended_remote_and_empty(self) -> None:
        samples = [
            LoggedFrame(0.1, "RX", _frame(0x12F, bytes([0x45, 0xFF, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]))),
            LoggedFrame(1.0, "TX", _frame(0x18DA10F1, b"\x02\x3E\x00", is_extended=True, dlc=3)),
            LoggedFrame(1.5, "RX", _frame(0x100, b"", is_remote=True, dlc=4)),
            LoggedFrame(2.0, "RX", _frame(0x330, b"", dlc=0)),
        ]
        for event in samples:
            again = parse_event(format_event(event))
            self.assertEqual(again.t_s, event.t_s)
            self.assertEqual(again.direction, event.direction)
            self.assertEqual(again.frame.arbitration_id, event.frame.arbitration_id)
            self.assertEqual(again.frame.is_extended, event.frame.is_extended)
            self.assertEqual(again.frame.is_remote, event.frame.is_remote)
            self.assertEqual(again.frame.dlc, event.frame.dlc)
            self.assertEqual(again.frame.data, event.frame.data)

    def test_remote_line_has_no_data_token(self) -> None:
        line = format_event(LoggedFrame(0, "RX", _frame(0x100, b"", is_remote=True, dlc=2)))
        self.assertTrue(line.endswith(" 0 1 2"))

    def test_bad_dlc_and_standard_id(self) -> None:
        with self.assertRaises(ValueError):
            parse_event("0 RX 800 0 0 1 00")
        with self.assertRaises(ValueError):
            parse_event("0 RX 12F 0 0 1")
        with self.assertRaises(ValueError):
            parse_event("0 RX 12F 0 1 2 AABB")


class FileTests(unittest.TestCase):
    def test_writer_then_load(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / "bus.canlog")
            writer = RecordingWriter(path, {"port": "COM5", "can_bitrate": "500000"})
            first = writer.add("RX", _frame(0x12F, bytes([0x45, 0xFF])), now=writer._t0 + 0.05)
            second = writer.add(
                "TX",
                _frame(0x6F1, bytes([0x10, 0x02, 0x3E, 0x00]), dlc=4),
                now=writer._t0 + 0.20,
            )
            writer.close()
            text = Path(path).read_text(encoding="utf-8")
            loaded = load_recording(path)
        self.assertTrue(text.startswith(f"# {MAGIC} {VERSION}\n"))
        self.assertIn("# port COM5\n", text)
        self.assertEqual(loaded.meta["port"], "COM5")
        self.assertEqual(loaded.meta["can_bitrate"], "500000")
        self.assertEqual(len(loaded.events), 2)
        self.assertAlmostEqual(loaded.events[0].t_s, first.t_s)
        self.assertEqual(loaded.events[0].frame.data, bytes([0x45, 0xFF]))
        self.assertEqual(loaded.events[1].direction, "TX")
        self.assertEqual(loaded.events[1].frame.dlc, 4)
        self.assertAlmostEqual(second.t_s, 0.20)

    def test_file_header_and_skip_comments(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bus.canlog"
            path.write_text(
                f"# {MAGIC} {VERSION}\n"
                "# note bench\n"
                "\n"
                "0.000000 RX 12F 0 0 2 45FF\n",
                encoding="utf-8",
            )
            loaded = load_recording(str(path))
        self.assertEqual(loaded.meta["note"], "bench")
        self.assertEqual(loaded.events[0].frame.arbitration_id, 0x12F)

    def test_missing_magic_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bus.canlog"
            path.write_text("0.000000 RX 1 0 0 1 00\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_recording(str(path))

    def test_bad_line_names_the_number(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bus.canlog"
            path.write_text(f"# {MAGIC} {VERSION}\nnie-ramka\n", encoding="utf-8")
            with self.assertRaises(ValueError) as ctx:
                load_recording(str(path))
        self.assertIn("Linia 2", str(ctx.exception))


class PlaybackTests(unittest.TestCase):
    def test_waits_follow_timestamps_and_speed(self) -> None:
        events = [
            LoggedFrame(0.5, "RX", _frame(1, b"\x00")),
            LoggedFrame(0.6, "RX", _frame(2, b"\x00")),
            LoggedFrame(1.0, "RX", _frame(3, b"\x00")),
        ]
        self.assertEqual([round(item, 6) for item in playback_waits(events)], [0.5, 0.1, 0.4])
        self.assertEqual([round(item, 6) for item in playback_waits(events, speed=2)], [0.25, 0.05, 0.2])

    def test_backwards_timestamp_does_not_wait_negative(self) -> None:
        events = [
            LoggedFrame(1.0, "RX", _frame(1, b"\x00")),
            LoggedFrame(0.2, "RX", _frame(2, b"\x00")),
        ]
        self.assertEqual(playback_waits(events), [1.0, 0.0])


class FilterTests(unittest.TestCase):
    def test_empty_filter_is_everything(self) -> None:
        self.assertIsNone(parse_id_filter("  "))

    def test_hex_tokens(self) -> None:
        self.assertEqual(parse_id_filter("12F, 0x330 6f1"), {0x12F, 0x330, 0x6F1})

    def test_bad_token(self) -> None:
        with self.assertRaises(ValueError):
            parse_id_filter("12F GG")


if __name__ == "__main__":
    unittest.main()
