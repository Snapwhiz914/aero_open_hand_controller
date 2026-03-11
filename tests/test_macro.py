"""Tests for hand.macro — MacroRecorder and MacroPlayer."""

import csv
import os
import time
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from hand.macro import (
    MacroRecorder, MacroPlayer, MacroMetadata,
    list_macros, delete_macro, _parse_metadata, _trim_trailing_idle, NUM_SERVOS,
)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _make_mock_hand(positions=None):
    """Return a mock Hand whose get_positions() returns *positions*."""
    hand = MagicMock()
    if positions is None:
        positions = [32768] * NUM_SERVOS
    hand.get_positions.return_value = list(positions)
    return hand


def _write_csv(path: Path, rows: list[tuple], name="test", hand_type="left",
               rate_hz=50):
    """Write a minimal macro CSV for testing."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        f.write(f"# name: {name}\n")
        f.write(f"# rate_hz: {rate_hz}\n")
        f.write(f"# hand_type: {hand_type}\n")
        f.write(f"# recorded: 2026-01-01T00:00:00\n")
        cols = ["timestamp"] + [f"pos{i}" for i in range(NUM_SERVOS)]
        f.write(",".join(cols) + "\n")
        for ts, positions in rows:
            f.write(f"{ts:.3f}," + ",".join(str(p) for p in positions) + "\n")


# ── MacroRecorder ────────────────────────────────────────────────────────────

class TestMacroRecorder:
    def test_start_stop_creates_file(self, tmp_path):
        hand = _make_mock_hand()
        rec = MacroRecorder(hand, macro_dir=tmp_path, hand_type="left")
        assert not rec.is_recording

        rec.start()
        assert rec.is_recording
        time.sleep(0.1)

        path = rec.stop()
        assert not rec.is_recording
        assert path is not None
        assert path.exists()
        assert path.suffix == ".csv"

    def test_csv_has_headers(self, tmp_path):
        hand = _make_mock_hand()
        rec = MacroRecorder(hand, macro_dir=tmp_path, hand_type="right")
        rec.start()
        time.sleep(0.05)
        path = rec.stop()

        text = path.read_text()
        assert "# name:" in text
        assert "# rate_hz:" in text
        assert "# hand_type: right" in text
        assert "# recorded:" in text
        assert "timestamp,pos0,pos1,pos2,pos3,pos4,pos5,pos6" in text

    def test_deferred_start_no_data_if_no_movement(self, tmp_path):
        """If positions never change, no data rows are written."""
        hand = _make_mock_hand([1000] * NUM_SERVOS)
        rec = MacroRecorder(hand, macro_dir=tmp_path, rate_hz=100)
        rec.start()
        time.sleep(0.15)
        path = rec.stop()

        data_lines = [l for l in path.read_text().splitlines()
                       if l and not l.startswith("#") and not l.startswith("timestamp")]
        assert len(data_lines) == 0

    def test_deferred_start_writes_after_movement(self, tmp_path):
        """Once positions change beyond threshold, rows are written."""
        positions = [1000] * NUM_SERVOS
        hand = _make_mock_hand(positions)
        call_count = 0

        def changing_positions():
            nonlocal call_count
            call_count += 1
            if call_count > 3:
                return [2000] * NUM_SERVOS  # beyond threshold
            return [1000] * NUM_SERVOS

        hand.get_positions.side_effect = changing_positions

        rec = MacroRecorder(hand, macro_dir=tmp_path, rate_hz=100)
        rec.start()
        time.sleep(0.3)
        path = rec.stop()

        data_lines = [l for l in path.read_text().splitlines()
                       if l and not l.startswith("#") and not l.startswith("timestamp")]
        assert len(data_lines) > 0

    def test_rename(self, tmp_path):
        hand = _make_mock_hand()
        rec = MacroRecorder(hand, macro_dir=tmp_path)
        rec.start()
        time.sleep(0.05)
        rec.stop()
        new_path = rec.rename("grab ball")
        assert new_path is not None
        assert "grab_ball" in new_path.stem
        assert new_path.exists()
        assert "# name: grab_ball" in new_path.read_text()

    def test_elapsed(self, tmp_path):
        hand = _make_mock_hand()
        rec = MacroRecorder(hand, macro_dir=tmp_path)
        assert rec.elapsed == 0.0
        rec.start()
        time.sleep(0.15)
        e = rec.elapsed
        assert e >= 0.1
        rec.stop()

    def test_creates_macro_dir(self, tmp_path):
        subdir = tmp_path / "nested" / "macros"
        hand = _make_mock_hand()
        rec = MacroRecorder(hand, macro_dir=subdir)
        rec.start()
        time.sleep(0.05)
        rec.stop()
        assert subdir.is_dir()


# ── MacroPlayer ──────────────────────────────────────────────────────────────

class TestMacroPlayer:
    def _sample_rows(self, n=10, interval=0.02):
        """Generate n sample rows at *interval* seconds apart."""
        rows = []
        for i in range(n):
            ts = i * interval
            positions = [1000 + i * 100] * NUM_SERVOS
            rows.append((ts, positions))
        return rows

    def test_load(self, tmp_path):
        rows = self._sample_rows()
        path = tmp_path / "test.csv"
        _write_csv(path, rows, name="load_test")

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        meta = player.load(path)

        assert meta.name == "load_test"
        assert meta.sample_count == len(rows)
        assert meta.duration == pytest.approx(rows[-1][0], abs=0.01)
        assert player.duration == pytest.approx(rows[-1][0], abs=0.01)

    def test_play_calls_set_positions(self, tmp_path):
        rows = self._sample_rows(5, interval=0.02)
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)

        done = threading.Event()
        player.play(speed=5.0, on_finished=done.set)
        done.wait(timeout=3.0)

        assert hand.set_positions.call_count >= 3
        assert player.progress == pytest.approx(1.0)
        assert not player.is_playing

    def test_pause_resume(self, tmp_path):
        rows = self._sample_rows(50, interval=0.02)  # 1 second of data
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)

        done = threading.Event()
        player.play(speed=1.0, on_finished=done.set)
        time.sleep(0.1)

        player.pause()
        assert player.is_paused
        time.sleep(0.05)  # let any in-flight call settle
        calls_at_pause = hand.set_positions.call_count
        time.sleep(0.15)
        # No new calls while paused (allow at most 1 in-flight)
        assert hand.set_positions.call_count <= calls_at_pause + 1

        player.resume()
        assert not player.is_paused
        done.wait(timeout=5.0)
        assert hand.set_positions.call_count > calls_at_pause

    def test_stop(self, tmp_path):
        rows = self._sample_rows(100, interval=0.02)
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)

        player.play(speed=1.0)
        time.sleep(0.1)
        player.stop()
        assert not player.is_playing

    def test_seek(self, tmp_path):
        rows = self._sample_rows(20, interval=0.05)  # 1 second
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)

        player.seek(0.5)
        assert player.progress == pytest.approx(0.5, abs=0.1)
        # set_positions called with the position at ~50%
        hand.set_positions.assert_called()

    def test_speed_multiplier(self, tmp_path):
        rows = self._sample_rows(20, interval=0.02)  # 0.38s of data
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)

        done = threading.Event()
        start = time.monotonic()
        player.play(speed=5.0, on_finished=done.set)
        done.wait(timeout=3.0)
        elapsed = time.monotonic() - start
        # At 5x speed, 0.38s should take ~0.076s (plus overhead)
        assert elapsed < 0.5

    def test_unload(self, tmp_path):
        rows = self._sample_rows()
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)
        player.unload()
        assert player.metadata is None
        assert player.duration == 0.0

    def test_on_finished_callback(self, tmp_path):
        rows = self._sample_rows(5, interval=0.01)
        path = tmp_path / "test.csv"
        _write_csv(path, rows)

        hand = _make_mock_hand()
        player = MacroPlayer(hand)
        player.load(path)

        finished = threading.Event()
        player.play(speed=10.0, on_finished=finished.set)
        assert finished.wait(timeout=3.0)


# ── list_macros / delete_macro ───────────────────────────────────────────────

class TestHelpers:
    def test_list_macros_empty(self, tmp_path):
        assert list_macros(tmp_path / "nonexistent") == []

    def test_list_macros(self, tmp_path):
        rows = [(0.0, [0]*7), (0.1, [100]*7)]
        _write_csv(tmp_path / "a.csv", rows, name="alpha")
        time.sleep(0.05)
        _write_csv(tmp_path / "b.csv", rows, name="beta")

        macros = list_macros(tmp_path)
        assert len(macros) == 2
        # Newest first
        assert macros[0].name == "beta"

    def test_delete_macro(self, tmp_path):
        path = tmp_path / "delete_me.csv"
        _write_csv(path, [(0.0, [0]*7)])
        assert path.exists()
        delete_macro(path)
        assert not path.exists()

    def test_parse_metadata(self, tmp_path):
        rows = [(0.0, [0]*7), (0.5, [100]*7), (1.0, [200]*7)]
        path = tmp_path / "meta_test.csv"
        _write_csv(path, rows, name="my_macro", hand_type="right", rate_hz=100)

        meta = _parse_metadata(path)
        assert meta.name == "my_macro"
        assert meta.hand_type == "right"
        assert meta.rate_hz == 100.0
        assert meta.sample_count == 3
        assert meta.duration == pytest.approx(1.0)

    def test_trim_trailing_idle(self, tmp_path):
        """Trailing rows with identical positions are trimmed, keeping one."""
        rows = [
            (0.0, [0, 100, 200, 300, 400, 500, 600]),
            (0.1, [10, 110, 210, 310, 410, 510, 610]),  # last change
            (0.2, [10, 110, 210, 310, 410, 510, 610]),  # idle
            (0.3, [10, 110, 210, 310, 410, 510, 610]),  # idle
            (0.4, [10, 110, 210, 310, 410, 510, 610]),  # idle
        ]
        path = tmp_path / "trim_test.csv"
        _write_csv(path, rows)
        _trim_trailing_idle(path)

        data_lines = [l for l in path.read_text().splitlines()
                       if l.strip() and not l.startswith("#") and not l.startswith("timestamp")]
        # Should keep rows at 0.0 and 0.1 (last row with a position change)
        assert len(data_lines) == 2

    def test_trim_trailing_idle_no_trim_needed(self, tmp_path):
        """If every row differs, nothing is trimmed."""
        rows = [
            (0.0, [0]*7),
            (0.1, [100]*7),
            (0.2, [200]*7),
        ]
        path = tmp_path / "no_trim.csv"
        _write_csv(path, rows)
        _trim_trailing_idle(path)

        data_lines = [l for l in path.read_text().splitlines()
                       if l.strip() and not l.startswith("#") and not l.startswith("timestamp")]
        assert len(data_lines) == 3

    def test_trim_trailing_idle_all_same(self, tmp_path):
        """If all rows are identical, keep just the first."""
        rows = [
            (0.0, [500]*7),
            (0.1, [500]*7),
            (0.2, [500]*7),
        ]
        path = tmp_path / "all_same.csv"
        _write_csv(path, rows)
        _trim_trailing_idle(path)

        data_lines = [l for l in path.read_text().splitlines()
                       if l.strip() and not l.startswith("#") and not l.startswith("timestamp")]
        assert len(data_lines) == 1
