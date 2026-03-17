"""Macro recording and playback for the Aero Open Hand.

Records servo positions to CSV files and plays them back with transport controls.
Operates against the abstract Hand base class (protocol-agnostic).
"""

import csv
import io
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from hand import Hand

NUM_SERVOS = 7
_DEFAULT_RATE_HZ = 50
_DEFERRED_START_THRESHOLD = 100  # ~0.15% of 65535


@dataclass
class MacroMetadata:
    name: str
    rate_hz: float
    hand_type: str
    recorded: str  # ISO 8601
    duration: float
    sample_count: int
    path: Path


def _parse_metadata(path: Path) -> MacroMetadata:
    """Read CSV comment headers and count data rows to build MacroMetadata."""
    meta = {"name": path.stem, "rate_hz": _DEFAULT_RATE_HZ,
            "hand_type": "unknown", "recorded": ""}
    sample_count = 0
    last_ts = 0.0
    with open(path, newline="") as f:
        for line in f:
            line = line.strip()
            if line.startswith("#"):
                parts = line.lstrip("# ").split(":", 1)
                if len(parts) == 2:
                    key = parts[0].strip()
                    val = parts[1].strip()
                    if key in meta:
                        meta[key] = val
            elif line and not line.startswith("timestamp"):
                sample_count += 1
                try:
                    last_ts = float(line.split(",", 1)[0])
                except ValueError:
                    pass
    try:
        rate = float(meta["rate_hz"])
    except ValueError:
        rate = _DEFAULT_RATE_HZ
    return MacroMetadata(
        name=meta["name"],
        rate_hz=rate,
        hand_type=meta["hand_type"],
        recorded=meta["recorded"],
        duration=last_ts,
        sample_count=sample_count,
        path=path,
    )


def list_macros(macro_dir: str | Path) -> list[MacroMetadata]:
    """Return metadata for all CSV macros in *macro_dir*, newest first."""
    macro_dir = Path(macro_dir)
    if not macro_dir.is_dir():
        return []
    results = []
    for p in sorted(macro_dir.glob("*.csv"), key=os.path.getmtime, reverse=True):
        try:
            results.append(_parse_metadata(p))
        except Exception:
            pass
    return results


def delete_macro(path: str | Path) -> None:
    Path(path).unlink(missing_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# MacroRecorder
# ─────────────────────────────────────────────────────────────────────────────

class MacroRecorder:
    """Records hand positions to a CSV file at ~50 Hz."""

    def __init__(self, hand: Hand, macro_dir: str | Path = "macros",
                 hand_type: str = "left", rate_hz: float = _DEFAULT_RATE_HZ):
        self._hand = hand
        self._macro_dir = Path(macro_dir)
        self._hand_type = hand_type
        self._rate_hz = rate_hz
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._start_time: float | None = None
        self._file: io.TextIOWrapper | None = None
        self._path: Path | None = None
        self._recording = False
        self._elapsed = 0.0

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def elapsed(self) -> float:
        if self._start_time is not None and self._recording:
            return time.monotonic() - self._start_time
        return self._elapsed

    def start(self) -> None:
        if self._recording:
            return
        self._macro_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = self._macro_dir / f"recording_{ts}.csv"
        self._stop_event.clear()
        self._recording = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> Path | None:
        """Stop recording and return the path to the saved CSV."""
        if not self._recording:
            return None
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._recording = False
        if self._path is not None and self._path.exists():
            _trim_trailing_idle(self._path)
        return self._path

    def rename(self, name: str) -> Path | None:
        """Rename the last recording file to include *name*. Returns new path."""
        if self._path is None or not self._path.exists():
            return None
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(c if (c.isalnum() or c in "-_") else "_" for c in name).strip()
        if not safe_name:
            safe_name = "macro"
        new_path = self._path.parent / f"{safe_name}_{ts}.csv"
        self._path.rename(new_path)
        self._path = new_path
        # Update the name header inside the file
        _rewrite_name_header(new_path, safe_name)
        return new_path

    def _run(self) -> None:
        interval = 1.0 / self._rate_hz
        initial_positions: list[int] | None = None
        writing_started = False
        self._start_time = time.monotonic()
        iso_now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")

        try:
            f = open(self._path, "w", newline="")
            self._file = f
            # Write headers
            f.write(f"# name: recording\n")
            f.write(f"# rate_hz: {self._rate_hz}\n")
            f.write(f"# hand_type: {self._hand_type}\n")
            f.write(f"# recorded: {iso_now}\n")
            cols = ["timestamp"] + [f"pos{i}" for i in range(NUM_SERVOS)]
            f.write(",".join(cols) + "\n")
            f.flush()

            while not self._stop_event.is_set():
                loop_start = time.monotonic()
                try:
                    positions = self._hand.get_positions()
                except Exception:
                    self._stop_event.wait(interval)
                    continue

                if initial_positions is None:
                    initial_positions = list(positions)

                if not writing_started:
                    if any(abs(positions[i] - initial_positions[i]) > _DEFERRED_START_THRESHOLD
                           for i in range(NUM_SERVOS)):
                        writing_started = True
                        self._start_time = time.monotonic()
                    else:
                        self._stop_event.wait(max(0, interval - (time.monotonic() - loop_start)))
                        continue

                ts = time.monotonic() - self._start_time
                row = f"{ts:.3f}," + ",".join(str(p) for p in positions) + "\n"
                f.write(row)
                if int(ts * 10) % 5 == 0:  # flush every ~0.5s
                    f.flush()

                elapsed = time.monotonic() - loop_start
                self._stop_event.wait(max(0, interval - elapsed))

        finally:
            self._elapsed = time.monotonic() - (self._start_time or time.monotonic())
            if self._file is not None:
                self._file.close()
                self._file = None


def _trim_trailing_idle(path: Path) -> None:
    """Remove trailing rows where positions don't change from the end."""
    lines = path.read_text().splitlines(keepends=True)
    # Separate header lines from data lines
    header = []
    data = []
    for line in lines:
        stripped = line.strip()
        if not data and (stripped.startswith("#") or stripped.startswith("timestamp") or not stripped):
            header.append(line)
        else:
            data.append(line)
    if len(data) < 2:
        return
    # Parse the last row's positions
    last_parts = data[-1].strip().split(",")
    if len(last_parts) < 1 + NUM_SERVOS:
        return
    last_positions = last_parts[1:]
    # Walk backwards to find the first row that differs from the last
    trim_to = len(data)
    for j in range(len(data) - 2, -1, -1):
        parts = data[j].strip().split(",")
        if len(parts) < 1 + NUM_SERVOS:
            break
        if parts[1:] != last_positions:
            # Keep one row past the last change so the final position is recorded
            trim_to = j + 2
            break
    else:
        # All rows identical — keep just the first one
        trim_to = 1
    if trim_to < len(data):
        data = data[:trim_to]
        path.write_text("".join(header + data))


def _rewrite_name_header(path: Path, new_name: str) -> None:
    """Replace the '# name:' line inside a CSV file."""
    text = path.read_text()
    lines = text.split("\n")
    for i, line in enumerate(lines):
        if line.startswith("# name:"):
            lines[i] = f"# name: {new_name}"
            break
    path.write_text("\n".join(lines))


# ─────────────────────────────────────────────────────────────────────────────
# MacroPlayer
# ─────────────────────────────────────────────────────────────────────────────

class MacroPlayer:
    """Plays back a recorded macro CSV through a Hand."""

    def __init__(self, hand: Hand):
        self._hand = hand
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # not paused initially

        self._rows: list[tuple[float, list[int]]] = []
        self._metadata: MacroMetadata | None = None
        self._playing = False
        self._paused = False
        self._progress = 0.0
        self._current_index = 0
        self._speed = 1.0
        self._on_finished: callable = None

    @property
    def is_playing(self) -> bool:
        return self._playing

    @property
    def is_paused(self) -> bool:
        return self._paused

    @property
    def progress(self) -> float:
        return self._progress

    @property
    def duration(self) -> float:
        if self._rows:
            return self._rows[-1][0]
        return 0.0

    @property
    def metadata(self) -> MacroMetadata | None:
        return self._metadata

    def load(self, path: str | Path) -> MacroMetadata:
        """Load a CSV macro into memory. Returns its metadata."""
        path = Path(path)
        self._rows = []
        with open(path, newline="") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or line.startswith("timestamp"):
                    continue
                parts = line.split(",")
                if len(parts) < 1 + NUM_SERVOS:
                    continue
                try:
                    ts = float(parts[0])
                    positions = [int(parts[i + 1]) for i in range(NUM_SERVOS)]
                    self._rows.append((ts, positions))
                except (ValueError, IndexError):
                    continue
        self._metadata = _parse_metadata(path)
        self._progress = 0.0
        self._current_index = 0
        return self._metadata

    def play(self, speed: float = 1.0, on_finished: callable = None) -> None:
        """Start playback from the current position."""
        if not self._rows or self._playing:
            return
        self._speed = speed
        self._on_finished = on_finished
        self._stop_event.clear()
        self._pause_event.set()
        self._playing = True
        self._paused = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def pause(self) -> None:
        if self._playing and not self._paused:
            self._paused = True
            self._pause_event.clear()

    def resume(self) -> None:
        if self._playing and self._paused:
            self._paused = False
            self._pause_event.set()

    def stop(self) -> None:
        if not self._playing:
            return
        self._stop_event.set()
        self._pause_event.set()  # unblock if paused
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._playing = False
        self._paused = False

    def seek(self, fraction: float) -> None:
        """Seek to a position (0.0–1.0) in the macro."""
        fraction = max(0.0, min(1.0, fraction))
        if not self._rows:
            return
        target_ts = fraction * self._rows[-1][0]
        # Binary search for the closest index
        lo, hi = 0, len(self._rows) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self._rows[mid][0] < target_ts:
                lo = mid + 1
            else:
                hi = mid
        with self._lock:
            self._current_index = lo
            self._progress = fraction
        # Apply the position at seek point immediately
        if self._rows:
            try:
                self._hand.set_positions(self._rows[lo][1])
            except Exception:
                pass

    def set_speed(self, speed: float) -> None:
        self._speed = max(0.1, min(5.0, speed))

    def unload(self) -> None:
        """Unload the current macro."""
        self.stop()
        self._rows = []
        self._metadata = None
        self._progress = 0.0
        self._current_index = 0

    def _run(self) -> None:
        total_duration = self._rows[-1][0] if self._rows else 0.0
        start_index = self._current_index
        # Wall-clock reference for timing
        wall_start = time.monotonic()
        ts_offset = self._rows[start_index][0] if start_index < len(self._rows) else 0.0

        try:
            i = start_index
            while i < len(self._rows) and not self._stop_event.is_set():
                # Handle pause
                if not self._pause_event.is_set():
                    pause_start = time.monotonic()
                    self._pause_event.wait()
                    if self._stop_event.is_set():
                        break
                    # Adjust wall_start to account for pause duration
                    wall_start += time.monotonic() - pause_start

                ts, positions = self._rows[i]
                # When should this frame play (wall-clock)?
                target_wall = wall_start + (ts - ts_offset) / self._speed
                now = time.monotonic()
                if target_wall > now:
                    # Wait, but check for stop/pause
                    wait_time = target_wall - now
                    if self._stop_event.wait(wait_time):
                        break

                try:
                    self._hand.set_positions(positions)
                except Exception:
                    pass

                with self._lock:
                    self._current_index = i
                    self._progress = ts / total_duration if total_duration > 0 else 1.0

                i += 1

            # Finished naturally
            with self._lock:
                self._progress = 1.0
        finally:
            self._playing = False
            self._paused = False
            if self._on_finished is not None:
                self._on_finished()
