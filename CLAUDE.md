# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Python control system for the **Aero Open Hand** — an anthropomorphic robotic hand with 7 Feetech HLS3606M servos driven via TTL serial (1 Mbps, half-duplex). The project replaces the original ESP32 firmware with direct Python-to-servo control.

## Commands

```bash
# Run the GUI
python apps/gui.py [--hand left|right]

# Run the curses CLI
python apps/cli.py [--hand left|right] [--verbose]

# Run tests
pytest tests/

# Run a single test file
pytest tests/test_servo_protocol.py

# Run a specific test
pytest tests/test_servo_protocol.py::TestClassName::test_method_name
```

## Architecture

The system uses a strict layered architecture:

```
apps/gui.py (PyQt6)  or  apps/cli.py (curses)
                  ↓
      src/hand/hand.py        ← abstract Hand base class (ABC)
            ↓                          ↓
src/hand/ttl/hand_ttl.py    src/hand/sdk/hand_esp32.py
  (Feetech TTL serial)        (TetherIA ESP32 SDK)
            ↓
  src/hand/ttl/servo.py      ← high-level Servo API
            ↓
  src/hand/ttl/servo_protocol.py  ← raw Feetech serial protocol
            ↓
       Serial Port
```

### Key Modules

**`src/hand/ttl/servo_protocol.py`** — Low-level protocol. Builds/parses Feetech packets (header `0xFF 0xFF`, checksum `~(ID+len+instr+params)&0xFF`). Thread-safe via internal `RLock`. Handles sign-magnitude 16-bit encoding and little-endian word packing. `sync_read` / `sync_write` for multi-servo efficiency.

**`src/hand/ttl/servo.py`** — Servo register map and mode switching. Three modes: `servo_mode()` (position), `wheel_mode()` (continuous), `ele_mode()` (torque/effort). Key registers: GOAL_POSITION(42), GOAL_TORQUE(44), PRESENT_POSITION(56), PRESENT_TEMPERATURE(63).

**`src/hand/hand.py`** — Abstract `Hand` base class (ABC) with 15 abstract methods. Defines the interface for all hand implementations.

**`src/hand/ttl/hand_ttl.py`** — TTL serial hand controller. Manages:
- Per-servo `ServoData` (grasp/extend calibration counts, direction), persisted in `configs/hand_config.json`
- Position normalization: raw servo counts (0–4095) ↔ normalized (0–65535)
- Three control modes: position, raw torque, PID-based torque
- Background sensor thread: continuous `sync_read` of position/velocity/temperature/current; uses non-blocking lock so control is never blocked by sensor reads
- Thermal protection: reduces torque above 70°C (`TEMP_CUTOFF_C`)
- Soft limits: prevents torque commands from pushing past calibrated extend/grasp bounds
- Homing sequence: auto-homing with current limit (`HOMING_CURRENT_LIMIT=500`) or interactive manual calibration

**`src/hand/sdk/hand_esp32.py`** — TetherIA ESP32 SDK hand controller. Alternative implementation of the `Hand` ABC.

**`src/hand/ttl/pid.py`** — `PIDController` per servo (default Kp=0.015, Ki=0, Kd=0) with anti-windup. `PIDConfig` loads/saves per-servo gains to `configs/pid_config.json`.

**`apps/cli.py`** — Curses interface with `LoggingSerial` proxy for live hex-dump of serial traffic. Includes interactive sliders, PID tuning, servo ID assignment, trim operations.

**`apps/gui.py`** — PyQt6 interface. `QGraphicsScene` renders a hand visualization. Background `QThread` handles sensor polling. Includes `PIDDialog`, `PortDialog`, and macro toolbar with recording/playback controls.

**`src/hand/macro.py`** — Macro recording and playback. Protocol-agnostic (operates on abstract `Hand`).
- `MacroRecorder`: samples `hand.get_positions()` at 50 Hz in a background thread, writes CSV to `runtime/macros/`. Deferred start (only writes once positions differ from initial snapshot by >100 counts). On stop, trims trailing idle rows where positions don't change.
- `MacroPlayer`: loads CSV into memory, plays back via `hand.set_positions()` timed to recorded timestamps. Supports pause/resume, seek, speed multiplier (0.1–5.0×). Playback does not auto-stop at end — user must click "End Playback".
- `MacroMetadata` dataclass: name, rate_hz, hand_type, recorded, duration, sample_count, path.
- `list_macros(macro_dir)` / `delete_macro(path)` helpers.

**`src/hand_detector/`** — Threaded MediaPipe hand landmark detection package.
- **`src/hand_detector/hand_detector.py`** — `HandDetector` class. Opens a webcam and runs the MediaPipe hand landmarker in a daemon thread (`RunningMode.VIDEO`). Exposes:
  - `register_callback(fn) -> str` — registers a callback, returns a UUID callback ID
  - `remove_callback(callback_id)` — unregisters by ID
  - `start()` / `stop()` / `is_running()`
  - Callbacks fire on the detector thread with the canonical hand data dict (see below). Use `pyqtSignal` or `QMetaObject.invokeMethod` to marshal into the Qt main thread.
  - Auto-downloads `hand_landmarker.task` if not present.
- **`src/hand_detector/_landmark_math.py`** — Pure geometry helpers (`format_result`, `format_hand`, `project_finger_2d`, `format_thumb_3d`, etc.) extracted from the reference server. No side effects.
- **`src/hand_detector/hand_landmarker.task`** — MediaPipe float16 model file.

#### Hand data callback format

Every callback receives a dict with this structure:

```python
{
    "timestamp": float,          # time.time()
    "hands": [
        {
            "handedness": "Left" | "Right",
            "confidence": float,
            "fingers": {
                "thumb":  {"cmc": [x,y,z], "mcp": [0,0,0], "ip": [x,y,z], "tip": [x,y,z]},
                "index":  {"mcp": [0,0], "pip": [u,v], "dip": [u,v], "tip": [u,v]},
                "middle": ...,
                "ring":   ...,
                "pinky":  ...,
            }
        }
    ]
}
```

Thumb joints are 3-D (world space, relative to MCP). Index–pinky joints are 2-D projections onto each finger's own bending plane.

### Configuration Files

- **`configs/hand_config.json`** — Calibrated per-servo extend/grasp raw counts, hand type. Written during homing/trimming.
- **`configs/pid_config.json`** — Per-servo PID gains. Written when tuned via GUI/CLI.
- **`configs/detector_params.json`** — Hand detector tuning parameters (coupling, scaling, smoothing).
- **`runtime/macros/*.csv`** — Recorded macro files. CSV with `#` comment headers (name, rate_hz, hand_type, recorded) and columns `timestamp,pos0,…,pos6`. Timestamps are float seconds relative to recording start; positions are normalised 0–65535 ints.

### Feetech Protocol Details

Full specification in `docs/feetech_protocol.md`. Key points:
- Packet: `[0xFF, 0xFF, ID, Length, Instruction, ...Params, Checksum]`
- Response: `[0xFF, 0xFF, ID, Length, Error, ...Data, Checksum]`
- Instructions: PING(1), READ(2), WRITE(3), REG_WRITE(4), REG_ACTION(5), SYNC_READ(130), SYNC_WRITE(131)
- Signed values use sign-magnitude encoding (bit 15 = sign), not two's complement

### Testing Approach

Tests in `tests/test_servo_protocol.py` use `MockSerial` (a `unittest.mock`-based serial stub) injected via `patch`. Tests verify exact byte sequences against worked examples in `docs/feetech_protocol.md`. When adding protocol tests, stage mock responses before calling the method under test, then assert the written bytes match the expected packet.

Tests in `tests/test_macro.py` use `unittest.mock.MagicMock` as a fake `Hand`. They cover recording (deferred start, CSV headers, trailing idle trim), playback (play, pause/resume, stop, seek, speed, on_finished callback), and helpers (list_macros, delete_macro, parse_metadata).

### Hardware Constants (src/hand/ttl/hand_ttl.py)

| Constant | Value | Purpose |
|---|---|---|
| `COUNTS_PER_DEGREE` | 11.375 | Servo resolution |
| `TEMP_CUTOFF_C` | 70 | Thermal protection threshold |
| `HOT_TORQUE_LIMIT` | 500 | Torque cap when hot |
| `HOMING_CURRENT_LIMIT` | 500 | Max current during homing |
| `SOFT_LIMIT_TORQUE` | 200 | Torque applied at soft limit boundary |
| `HOLD_MAG` | 5 | Minimum hold torque magnitude |
