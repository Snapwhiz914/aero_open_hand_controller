#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ESP32 SDK hand controller — wraps TetherIA's serial protocol.

UNSUPPORTED features (raise NotImplementedError):
  - set_torques()          — ESP32 ctrl_torque only takes unsigned magnitudes;
                             direction (grasp vs extend) is not exposed.
  - set_torque_positions() — PID torque mode is not implemented for ESP32.
  - home_manual()          — ESP32 firmware only supports automatic homing.

DEGRADED features (functional but different semantics):
  - get_velocities()  — returns RPM (not raw Feetech speed units).
  - get_currents()    — returns mA  (not raw Feetech current units).
  - grasp()           — speed_dps and torque_limit params are ignored;
                        the ESP32 protocol does not expose per-move limits.
  - trim()            — return value is the ESP32 extend count (uint16),
                        not a Feetech raw count (0–4095).
  - ping_all()        — always returns {i: False for i in range(7)};
                        the ESP32 SDK has no ping mechanism.
  - update_pid()      — no-op; ESP32 has no software PID layer.
"""

import os
import struct
import threading
import time
from collections.abc import Callable
from serial import Serial, SerialTimeoutException

from ..hand import Hand
from .aero_hand_constants import AeroHandConstants

## Setup Modes
HOMING_MODE = 0x01
SET_ID_MODE = 0x02
TRIM_MODE   = 0x03

## Command Modes
CTRL_POS = 0x11
CTRL_TOR = 0x12

## Request Modes
GET_POS  = 0x22
GET_VEL  = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25

_UINT16_MAX = 65535
NUM_SERVOS  = 7


# ---------------------------------------------------------------------------
# PID config stub (no PID support on ESP32)
# ---------------------------------------------------------------------------

class _NullPIDConfig:
    """Read-only PIDConfig substitute that returns zeroed gains."""

    def get(self, channel: int) -> dict:
        return {"kp": 0.0, "ki": 0.0, "kd": 0.0}

    def set(self, channel: int, kp: float, ki: float, kd: float) -> None:
        pass  # no-op


# ---------------------------------------------------------------------------
# Compatibility shims so existing CLI/GUI internal accesses don't crash
# ---------------------------------------------------------------------------

class _ProtocolShim:
    """Minimal shim that exposes _serial (pointing at the ESP32 serial port)
    and a dummy lock, so LoggingSerial injection in cli.py works without
    crashing (the log will capture raw ESP32 frames, not Feetech packets)."""

    def __init__(self, serial_port):
        self._serial = serial_port
        self.lock = threading.RLock()

    def close(self):
        pass


class _ServoDataShim:
    """Minimal shim that exposes extend_count so TrimDialog can display it."""

    def __init__(self):
        self.extend_count = 0
        self.grasp_count = 0
        self.servo_direction = 1


# ---------------------------------------------------------------------------
# HandESP32
# ---------------------------------------------------------------------------

class HandESP32(Hand):
    """Hand controller that communicates via an ESP32 running TetherIA firmware.

    The ESP32 sits between Python and the Feetech servos and exposes a
    higher-level binary framing protocol at 921 600 baud.
    """

    def __init__(self, port: str, hand_type: str = "left",
                 baudrate: int = 921600):
        """
        Args:
            port:      Serial port string (e.g. "COM3" or "/dev/ttyUSB0").
            hand_type: "left" or "right" (stored but not used by ESP32 firmware).
            baudrate:  Serial baud rate (default 921 600).
        """
        self._hand_type = hand_type.lower()
        self.ser = Serial(port, baudrate, timeout=0.05, write_timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        constants = AeroHandConstants()
        self._actuation_lower = constants.actuation_lower_limits
        self._actuation_upper = constants.actuation_upper_limits

        # Cached sensor data (updated by background thread)
        self._metrics_lock = threading.Lock()
        self._positions    = [0] * NUM_SERVOS  # 0–65535 normalised
        self._velocities   = [0] * NUM_SERVOS  # RPM (signed)
        self._currents     = [0] * NUM_SERVOS  # mA  (signed)
        self._temperatures = [0] * NUM_SERVOS  # °C

        self._homing = False

        # Serial lock — ensures only one thread accesses the port at a time
        self._serial_lock = threading.Lock()

        # Stubs expected by CLI/GUI internal accesses
        self._protocol = _ProtocolShim(self.ser)
        self._sd       = [_ServoDataShim() for _ in range(NUM_SERVOS)]

        self._pid_config_obj = _NullPIDConfig()

        # Background sensor polling thread
        self._running = True
        self._sensor_thread = threading.Thread(
            target=self._sensor_loop, daemon=True)
        self._sensor_thread.start()

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def close(self) -> None:
        self._running = False
        if self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=1.0)
        self.ser.close()

    # ── Background sensor polling ────────────────────────────────────────────

    def _sensor_loop(self):
        while self._running:
            if self._homing:
                time.sleep(0.1)
                continue

            acquired = self._serial_lock.acquire(blocking=False)
            if not acquired:
                time.sleep(0.02)
                continue
            try:
                pos  = self._read_positions_raw()
                vel  = self._read_velocities_raw()
                curr = self._read_currents_raw()
                temp = self._read_temperatures_raw()
            finally:
                self._serial_lock.release()

            with self._metrics_lock:
                if pos  is not None: self._positions    = pos
                if vel  is not None: self._velocities   = vel
                if curr is not None: self._currents     = curr
                if temp is not None: self._temperatures = temp

            time.sleep(0.05)

    # ── Low-level serial helpers ──────────────────────────────────────────────

    def _send_data(self, header: int, payload: list[int] = None):
        if payload is None:
            payload = [0] * 7
        assert len(payload) == 7
        msg = struct.pack("<2B7H", header & 0xFF, 0x00,
                         *(v & 0xFFFF for v in payload))
        self.ser.write(msg)
        self.ser.flush()

    def _wait_for_ack(self, opcode: int, timeout_s: float) -> "bytes | None":
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.ser.read(16)
            if len(frame) != 16:
                continue
            if frame[0] == (opcode & 0xFF) and frame[1] == 0x00:
                return frame[2:]
        return None

    def _read_sensor_block(self, opcode: int) -> "list[int] | None":
        """Send a GET_* request and decode the 7-value uint16 response."""
        try:
            self.ser.reset_input_buffer()
            self._send_data(opcode)
        except SerialTimeoutException:
            return None
        resp = self.ser.read(16)
        if len(resp) != 16:
            return None
        data = struct.unpack("<2B7H", resp)
        if data[0] != opcode:
            self.ser.reset_input_buffer()
            return None
        return list(data[2:])

    def _read_sensor_block_signed(self, opcode: int) -> "list[int] | None":
        """Send a GET_* request and decode the 7-value int16 response."""
        try:
            self.ser.reset_input_buffer()
            self._send_data(opcode)
        except SerialTimeoutException:
            return None
        resp = self.ser.read(16)
        if len(resp) != 16:
            return None
        data = struct.unpack("<2B7h", resp)
        if data[0] != opcode:
            self.ser.reset_input_buffer()
            return None
        return list(data[2:])

    # ── Actuation ↔ normalised (0–65535) conversion ──────────────────────────

    def _act_deg_to_u16(self, i: int, act_deg: float) -> int:
        lo = self._actuation_lower[i]
        hi = self._actuation_upper[i]
        if hi == lo:
            return 0
        return max(0, min(_UINT16_MAX, int((act_deg - lo) / (hi - lo) * _UINT16_MAX)))

    def _u16_to_act_deg(self, i: int, u16: int) -> float:
        lo = self._actuation_lower[i]
        hi = self._actuation_upper[i]
        return lo + (u16 / _UINT16_MAX) * (hi - lo)

    # ── Raw sensor reads (called while _serial_lock is held) ──────────────────

    def _read_positions_raw(self) -> "list[int] | None":
        vals = self._read_sensor_block(GET_POS)
        if vals is None:
            return None
        # vals are uint16 normalised actuations from the ESP32 firmware
        return [
            self._act_deg_to_u16(
                i,
                self._actuation_lower[i]
                + (vals[i] / _UINT16_MAX)
                * (self._actuation_upper[i] - self._actuation_lower[i])
            )
            for i in range(NUM_SERVOS)
        ]

    def _read_velocities_raw(self) -> "list[int] | None":
        vals = self._read_sensor_block_signed(GET_VEL)
        if vals is None:
            return None
        # Feetech docs: 1 unit = 0.732 RPM
        return [int(v * 0.732) for v in vals]

    def _read_currents_raw(self) -> "list[int] | None":
        vals = self._read_sensor_block_signed(GET_CURR)
        if vals is None:
            return None
        # Feetech docs: 1 unit = 6.5 mA
        return [int(v * 6.5) for v in vals]

    def _read_temperatures_raw(self) -> "list[int] | None":
        vals = self._read_sensor_block(GET_TEMP)
        if vals is None:
            return None
        return [int(v) for v in vals]

    # ── Control: position mode ────────────────────────────────────────────────

    def set_positions(self, positions: list[int]) -> None:
        """Set normalised positions (0–65535) for all 7 servos.

        The ESP32 firmware accepts the same 0–65535 actuation scale, so no
        kinematic conversion is needed — values are forwarded directly.
        """
        if self._homing:
            return
        payload = [max(0, min(_UINT16_MAX, p)) for p in positions]
        with self._serial_lock:
            try:
                self._send_data(CTRL_POS, payload)
            except SerialTimeoutException as e:
                print(f"HandESP32.set_positions: serial timeout: {e}")

    # ── Control: raw torque mode ──────────────────────────────────────────────

    def set_torques(self, torques: list[int]) -> None:
        raise NotImplementedError(
            "HandESP32 does not support directional torque control. "
            "The ESP32 ctrl_torque command only accepts unsigned magnitudes "
            "and does not expose a grasp/extend direction."
        )

    # ── Control: PID torque mode ──────────────────────────────────────────────

    def set_torque_positions(self, positions: list[int]) -> None:
        raise NotImplementedError(
            "HandESP32 does not support PID-based torque control. "
            "Use set_positions() for position control instead."
        )

    # ── Per-servo configuration ───────────────────────────────────────────────

    def set_speed(self, channel: int, speed: int) -> None:
        """Set maximum move speed for one actuator (0–32766)."""
        speed = max(0, min(32766, speed))
        with self._serial_lock:
            try:
                payload = [0] * 7
                payload[0] = channel & 0xFFFF
                payload[1] = speed & 0xFFFF
                self._send_data(0x31, payload)  # SET_SPE
                self._wait_for_ack(0x31, 2.0)
            except SerialTimeoutException:
                pass

    def set_torque_limit(self, channel: int, torque: int) -> None:
        """Set torque limit for one actuator (0–1000)."""
        torque = max(0, min(1000, torque))
        with self._serial_lock:
            try:
                payload = [0] * 7
                payload[0] = channel & 0xFFFF
                payload[1] = torque & 0xFFFF
                self._send_data(0x32, payload)  # SET_TOR
                self._wait_for_ack(0x32, 2.0)
            except SerialTimeoutException:
                pass

    # ── PID configuration (stub) ──────────────────────────────────────────────

    @property
    def pid_config(self) -> _NullPIDConfig:
        return self._pid_config_obj

    def update_pid(self, channel: int, kp: float, ki: float, kd: float) -> None:
        pass  # ESP32 has no software PID layer

    # ── Homing ────────────────────────────────────────────────────────────────

    @property
    def is_homing(self) -> bool:
        return self._homing

    def home(self) -> None:
        """Run the automatic ESP32 homing sequence. Blocks until complete (~175 s)."""
        self._homing = True
        try:
            with self._serial_lock:
                self.ser.reset_input_buffer()
                self._send_data(HOMING_MODE)
                payload = self._wait_for_ack(HOMING_MODE, 175.0)
            if payload is None:
                raise TimeoutError("Homing ACK not received within 175 s")
            if not all(b == 0 for b in payload):
                raise ValueError(f"Unexpected homing payload: {payload.hex()}")
        finally:
            self._homing = False

    def home_manual(self, step_event: threading.Event,
                    on_prompt: Callable[[str], None]) -> None:
        raise NotImplementedError(
            "HandESP32 does not support manual homing. "
            "Use home() for the automatic homing sequence instead."
        )

    # ── Grasp ─────────────────────────────────────────────────────────────────

    def grasp(self, speed_dps: float = 15.0, torque_limit: int = 500) -> None:
        """Move all actuators to their fully-grasped positions.

        Note: speed_dps and torque_limit are ignored by the ESP32 protocol.
        """
        self.set_positions([_UINT16_MAX] * NUM_SERVOS)

    # ── Sensor data access ────────────────────────────────────────────────────

    def get_positions(self) -> list[int]:
        with self._metrics_lock:
            return list(self._positions)

    def get_velocities(self) -> list[int]:
        """Returns velocities in RPM (not raw Feetech units)."""
        with self._metrics_lock:
            return list(self._velocities)

    def get_currents(self) -> list[int]:
        """Returns currents in mA (not raw Feetech units)."""
        with self._metrics_lock:
            return list(self._currents)

    def get_temperatures(self) -> list[int]:
        with self._metrics_lock:
            return list(self._temperatures)

    # ── Trimming ──────────────────────────────────────────────────────────────

    def trim(self, channel: int, degrees: float) -> int:
        """Adjust the extend calibration for one actuator.

        Returns the new extend count as reported by the ESP32 firmware.
        """
        if not (0 <= channel <= 6):
            raise ValueError(f"Invalid channel: {channel}")
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                payload = [0] * 7
                payload[0] = channel & 0xFFFF
                payload[1] = int(degrees) & 0xFFFF
                self._send_data(TRIM_MODE, payload)
                resp = self._wait_for_ack(TRIM_MODE, 2.0)
            except SerialTimeoutException:
                return 0
        if resp is None:
            return 0
        _, extend = struct.unpack_from("<HH", resp, 0)
        # Update shim so TrimDialog can display current value
        self._sd[channel].extend_count = extend
        return extend

    # ── Servo ID ──────────────────────────────────────────────────────────────

    def set_servo_id(self, new_id: int, current_limit: int = 1023) -> bool:
        """Change the ID of the single servo connected to the bus.

        The ESP32 firmware handles bus scanning internally, same as the TTL
        version. Only one servo should be connected when calling this.
        Returns True on success.
        """
        if not (0 <= new_id <= 253 and new_id != 0xFE):
            return False
        current_limit = max(0, min(1023, current_limit))
        with self._serial_lock:
            try:
                self.ser.reset_input_buffer()
                payload = [0] * 7
                payload[0] = new_id & 0xFF
                payload[1] = current_limit & 0x03FF
                self._send_data(SET_ID_MODE, payload)
                resp = self._wait_for_ack(SET_ID_MODE, 5.0)
            except SerialTimeoutException:
                return False
        return resp is not None

    # ── Ping ──────────────────────────────────────────────────────────────────

    def ping_all(self) -> dict[int, bool]:
        """Always returns no-response: the ESP32 SDK has no ping mechanism."""
        return {i: False for i in range(NUM_SERVOS)}
