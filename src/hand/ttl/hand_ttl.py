import json
import os
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum

from ..hand import Hand
from .servo_protocol import ServoProtocol
from .servo import Servo
from .pid import PIDController, PIDConfig


# ---- Servo data (from Homing.cpp) ----

@dataclass
class ServoData:
    grasp_count: int
    extend_count: int
    servo_direction: int  # +1 or -1


SD_BASE_LEFT = [
    ServoData(3186, 2048, 1), #thumb abd
    ServoData(2048, 865, 1), #thumb tend
    ServoData(0, 2980, 1), # thumb flex
    ServoData(4095, 817, 1), # index flex
    ServoData(4095, 817, 1), # middle flex
    ServoData(4095, 817, -1),
    ServoData(4095, 817, -1),
]

SD_BASE_RIGHT = [
    ServoData(910, 2048, -1),
    ServoData(2048, 3231, 1),
    ServoData(4095, 1115, -1),
    ServoData(0, 3278, 1),
    ServoData(0, 3278, 1),
    ServoData(0, 3278, 1),
    ServoData(0, 3278, 1),
]

SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]
NUM_SERVOS = 7

# ---- Constants (from firmware.ino) ----

HOLD_MAG = 5
TEMP_CUTOFF_C = 70
HOT_TORQUE_LIMIT = 500
COUNTS_PER_DEGREE = 11.375
SOFT_LIMIT_TORQUE = 200

HOMING_CURRENT_LIMIT = 500
HOMING_SPEED = 500

# Feetech sensor unit conversions (from Feetech documentation, also used in aero_hand.py)
CURRENT_MA_PER_UNIT = 6.5    # 1 raw unit = 6.5 mA
SPEED_RPM_PER_UNIT  = 0.732  # 1 raw unit = 0.732 RPM

class ControlMode(Enum):
    POSITION = 0
    TORQUE = 2


class HandTTL(Hand):
    """TTL serial hand controller (Feetech HLS3606M servos, 1 Mbps).

    Manages servo communication, control modes, homing, sensor reading,
    trimming, and PID-based torque control.
    """

    def __init__(self, port: "str | object", hand_type: str = "left",
                 config_dir: str = ""):
        """
        Args:
            port: Serial port (e.g. "COM3" or "/dev/ttyUSB0"), or a
                  serial-compatible object (e.g. HandSimulator).
            hand_type: "left" or "right".
            config_dir: Directory containing hand_config.json and pid_config.json.
        """
        self._config_dir = config_dir

        # Serial + servo API
        self._protocol = ServoProtocol(port)
        self._servo = Servo(self._protocol)

        # Hand configuration
        self._hand_type = hand_type.lower()
        if self._hand_type == "right":
            baseline = SD_BASE_RIGHT
        else:
            baseline = SD_BASE_LEFT
        self._sd = [ServoData(s.grasp_count, s.extend_count, s.servo_direction)
                     for s in baseline]

        # Load persisted extend_count values
        self._extends_path = os.path.join(config_dir, "hand_config.json")
        self._load_extends()

        # DEBUG: print effective SD values after config load
        # for i in range(NUM_SERVOS):
        #     print(f"  [DEBUG] servo {i}: extend={self._sd[i].extend_count}, "
        #           f"grasp={self._sd[i].grasp_count}, dir={self._sd[i].servo_direction}")

        # Control state
        self._current_mode = ControlMode.POSITION
        self._speed = [32766] * NUM_SERVOS
        self._accel = [0] * NUM_SERVOS
        self._torque_limit = [700] * NUM_SERVOS
        self._last_torque_cmd = [0] * NUM_SERVOS
        self._torque_limited = [False] * NUM_SERVOS

        # PID controllers
        pid_config_path = os.path.join(config_dir, "pid_config.json")
        self._pid_config = PIDConfig(pid_config_path)
        self._pid_controllers = self._pid_config.create_controllers()
        self._last_pid_time = time.monotonic()

        # Sensor data (thread-safe)
        self._metrics_lock = threading.Lock()
        self._positions = [0] * NUM_SERVOS
        self._velocities = [0] * NUM_SERVOS
        self._currents = [0] * NUM_SERVOS
        self._temperatures = [0] * NUM_SERVOS

        # Homing state
        self._homing = False

        # Sensor reading thread
        self._running = True
        self._sensor_thread = threading.Thread(
            target=self._sensor_read_loop, daemon=True)
        self._sensor_thread.start()

    def close(self):
        self._running = False
        if self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=1.0)
        self._protocol.close()

    # ---- Persistence ----

    def _load_extends(self):
        if os.path.exists(self._extends_path):
            with open(self._extends_path, "r") as f:
                data = json.load(f)
            extends = data.get("extend_counts", [])
            for i in range(min(len(extends), NUM_SERVOS)):
                v = extends[i]
                if 0 <= v <= 4095:
                    self._sd[i].extend_count = v
            grasps = data.get("grasp_counts", [])
            for i in range(min(len(grasps), NUM_SERVOS)):
                v = grasps[i]
                if 0 <= v <= 4095:
                    self._sd[i].grasp_count = v

    def _save_extends(self):
        data = {
            "hand_type": self._hand_type,
            "extend_counts": [self._sd[i].extend_count for i in range(NUM_SERVOS)],
            "grasp_counts":  [self._sd[i].grasp_count  for i in range(NUM_SERVOS)],
        }
        with open(self._extends_path, "w") as f:
            json.dump(data, f, indent=2)

    # ---- Position mapping (from firmware.ino) ----

    def _map_raw_to_u16(self, channel: int, raw: int) -> int:
        ext = self._sd[channel].extend_count
        gra = self._sd[channel].grasp_count
        span = gra - ext
        if span == 0:
            # DEBUG
            # if channel == 4:
            #     print(f"  [DEBUG] _map_raw_to_u16(ch=4, raw={raw}): SPAN=0 → returning 0")
            return 0
        val = ((raw - ext) * 65535) // span
        return max(0, min(65535, val))

    def _map_u16_to_raw(self, channel: int, u16: int) -> int:
        ext = self._sd[channel].extend_count
        gra = self._sd[channel].grasp_count
        if ext == 0 and gra == 0:
            raw = (u16 * 4095) // 65535
        else:
            raw = ext + (u16 * (gra - ext)) // 65535
        raw = max(0, min(4095, raw))
        # DEBUG: trace middle finger mapping
        # if channel == 4:
        #     print(f"  [DEBUG] _map_u16_to_raw(ch=4, u16={u16}): ext={ext}, gra={gra}, raw={raw}")
        return raw

    # ---- Thermal protection ----

    def _get_temp(self, channel: int) -> int:
        with self._metrics_lock:
            return self._temperatures[channel]

    def _is_hot(self, channel: int) -> bool:
        return self._get_temp(channel) >= TEMP_CUTOFF_C

    # ---- Control: Position mode ----

    def set_positions(self, positions: list[int]) -> None:
        """Set positions for all 7 servos.

        Args:
            positions: List of 7 values, each 0-65535 (0=extend, 65535=grasp).
        """
        if self._homing:
            return

        raw_positions = [self._map_u16_to_raw(i, positions[i])
                         for i in range(NUM_SERVOS)]

        torques_eff = []
        for i in range(NUM_SERVOS):
            t = self._torque_limit[i]
            if self._is_hot(i):
                t = min(t, HOT_TORQUE_LIMIT)
            torques_eff.append(t)

        with self._protocol.lock:
            if self._current_mode != ControlMode.POSITION:
                for sid in SERVO_IDS:
                    self._servo.servo_mode(sid)
                self._current_mode = ControlMode.POSITION

            self._servo.sync_write_pos_ex(
                SERVO_IDS, raw_positions, self._speed,
                self._accel, torques_eff)

    # ---- Control: Raw torque mode ----

    def set_torques(self, torques: list[int]) -> None:
        """Set raw torque for all 7 servos.

        Args:
            torques: List of 7 signed values (-1000 to +1000). Positive = grasp, negative = extend, 0 = no torque.
        """
        if self._homing:
            return

        torque_cmds = []
        for i in range(NUM_SERVOS):
            signed = max(-1000, min(1000, torques[i]))
            if signed != 0:
                mag = max(HOLD_MAG, min(1000, abs(signed)))
                if self._is_hot(i):
                    mag = min(mag, HOT_TORQUE_LIMIT)
                signed = mag if signed > 0 else -mag
            grasp_sign = 1 if self._sd[i].extend_count > self._sd[i].grasp_count else -1
            torque_cmds.append(grasp_sign * signed)

        for i in range(NUM_SERVOS):
            self._last_torque_cmd[i] = torque_cmds[i]

        with self._protocol.lock:
            if self._current_mode != ControlMode.TORQUE:
                for sid in SERVO_IDS:
                    self._servo.ele_mode(sid)
                self._current_mode = ControlMode.TORQUE

            for i in range(NUM_SERVOS):
                self._servo.write_ele(SERVO_IDS[i], torque_cmds[i])

    # ---- Control: PID-based torque mode ----

    def set_torque_positions(self, positions: list[int]) -> None:
        """PID-based torque control targeting positions.

        Args:
            positions: List of 7 target positions (0-65535 normalized).
        """
        now = time.monotonic()
        dt = now - self._last_pid_time
        self._last_pid_time = now
        if dt <= 0:
            dt = 0.01

        current_positions = self.get_positions()
        torques = []
        for i in range(NUM_SERVOS):
            output = self._pid_controllers[i].update(
                positions[i], current_positions[i], dt)
            torques.append(int(output))

        self.set_torques(torques)

    # ---- Per-servo configuration ----

    def set_speed(self, channel: int, speed: int) -> None:
        self._speed[channel] = max(0, min(32766, speed))

    def set_torque_limit(self, channel: int, torque: int) -> None:
        self._torque_limit[channel] = max(0, min(1023, torque))

    # ---- PID configuration ----

    @property
    def pid_config(self) -> PIDConfig:
        return self._pid_config

    def update_pid(self, channel: int, kp: float, ki: float, kd: float):
        self._pid_config.set(channel, kp, ki, kd)
        self._pid_controllers[channel] = PIDController(kp=kp, ki=ki, kd=kd)

    # ---- Sensor reading thread ----

    def _sensor_read_loop(self):
        while self._running:
            if self._homing:
                time.sleep(0.01)
                continue

            # Try to acquire bus lock non-blocking
            acquired = self._protocol.lock.acquire(blocking=False)
            if not acquired:
                time.sleep(0.01)
                continue

            try:
                # Use sync_read without acquiring lock again (we hold it)
                # Call the lower-level methods directly
                self._protocol._flush_input()

                # Build and send sync read packet
                n_servos = NUM_SERVOS
                mem_addr = 56  # SENSOR_BLOCK_START
                length = 15   # SENSOR_BLOCK_LEN
                timeout = 0.008 + self._protocol._timeout * n_servos

                pkt_len = n_servos + 4
                checksum_sum = (0xFE + pkt_len + 0x82 + mem_addr + length)
                for sid in SERVO_IDS:
                    checksum_sum += sid
                checksum = (~checksum_sum) & 0xFF

                pkt = bytes([0xFF, 0xFF, 0xFE, pkt_len, 0x82,
                             mem_addr, length])
                pkt += bytes(SERVO_IDS) + bytes([checksum])
                self._protocol._serial.write(pkt)

                # Read response
                max_resp_len = n_servos * (length + 6)
                old_timeout = self._protocol._serial.timeout
                self._protocol._serial.timeout = timeout
                buf = self._protocol._serial.read(max_resp_len)
                self._protocol._serial.timeout = old_timeout

                # Parse responses
                # Read current values so unresponsive servos keep their last reading
                with self._metrics_lock:
                    pos = list(self._positions)
                    vel = list(self._velocities)
                    tmp = list(self._temperatures)
                    cur = list(self._currents)

                idx = 0
                for i, target_id in enumerate(SERVO_IDS):
                    found = False
                    while idx + 5 + length <= len(buf):
                        if (buf[idx] == 0xFF and buf[idx + 1] == 0xFF
                                and buf[idx + 2] != 0xFF):
                            if buf[idx + 2] == target_id:
                                resp_len = buf[idx + 3]
                                if resp_len == length + 2:
                                    data_start = idx + 5
                                    data_end = data_start + length
                                    if data_end + 1 <= len(buf):
                                        data = buf[data_start:data_end]
                                        ck = (target_id + resp_len +
                                              buf[idx + 4] + sum(data))
                                        if buf[data_end] == (~ck & 0xFF):
                                            raw_pos = data[0] | (data[1] << 8)
                                            pos[i] = self._map_raw_to_u16(i, raw_pos)
                                            # Speed: sign-magnitude, convert to RPM
                                            spd_raw = data[2] | (data[3] << 8)
                                            if spd_raw & (1 << 15):
                                                vel[i] = int(-(spd_raw & ~(1 << 15)) * SPEED_RPM_PER_UNIT)
                                            else:
                                                vel[i] = int(spd_raw * SPEED_RPM_PER_UNIT)
                                            tmp[i] = data[7]
                                            # Current: sign-magnitude, convert to mA
                                            cur_raw = data[13] | (data[14] << 8)
                                            if cur_raw & (1 << 15):
                                                cur[i] = int(-(cur_raw & ~(1 << 15)) * CURRENT_MA_PER_UNIT)
                                            else:
                                                cur[i] = int(cur_raw * CURRENT_MA_PER_UNIT)
                                            idx = data_end + 1
                                            found = True
                                            break
                            idx += 1
                        else:
                            idx += 1
                    if not found:
                        idx = 0  # reset search position for next servo

                with self._metrics_lock:
                    self._positions = pos
                    self._velocities = vel
                    self._temperatures = tmp
                    self._currents = cur
            finally:
                self._protocol.lock.release()

            # Soft limit enforcement
            self._check_soft_limits()

            time.sleep(0.01)

    # ---- Soft limits (from firmware.ino) ----

    def _check_soft_limits(self):
        if self._current_mode != ControlMode.TORQUE:
            for i in range(NUM_SERVOS):
                self._torque_limited[i] = False
            return

        with self._metrics_lock:
            positions = list(self._positions)

        for i in range(NUM_SERVOS):
            # Convert normalized position back to raw for bounds checking
            ext = self._sd[i].extend_count
            gra = self._sd[i].grasp_count
            raw_min = min(ext, gra)
            raw_max = max(ext, gra)

            # Get approximate raw position from normalized
            span = gra - ext
            if span == 0:
                continue
            raw = ext + (positions[i] * span) // 65535
            in_range = raw_min <= raw <= raw_max

            if not in_range and not self._torque_limited[i]:
                limited = SOFT_LIMIT_TORQUE if self._last_torque_cmd[i] >= 0 else -SOFT_LIMIT_TORQUE
                with self._protocol.lock:
                    self._servo.write_ele(SERVO_IDS[i], limited)
                self._torque_limited[i] = True
            elif in_range and self._torque_limited[i]:
                with self._protocol.lock:
                    self._servo.write_ele(SERVO_IDS[i], self._last_torque_cmd[i])
                self._torque_limited[i] = False

    # ---- Sensor data access ----

    def get_positions(self) -> list[int]:
        with self._metrics_lock:
            return list(self._positions)

    def get_velocities(self) -> list[int]:
        with self._metrics_lock:
            return list(self._velocities)

    def get_currents(self) -> list[int]:
        with self._metrics_lock:
            return list(self._currents)

    def get_temperatures(self) -> list[int]:
        with self._metrics_lock:
            return list(self._temperatures)

    # ---- Homing (from Homing.cpp) ----

    @property
    def is_homing(self) -> bool:
        return self._homing

    def home(self) -> None:
        """Full homing sequence. Blocks until complete."""
        self._homing = True
        try:
            self._reset_to_baseline()
            with self._protocol.lock:
                self._zero_with_current(0, self._sd[0].servo_direction, HOMING_CURRENT_LIMIT)
                self._zero_with_current(1, self._sd[1].servo_direction, HOMING_CURRENT_LIMIT)
                self._zero_with_current(2, self._sd[2].servo_direction, HOMING_CURRENT_LIMIT)
                self._zero_fingers_parallel(3, 4, HOMING_CURRENT_LIMIT)

                # Post-homing: move all to extend
                for i in range(NUM_SERVOS):
                    self._servo.write_pos_ex(
                        SERVO_IDS[i], self._sd[i].extend_count, 2400, 0, 1023)

            self._save_extends()
        finally:
            self._homing = False

    def home_manual(self, step_event: threading.Event,
                    on_prompt: Callable[[str], None]) -> None:
        """Interactive manual homing sequence.  Blocks until all 7 servos are done.

        The caller advances through each step by setting step_event.
        on_prompt(msg) is called from this thread to send status strings to the UI.

        Sequence for each servo:
          1. Read current raw position -> extend_count (user placed servo at extend).
          2. Wait for step_event (user presses Space).
          3. Ramp torque in grasp direction at 100 units/second until step_event.
          4. Record current raw position as grasp_count.
          5. Return servo to position mode at grasp position.
          6. Servo 2 only: thumb-tendon nudge (same movement as auto homing, no
             calibration_ofs so the coordinate system stays consistent).
        After all servos: move all to extend, save calibration.
        """
        self._homing = True
        try:
            # Use hardcoded baseline to determine grasp direction — immune to
            # corrupted persisted config or Step 0 overwriting extend_count.
            baseline = SD_BASE_RIGHT if self._hand_type == "right" else SD_BASE_LEFT
            baseline_grasp_signs = [
                1 if baseline[i].extend_count > baseline[i].grasp_count else -1
                for i in range(NUM_SERVOS)
            ]

            # Step 0: read all extend positions while servos are still at rest
            with self._protocol.lock:
                for i in range(NUM_SERVOS):
                    pos = self._servo.read_pos(SERVO_IDS[i])
                    if pos is not None:
                        self._sd[i].extend_count = max(0, min(4095, pos))

            for i in range(NUM_SERVOS):
                sid = SERVO_IDS[i]

                # -- Phase 1: prompt, wait for first Space --
                on_prompt(
                    f"[{i+1}/7] Servo {i}: extend={self._sd[i].extend_count}. "
                    f"Press Space to begin torque ramp.")
                step_event.clear()
                step_event.wait()

                # -- Phase 2: ramp torque until second Space --
                on_prompt(
                    f"[{i+1}/7] Servo {i}: ramping torque. "
                    f"Press Space to record grasp position.")
                step_event.clear()

                grasp_sign = baseline_grasp_signs[i]

                with self._protocol.lock:
                    self._servo.ele_mode(sid)

                torque = 0
                t_last = time.monotonic()
                while not step_event.is_set():
                    t_now = time.monotonic()
                    torque = min(1000, torque + int(100 * (t_now - t_last)))
                    t_last = t_now
                    torque_cmd = grasp_sign * torque
                    with self._protocol.lock:
                        self._servo.write_ele(sid, torque_cmd)
                    time.sleep(0.05)

                # -- Record grasp position, stop torque --
                with self._protocol.lock:
                    grasp_pos = self._servo.read_pos(sid)
                    if grasp_pos is not None:
                        self._sd[i].grasp_count = max(0, min(4095, grasp_pos))
                    else:
                        grasp_pos = self._sd[i].grasp_count

                    self._servo.servo_mode(sid)
                    self._servo.write_pos_ex(sid, self._sd[i].grasp_count,
                                             300, 0, 500)

                    # Thumb tendon nudge (servo 2) – mechanical only, no calibration_ofs
                    if sid == 2:
                        direction = self._sd[i].servo_direction
                        self._servo.write_pos_ex(
                            sid, self._sd[i].grasp_count + direction * 2048,
                            2400, 0, 1000)
                        time.sleep(0.25)
                        self._servo.write_pos_ex(
                            sid,
                            self._sd[i].extend_count - direction * 625,
                            2400, 0, 1000)
                        time.sleep(0.25)

            # Move all to extend
            on_prompt("All servos homed. Moving to extend…")
            with self._protocol.lock:
                for i in range(NUM_SERVOS):
                    self._servo.servo_mode(SERVO_IDS[i])
                    self._servo.write_pos_ex(
                        SERVO_IDS[i], self._sd[i].extend_count, 2400, 0, 1023)

            self._save_extends()
            on_prompt("Manual homing complete.")
        finally:
            self._homing = False

    def grasp(self, speed_dps: float = 15.0,
              torque_limit: int = HOMING_CURRENT_LIMIT) -> None:
        """Move all servos to their grasp positions.

        Args:
            speed_dps: Closing speed in degrees per second.
            torque_limit: Maximum torque applied during movement (0–1023).
        """
        if self._homing:
            return
        speed = max(1, int(speed_dps * COUNTS_PER_DEGREE))
        torque_limit = max(0, min(1023, torque_limit))
        with self._protocol.lock:
            if self._current_mode != ControlMode.POSITION:
                for sid in SERVO_IDS:
                    self._servo.servo_mode(sid)
                self._current_mode = ControlMode.POSITION
            self._servo.sync_write_pos_ex(
                SERVO_IDS,
                [self._sd[i].grasp_count for i in range(NUM_SERVOS)],
                [speed] * NUM_SERVOS,
                [0] * NUM_SERVOS,
                [torque_limit] * NUM_SERVOS)

    def _reset_to_baseline(self):
        if self._hand_type == "right":
            baseline = SD_BASE_RIGHT
        else:
            baseline = SD_BASE_LEFT
        for i in range(NUM_SERVOS):
            self._sd[i] = ServoData(
                baseline[i].grasp_count,
                baseline[i].extend_count,
                baseline[i].servo_direction)

    def _zero_with_current(self, index: int, direction: int,
                           current_limit: int):
        """Home a single servo using current feedback (thumb servos).

        Must be called with protocol lock held.
        """
        servo_id = SERVO_IDS[index]

        # Open travel window fully
        self._servo.set_angle_limits(servo_id, 0, 0)
        self._servo.servo_mode(servo_id)
        self._servo.feedback(servo_id)

        current = 0
        position = 0
        t0 = time.monotonic()

        while abs(current) < current_limit:
            self._servo.write_pos_ex(servo_id, 50000 * direction, HOMING_SPEED, 0,
                                     current_limit)
            cur = self._servo.read_current(servo_id)
            pos = self._servo.read_pos(servo_id)
            if cur is not None:
                current = cur
            if pos is not None:
                position = pos
            if time.monotonic() - t0 > 10.0:
                break
            time.sleep(0.001)

        # Primary calibration at contact
        self._servo.write_pos_ex(servo_id, position, 2400, 0, 1000)
        time.sleep(0.03)
        self._servo.calibration_ofs(servo_id)
        time.sleep(0.03)
        pos = self._servo.read_pos(servo_id)
        if pos is not None:
            position = pos

        if servo_id == 0:
            # Thumb abduction: hold grasp posture
            self._servo.write_pos_ex(
                servo_id, self._sd[index].grasp_count, 2400, 0, 1000)
            time.sleep(0.25)
        elif servo_id == 1:
            # Thumb flexion: go to extend
            self._servo.write_pos_ex(
                servo_id, self._sd[index].extend_count, 2400, 0, 1000)
            time.sleep(0.25)
        elif servo_id == 2:
            # Thumb tendon: nudge and recalibrate, then extend
            self._servo.write_pos_ex(
                servo_id, position + (direction * 2048), 2400, 0, 1000)
            time.sleep(0.25)
            self._servo.calibration_ofs(servo_id)
            time.sleep(0.03)
            self._servo.write_pos_ex(
                servo_id,
                self._sd[index].extend_count - (direction * 625),
                2400, 0, 1000)
            time.sleep(0.03)

    def _zero_fingers_parallel(self, first_idx: int, count: int,
                               current_limit: int):
        """Home finger servos (channels 3-6) in parallel.

        Must be called with protocol lock held.
        """
        if count == 0 or first_idx > 6:
            return
        if first_idx + count > 7:
            count = 7 - first_idx

        done = [False] * NUM_SERVOS
        contact_pos = [0] * NUM_SERVOS
        cur = [0] * NUM_SERVOS
        pos = [0] * NUM_SERVOS

        for i in range(count):
            idx = first_idx + i
            servo_id = SERVO_IDS[idx]
            self._servo.set_angle_limits(servo_id, 0, 0)
            self._servo.servo_mode(servo_id)
            self._servo.feedback(servo_id)

        t0 = time.monotonic()
        while True:
            all_done = all(done[first_idx + i] for i in range(count))
            if all_done:
                break
            if time.monotonic() - t0 > 10.0:
                break

            for i in range(count):
                idx = first_idx + i
                if done[idx]:
                    continue
                servo_id = SERVO_IDS[idx]
                direction = self._sd[idx].servo_direction
                self._servo.write_pos_ex(
                    servo_id, 50000 * direction, HOMING_SPEED, 0, current_limit)

            for i in range(count):
                idx = first_idx + i
                if done[idx]:
                    continue
                servo_id = SERVO_IDS[idx]
                c = self._servo.read_current(servo_id)
                p = self._servo.read_pos(servo_id)
                if c is not None:
                    cur[idx] = c
                if p is not None:
                    pos[idx] = p
                if abs(cur[idx]) >= current_limit:
                    done[idx] = True
                    contact_pos[idx] = pos[idx]
                    self._servo.write_pos_ex(
                        servo_id, contact_pos[idx], 60, 50, 1000)

            time.sleep(0.001)

        # Post-calibration for each finger
        for i in range(count):
            idx = first_idx + i
            servo_id = SERVO_IDS[idx]
            direction = self._sd[idx].servo_direction

            p = contact_pos[idx] if done[idx] else pos[idx]

            self._servo.write_pos_ex(servo_id, p, 2400, 0, 1000)
            time.sleep(0.03)
            self._servo.calibration_ofs(servo_id)
            time.sleep(0.03)
            p_new = self._servo.read_pos(servo_id)
            if p_new is not None:
                p = p_new
            self._servo.write_pos_ex(
                servo_id, p + (direction * 2048), 2400, 0, 1000)
            time.sleep(0.3)
            self._servo.calibration_ofs(servo_id)
            time.sleep(0.03)
            self._servo.write_pos_ex(
                servo_id, self._sd[idx].extend_count, 2400, 0, 1000)

    # ---- Trimming (from firmware.ino handleTrimCmd) ----

    def trim(self, channel: int, degrees: float) -> int:
        """Adjust extend_count by degrees. Returns new extend_count."""
        if channel < 0 or channel >= NUM_SERVOS:
            raise ValueError(f"Invalid channel: {channel}")
        delta_counts = int(degrees * COUNTS_PER_DEGREE)
        new_ext = self._sd[channel].extend_count + delta_counts
        new_ext = max(0, min(4095, new_ext))
        self._sd[channel].extend_count = new_ext
        self._save_extends()
        return new_ext

    # ---- Set servo ID (from firmware.ino handleSetIdCmd) ----

    def set_servo_id(self, new_id: int, current_limit: int = 1023) -> bool:
        """Scan for a single servo on the bus and change its ID.

        Only one servo should be connected when calling this.
        Returns True on success.
        """
        if new_id > 253 or new_id == 0xFE:
            return False

        current_limit = min(current_limit, 1023)

        # Scan for a single servo
        found_id = None
        found_count = 0
        for scan_id in range(254):
            if scan_id == 0xFE:
                continue
            if self._protocol.ping(scan_id):
                if found_count == 0:
                    found_id = scan_id
                found_count += 1
                if found_count > 1:
                    break

        print(f"ssid: found {found_count} servos")
        if found_count != 1:
            return False

        # Check if new_id is already taken (if different from current)
        if new_id != found_id:
            if self._protocol.ping(new_id):
                return False

        print(f"ssid: acquiring lock...")
        with self._protocol.lock:
            print(f"unlock eerprom: {self._servo.unlock_eprom(found_id)}")
            print(f"write word: {self._protocol.write_word(found_id, 28, current_limit)}")  # REG_CURRENT_LIMIT
            target_id = found_id
            if new_id != found_id:
                self._protocol.write_byte(found_id, 5, new_id)  # REG_ID
                target_id = new_id
            self._servo.lock_eprom(target_id)

        return True

    # ---- Ping ----

    def ping_all(self) -> dict[int, bool]:
        """Ping all 7 servos, return {id: responded}."""
        result = {}
        for sid in SERVO_IDS:
            result[sid] = self._protocol.ping(sid)
        return result
