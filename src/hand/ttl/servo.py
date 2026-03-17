from .servo_protocol import ServoProtocol, _to_sign_mag16, _from_sign_mag16

# Register addresses (from HLSCL.h)
# EPROM (read/write)
REG_ID = 5
REG_BAUD_RATE = 6
REG_MIN_ANGLE_LIMIT = 9
REG_MAX_ANGLE_LIMIT = 11
REG_CW_DEAD = 26
REG_CCW_DEAD = 27
REG_OFS = 31
REG_MODE = 33

# SRAM (read/write)
REG_TORQUE_ENABLE = 40
REG_ACC = 41
REG_GOAL_POSITION = 42
REG_GOAL_TORQUE = 44
REG_GOAL_SPEED = 46
REG_TORQUE_LIMIT = 48
REG_LOCK = 55

# SRAM (read-only) — feedback registers
REG_PRESENT_POSITION = 56
REG_PRESENT_SPEED = 58
REG_PRESENT_LOAD = 60
REG_PRESENT_VOLTAGE = 62
REG_PRESENT_TEMPERATURE = 63
REG_MOVING = 66
REG_PRESENT_CURRENT = 69

# Sensor block for sync read
SENSOR_BLOCK_START = REG_PRESENT_POSITION  # 56
SENSOR_BLOCK_LEN = REG_PRESENT_CURRENT + 2 - REG_PRESENT_POSITION  # 15 bytes

REG_CURRENT_LIMIT = 28


class Servo:
    """High-level servo control API.

    Mirrors the HLSCL class from the Arduino library, providing named
    operations on top of the raw ServoProtocol.
    """

    def __init__(self, protocol: ServoProtocol):
        self._protocol = protocol

    @property
    def protocol(self) -> ServoProtocol:
        return self._protocol

    # ---- Control mode switching ----

    def servo_mode(self, servo_id: int) -> bool:
        return self._protocol.write_byte(servo_id, REG_MODE, 0)

    def wheel_mode(self, servo_id: int) -> bool:
        return self._protocol.write_byte(servo_id, REG_MODE, 1)

    def ele_mode(self, servo_id: int) -> bool:
        return self._protocol.write_byte(servo_id, REG_MODE, 2)

    # ---- Position control ----

    def write_pos_ex(self, servo_id: int, position: int, speed: int,
                     acc: int = 0, torque: int = 0) -> bool:
        """Write position with speed, acceleration, and torque limit.

        Data layout at REG_ACC (register 41):
        [ACC(1), PosL, PosH, TorqueL, TorqueH, SpeedL, SpeedH]
        """
        pos_enc = _to_sign_mag16(position)
        data = bytearray(7)
        data[0] = acc & 0xFF
        data[1] = pos_enc & 0xFF
        data[2] = (pos_enc >> 8) & 0xFF
        data[3] = torque & 0xFF
        data[4] = (torque >> 8) & 0xFF
        data[5] = speed & 0xFF
        data[6] = (speed >> 8) & 0xFF
        return self._protocol.gen_write(servo_id, REG_ACC, bytes(data))

    def sync_write_pos_ex(self, servo_ids: list[int],
                          positions: list[int], speeds: list[int],
                          accs: list[int], torques: list[int]) -> None:
        """Sync write positions for multiple servos."""
        data_per_servo = []
        for i in range(len(servo_ids)):
            pos_enc = _to_sign_mag16(positions[i])
            d = bytearray(7)
            d[0] = (accs[i] if accs else 0) & 0xFF
            d[1] = pos_enc & 0xFF
            d[2] = (pos_enc >> 8) & 0xFF
            d[3] = torques[i] & 0xFF
            d[4] = (torques[i] >> 8) & 0xFF
            d[5] = speeds[i] & 0xFF
            d[6] = (speeds[i] >> 8) & 0xFF
            data_per_servo.append(bytes(d))
        self._protocol.sync_write(servo_ids, REG_ACC, data_per_servo)

    # ---- Torque (ele) control ----

    def write_ele(self, servo_id: int, torque: int) -> bool:
        """Write torque command in sign-magnitude format."""
        encoded = _to_sign_mag16(torque)
        return self._protocol.write_word(servo_id, REG_GOAL_TORQUE, encoded)

    # ---- Torque enable ----

    def enable_torque(self, servo_id: int, enable: bool) -> bool:
        return self._protocol.write_byte(
            servo_id, REG_TORQUE_ENABLE, 1 if enable else 0)

    # ---- EPROM lock/unlock ----

    def unlock_eprom(self, servo_id: int) -> bool:
        self.enable_torque(servo_id, False)
        return self._protocol.write_byte(servo_id, REG_LOCK, 0)

    def lock_eprom(self, servo_id: int) -> bool:
        return self._protocol.write_byte(servo_id, REG_LOCK, 1)

    # ---- Calibration ----

    def calibration_ofs(self, servo_id: int) -> bool:
        self.enable_torque(servo_id, False)
        self.unlock_eprom(servo_id)
        return self._protocol.recal(servo_id)

    # ---- Angle limits ----

    def set_angle_limits(self, servo_id: int,
                         min_limit: int, max_limit: int) -> None:
        min_limit = max(0, min(4095, min_limit))
        max_limit = max(0, min(4095, max_limit))
        self.unlock_eprom(servo_id)
        self._protocol.write_word(servo_id, REG_MIN_ANGLE_LIMIT, min_limit)
        self._protocol.write_word(servo_id, REG_MAX_ANGLE_LIMIT, max_limit)
        self.lock_eprom(servo_id)
        self.enable_torque(servo_id, True)

    # ---- Individual sensor reads ----

    def read_pos(self, servo_id: int) -> int | None:
        raw = self._protocol.read_word(servo_id, REG_PRESENT_POSITION)
        if raw is None:
            return None
        return _from_sign_mag16(raw)

    def read_speed(self, servo_id: int) -> int | None:
        raw = self._protocol.read_word(servo_id, REG_PRESENT_SPEED)
        if raw is None:
            return None
        return _from_sign_mag16(raw)

    def read_load(self, servo_id: int) -> int | None:
        raw = self._protocol.read_word(servo_id, REG_PRESENT_LOAD)
        if raw is None:
            return None
        # Load uses bit 10 as sign bit
        if raw & (1 << 10):
            return -(raw & ~(1 << 10))
        return raw

    def read_voltage(self, servo_id: int) -> int | None:
        return self._protocol.read_byte(servo_id, REG_PRESENT_VOLTAGE)

    def read_temperature(self, servo_id: int) -> int | None:
        return self._protocol.read_byte(servo_id, REG_PRESENT_TEMPERATURE)

    def read_moving(self, servo_id: int) -> int | None:
        return self._protocol.read_byte(servo_id, REG_MOVING)

    def read_current(self, servo_id: int) -> int | None:
        raw = self._protocol.read_word(servo_id, REG_PRESENT_CURRENT)
        if raw is None:
            return None
        return _from_sign_mag16(raw)

    # ---- Bulk sensor read ----

    def feedback(self, servo_id: int) -> dict | None:
        """Read all feedback registers (15 bytes) for a single servo."""
        data = self._protocol.read_block(
            servo_id, SENSOR_BLOCK_START, SENSOR_BLOCK_LEN)
        if data is None or len(data) < SENSOR_BLOCK_LEN:
            return None
        return self._parse_sensor_block(data)

    def sync_read_sensors(self, servo_ids: list[int]
                          ) -> dict[int, dict]:
        """Sync read 15-byte sensor blocks from all servos.

        Returns {servo_id: {pos, speed, load, voltage, temp, moving, current}}
        """
        raw = self._protocol.sync_read(
            servo_ids, SENSOR_BLOCK_START, SENSOR_BLOCK_LEN)
        result = {}
        for sid, data in raw.items():
            if len(data) >= SENSOR_BLOCK_LEN:
                result[sid] = self._parse_sensor_block(data)
        return result

    @staticmethod
    def _parse_sensor_block(data: bytes) -> dict:
        """Parse a 15-byte sensor block into named values.

        Byte layout (offsets from register 56):
          0-1: Position (16-bit, sign-magnitude)
          2-3: Speed (16-bit, sign-magnitude)
          4-5: Load (16-bit, bit 10 sign)
          6:   Voltage (8-bit)
          7:   Temperature (8-bit)
          10:  Moving (8-bit)
          13-14: Current (16-bit, sign-magnitude)
        """
        pos_raw = data[0] | (data[1] << 8)
        spd_raw = data[2] | (data[3] << 8)
        load_raw = data[4] | (data[5] << 8)
        voltage = data[6]
        temperature = data[7]
        moving = data[10] if len(data) > 10 else 0
        cur_raw = data[13] | (data[14] << 8)

        return {
            "position": _from_sign_mag16(pos_raw),
            "speed": _from_sign_mag16(spd_raw),
            "load": -(load_raw & ~(1 << 10)) if load_raw & (1 << 10) else load_raw,
            "voltage": voltage,
            "temperature": temperature,
            "moving": moving,
            "current": _from_sign_mag16(cur_raw),
        }
