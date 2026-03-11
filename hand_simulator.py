"""hand_simulator.py — Software digital twin for the Aero Open Hand.

Implements a serial.Serial drop-in (HandSimulator) that:
  * Parses Feetech packets written to it and generates correct responses.
  * Maintains a per-servo register map (SimulatedServo).
  * Supports drop rules (DropRule) to suppress responses from specific
    servos / instructions so that bus-flooding bugs can be reproduced.
  * Writes a timestamped log file (sim_YYYYMMDD_HHMMSS.log).

Typical use:

    from hand_simulator import HandSimulator, SERVO_IDS
    from hand import Hand

    sim = HandSimulator(servo_ids=SERVO_IDS)
    sim.add_drop_rule(servo_id=2)          # freeze middle finger
    hand = Hand(sim, hand_type='left')
"""

import collections
import datetime
import os
import sys
import threading
import time

from hand.ttl.servo_protocol import (
    INST_PING, INST_READ, INST_WRITE, INST_REG_WRITE, INST_REG_ACTION,
    INST_SYNC_READ, INST_SYNC_WRITE, INST_RECOVERY, INST_RESET, INST_CAL,
    BROADCAST_ID,
)

# Default servo IDs (matches hand/hand.py)
SERVO_IDS = [0, 1, 2, 3, 4, 5, 6]

# Register addresses (must match hand/servo.py)
_REG_GOAL_POSITION    = 42
_REG_PRESENT_POSITION = 56
_REG_PRESENT_VOLTAGE  = 62
_REG_PRESENT_TEMPERATURE = 63

# Human-readable instruction names for logging / drop-rule API
_INST_NAMES = {
    INST_PING:       "PING",
    INST_READ:       "READ",
    INST_WRITE:      "WRITE",
    INST_REG_WRITE:  "REG_WRITE",
    INST_REG_ACTION: "REG_ACTION",
    INST_SYNC_READ:  "SYNC_READ",
    INST_SYNC_WRITE: "SYNC_WRITE",
    INST_RECOVERY:   "RECOVERY",
    INST_RESET:      "RESET",
    INST_CAL:        "CAL",
}

# Reverse map: "SYNC_READ" → 0x82, etc.
_INST_BY_NAME = {v: k for k, v in _INST_NAMES.items()}

_FLOOD_THRESHOLD_DEFAULT = 200   # packets / second


# ─────────────────────────────────────────────────────────────────────────────
# SimulatedServo
# ─────────────────────────────────────────────────────────────────────────────

class SimulatedServo:
    """128-byte register map for one simulated servo."""

    REG_SIZE = 128

    def __init__(self, servo_id: int):
        self.servo_id = servo_id
        self._regs = bytearray(self.REG_SIZE)

        # Register defaults
        self._write16(_REG_PRESENT_POSITION, 2048)    # mid-range
        self._regs[_REG_PRESENT_TEMPERATURE] = 30     # 30 °C
        self._regs[_REG_PRESENT_VOLTAGE]     = 120    # 12.0 V
        self._regs[5] = servo_id                      # REG_ID

    # ── private helpers ───────────────────────────────────────────────────────

    def _write16(self, addr: int, value: int):
        if addr + 1 < self.REG_SIZE:
            self._regs[addr]     = value & 0xFF
            self._regs[addr + 1] = (value >> 8) & 0xFF

    def _read16(self, addr: int) -> int:
        if addr + 1 < self.REG_SIZE:
            return self._regs[addr] | (self._regs[addr + 1] << 8)
        return 0

    # ── public API ────────────────────────────────────────────────────────────

    def read_registers(self, addr: int, length: int) -> bytes:
        """Return *length* bytes starting at *addr*, zero-padded if needed."""
        end  = min(addr + length, self.REG_SIZE)
        data = bytes(self._regs[addr:end])
        if len(data) < length:
            data = data + bytes(length - len(data))
        return data

    def write_registers(self, addr: int, data: bytes):
        """Write *data* at *addr*.  Mirrors GOAL_POSITION → PRESENT_POSITION."""
        for i, b in enumerate(data):
            if addr + i < self.REG_SIZE:
                self._regs[addr + i] = b

        # Mirror: any write that touches register 42 (GOAL_POSITION) is
        # immediately reflected in register 56 (PRESENT_POSITION) so that
        # position-mode sliders appear interactive.
        if addr <= _REG_GOAL_POSITION < addr + len(data):
            goal = self._read16(_REG_GOAL_POSITION)
            self._write16(_REG_PRESENT_POSITION, goal)


# ─────────────────────────────────────────────────────────────────────────────
# DropRule
# ─────────────────────────────────────────────────────────────────────────────

class DropRule:
    """Suppress servo responses that match this rule.

    Parameters
    ----------
    instruction : int | None
        INST_* constant to match, or None to match any instruction.
    servo_id : int | None
        Servo ID to match, or None to match any servo.
    after_n : int | None
        If set, the rule only starts dropping *after* after_n successful
        (non-dropped) calls have been seen.
    name : str | None
        Label used by remove_drop_rule() and log lines.
    """

    def __init__(self, instruction=None, servo_id=None, after_n=None,
                 name=None):
        self.instruction = instruction
        self.servo_id    = servo_id
        self.after_n     = after_n
        self.name        = name or self._auto_name()
        self._call_count = 0   # total calls that matched the inst/id filter

    def _auto_name(self) -> str:
        parts = []
        if self.servo_id is not None:
            parts.append(f"servo_id={self.servo_id}")
        if self.instruction is not None:
            parts.append(
                f"instr={_INST_NAMES.get(self.instruction, str(self.instruction))}")
        if self.after_n is not None:
            parts.append(f"after_n={self.after_n}")
        return ",".join(parts) or "drop_all"

    def matches(self, instruction: int, servo_id: int) -> bool:
        """Return True if this rule should suppress the response."""
        if self.instruction is not None and self.instruction != instruction:
            return False
        if self.servo_id is not None and self.servo_id != servo_id:
            return False
        self._call_count += 1
        if self.after_n is not None and self._call_count <= self.after_n:
            return False
        return True

    def reset_counter(self):
        self._call_count = 0


# ─────────────────────────────────────────────────────────────────────────────
# HandSimulator  (serial.Serial drop-in)
# ─────────────────────────────────────────────────────────────────────────────

class HandSimulator:
    """A serial.Serial drop-in that simulates Feetech servos.

    Pass an instance wherever a port string or serial.Serial object is
    accepted (requires the servo_protocol.py patch).

    Quick start::

        sim = HandSimulator(servo_ids=[0, 1, 2, 3, 4, 5, 6])
        sim.add_drop_rule(servo_id=2)          # freeze servo 2
        hand = Hand(sim, hand_type='left')
    """

    def __init__(self, servo_ids: list | None = None,
                 flood_threshold: int = _FLOOD_THRESHOLD_DEFAULT):
        if servo_ids is None:
            servo_ids = SERVO_IDS[:]

        # serial.Serial-compatible attributes
        self.baudrate = 1_000_000
        self.timeout  = 0.01
        self.is_open  = True

        # Servo register maps
        self._servos: dict[int, SimulatedServo] = {
            sid: SimulatedServo(sid) for sid in servo_ids
        }

        # Buffers and sync
        self._write_buf = bytearray()
        self._read_buf  = collections.deque()
        self._lock      = threading.Lock()

        # Drop rules
        self._drop_rules: list[DropRule] = []

        # Flood detection threshold
        self._flood_threshold = flood_threshold

        # Stats
        self._stats = {
            "total_rx":              0,
            "total_dropped":         0,
            "by_instruction":        collections.defaultdict(int),
            "dropped_by_instruction": collections.defaultdict(int),
        }

        # Rolling 1-second window for packets/sec calculation
        self._pkt_times: collections.deque = collections.deque()

        # Log file
        _log_dir = os.path.join("logs", "sim")
        os.makedirs(_log_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(_log_dir, f"sim_{ts}.log")
        self._log_file = open(self._log_path, "w", buffering=1)  # line-buffered

    # ── serial.Serial interface ───────────────────────────────────────────────

    def reset_input_buffer(self):
        with self._lock:
            self._read_buf.clear()

    def write(self, data: bytes) -> int:
        data = bytes(data)
        with self._lock:
            self._log("TX", data)
            self._write_buf.extend(data)
            self._try_process()
        return len(data)

    def read(self, size: int = 1) -> bytes:
        result = bytearray()
        with self._lock:
            for _ in range(size):
                if self._read_buf:
                    result.append(self._read_buf.popleft())
                else:
                    break
        return bytes(result)

    def close(self):
        if self._log_file:
            self._log_file.flush()
            self._log_file.close()
            self._log_file = None
        self.is_open = False

    # ── log file helpers ──────────────────────────────────────────────────────

    def _log(self, direction: str, payload: bytes, extra: str = ""):
        """Write one line to the log file.  direction is padded to 7 chars."""
        if not self._log_file:
            return
        ts   = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_ = " ".join(f"{b:02X}" for b in payload)
        line = f"{ts}  {direction:<7}  {hex_}"
        if extra:
            line += f"  {extra}"
        self._log_file.write(line + "\n")

    def _log_dropped(self, servo_id: int, instruction: int, rule: DropRule):
        if not self._log_file:
            return
        ts        = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        inst_name = _INST_NAMES.get(instruction, f"0x{instruction:02X}")
        self._log_file.write(
            f"{ts}  {'DROPPED':<7}  "
            f"servo={servo_id}  instr={inst_name}  rule=\"{rule.name}\"\n")

    # ── packet processing ─────────────────────────────────────────────────────

    def _try_process(self):
        """Parse and dispatch all complete packets from _write_buf."""
        while True:
            pkt = self._try_extract_packet()
            if pkt is None:
                break
            self._dispatch(*pkt)

    def _try_extract_packet(self):
        """Extract one complete packet from _write_buf.

        Frame layout::

            FF FF  ID  Length  Instruction  [Params...]  Checksum
              2    1     1         1         Length-2       1

        Total frame = Length + 4 bytes.

        Returns (servo_id, instruction, params_bytes) or None.
        """
        buf = self._write_buf

        # Find 0xFF 0xFF header
        start = -1
        for i in range(len(buf) - 1):
            if buf[i] == 0xFF and buf[i + 1] == 0xFF:
                start = i
                break

        if start < 0:
            # No header — keep trailing 0xFF in case it's the start of one
            self._write_buf = bytearray([0xFF]) if buf and buf[-1] == 0xFF else bytearray()
            return None

        if start > 0:
            self._write_buf = buf[start:]
            buf = self._write_buf

        # Need at least 4 bytes to read servo_id and length
        if len(buf) < 4:
            return None

        servo_id = buf[2]
        length   = buf[3]          # = n_params + 2  (instruction + checksum)
        total    = length + 4      # full frame size

        if len(buf) < total:
            return None            # frame not yet complete

        frame       = buf[:total]
        instruction = frame[4]
        params      = bytes(frame[5 : 3 + length])  # length-2 bytes
        checksum    = frame[3 + length]              # last byte of frame

        # Validate checksum
        expected = (~(servo_id + length + instruction + sum(params))) & 0xFF
        if checksum != expected:
            # Bad checksum — skip 2 bytes and retry to re-sync
            self._write_buf = buf[2:]
            return None

        self._write_buf = buf[total:]
        return servo_id, instruction, params

    def _dispatch(self, servo_id: int, instruction: int, params: bytes):
        """Route a parsed packet to the correct handler."""
        self._stats["total_rx"] += 1
        self._stats["by_instruction"][instruction] += 1

        # Rolling flood detection
        now = time.monotonic()
        self._pkt_times.append(now)
        while self._pkt_times and now - self._pkt_times[0] > 1.0:
            self._pkt_times.popleft()
        pps = len(self._pkt_times)
        if pps > self._flood_threshold:
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(
                f"{ts}  WARNING  Bus flood detected: {pps} pkt/s "
                f"(threshold={self._flood_threshold})",
                file=sys.stderr,
            )

        if instruction == INST_SYNC_READ:
            self._handle_sync_read(params)
        elif instruction == INST_SYNC_WRITE:
            self._handle_sync_write(params)
        else:
            # Single-servo instructions: check drop rules first
            if self._should_drop(instruction, servo_id):
                return
            if instruction == INST_PING:
                self._enqueue_response(servo_id)
            elif instruction == INST_READ:
                self._handle_read(servo_id, params)
            elif instruction in (INST_WRITE, INST_REG_WRITE):
                self._handle_write(servo_id, params)
                if servo_id != BROADCAST_ID:
                    self._enqueue_response(servo_id)
            elif instruction == INST_REG_ACTION:
                if servo_id != BROADCAST_ID:
                    self._enqueue_response(servo_id)
            elif instruction in (INST_RESET, INST_CAL, INST_RECOVERY):
                if servo_id != BROADCAST_ID:
                    self._enqueue_response(servo_id)

    def _should_drop(self, instruction: int, servo_id: int) -> bool:
        """Return True if the response for this (instruction, servo_id) should be suppressed."""
        for rule in self._drop_rules:
            if rule.matches(instruction, servo_id):
                self._stats["total_dropped"] += 1
                self._stats["dropped_by_instruction"][instruction] += 1
                self._log_dropped(servo_id, instruction, rule)
                return True
        return False

    def _enqueue_response(self, servo_id: int, data: bytes = b""):
        pkt = self._build_response(servo_id, data)
        self._log("RX", pkt)
        self._read_buf.extend(pkt)

    @staticmethod
    def _build_response(servo_id: int, data: bytes = b"") -> bytes:
        """Build a standard Feetech status/response packet."""
        length = len(data) + 2   # status byte + checksum byte
        status = 0x00
        ck     = (~(servo_id + length + status + sum(data))) & 0xFF
        return bytes([0xFF, 0xFF, servo_id, length, status]) + data + bytes([ck])

    # ── instruction handlers ──────────────────────────────────────────────────

    def _handle_read(self, servo_id: int, params: bytes):
        if len(params) < 2:
            self._enqueue_response(servo_id)
            return
        addr, length = params[0], params[1]
        servo = self._servos.get(servo_id)
        if servo is None:
            return   # unknown servo — no response (matches real hardware)
        data = servo.read_registers(addr, length)
        self._enqueue_response(servo_id, data)

    def _handle_write(self, servo_id: int, params: bytes):
        if len(params) < 1:
            return
        addr  = params[0]
        data  = params[1:]
        servo = self._servos.get(servo_id)
        if servo is not None:
            servo.write_registers(addr, data)

    def _handle_sync_read(self, params: bytes):
        """SYNC_READ: params = [addr, data_len, id0, id1, ...]

        Emit one response per servo that is not suppressed by a drop rule.
        Missing servos leave gaps in the byte stream — exactly what happens
        on real hardware when a servo stops responding.
        """
        if len(params) < 3:
            return
        addr     = params[0]
        data_len = params[1]
        ids      = list(params[2:])

        for sid in ids:
            if self._should_drop(INST_SYNC_READ, sid):
                continue
            servo = self._servos.get(sid)
            if servo is None:
                continue
            data = servo.read_registers(addr, data_len)
            self._enqueue_response(sid, data)

    def _handle_sync_write(self, params: bytes):
        """SYNC_WRITE: params = [addr, data_len, id0, data0..., id1, data1..., ...]

        Updates each servo's register map.  No response is generated
        (broadcast command).  Drop rules are honoured per servo: if a rule
        matches, the register write is suppressed and a DROPPED line is emitted
        (matching the behaviour of _handle_sync_read).
        """
        if len(params) < 3:
            return
        addr     = params[0]
        data_len = params[1]
        rest     = params[2:]
        stride   = 1 + data_len   # servo_id byte + data bytes

        i = 0
        while i + stride <= len(rest):
            sid  = rest[i]
            data = rest[i + 1 : i + stride]
            if not self._should_drop(INST_SYNC_WRITE, sid):
                servo = self._servos.get(sid)
                if servo is not None:
                    servo.write_registers(addr, data)
            i += stride

    # ── drop rule API ─────────────────────────────────────────────────────────

    def add_drop_rule(self, servo_id=None, instruction=None,
                      after_n=None, name=None) -> DropRule:
        """Add a response suppression rule.

        Parameters
        ----------
        servo_id : int | None
            Suppress responses for this servo ID (None = any servo).
        instruction : int | str | None
            Suppress this instruction (None = any).  May be a string like
            ``'SYNC_READ'``, ``'PING'``, etc.
        after_n : int | None
            Only start dropping *after* after_n successful responses.
        name : str | None
            Label for remove_drop_rule() and log lines.  Auto-generated if
            not supplied.

        Returns the new DropRule object.
        """
        if isinstance(instruction, str):
            instruction = _INST_BY_NAME[instruction.upper()]
        rule = DropRule(instruction=instruction, servo_id=servo_id,
                        after_n=after_n, name=name)
        with self._lock:
            self._drop_rules.append(rule)
        return rule

    def remove_drop_rule(self, name: str):
        """Remove the first drop rule with the given name."""
        with self._lock:
            for i, rule in enumerate(self._drop_rules):
                if rule.name == name:
                    del self._drop_rules[i]
                    return

    def clear_drop_rules(self):
        """Remove all drop rules."""
        with self._lock:
            self._drop_rules.clear()

    # ── stats / flood detection ───────────────────────────────────────────────

    def get_stats(self) -> dict:
        """Return a snapshot of traffic statistics."""
        with self._lock:
            now    = time.monotonic()
            recent = sum(1 for t in self._pkt_times if now - t <= 1.0)
            return {
                "total_rx":               self._stats["total_rx"],
                "total_dropped":          self._stats["total_dropped"],
                "by_instruction":         dict(self._stats["by_instruction"]),
                "dropped_by_instruction": dict(self._stats["dropped_by_instruction"]),
                "packets_per_second":     recent,
                "log_path":               self._log_path,
            }

    def reset_stats(self):
        """Reset all counters (does not clear drop rules)."""
        with self._lock:
            self._stats["total_rx"]   = 0
            self._stats["total_dropped"] = 0
            self._stats["by_instruction"].clear()
            self._stats["dropped_by_instruction"].clear()
            self._pkt_times.clear()

    def print_stats(self):
        """Pretty-print statistics to stdout."""
        s = self.get_stats()
        print("HandSimulator stats:")
        print(f"  total_rx:      {s['total_rx']}")
        print(f"  total_dropped: {s['total_dropped']}")
        print(f"  packets/sec:   {s['packets_per_second']}")
        print(f"  log file:      {s['log_path']}")
        if s["by_instruction"]:
            print("  by instruction:")
            for code, count in sorted(s["by_instruction"].items()):
                name    = _INST_NAMES.get(code, f"0x{code:02X}")
                dropped = s["dropped_by_instruction"].get(code, 0)
                print(f"    {name:<12}  rx={count}  dropped={dropped}")


# ─────────────────────────────────────────────────────────────────────────────
# Self-test / demo
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import atexit
    from hand.ttl import HandTTL

    sim = HandSimulator(servo_ids=SERVO_IDS)

    # Uncomment to reproduce the "middle finger stalls" condition:
    # sim.add_drop_rule(servo_id=2)

    # Uncomment to trigger bus flooding after 20 SYNC_READ cycles:
    # sim.add_drop_rule(instruction='SYNC_READ', after_n=20)

    atexit.register(sim.print_stats)

    hand = HandTTL(sim, hand_type="left")
    print("Hand created with simulated serial.  Reading sensors for 2 s...")
    for _ in range(4):
        time.sleep(0.5)
        print(f"  positions: {hand.get_positions()}")
    hand.close()
