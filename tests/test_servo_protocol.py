"""Test suite that verifies Servo and ServoProtocol produce the exact byte
sequences from the worked examples in feetech_protocol.md.

A MockSerial replaces serial.Serial so no hardware is needed.  Each test
calls the high-level API, then checks that the bytes written to the serial
line match the expected hex from the protocol document.
"""

import unittest
from unittest.mock import patch, MagicMock
from collections import deque

# ---------------------------------------------------------------------------
# Mock serial port that records writes and plays back configured responses
# ---------------------------------------------------------------------------

class MockSerial:
    """Drop-in replacement for serial.Serial used in tests.

    * All bytes passed to write() are appended to `written`.
    * Bytes queued via `stage_response()` are returned by read().
    """

    def __init__(self, **kwargs):
        self.written = bytearray()
        self._rx_buf = deque()  # bytes available for read()
        self.timeout = kwargs.get("timeout", 0.01)

    # -- write side ----------------------------------------------------------

    def write(self, data):
        self.written.extend(data)
        return len(data)

    # -- read side -----------------------------------------------------------

    def stage_response(self, data: bytes):
        """Queue bytes that subsequent read() calls will return."""
        self._rx_buf.extend(data)

    def read(self, size=1):
        out = bytearray()
        for _ in range(size):
            if self._rx_buf:
                out.append(self._rx_buf.popleft())
            else:
                break
        return bytes(out)

    # -- misc ----------------------------------------------------------------

    def reset_input_buffer(self):
        # In real hardware, this clears stale bytes already in the buffer.
        # In the mock, staged bytes represent future servo responses that
        # arrive *after* the command is sent, so we intentionally do not
        # clear them here.
        pass

    def close(self):
        pass


def _make_ack(servo_id: int) -> bytes:
    """Build a standard 6-byte acknowledgment response for *servo_id*."""
    status = 0x00
    length = 0x02
    checksum = (~(servo_id + length + status)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, status, checksum])


def _make_read_response(servo_id: int, data: bytes) -> bytes:
    """Build a read-response packet carrying *data* for *servo_id*."""
    status = 0x00
    length = len(data) + 2  # status + data + checksum
    checksum = (~(servo_id + length + status + sum(data))) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, status]) + data + bytes([checksum])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _hex(bs: bytes | bytearray) -> str:
    """Pretty-print bytes as uppercase hex for assertion messages."""
    return " ".join(f"{b:02X}" for b in bs)


def _create_protocol_and_servo(mock_serial: MockSerial):
    """Instantiate ServoProtocol and Servo using a pre-built MockSerial."""
    from hand.ttl.servo_protocol import ServoProtocol
    from hand.ttl.servo import Servo

    with patch("hand.servo_protocol.serial.Serial", return_value=mock_serial):
        protocol = ServoProtocol("FAKE_PORT")
    servo = Servo(protocol)
    return protocol, servo


# ===========================================================================
# Tests — one per worked example in feetech_protocol.md §10
# ===========================================================================

class TestProtocolExamples(unittest.TestCase):
    """Each test corresponds to a numbered example in the protocol doc."""

    # -----------------------------------------------------------------------
    # Example 1 — Ping Servo ID 3
    # -----------------------------------------------------------------------
    def test_example1_ping(self):
        ms = MockSerial()
        protocol, _ = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF 03 02 01 F9")
        response = bytes.fromhex("FF FF 03 02 00 FA")
        ms.stage_response(response)

        result = protocol.ping(3)

        self.assertTrue(result)
        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 2 — Read 2 bytes (position) from servo 1, register 56
    # -----------------------------------------------------------------------
    def test_example2_read_position(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF 01 04 02 38 02 BE")
        # Response: position = 2048 = 0x0800, little-endian data: 00 08
        response = bytes.fromhex("FF FF 01 04 00 00 08 F2")
        ms.stage_response(response)

        pos = servo.read_pos(1)

        self.assertEqual(pos, 2048)
        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 3 — WritePosEx: position=1000, speed=500, servo 5
    # -----------------------------------------------------------------------
    def test_example3_write_pos_ex(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex(
            "FF FF 05 0A 03 29 00 E8 03 00 00 F4 01 E4")
        response = bytes.fromhex("FF FF 05 02 00 F8")
        ms.stage_response(response)

        servo.write_pos_ex(5, position=1000, speed=500, acc=0, torque=0)

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 4 — WritePosEx: position=-500, speed=300, acc=10, torque=200,
    #              servo 2
    # -----------------------------------------------------------------------
    def test_example4_write_pos_ex_negative(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex(
            "FF FF 02 0A 03 29 0A F4 81 C8 00 2C 01 53")
        response = bytes.fromhex("FF FF 02 02 00 FB")
        ms.stage_response(response)

        servo.write_pos_ex(2, position=-500, speed=300, acc=10, torque=200)

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 5 — Enable torque on servo 4
    # -----------------------------------------------------------------------
    def test_example5_enable_torque(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF 04 04 03 28 01 CB")
        ms.stage_response(_make_ack(4))

        servo.enable_torque(4, True)

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 6 — Set Ele mode on servo 0
    # -----------------------------------------------------------------------
    def test_example6_ele_mode(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF 00 04 03 21 02 D5")
        ms.stage_response(_make_ack(0))

        servo.ele_mode(0)

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 7 — WriteEle: torque=-300, servo 1
    # -----------------------------------------------------------------------
    def test_example7_write_ele(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF 01 05 03 2C 2C 81 1D")
        ms.stage_response(_make_ack(1))

        servo.write_ele(1, -300)

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 8 — Sync write positions to servos 0, 1, 2
    # -----------------------------------------------------------------------
    def test_example8_sync_write_pos_ex(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex(
            "FF FF FE 1C 83 29 07"
            "00 00 E8 03 00 00 F4 01"
            "01 05 00 08 64 00 2C 01"
            "02 0A B8 0B 00 00 90 01"
            "53")

        servo.sync_write_pos_ex(
            servo_ids=[0, 1, 2],
            positions=[1000, 2048, 3000],
            speeds=[500, 300, 400],
            accs=[0, 5, 10],
            torques=[0, 100, 0],
        )

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 9 — Sync read 15-byte sensor block from servos 0, 1, 2
    #             (verify the command packet only; response parsing tested
    #              by checking the returned dict)
    # -----------------------------------------------------------------------
    def test_example9_sync_read_command(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF FE 07 82 38 0F 00 01 02 2E")

        # Build the response for servo 0 from the doc example:
        # pos=2048, speed=0, load=0, voltage=75, temp=35, moving=0, current=50
        sensor_data_0 = bytes.fromhex(
            "00 08 00 00 00 00 4B 23 00 00 00 00 00 32 00")
        resp_0 = _make_read_response(0, sensor_data_0)

        # Fabricate simple responses for servos 1 and 2 (all zeros)
        sensor_data_1 = bytes(15)
        sensor_data_2 = bytes(15)
        resp_1 = _make_read_response(1, sensor_data_1)
        resp_2 = _make_read_response(2, sensor_data_2)

        ms.stage_response(resp_0 + resp_1 + resp_2)

        result = servo.sync_read_sensors([0, 1, 2])

        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

        # Also verify the parsed sensor values for servo 0
        self.assertIn(0, result)
        self.assertEqual(result[0]["position"], 2048)
        self.assertEqual(result[0]["speed"], 0)
        self.assertEqual(result[0]["load"], 0)
        self.assertEqual(result[0]["voltage"], 75)
        self.assertEqual(result[0]["temperature"], 35)
        self.assertEqual(result[0]["moving"], 0)
        self.assertEqual(result[0]["current"], 50)

    # -----------------------------------------------------------------------
    # Example 10 — CAL on servo 6
    # -----------------------------------------------------------------------
    def test_example10_cal(self):
        ms = MockSerial()
        protocol, _ = _create_protocol_and_servo(ms)

        expected_cmd = bytes.fromhex("FF FF 06 02 0B EC")
        response = bytes.fromhex("FF FF 06 02 00 F7")
        ms.stage_response(response)

        result = protocol.recal(6)

        self.assertTrue(result)
        self.assertEqual(
            bytes(ms.written), expected_cmd,
            f"Expected: {_hex(expected_cmd)}\n"
            f"Got:      {_hex(ms.written)}")

    # -----------------------------------------------------------------------
    # Example 11 — Full EPROM unlock → write ID → lock sequence
    #              (servo ID 1 → 5)
    #
    # This is a multi-step sequence.  We verify each command packet in order.
    # Steps:
    #   1. Disable torque on servo 1:  FF FF 01 04 03 28 00 CF
    #   2. Unlock EPROM on servo 1:    FF FF 01 04 03 37 00 C0
    #      (unlock_eprom also disables torque first, so step 1 is repeated)
    #   3. Write new ID (5) to reg 5:  FF FF 01 04 03 05 05 ED
    #   4. Lock EPROM on servo 5:      FF FF 05 04 03 37 01 BB
    #   5. Enable torque on servo 5:   FF FF 05 04 03 28 01 CA
    #
    # Note: The Servo class's unlock_eprom() calls enable_torque(False)
    # internally, so the full byte sequence produced by calling:
    #   servo.unlock_eprom(1)  → disable torque + unlock
    #   protocol.write_byte(1, 5, 5)  → write new ID
    #   servo.lock_eprom(5)    → lock
    #   servo.enable_torque(5, True) → enable torque
    # will match the doc's 5 packets.
    # -----------------------------------------------------------------------
    def test_example11_eprom_id_change(self):
        ms = MockSerial()
        protocol, servo = _create_protocol_and_servo(ms)

        # Each non-broadcast write expects an ack response
        # Step 1+2 are on servo 1, step 3 on servo 1, steps 4+5 on servo 5
        ms.stage_response(_make_ack(1))  # ack for disable torque (inside unlock_eprom)
        ms.stage_response(_make_ack(1))  # ack for unlock eprom
        ms.stage_response(_make_ack(1))  # ack for write ID
        ms.stage_response(_make_ack(5))  # ack for lock eprom
        ms.stage_response(_make_ack(5))  # ack for enable torque

        # Execute the sequence
        servo.unlock_eprom(1)              # disable torque + unlock
        protocol.write_byte(1, 5, 5)       # write new ID = 5 to register 5
        servo.lock_eprom(5)                 # lock EPROM on new ID
        servo.enable_torque(5, True)        # re-enable torque on new ID

        # Expected full byte stream (5 packets concatenated)
        expected = (
            bytes.fromhex("FF FF 01 04 03 28 00 CF")   # step 1: disable torque
            + bytes.fromhex("FF FF 01 04 03 37 00 C0")  # step 2: unlock EPROM
            + bytes.fromhex("FF FF 01 04 03 05 05 ED")  # step 3: write ID=5
            + bytes.fromhex("FF FF 05 04 03 37 01 BB")  # step 4: lock EPROM
            + bytes.fromhex("FF FF 05 04 03 28 01 CA")  # step 5: enable torque
        )

        self.assertEqual(
            bytes(ms.written), expected,
            f"Expected: {_hex(expected)}\n"
            f"Got:      {_hex(ms.written)}")


if __name__ == "__main__":
    unittest.main()
