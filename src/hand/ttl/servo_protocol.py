import threading
import serial

# Instruction codes (from INST.h)
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_REG_ACTION = 0x05
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
INST_RECOVERY = 0x06
INST_RESET = 0x0A
INST_CAL = 0x0B

BROADCAST_ID = 0xFE


def _to_sign_mag16(value: int) -> int:
    """Encode a signed int as sign-magnitude 16-bit (bit 15 = sign)."""
    if value < 0:
        return (-value) | (1 << 15)
    return value


def _from_sign_mag16(raw: int) -> int:
    """Decode a sign-magnitude 16-bit value to a signed int."""
    if raw & (1 << 15):
        return -(raw & ~(1 << 15))
    return raw


class ServoProtocol:
    """Low-level Feetech serial servo protocol.

    Implements the packet framing from SCSerial + SCS (Arduino library).
    All methods are thread-safe via an internal lock.
    """

    def __init__(self, port: "str | object", baudrate: int = 1_000_000,
                 timeout: float = 0.01):
        if isinstance(port, str):
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )
        else:
            self._serial = port   # accept any serial-like object (e.g. HandSimulator)
        self._lock = threading.RLock()
        self._timeout = timeout
        self._logger = None   # optional SerialFileLogger; set externally

    def close(self):
        self._serial.close()

    @property
    def lock(self) -> threading.RLock:
        return self._lock

    # ---- Low-level packet I/O ----

    def _flush_input(self):
        self._serial.reset_input_buffer()

    def _write_packet(self, servo_id: int, instruction: int,
                      params: bytes | None = None):
        """Build and send a protocol frame.

        Frame: [0xFF 0xFF ID Length Instruction [Params...] Checksum]
        Length = len(params) + 2  (instruction byte + checksum byte count in length)
        """
        if params is None:
            params = b""
        length = len(params) + 2
        header = bytes([0xFF, 0xFF, servo_id, length, instruction])
        checksum = (~(servo_id + length + instruction + sum(params))) & 0xFF
        pkt = header + params + bytes([checksum])
        self._serial.write(pkt)
        if self._logger is not None:
            self._logger.log_tx(pkt)

    def _read_response(self, expected_id: int, expected_data_len: int
                       ) -> bytes | None:
        """Read and validate a response packet.

        Returns the data payload (without header/checksum) or None on failure.
        """
        # Find header 0xFF 0xFF
        attempts = 0
        prev = 0
        while attempts < 20:
            b = self._serial.read(1)
            if len(b) == 0:
                return None
            cur = b[0]
            if prev == 0xFF and cur == 0xFF:
                break
            prev = cur
            attempts += 1
        else:
            return None

        # Read ID, Length, Error/Status
        hdr = self._serial.read(3)
        if len(hdr) != 3:
            return None
        resp_id, resp_len, status = hdr
        if resp_id != expected_id and expected_id != BROADCAST_ID:
            return None
        data_len = resp_len - 2  # subtract status + checksum
        if data_len < 0:
            # Length field includes the error byte and checksum.
            # For a ping response, resp_len == 2, so data_len == 0.
            data_len = 0

        data = b""
        if data_len > 0:
            data = self._serial.read(data_len)
            if len(data) != data_len:
                return None

        # Read checksum
        ckb = self._serial.read(1)
        if len(ckb) != 1:
            return None
        expected_cksum = (~(resp_id + resp_len + status + sum(data))) & 0xFF
        if ckb[0] != expected_cksum:
            return None
        if self._logger is not None:
            full_pkt = bytes([0xFF, 0xFF, resp_id, resp_len, status]) + data + ckb
            self._logger.log_rx(full_pkt)
        return data

    # ---- Public API ----

    def ping(self, servo_id: int) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_PING)
            # Ping response: header + ID(1) + Length(1=2) + Error(1) + Checksum(1)
            # _read_response expects data_len; for ping, data_len = 0
            resp = self._read_response(servo_id, 0)
            return resp is not None

    def gen_write(self, servo_id: int, mem_addr: int, data: bytes) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_WRITE,
                               bytes([mem_addr]) + data)
            if servo_id == BROADCAST_ID:
                return True
            return self._read_response(servo_id, 0) is not None

    def reg_write(self, servo_id: int, mem_addr: int, data: bytes) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_REG_WRITE,
                               bytes([mem_addr]) + data)
            if servo_id == BROADCAST_ID:
                return True
            return self._read_response(servo_id, 0) is not None

    def reg_write_action(self, servo_id: int = BROADCAST_ID) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_REG_ACTION)
            if servo_id == BROADCAST_ID:
                return True
            return self._read_response(servo_id, 0) is not None

    def write_byte(self, servo_id: int, mem_addr: int, value: int) -> bool:
        return self.gen_write(servo_id, mem_addr, bytes([value & 0xFF]))

    def write_word(self, servo_id: int, mem_addr: int, value: int) -> bool:
        # Little-endian: low byte first (End=0 in Arduino code)
        lo = value & 0xFF
        hi = (value >> 8) & 0xFF
        return self.gen_write(servo_id, mem_addr, bytes([lo, hi]))

    def read_block(self, servo_id: int, mem_addr: int, length: int
                   ) -> bytes | None:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_READ,
                               bytes([mem_addr, length]))
            return self._read_response(servo_id, length)

    def read_byte(self, servo_id: int, mem_addr: int) -> int | None:
        data = self.read_block(servo_id, mem_addr, 1)
        if data is None or len(data) < 1:
            return None
        return data[0]

    def read_word(self, servo_id: int, mem_addr: int) -> int | None:
        data = self.read_block(servo_id, mem_addr, 2)
        if data is None or len(data) < 2:
            return None
        return data[0] | (data[1] << 8)

    def sync_write(self, servo_ids: list[int], mem_addr: int,
                   data_per_servo: list[bytes]) -> None:
        """Sync write: broadcast write different data to multiple servos."""
        if not servo_ids:
            return
        n_servos = len(servo_ids)
        data_len = len(data_per_servo[0])
        # Build params: [MemAddr, DataLen, ID0, Data0..., ID1, Data1..., ...]
        params = bytearray([mem_addr, data_len])
        for i in range(n_servos):
            params.append(servo_ids[i])
            params.extend(data_per_servo[i])

        # Sync write uses broadcast ID and a special frame structure.
        # Frame: [0xFF 0xFF 0xFE Length INST_SYNC_WRITE MemAddr DataLen
        #         ID0 Data0... ID1 Data1... Checksum]
        length = (data_len + 1) * n_servos + 4
        checksum_sum = (BROADCAST_ID + length + INST_SYNC_WRITE +
                        mem_addr + data_len)
        for i in range(n_servos):
            checksum_sum += servo_ids[i] + sum(data_per_servo[i])
        checksum = (~checksum_sum) & 0xFF

        header = bytes([0xFF, 0xFF, BROADCAST_ID, length, INST_SYNC_WRITE,
                        mem_addr, data_len])
        body = bytearray()
        for i in range(n_servos):
            body.append(servo_ids[i])
            body.extend(data_per_servo[i])

        with self._lock:
            self._flush_input()
            pkt = header + bytes(body) + bytes([checksum])
            self._serial.write(pkt)
            if self._logger is not None:
                self._logger.log_tx(pkt)

    def sync_read(self, servo_ids: list[int], mem_addr: int, length: int,
                  timeout: float | None = None
                  ) -> dict[int, bytes]:
        """Sync read: request data from multiple servos in one transaction.

        Returns dict mapping servo_id -> bytes of data read.
        """
        if timeout is None:
            timeout = self._timeout * len(servo_ids) + 0.008
        n_servos = len(servo_ids)

        with self._lock:
            self._flush_input()
            # Build sync read request packet
            # Frame: [0xFF 0xFF 0xFE Length INST_SYNC_READ MemAddr DataLen
            #         ID0 ID1 ... Checksum]
            pkt_len = n_servos + 4
            checksum_sum = (BROADCAST_ID + pkt_len + INST_SYNC_READ +
                            mem_addr + length)
            for sid in servo_ids:
                checksum_sum += sid
            checksum = (~checksum_sum) & 0xFF

            pkt = bytes([0xFF, 0xFF, BROADCAST_ID, pkt_len, INST_SYNC_READ,
                         mem_addr, length])
            pkt += bytes(servo_ids) + bytes([checksum])
            self._serial.write(pkt)
            if self._logger is not None:
                self._logger.log_tx(pkt)

            # Read all response bytes with extended timeout
            max_resp_len = n_servos * (length + 6)
            old_timeout = self._serial.timeout
            self._serial.timeout = timeout
            buf = self._serial.read(max_resp_len)
            self._serial.timeout = old_timeout

            # Parse individual responses from the buffer
            result = {}
            idx = 0
            for target_id in servo_ids:
                # Search for header 0xFF 0xFF followed by target_id
                found = False
                while idx + 5 + length <= len(buf):
                    if (buf[idx] == 0xFF and idx + 1 < len(buf)
                            and buf[idx + 1] == 0xFF):
                        if idx + 2 < len(buf) and buf[idx + 2] == target_id:
                            # Validate length field
                            resp_len = buf[idx + 3]
                            if resp_len == length + 2:
                                # status = buf[idx + 4]
                                data_start = idx + 5
                                data_end = data_start + length
                                if data_end + 1 <= len(buf):
                                    data = buf[data_start:data_end]
                                    # Verify checksum
                                    ck_sum = (target_id + resp_len +
                                              buf[idx + 4] + sum(data))
                                    expected_ck = (~ck_sum) & 0xFF
                                    if buf[data_end] == expected_ck:
                                        result[target_id] = data
                                        if self._logger is not None:
                                            self._logger.log_rx(
                                                bytes(buf[idx : data_end + 1]))
                                        idx = data_end + 1
                                        found = True
                                        break
                            idx += 1
                        else:
                            idx += 1
                    else:
                        idx += 1
                if not found:
                    # Try searching rest of buffer for this ID
                    pass

            return result

    def reset(self, servo_id: int) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_RESET)
            if servo_id == BROADCAST_ID:
                return True
            return self._read_response(servo_id, 0) is not None

    def recal(self, servo_id: int) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_CAL)
            if servo_id == BROADCAST_ID:
                return True
            return self._read_response(servo_id, 0) is not None

    def recovery(self, servo_id: int) -> bool:
        with self._lock:
            self._flush_input()
            self._write_packet(servo_id, INST_RECOVERY)
            if servo_id == BROADCAST_ID:
                return True
            return self._read_response(servo_id, 0) is not None
