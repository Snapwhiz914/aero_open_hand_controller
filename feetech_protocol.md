# Feetech Servo Serial Protocol — Low-Level Specification

This document describes the exact byte-level serial protocol used by Feetech HLS-series servo motors (e.g. HLS3606M), as implemented in the FTServo Arduino library.

---

## 1. Physical Layer

- **Interface**: TTL-level UART (half-duplex on a shared bus)
- **Baud rate**: 1,000,000 bps (default, configurable via register 6)
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1
- **Byte order**: Little-endian (`End=0` in the library) — when a 16-bit word is split into two bytes, the **low byte is sent first**, then the high byte.

---

## 2. Packet Framing

Every packet (both command and response) uses the same frame structure:

```
┌──────┬──────┬──────┬────────┬─────────────┬───────────────┬──────────┐
│ 0xFF │ 0xFF │  ID  │ Length │ Instruction │ Parameters... │ Checksum │
└──────┴──────┴──────┴────────┴─────────────┴───────────────┴──────────┘
```

| Field         | Size    | Description |
|---------------|---------|-------------|
| Header        | 2 bytes | Always `0xFF 0xFF` |
| ID            | 1 byte  | Servo ID (0–253), or `0xFE` for broadcast |
| Length        | 1 byte  | Number of bytes **after** Length, i.e. `1 (instruction) + N (parameters) + 1 (checksum)` = `N + 2` |
| Instruction   | 1 byte  | Instruction code (see Section 3) |
| Parameters    | N bytes | Instruction-specific payload (may be 0 bytes) |
| Checksum      | 1 byte  | `~(ID + Length + Instruction + Param[0] + ... + Param[N-1]) & 0xFF` |

The **Length** field counts: the instruction byte, all parameter bytes, and the checksum byte itself. So for a packet with 0 parameter bytes, Length = 2. For a packet with 1 parameter byte, Length = 3, etc.

### Checksum Calculation

```
Checksum = ~(ID + Length + Instruction + sum_of_all_parameter_bytes) & 0xFF
```

The bitwise NOT (`~`) is applied, then masked to 8 bits.

---

## 3. Instruction Codes

| Code   | Name            | Description |
|--------|-----------------|-------------|
| `0x01` | PING            | Check if a servo is online |
| `0x02` | READ            | Read data from servo memory |
| `0x03` | WRITE           | Write data to servo memory |
| `0x04` | REG_WRITE       | Buffer a write (executed later by REG_ACTION) |
| `0x05` | REG_ACTION      | Execute all buffered REG_WRITE commands |
| `0x06` | RECOVERY        | Recovery instruction |
| `0x0A` | RESET           | Reset servo state |
| `0x0B` | CAL             | Calibrate servo midpoint |
| `0x82` | SYNC_READ       | Read same memory region from multiple servos |
| `0x83` | SYNC_WRITE      | Write different data to multiple servos |

---

## 4. Response Packets

When a command is sent to a specific servo ID (not broadcast `0xFE`), the servo responds with:

```
┌──────┬──────┬──────┬────────┬────────┬───────────────┬──────────┐
│ 0xFF │ 0xFF │  ID  │ Length │ Status │    Data...    │ Checksum │
└──────┴──────┴──────┴────────┴────────┴───────────────┴──────────┘
```

| Field    | Size    | Description |
|----------|---------|-------------|
| Header   | 2 bytes | `0xFF 0xFF` |
| ID       | 1 byte  | Responding servo's ID |
| Length   | 1 byte  | `2 + data_length` (status byte + data bytes + checksum) |
| Status   | 1 byte  | Error/status flags (0 = no error) |
| Data     | N bytes | Requested data (only present for READ responses) |
| Checksum | 1 byte  | `~(ID + Length + Status + sum_of_data_bytes) & 0xFF` |

**Broadcast commands (`ID = 0xFE`) produce no response.**

### Acknowledgment Response (Write/Ping/Action)

For non-read commands (WRITE, REG_WRITE, REG_ACTION, PING, RESET, CAL), the response contains no data:

```
Length = 2, Data = (none)
```

So the full response is 6 bytes: `FF FF ID 02 Status Checksum`

### Read Response

For READ commands, the response contains the requested data:

```
Length = 2 + requested_data_length
```

---

## 5. Data Encoding

### 16-bit Word Encoding (Little-Endian)

When `End = 0` (the default used by this library), 16-bit values are split as:

```
Low byte  = value & 0xFF          (sent/stored first, at lower address)
High byte = (value >> 8) & 0xFF   (sent/stored second, at higher address)
```

This is standard little-endian: `[LowByte, HighByte]`.

The `Host2SCS` function with `End=0` maps:
- `DataL` (first parameter, lower memory address) ← `value & 0xFF`
- `DataH` (second parameter, higher memory address) ← `(value >> 8) & 0xFF`

### Sign-Magnitude 16-bit Encoding

Signed 16-bit values (position, speed, torque, current) use **sign-magnitude** encoding, NOT two's complement:

```
Bit 15:    Sign bit (0 = positive, 1 = negative)
Bits 14-0: Magnitude (absolute value)
```

**Encoding** (value → raw):
```
if value < 0:
    raw = |value| | 0x8000
else:
    raw = value
```

**Decoding** (raw → value):
```
if raw & 0x8000:
    value = -(raw & 0x7FFF)
else:
    value = raw
```

This raw 16-bit value is then split into two bytes using the little-endian convention above.

### Load Encoding (Special Case)

The Load register uses **bit 10** as the sign bit (not bit 15):

```
Bit 10:   Sign bit (0 = positive, 1 = negative)
Bits 9-0: Magnitude
```

**Decoding**:
```
if raw & 0x0400:
    value = -(raw & ~0x0400)
else:
    value = raw
```

---

## 6. Instruction Details

### 6.1 PING (`0x01`)

Checks if a servo is online. No parameters.

**Command packet:**
```
FF FF ID 02 01 Checksum
```
- Length = 2 (instruction + checksum, no parameters)
- Checksum = `~(ID + 0x02 + 0x01) & 0xFF`

**Response packet (6 bytes):**
```
FF FF ID 02 Status Checksum
```

---

### 6.2 READ (`0x02`)

Reads `N` bytes from servo memory starting at a given address.

**Command packet:**
```
FF FF ID 04 02 MemAddr DataLen Checksum
```
- Length = 4 (instruction + 2 params + checksum)
- `MemAddr`: Starting register address (1 byte)
- `DataLen`: Number of bytes to read (1 byte)
- Checksum = `~(ID + 0x04 + 0x02 + MemAddr + DataLen) & 0xFF`

**Response packet:**
```
FF FF ID (DataLen+2) Status Data[0] Data[1] ... Data[N-1] Checksum
```
- Length = `DataLen + 2`
- The data bytes are in register order (low address first)

---

### 6.3 WRITE (`0x03`)

Writes data to servo memory starting at a given address.

**Command packet:**
```
FF FF ID (N+3) 03 MemAddr Data[0] Data[1] ... Data[N-1] Checksum
```
- Length = `N + 3` (instruction + 1 addr byte + N data bytes + checksum)
- `MemAddr`: Starting register address (1 byte)
- `Data[0..N-1]`: Bytes to write
- Checksum = `~(ID + Length + 0x03 + MemAddr + sum(Data)) & 0xFF`

**Response packet (if not broadcast):**
```
FF FF ID 02 Status Checksum
```

#### Writing a Single Byte

To write 1 byte to register `Addr`:
```
FF FF ID 04 03 Addr Value Checksum
```

#### Writing a 16-bit Word

To write a 16-bit value `W` to register `Addr` (little-endian):
```
FF FF ID 05 03 Addr (W & 0xFF) ((W >> 8) & 0xFF) Checksum
```

---

### 6.4 REG_WRITE (`0x04`)

Identical to WRITE in packet format, but the write is **buffered** and not applied until a REG_ACTION command is sent. This allows synchronizing writes across multiple servos.

**Command packet:**
```
FF FF ID (N+3) 04 MemAddr Data[0] ... Data[N-1] Checksum
```

**Response packet (if not broadcast):**
```
FF FF ID 02 Status Checksum
```

---

### 6.5 REG_ACTION (`0x05`)

Executes all previously buffered REG_WRITE commands. Typically sent to broadcast address `0xFE`.

**Command packet:**
```
FF FF ID 02 05 Checksum
```
- No parameters (Length = 2)
- Usually `ID = 0xFE` (broadcast to all servos)

**Response packet (if not broadcast):**
```
FF FF ID 02 Status Checksum
```

---

### 6.6 RECOVERY (`0x06`)

Recovery instruction.

**Command packet:**
```
FF FF ID 02 06 Checksum
```

**Response packet (if not broadcast):**
```
FF FF ID 02 Status Checksum
```

---

### 6.7 RESET (`0x0A`)

Resets servo state.

**Command packet:**
```
FF FF ID 02 0A Checksum
```

**Response packet (if not broadcast):**
```
FF FF ID 02 Status Checksum
```

---

### 6.8 CAL (`0x0B`)

Calibrates the servo midpoint (zero-point recalibration).

**Command packet:**
```
FF FF ID 02 0B Checksum
```

**Response packet (if not broadcast):**
```
FF FF ID 02 Status Checksum
```

---

### 6.9 SYNC_WRITE (`0x83`)

Writes different data to multiple servos in a single packet. Always uses broadcast ID `0xFE`. **No response is sent.**

**Command packet:**
```
FF FF FE Length 83 MemAddr DataLen ID0 Data0[0..N-1] ID1 Data1[0..N-1] ... Checksum
```

| Field     | Description |
|-----------|-------------|
| `0xFE`    | Broadcast ID (always) |
| Length    | `(DataLen + 1) * NumServos + 4` |
| `0x83`   | SYNC_WRITE instruction |
| MemAddr   | Starting register address |
| DataLen   | Number of data bytes per servo |
| ID_i      | Servo ID for servo i |
| Data_i    | `DataLen` bytes of data for servo i |
| Checksum  | `~(0xFE + Length + 0x83 + MemAddr + DataLen + sum(all_IDs) + sum(all_data)) & 0xFF` |

The body consists of repeating `[ID, Data[0], Data[1], ..., Data[DataLen-1]]` groups, one per servo.

---

### 6.10 SYNC_READ (`0x82`)

Reads the same memory region from multiple servos. Always uses broadcast ID `0xFE`.

**Command packet:**
```
FF FF FE Length 82 MemAddr DataLen ID0 ID1 ID2 ... Checksum
```

| Field     | Description |
|-----------|-------------|
| `0xFE`    | Broadcast ID (always) |
| Length    | `NumServos + 4` |
| `0x82`   | SYNC_READ instruction |
| MemAddr   | Starting register address |
| DataLen   | Number of bytes to read from each servo |
| ID_0..N   | List of servo IDs to read from |
| Checksum  | `~(0xFE + Length + 0x82 + MemAddr + DataLen + sum(all_IDs)) & 0xFF` |

**Response:** Each servo responds individually with a standard response packet:
```
FF FF ID_i (DataLen+2) Status Data[0..DataLen-1] Checksum
```

The responses arrive sequentially in the order the IDs were listed. The host must read all responses from the bus. The total maximum response length is `NumServos * (DataLen + 6)` bytes.

---

## 7. Register Map (HLS Series)

### EPROM Registers (Read-Only)

| Address | Name       | Size   | Description |
|---------|------------|--------|-------------|
| 3       | MODEL_L    | 1 byte | Model number low byte |
| 4       | MODEL_H    | 1 byte | Model number high byte |

### EPROM Registers (Read/Write)

| Address | Name             | Size    | Description |
|---------|------------------|---------|-------------|
| 5       | ID               | 1 byte  | Servo ID (0–253) |
| 6       | BAUD_RATE        | 1 byte  | Baud rate index (0=1M, 1=500K, 2=250K, ...) |
| 7       | SECOND_ID        | 1 byte  | Secondary ID |
| 9–10    | MIN_ANGLE_LIMIT  | 2 bytes | Minimum angle limit (little-endian) |
| 11–12   | MAX_ANGLE_LIMIT  | 2 bytes | Maximum angle limit (little-endian) |
| 26      | CW_DEAD          | 1 byte  | Clockwise dead zone |
| 27      | CCW_DEAD         | 1 byte  | Counter-clockwise dead zone |
| 31–32   | OFS              | 2 bytes | Offset (little-endian) |
| 33      | MODE             | 1 byte  | Operating mode: 0=Servo, 1=Wheel(constant speed), 2=Ele(constant torque) |

### SRAM Registers (Read/Write)

| Address | Name             | Size    | Description |
|---------|------------------|---------|-------------|
| 40      | TORQUE_ENABLE    | 1 byte  | 0=disabled, 1=enabled |
| 41      | ACC              | 1 byte  | Acceleration |
| 42–43   | GOAL_POSITION    | 2 bytes | Target position (sign-magnitude, little-endian) |
| 44–45   | GOAL_TORQUE      | 2 bytes | Target torque (sign-magnitude, little-endian) |
| 46–47   | GOAL_SPEED       | 2 bytes | Target speed (sign-magnitude, little-endian) |
| 48–49   | TORQUE_LIMIT     | 2 bytes | Maximum torque limit (little-endian) |
| 55      | LOCK             | 1 byte  | EPROM lock: 0=unlocked, 1=locked |

### SRAM Registers (Read-Only / Feedback)

| Address | Name                | Size    | Encoding |
|---------|---------------------|---------|----------|
| 56–57   | PRESENT_POSITION    | 2 bytes | Sign-magnitude (bit 15 = sign) |
| 58–59   | PRESENT_SPEED       | 2 bytes | Sign-magnitude (bit 15 = sign) |
| 60–61   | PRESENT_LOAD        | 2 bytes | Sign-magnitude (bit 10 = sign) |
| 62      | PRESENT_VOLTAGE     | 1 byte  | Unsigned |
| 63      | PRESENT_TEMPERATURE | 1 byte  | Unsigned (°C) |
| 66      | MOVING              | 1 byte  | 0=stopped, 1=moving |
| 69–70   | PRESENT_CURRENT     | 2 bytes | Sign-magnitude (bit 15 = sign) |

### Sensor Block for Bulk Read

A contiguous block from register 56 to register 70 (15 bytes) can be read in one READ or SYNC_READ operation to get all feedback data at once.

Byte offsets within the 15-byte block:

| Offset | Register       | Size   |
|--------|----------------|--------|
| 0–1    | Position       | 2 bytes |
| 2–3    | Speed          | 2 bytes |
| 4–5    | Load           | 2 bytes |
| 6      | Voltage        | 1 byte  |
| 7      | Temperature    | 1 byte  |
| 8–9    | (unused/gap)   | 2 bytes |
| 10     | Moving         | 1 byte  |
| 11–12  | (unused/gap)   | 2 bytes |
| 13–14  | Current        | 2 bytes |

---

## 8. High-Level Command Sequences

### 8.1 WritePosEx — Position Command with Speed/Acc/Torque

Writes 7 bytes to register 41 (ACC), which covers registers 41–47 in a single write. This is the primary position control command.

**Data layout (7 bytes starting at register 41):**

| Byte | Register | Content |
|------|----------|---------|
| 0    | 41 (ACC) | Acceleration value (unsigned 8-bit) |
| 1    | 42 (GOAL_POSITION_L) | Position low byte |
| 2    | 43 (GOAL_POSITION_H) | Position high byte |
| 3    | 44 (GOAL_TORQUE_L) | Torque limit low byte |
| 4    | 45 (GOAL_TORQUE_H) | Torque limit high byte |
| 5    | 46 (GOAL_SPEED_L) | Speed low byte |
| 6    | 47 (GOAL_SPEED_H) | Speed high byte |

Position is sign-magnitude encoded before splitting into bytes.

### 8.2 WriteEle — Torque Control

Writes a sign-magnitude encoded torque value to register 44 (GOAL_TORQUE) as a 16-bit word.

### 8.3 Mode Switching

Write a single byte to register 33 (MODE):
- `0x00` → Servo mode (position control)
- `0x01` → Wheel mode (constant speed)
- `0x02` → Ele mode (constant torque)

### 8.4 EPROM Access

Before writing to EPROM registers (like ID, angle limits, etc.):
1. Disable torque: write `0x00` to register 40
2. Unlock EPROM: write `0x00` to register 55
3. Perform EPROM writes
4. Lock EPROM: write `0x01` to register 55
5. Re-enable torque: write `0x01` to register 40

### 8.5 Calibration

1. Disable torque: write `0x00` to register 40
2. Unlock EPROM: write `0x00` to register 55
3. Send CAL instruction (`0x0B`)

---

## 9. Baud Rate Table

| Index | Baud Rate |
|-------|-----------|
| 0     | 1,000,000 |
| 1     | 500,000   |
| 2     | 250,000   |
| 3     | 128,000   |
| 4     | 115,200   |
| 5     | 76,800    |
| 6     | 57,600    |
| 7     | 38,400    |
| 8     | 19,200    |
| 9     | 14,400    |
| 10    | 9,600     |
| 11    | 4,800     |

---

## 10. Example Byte Sequences

All examples use `End=0` (little-endian).

### Example 1: Ping Servo ID 3

**Command (6 bytes):**
```
FF FF 03 02 01 F9
```
- ID = 0x03
- Length = 0x02
- Instruction = 0x01 (PING)
- Checksum = ~(0x03 + 0x02 + 0x01) & 0xFF = ~0x06 & 0xFF = 0xF9

**Response (6 bytes, if servo is online):**
```
FF FF 03 02 00 FA
```
- ID = 0x03
- Length = 0x02
- Status = 0x00 (no error)
- Checksum = ~(0x03 + 0x02 + 0x00) & 0xFF = ~0x05 & 0xFF = 0xFA

---

### Example 2: Read 2 Bytes from Register 56 (Present Position) of Servo ID 1

**Command (8 bytes):**
```
FF FF 01 04 02 38 02 BE
```
- ID = 0x01
- Length = 0x04
- Instruction = 0x02 (READ)
- MemAddr = 0x38 (56 decimal)
- DataLen = 0x02 (2 bytes)
- Checksum = ~(0x01 + 0x04 + 0x02 + 0x38 + 0x02) & 0xFF = ~0x41 & 0xFF = 0xBE

**Response (8 bytes, if position = 2048 = 0x0800):**
```
FF FF 01 04 00 00 08 F2
```
- ID = 0x01
- Length = 0x04
- Status = 0x00
- Data = 0x00, 0x08 (little-endian: 0x0800 = 2048)
- Checksum = ~(0x01 + 0x04 + 0x00 + 0x00 + 0x08) & 0xFF = ~0x0D & 0xFF = 0xF2

---

### Example 3: Write Position 1000 with Speed 500 to Servo ID 5

Position = 1000 (positive, no sign bit) = 0x03E8
Speed = 500 = 0x01F4
ACC = 0
Torque limit = 0

**Data payload (7 bytes at register 41):**
```
Byte 0 (ACC):         0x00
Byte 1 (Pos low):     0xE8   (0x03E8 & 0xFF)
Byte 2 (Pos high):    0x03   (0x03E8 >> 8)
Byte 3 (Torque low):  0x00
Byte 4 (Torque high): 0x00
Byte 5 (Speed low):   0xF4   (0x01F4 & 0xFF)
Byte 6 (Speed high):  0x01   (0x01F4 >> 8)
```

**Command packet (14 bytes):**
```
FF FF 05 0A 03 29 00 E8 03 00 00 F4 01 E4
```
- ID = 0x05
- Length = 0x0A (7 data + 1 addr + 1 instruction + 1 checksum = 10)
- Instruction = 0x03 (WRITE)
- MemAddr = 0x29 (41 decimal)
- Data = 00 E8 03 00 00 F4 01
- Checksum = ~(0x05 + 0x0A + 0x03 + 0x29 + 0x00 + 0xE8 + 0x03 + 0x00 + 0x00 + 0xF4 + 0x01) & 0xFF
           = ~(0x21B) & 0xFF = ~0x1B & 0xFF = 0xE4

**Response (6 bytes):**
```
FF FF 05 02 00 F8
```

---

### Example 4: Write Negative Position -500 with Speed 300 to Servo ID 2

Position = -500 → sign-magnitude: `500 | 0x8000` = `0x81F4`
Speed = 300 = 0x012C
ACC = 10
Torque limit = 200 = 0x00C8

**Data payload (7 bytes):**
```
Byte 0 (ACC):         0x0A
Byte 1 (Pos low):     0xF4   (0x81F4 & 0xFF)
Byte 2 (Pos high):    0x81   (0x81F4 >> 8)
Byte 3 (Torque low):  0xC8   (0x00C8 & 0xFF)
Byte 4 (Torque high): 0x00
Byte 5 (Speed low):   0x2C   (0x012C & 0xFF)
Byte 6 (Speed high):  0x01
```

**Command packet (14 bytes):**
```
FF FF 02 0A 03 29 0A F4 81 C8 00 2C 01 53
```
- Checksum = ~(0x02 + 0x0A + 0x03 + 0x29 + 0x0A + 0xF4 + 0x81 + 0xC8 + 0x00 + 0x2C + 0x01) & 0xFF
           = ~(0x3AC) & 0xFF = ~0xAC & 0xFF = 0x53

---

### Example 5: Write Byte — Enable Torque on Servo ID 4

Write `0x01` to register 40 (TORQUE_ENABLE).

**Command packet (8 bytes):**
```
FF FF 04 04 03 28 01 CB
```
- ID = 0x04
- Length = 0x04
- Instruction = 0x03 (WRITE)
- MemAddr = 0x28 (40 decimal)
- Data = 0x01
- Checksum = ~(0x04 + 0x04 + 0x03 + 0x28 + 0x01) & 0xFF = ~0x34 & 0xFF = 0xCB

---

### Example 6: Set Mode to Ele (Torque) Mode on Servo ID 0

Write `0x02` to register 33 (MODE).

**Command packet (8 bytes):**
```
FF FF 00 04 03 21 02 D5
```
- Checksum = ~(0x00 + 0x04 + 0x03 + 0x21 + 0x02) & 0xFF = ~0x2A & 0xFF = 0xD5

---

### Example 7: WriteEle — Write Torque -300 to Servo ID 1

Torque = -300 → sign-magnitude: `300 | 0x8000` = `0x812C`
Write 16-bit word to register 44 (GOAL_TORQUE).

**Command packet (9 bytes):**
```
FF FF 01 05 03 2C 2C 81 1D
```
- ID = 0x01
- Length = 0x05
- Instruction = 0x03 (WRITE)
- MemAddr = 0x2C (44 decimal)
- Data = 0x2C, 0x81 (little-endian: low byte 0x2C first, high byte 0x81 second)
- Checksum = ~(0x01 + 0x05 + 0x03 + 0x2C + 0x2C + 0x81) & 0xFF = ~0xE2 & 0xFF = 0x1D

---

### Example 8: Sync Write Positions to Servos 0, 1, and 2

Servo 0: position=1000 (0x03E8), speed=500 (0x01F4), acc=0, torque=0
Servo 1: position=2048 (0x0800), speed=300 (0x012C), acc=5, torque=100 (0x0064)
Servo 2: position=3000 (0x0BB8), speed=400 (0x0190), acc=10, torque=0

DataLen = 7 (bytes per servo)
NumServos = 3
Length = (7 + 1) * 3 + 4 = 28 = 0x1C

**Command packet:**
```
FF FF FE 1C 83 29 07
  00 00 E8 03 00 00 F4 01
  01 05 00 08 64 00 2C 01
  02 0A B8 0B 00 00 90 01
  Checksum
```

Breakdown:
- Header: `FF FF FE`
- Length: `1C` (28)
- Instruction: `83` (SYNC_WRITE)
- MemAddr: `29` (41)
- DataLen: `07` (7)
- Servo 0: `00` `00 E8 03 00 00 F4 01`
- Servo 1: `01` `05 00 08 64 00 2C 01`
- Servo 2: `02` `0A B8 0B 00 00 90 01`
- Checksum = ~(0xFE + 0x1C + 0x83 + 0x29 + 0x07 + 0x00 + 0x00 + 0xE8 + 0x03 + 0x00 + 0x00 + 0xF4 + 0x01 + 0x01 + 0x05 + 0x00 + 0x08 + 0x64 + 0x00 + 0x2C + 0x01 + 0x02 + 0x0A + 0xB8 + 0x0B + 0x00 + 0x00 + 0x90 + 0x01) & 0xFF
           = ~(0x5AC) & 0xFF = ~0xAC & 0xFF = 0x53

Full packet:
```
FF FF FE 1C 83 29 07 00 00 E8 03 00 00 F4 01 01 05 00 08 64 00 2C 01 02 0A B8 0B 00 00 90 01 53
```

**No response** (broadcast command).

---

### Example 9: Sync Read 15 Bytes (Sensor Block) from Servos 0, 1, 2

MemAddr = 56 (0x38), DataLen = 15 (0x0F), NumServos = 3
Length = 3 + 4 = 7 = 0x07

**Command packet (12 bytes):**
```
FF FF FE 07 82 38 0F 00 01 02 Checksum
```
- Checksum = ~(0xFE + 0x07 + 0x82 + 0x38 + 0x0F + 0x00 + 0x01 + 0x02) & 0xFF
           = ~(0x1D1) & 0xFF = ~0xD1 & 0xFF = 0x2E

Full command:
```
FF FF FE 07 82 38 0F 00 01 02 2E
```

**Response** (3 individual response packets, one per servo):

Each servo responds with 21 bytes:
```
FF FF ID 11 Status [15 bytes of sensor data] Checksum
```
- Length = 0x11 (15 + 2 = 17)

For example, servo 0 responding with position=2048 (0x0800), speed=0, load=0, voltage=75 (7.5V), temp=35°C, moving=0, current=50 (0x0032):
```
FF FF 00 11 00 00 08 00 00 00 00 4B 23 00 00 00 00 00 32 00 41
```

Byte layout within the 15-byte data:
```
Offset 0-1:   Position  = 00 08 → 0x0800 = 2048
Offset 2-3:   Speed     = 00 00 → 0
Offset 4-5:   Load      = 00 00 → 0
Offset 6:     Voltage   = 4B → 75
Offset 7:     Temp      = 23 → 35
Offset 8-9:   (gap)
Offset 10:    Moving    = 00 → 0
Offset 11-12: (gap)
Offset 13-14: Current   = 32 00 → 0x0032 = 50
```

Total response bytes for 3 servos: up to 63 bytes (3 × 21).

---

### Example 10: CAL (Calibrate Midpoint) on Servo ID 6

**Command (6 bytes):**
```
FF FF 06 02 0B EC
```
- Checksum = ~(0x06 + 0x02 + 0x0B) & 0xFF = ~0x13 & 0xFF = 0xEC

**Response (6 bytes):**
```
FF FF 06 02 00 F7
```

---

### Example 11: Full EPROM Unlock → Write ID → Lock Sequence

Changing servo ID from 1 to 5:

**Step 1: Disable torque** — write 0x00 to register 40 on servo 1:
```
FF FF 01 04 03 28 00 CF
```

**Step 2: Unlock EPROM** — write 0x00 to register 55 on servo 1:
```
FF FF 01 04 03 37 00 C0
```

**Step 3: Write new ID** — write 0x05 to register 5 on servo 1:
```
FF FF 01 04 03 05 05 ED
```

**Step 4: Lock EPROM** — write 0x01 to register 55 on (now) servo 5:
```
FF FF 05 04 03 37 01 BB
```

**Step 5: Re-enable torque** — write 0x01 to register 40 on servo 5:
```
FF FF 05 04 03 28 01 CA
```
