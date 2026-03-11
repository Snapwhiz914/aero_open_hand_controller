"""sim_log_analyze.py — Analyze HandSimulator log files for bus/protocol anomalies.

Usage:
    python sim_log_analyze.py  sim_20260228_134031.log  [--servo N] [--csv out.csv]

The log format written by HandSimulator is:

    HH:MM:SS.mmm  TX       FF FF FE 0B 82 38 0F 00 01 02 03 04 05 06 4B
    HH:MM:SS.mmm  RX       FF FF 00 11 00 [15 data bytes] CK
    HH:MM:SS.mmm  DROPPED  servo=2  instr=SYNC_READ  rule="servo_id=2"

A SYNC_READ transaction = one TX line followed by per-servo RX/DROPPED lines.
A SYNC_WRITE produces only a TX line (broadcast — no servo responses).
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import dataclass, field
from statistics import median
from typing import Optional


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

INST_SYNC_READ  = 0x82
INST_SYNC_WRITE = 0x83
INST_PING       = 0x01
INST_READ       = 0x02
INST_WRITE      = 0x03

BROADCAST_ID = 0xFE

TEMP_CUTOFF_C   = 70          # thermal threshold (hand/hand.py)
FLOOD_THRESHOLD = 100         # TX packets/s
DROPOUT_WINDOW  = 5.0         # seconds for rolling response-rate check
DROPOUT_MIN_PCT = 0.90        # below this → SERVO_DROPOUT

# Sensor block offsets (relative to response payload at byte 5 of the full packet)
SENSOR_POSITION_OFF   = 0
SENSOR_SPEED_OFF      = 2
SENSOR_LOAD_OFF       = 4
SENSOR_VOLTAGE_OFF    = 6
SENSOR_TEMP_OFF       = 7
SENSOR_MOVING_OFF     = 10
SENSOR_CURRENT_OFF    = 13
SENSOR_BLOCK_LEN      = 15

# Regex matching one log line
_LINE_RE = re.compile(
    r'^(\d{2}:\d{2}:\d{2}\.\d{3})'   # group 1: timestamp
    r'\s{2}(TX|RX|DROPPED)'           # group 2: direction
    r'\s{2,7}(.*)$'                   # group 3: remainder
)

# Regex for DROPPED fields
_DROPPED_RE = re.compile(
    r'servo=(\d+)\s+instr=(\S+)\s+rule="([^"]*)"'
)


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class LogLine:
    ts: float           # seconds since midnight
    direction: str      # 'TX' | 'RX' | 'DROPPED'
    data: Optional[bytes]   # parsed hex (None for DROPPED)
    meta: dict = field(default_factory=dict)  # DROPPED fields


@dataclass
class Transaction:
    """One SYNC_READ request + its per-servo responses."""
    ts: float
    tx_bytes: bytes
    requested_ids: list[int]
    responses: dict[int, bytes]    # servo_id → raw response packet bytes
    dropped: list[dict]            # list of DROPPED meta dicts
    missing: list[int]             # in requested_ids but absent from both


@dataclass
class ControlEvent:
    """Non-SYNC_READ TX line (SYNC_WRITE, WRITE, PING, …)."""
    ts: float
    instruction: int
    raw: bytes


# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 — Parse log lines
# ─────────────────────────────────────────────────────────────────────────────

def _ts_to_sec(ts_str: str) -> float:
    """Convert 'HH:MM:SS.mmm' → float seconds since midnight."""
    h, m, rest = ts_str.split(":")
    s, ms = rest.split(".")
    return int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000.0


def parse_log(path: str) -> list[LogLine]:
    lines: list[LogLine] = []
    with open(path, "r") as fh:
        for raw in fh:
            raw = raw.rstrip("\n")
            m = _LINE_RE.match(raw)
            if not m:
                continue
            ts_str, direction, remainder = m.group(1), m.group(2), m.group(3).strip()
            ts = _ts_to_sec(ts_str)

            if direction in ("TX", "RX"):
                try:
                    data = bytes(int(x, 16) for x in remainder.split())
                except ValueError:
                    data = b""
                lines.append(LogLine(ts=ts, direction=direction, data=data))
            else:  # DROPPED
                dm = _DROPPED_RE.search(remainder)
                meta: dict = {}
                if dm:
                    meta = {
                        "servo": int(dm.group(1)),
                        "instr": dm.group(2),
                        "rule":  dm.group(3),
                    }
                lines.append(LogLine(ts=ts, direction="DROPPED", data=None, meta=meta))

    return lines


# ─────────────────────────────────────────────────────────────────────────────
# Phase 2 — Reconstruct transactions
# ─────────────────────────────────────────────────────────────────────────────

def _is_sync_read_tx(data: bytes) -> bool:
    """Return True if this TX packet is a SYNC_READ to broadcast."""
    return (len(data) >= 5 and
            data[0] == 0xFF and data[1] == 0xFF and
            data[2] == BROADCAST_ID and
            data[4] == INST_SYNC_READ)


def _is_sync_write_tx(data: bytes) -> bool:
    return (len(data) >= 5 and
            data[0] == 0xFF and data[1] == 0xFF and
            data[4] == INST_SYNC_WRITE)


def _parse_sync_read_ids(tx_data: bytes) -> list[int]:
    """Extract requested servo IDs from a SYNC_READ TX packet."""
    # Packet: FF FF FE len 82  addr data_len  id0 id1 ...  ck
    if len(tx_data) < 8:
        return []
    # params start at byte 5, first 2 are addr+data_len, rest are IDs
    params = tx_data[5:-1]   # strip checksum
    if len(params) < 3:
        return []
    return list(params[2:])


def _parse_rx_servo_id(rx_data: bytes) -> Optional[int]:
    """Extract servo ID from an RX response packet."""
    if len(rx_data) >= 3 and rx_data[0] == 0xFF and rx_data[1] == 0xFF:
        return rx_data[2]
    return None


def _sensor_payload(rx_data: bytes) -> Optional[bytes]:
    """Extract the sensor data payload (bytes after header/error, before checksum)."""
    # RX: FF FF ID len status data... ck
    if len(rx_data) < 6:
        return None
    payload_len = rx_data[3] - 2   # length field includes status + checksum
    payload = rx_data[5: 5 + payload_len]
    if len(payload) < payload_len:
        return None
    return payload


def reconstruct(lines: list[LogLine]) -> tuple[list[Transaction], list[ControlEvent]]:
    transactions: list[Transaction] = []
    control_events: list[ControlEvent] = []

    i = 0
    while i < len(lines):
        line = lines[i]

        if line.direction != "TX" or not line.data:
            i += 1
            continue

        tx = line.data

        # ── SYNC_READ transaction ─────────────────────────────────────────────
        if _is_sync_read_tx(tx):
            requested_ids = _parse_sync_read_ids(tx)
            responses: dict[int, bytes] = {}
            dropped_list: list[dict] = []

            i += 1
            # Consume following RX/DROPPED lines that belong to this transaction
            while i < len(lines) and lines[i].direction in ("RX", "DROPPED"):
                resp_line = lines[i]
                if resp_line.direction == "RX" and resp_line.data:
                    sid = _parse_rx_servo_id(resp_line.data)
                    if sid is not None:
                        responses[sid] = resp_line.data
                elif resp_line.direction == "DROPPED":
                    dropped_list.append(resp_line.meta)
                i += 1

            dropped_ids = {d["servo"] for d in dropped_list if "servo" in d}
            missing = [sid for sid in requested_ids
                       if sid not in responses and sid not in dropped_ids]

            # Last response timestamp for latency calculation
            # Find it by scanning backwards through the consumed lines
            last_rx_ts = line.ts
            for j in range(i - 1, i - 1 - len(requested_ids) - 1, -1):
                if j < 0:
                    break
                if lines[j].direction in ("RX", "DROPPED"):
                    last_rx_ts = lines[j].ts
                    break

            transactions.append(Transaction(
                ts=line.ts,
                tx_bytes=tx,
                requested_ids=requested_ids,
                responses=responses,
                dropped=dropped_list,
                missing=missing,
            ))

        # ── SYNC_WRITE or other control event ─────────────────────────────────
        else:
            instr = tx[4] if len(tx) >= 5 else 0
            control_events.append(ControlEvent(ts=line.ts, instruction=instr, raw=tx))
            i += 1

    return transactions, control_events


# ─────────────────────────────────────────────────────────────────────────────
# Phase 3 — Decode sensor block
# ─────────────────────────────────────────────────────────────────────────────

def _sign_magnitude(word: int) -> int:
    """Feetech sign-magnitude 16-bit → signed Python int."""
    sign = (word >> 15) & 1
    mag  = word & 0x7FFF
    return -mag if sign else mag


def _read16_le(data: bytes, offset: int) -> int:
    if offset + 1 >= len(data):
        return 0
    return data[offset] | (data[offset + 1] << 8)


def decode_sensor(payload: bytes) -> dict:
    """Decode the 15-byte sensor block into a dict of named values."""
    if len(payload) < SENSOR_BLOCK_LEN:
        return {}
    pos     = _sign_magnitude(_read16_le(payload, SENSOR_POSITION_OFF))
    speed   = _sign_magnitude(_read16_le(payload, SENSOR_SPEED_OFF))
    load    = _sign_magnitude(_read16_le(payload, SENSOR_LOAD_OFF))
    voltage = payload[SENSOR_VOLTAGE_OFF] if SENSOR_VOLTAGE_OFF < len(payload) else 0
    temp    = payload[SENSOR_TEMP_OFF]    if SENSOR_TEMP_OFF    < len(payload) else 0
    moving  = payload[SENSOR_MOVING_OFF]  if SENSOR_MOVING_OFF  < len(payload) else 0
    current = _sign_magnitude(_read16_le(payload, SENSOR_CURRENT_OFF))
    return {
        "position": pos,
        "speed":    speed,
        "load":     load,
        "voltage":  voltage / 10.0,
        "temp":     temp,
        "moving":   moving,
        "current":  current,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Phase 3 — Compute metrics
# ─────────────────────────────────────────────────────────────────────────────

def compute_metrics(transactions: list[Transaction],
                    control_events: list[ControlEvent]) -> dict:
    if not transactions:
        return {}

    t0 = transactions[0].ts
    t_end = max(transactions[-1].ts,
                control_events[-1].ts if control_events else transactions[-1].ts)
    duration = t_end - t0

    # Per-transaction latency: time from TX to last expected response
    # We approximate as (next TX ts - this TX ts) clipped to reasonable range,
    # but we don't store per-line timestamps inside the transaction structure.
    # Instead, latency = gap between consecutive TX lines as a proxy.
    latencies: list[float] = []
    for idx in range(len(transactions) - 1):
        gap_ms = (transactions[idx + 1].ts - transactions[idx].ts) * 1000
        if 0 < gap_ms < 500:   # ignore unreasonable gaps
            latencies.append(gap_ms)

    latency_p50 = median(latencies) if latencies else 0.0
    latency_p99 = sorted(latencies)[int(len(latencies) * 0.99)] if latencies else 0.0

    # TX rate: rolling 1-s windows
    tx_times = [t.ts for t in transactions]
    peak_rate = 0
    for idx, ts in enumerate(tx_times):
        window = [t for t in tx_times if ts - 1.0 < t <= ts]
        peak_rate = max(peak_rate, len(window))

    avg_rate = len(transactions) / duration if duration > 0 else 0.0

    # Collect all servo IDs seen across transactions
    all_sids: set[int] = set()
    for t in transactions:
        all_sids.update(t.requested_ids)

    # Per-servo response counts
    servo_responded: dict[int, int] = {sid: 0 for sid in all_sids}
    servo_dropped:   dict[int, int] = {sid: 0 for sid in all_sids}
    servo_missing:   dict[int, int] = {sid: 0 for sid in all_sids}

    for t in transactions:
        for sid in t.requested_ids:
            if sid in t.responses:
                servo_responded[sid] = servo_responded.get(sid, 0) + 1
            else:
                if any(d.get("servo") == sid for d in t.dropped):
                    servo_dropped[sid] = servo_dropped.get(sid, 0) + 1
                else:
                    servo_missing[sid] = servo_missing.get(sid, 0) + 1

    total_per_servo: dict[int, int] = {}
    for sid in all_sids:
        total_per_servo[sid] = (servo_responded.get(sid, 0) +
                                servo_dropped.get(sid, 0) +
                                servo_missing.get(sid, 0))

    # Temperature history and peaks per servo
    temp_peaks: dict[int, int] = {}
    pos_history: dict[int, list[tuple[float, int]]] = {sid: [] for sid in all_sids}

    for t in transactions:
        for sid, rx_data in t.responses.items():
            payload = _sensor_payload(rx_data)
            if payload:
                s = decode_sensor(payload)
                if s:
                    temp_peaks[sid] = max(temp_peaks.get(sid, 0), s["temp"])
                    pos_history.setdefault(sid, []).append((t.ts, s["position"]))

    # Control event counts
    sync_write_count = sum(1 for e in control_events if e.instruction == INST_SYNC_WRITE)
    ping_count       = sum(1 for e in control_events if e.instruction == INST_PING)

    return {
        "duration":         duration,
        "t0":               t0,
        "n_sync_read":      len(transactions),
        "n_sync_write":     sync_write_count,
        "n_ping":           ping_count,
        "avg_rate_hz":      avg_rate,
        "peak_rate_hz":     peak_rate,
        "latency_p50":      latency_p50,
        "latency_p99":      latency_p99,
        "all_sids":         sorted(all_sids),
        "servo_responded":  servo_responded,
        "servo_dropped":    servo_dropped,
        "servo_missing":    servo_missing,
        "total_per_servo":  total_per_servo,
        "temp_peaks":       temp_peaks,
        "pos_history":      pos_history,
        "latencies":        latencies,
        "tx_times":         tx_times,
    }


# ─────────────────────────────────────────────────────────────────────────────
# Phase 4 — Anomaly detection
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Anomaly:
    label: str
    ts: Optional[float]   # None for global advisories
    detail: str


def detect_anomalies(transactions: list[Transaction],
                     control_events: list[ControlEvent],
                     metrics: dict,
                     lines: list[LogLine]) -> list[Anomaly]:
    anomalies: list[Anomaly] = []
    t0 = metrics.get("t0", 0.0)
    all_sids = metrics.get("all_sids", [])

    # ── FLOOD_WARNING: TX rate > 100 pkt/s in any 1-s window ─────────────────
    tx_times = metrics.get("tx_times", [])
    for idx, ts in enumerate(tx_times):
        window = [t for t in tx_times if ts - 1.0 < t <= ts]
        if len(window) > FLOOD_THRESHOLD:
            anomalies.append(Anomaly(
                label="FLOOD_WARNING",
                ts=ts,
                detail=f"TX rate {len(window)} pkt/s in 1-s window (threshold={FLOOD_THRESHOLD})",
            ))
            break   # report first occurrence only

    # ── SERVO_DROPOUT: rolling 5-s response rate < 90 % ─────────────────────
    # For each transaction, compute the rolling 5-s window ending at that point
    for sid in all_sids:
        dropout_reported = False
        for idx, t in enumerate(transactions):
            if dropout_reported:
                break
            window_start = t.ts - DROPOUT_WINDOW
            window_txns = [tx for tx in transactions
                           if window_start <= tx.ts <= t.ts and sid in tx.requested_ids]
            if len(window_txns) < 10:   # not enough data yet
                continue
            responded = sum(1 for tx in window_txns if sid in tx.responses)
            rate = responded / len(window_txns)
            if rate < DROPOUT_MIN_PCT:
                anomalies.append(Anomaly(
                    label="SERVO_DROPOUT",
                    ts=t.ts,
                    detail=f"servo={sid}  response rate {rate:.0%} over {DROPOUT_WINDOW:.0f}-s window",
                ))
                dropout_reported = True

    # ── SILENT_DROPOUT: missing (not in responses AND not in dropped) ─────────
    for t in transactions:
        if t.missing:
            for sid in t.missing:
                anomalies.append(Anomaly(
                    label="SILENT_DROPOUT",
                    ts=t.ts,
                    detail=f"servo={sid}  absent from both RX and DROPPED lines",
                ))
            break   # report first occurrence only

    # ── THERMAL: any servo ≥ 70 °C ────────────────────────────────────────────
    for sid, peak in metrics.get("temp_peaks", {}).items():
        if peak >= TEMP_CUTOFF_C:
            anomalies.append(Anomaly(
                label="THERMAL",
                ts=None,
                detail=f"servo={sid}  peak temperature {peak} °C ≥ {TEMP_CUTOFF_C} °C",
            ))

    # ── TIMEOUT: mismatch > 2 between requested and (responded+dropped) ───────
    for t in transactions:
        n_missing = len(t.missing)
        if n_missing > 2:
            anomalies.append(Anomaly(
                label="TIMEOUT",
                ts=t.ts,
                detail=f"{n_missing} servos missing (not dropped, not responded) in one transaction",
            ))
            break   # report first occurrence only

    # ── DROP_RULE_INACTIVE: SYNC_WRITE events but no DROPPED for SYNC_WRITE ──
    has_sync_write = metrics.get("n_sync_write", 0) > 0
    dropped_sync_write = any(
        line.direction == "DROPPED" and line.meta.get("instr") == "SYNC_WRITE"
        for line in lines
    )
    if has_sync_write and not dropped_sync_write:
        # Check if any SYNC_WRITE drop rules appear in the DROPPED lines' rules
        # Even if there are no SYNC_WRITE drops, we flag this as suspicious
        # (the _handle_sync_write bug: it never calls _should_drop)
        anomalies.append(Anomaly(
            label="DROP_RULE_INACTIVE",
            ts=None,
            detail=(
                "SYNC_WRITE packets present but no DROPPED lines emitted for SYNC_WRITE — "
                "_handle_sync_write does not call _should_drop; "
                "SYNC_WRITE drop rules are silently ignored"
            ),
        ))

    return anomalies


def _fmt_ts(ts: Optional[float]) -> str:
    if ts is None:
        return "--"
    h = int(ts // 3600)
    m = int((ts % 3600) // 60)
    s = ts % 60
    return f"{h:02d}:{m:02d}:{s:06.3f}"


# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — Report output
# ─────────────────────────────────────────────────────────────────────────────

def print_report(path: str,
                 transactions: list[Transaction],
                 control_events: list[ControlEvent],
                 metrics: dict,
                 anomalies: list[Anomaly],
                 filter_servo: Optional[int] = None):
    all_sids = metrics.get("all_sids", [])
    total_per_servo = metrics.get("total_per_servo", {})
    servo_responded = metrics.get("servo_responded", {})
    temp_peaks      = metrics.get("temp_peaks", {})

    print(f"\n=== sim_log_analyze: {path} ===")
    print()
    print(f"  Duration        : {metrics.get('duration', 0):.1f} s")
    print(f"  Transactions    : "
          f"{metrics.get('n_sync_read', 0)} SYNC_READ, "
          f"{metrics.get('n_sync_write', 0)} SYNC_WRITE, "
          f"{metrics.get('n_ping', 0)} PING")
    print(f"  TX rate (avg)   : {metrics.get('avg_rate_hz', 0):.1f} pkt/s   "
          f"peak: {metrics.get('peak_rate_hz', 0):.0f} pkt/s")
    print(f"  Latency         : "
          f"p50={metrics.get('latency_p50', 0):.0f} ms  "
          f"p99={metrics.get('latency_p99', 0):.0f} ms")
    print()

    # Per-servo response rates
    print("  Per-servo response rate:")
    row: list[str] = []
    dropout_sids: set[int] = set()
    for sid in all_sids:
        total = total_per_servo.get(sid, 0)
        resp  = servo_responded.get(sid, 0)
        pct   = (resp / total * 100) if total else 0.0
        flag  = " ← SERVO_DROPOUT" if pct < (DROPOUT_MIN_PCT * 100) else ""
        if pct < (DROPOUT_MIN_PCT * 100):
            dropout_sids.add(sid)
        row.append(f"Servo {sid}: {pct:3.0f} %{flag}")

    # Print 3 per row
    for start in range(0, len(row), 3):
        print("    " + "     ".join(row[start:start + 3]))
    print()

    # Anomalies
    if anomalies:
        print("  Anomalies:")
        for a in anomalies:
            ts_str = _fmt_ts(a.ts)
            print(f"    [{ts_str}] {a.label}  {a.detail}")
        print()
    else:
        print("  No anomalies detected.")
        print()

    # Temperature peaks
    if temp_peaks:
        print("  Temperature peaks (°C):")
        temp_parts = []
        for sid in all_sids:
            peak = temp_peaks.get(sid, "n/a")
            extra = " (last known before dropout)" if sid in dropout_sids else ""
            temp_parts.append(f"Servo {sid}: {peak}{extra}")
        print("    " + "   ".join(temp_parts))
        print()

    # Summary of anomaly labels seen
    labels = {a.label for a in anomalies}
    for expected in ("FLOOD_WARNING", "TIMEOUT"):
        if expected not in labels:
            print(f"  No {expected} anomalies.")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# CSV export
# ─────────────────────────────────────────────────────────────────────────────

def export_csv(path: str,
               transactions: list[Transaction],
               metrics: dict):
    all_sids = sorted(metrics.get("all_sids", []))
    tx_times = metrics.get("tx_times", [])

    # Pre-compute rolling 1-s rate at each transaction timestamp
    rate_at: list[float] = []
    for ts in tx_times:
        window = sum(1 for t in tx_times if ts - 1.0 < t <= ts)
        rate_at.append(window)

    # Approximate latency per transaction
    latencies = metrics.get("latencies", [])

    with open(path, "w", newline="") as fh:
        header = ["ts", "tx_rate_hz", "latency_ms"]
        for sid in all_sids:
            header += [f"servo{sid}_pos", f"servo{sid}_temp", f"servo{sid}_responded"]
        writer = csv.DictWriter(fh, fieldnames=header)
        writer.writeheader()

        for idx, t in enumerate(transactions):
            row: dict = {
                "ts":          f"{t.ts:.3f}",
                "tx_rate_hz":  f"{rate_at[idx]:.1f}",
                "latency_ms":  f"{latencies[idx]:.1f}" if idx < len(latencies) else "",
            }
            for sid in all_sids:
                responded = sid in t.responses
                pos  = ""
                temp = ""
                if responded:
                    payload = _sensor_payload(t.responses[sid])
                    if payload:
                        s = decode_sensor(payload)
                        if s:
                            pos  = s["position"]
                            temp = s["temp"]
                row[f"servo{sid}_pos"]       = pos
                row[f"servo{sid}_temp"]      = temp
                row[f"servo{sid}_responded"] = int(responded)
            writer.writerow(row)

    print(f"  CSV written to: {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Analyze HandSimulator log files for bus and protocol anomalies.")
    parser.add_argument("log", help="Path to sim_YYYYMMDD_HHMMSS.log file")
    parser.add_argument("--servo", type=int, default=None,
                        help="Filter output to a single servo ID")
    parser.add_argument("--csv", metavar="OUT.CSV", default=None,
                        help="Export per-transaction rows to CSV")
    args = parser.parse_args()

    print(f"Parsing {args.log} …", end=" ", flush=True)
    lines = parse_log(args.log)
    print(f"{len(lines)} lines", flush=True)

    transactions, control_events = reconstruct(lines)

    if not transactions:
        print("No SYNC_READ transactions found.  Nothing to analyze.")
        sys.exit(0)

    metrics   = compute_metrics(transactions, control_events)
    anomalies = detect_anomalies(transactions, control_events, metrics, lines)

    print_report(args.log, transactions, control_events, metrics, anomalies,
                 filter_servo=args.servo)

    if args.csv:
        export_csv(args.csv, transactions, metrics)


if __name__ == "__main__":
    main()
