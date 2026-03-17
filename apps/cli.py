#!/usr/bin/env python3
"""
cli.py  —  Interactive curses CLI for the Aero Open Hand Controller.

Usage:
    python cli.py [--hand left|right] [--verbose]

Windows note: install windows-curses if curses is not found.
    pip install pyserial windows-curses
"""

import argparse
import curses
import datetime
import os
import sys
import threading
import time
from collections import deque

import serial.tools.list_ports

from hand import Hand
from hand.ttl import HandTTL
from hand.sdk import HandESP32

NUM_SERVOS = 7

_LOG_DIR = os.path.join("runtime", "logs")

# ─────────────────────────────────────────────────────────────────────────────
# Serial logger
# ─────────────────────────────────────────────────────────────────────────────

def _now_ms() -> str:
    """Current wall-clock time as HH:MM:SS.mmm."""
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class LoggingSerial:
    """
    Transparent proxy around a serial.Serial instance.

    Every write() and read() is forwarded to the real port; the byte
    payloads are also handed to the supplied TX / RX callbacks so the
    CLI can display them live.
    """

    def __init__(self, real_serial, on_tx, on_rx):
        # Store on the object dict directly to avoid triggering __setattr__
        object.__setattr__(self, "_s",     real_serial)
        object.__setattr__(self, "_on_tx", on_tx)
        object.__setattr__(self, "_on_rx", on_rx)

    # Intercept write / read; everything else proxies automatically.

    def write(self, data):
        result = self._s.write(data)
        if data:
            self._on_tx(bytes(data))
        return result

    def read(self, size=1):
        data = self._s.read(size)
        if data:
            self._on_rx(data)
        return data

    # Attribute proxy ─────────────────────────────────────────────────────────

    def __getattr__(self, name):
        # Called for any attribute not found on LoggingSerial itself
        return getattr(self._s, name)

    def __setattr__(self, name, value):
        # All attribute sets (e.g. .timeout) go to the real serial
        setattr(self._s, name, value)


# ─────────────────────────────────────────────────────────────────────────────
# Port selection  (plain terminal, before curses starts)
# ─────────────────────────────────────────────────────────────────────────────

def select_port() -> "str | None":
    ports = list(serial.tools.list_ports.comports())

    print("\nAvailable serial ports:")
    if ports:
        for i, p in enumerate(ports):
            desc = p.description or "unknown"
            print(f"  [{i + 1}]  {p.device:<16}  {desc}")
    else:
        print("  (none detected)")
    print("  [0]  Enter port name manually")

    while True:
        try:
            raw = input("\nSelect port: ").strip()
        except (EOFError, KeyboardInterrupt):
            return None

        if raw == "0":
            try:
                manual = input("Port name (e.g. COM3 or /dev/ttyUSB0): ").strip()
            except (EOFError, KeyboardInterrupt):
                return None
            if manual:
                return manual
        else:
            try:
                idx = int(raw) - 1
                if 0 <= idx < len(ports):
                    return ports[idx].device
            except ValueError:
                pass
        print("  Invalid — try again.")


# ─────────────────────────────────────────────────────────────────────────────
# Menu definition
# ─────────────────────────────────────────────────────────────────────────────

MENU = [
    ("1", "Set position  —  position mode        (single servo)"),
    ("2", "Set position  —  raw torque mode      (single servo)"),
    ("3", "Set position  —  PID torque mode      (single servo)"),
    ("4", "Set PID constants                     (single servo)"),
    ("5", "Show sensor readings"),
    ("6", "Home hand"),
    ("7", "Trim servo"),
    ("8", "Set servo ID"),
    ("9", "Ping servos"),
    ("v", "Toggle verbose serial log"),
    ("q", "Quit"),
]

# How many log entries to keep in the ring buffer
LOG_BUF_SIZE = 500
# Max rows the serial log panel will occupy
LOG_PANEL_MAX = 14


# ─────────────────────────────────────────────────────────────────────────────
# Main TUI class
# ─────────────────────────────────────────────────────────────────────────────

class HandCLI:
    REFRESH_S = 0.4

    def __init__(self, hand: Hand, port: str, hand_type: str,
                 verbose: bool = False):
        self.hand      = hand
        self.port      = port
        self.hand_type = hand_type

        # Last-commanded state — lets single-servo commands leave others alone
        self._cmd_positions = [32767] * NUM_SERVOS  # mid-range default
        self._cmd_torques   = [0]     * NUM_SERVOS
        self._cmd_pid_pos   = [32767] * NUM_SERVOS

        self._status      = "Ready.  Press a key to run a command."
        self._status_lock = threading.Lock()

        # Serial log
        self._verbose   = verbose
        self._log_buf   = deque(maxlen=LOG_BUF_SIZE)
        self._log_lock  = threading.Lock()

    # ── Serial log callbacks ─────────────────────────────────────────────────

    def _log_tx(self, data: bytes):
        entry = (_now_ms(), "TX", _hex(data))
        with self._log_lock:
            self._log_buf.append(entry)

    def _log_rx(self, data: bytes):
        entry = (_now_ms(), "RX", _hex(data))
        with self._log_lock:
            self._log_buf.append(entry)

    def inject_logger(self):
        """Replace the Hand's serial object with a LoggingSerial wrapper."""
        real = self.hand._protocol._serial
        self.hand._protocol._serial = LoggingSerial(real, self._log_tx, self._log_rx)

    # ── Entry point ──────────────────────────────────────────────────────────

    def run(self, stdscr: "curses._CursesWindow"):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)

        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_CYAN,   -1)  # header
            curses.init_pair(2, curses.COLOR_GREEN,  -1)  # TX / normal sensor
            curses.init_pair(3, curses.COLOR_YELLOW, -1)  # RX / menu hints
            curses.init_pair(4, curses.COLOR_RED,    -1)  # hot / error
            curses.init_pair(5, curses.COLOR_WHITE,  -1)  # log panel label

        last_draw = 0.0
        while True:
            now = time.monotonic()
            if now - last_draw >= self.REFRESH_S:
                self._draw(stdscr)
                last_draw = now

            key = stdscr.getch()
            if key == -1:
                time.sleep(0.05)
                continue

            ch = chr(key).lower() if 0 < key < 256 else ""

            if   ch == "q": break
            elif ch == "v": self._verbose = not self._verbose
            elif ch == "1": self._set_position_mode(stdscr)
            elif ch == "2": self._set_raw_torque(stdscr)
            elif ch == "3": self._set_pid_torque(stdscr)
            elif ch == "4": self._set_pid_constants(stdscr)
            elif ch == "5": self._show_sensors(stdscr)
            elif ch == "6": self._home(stdscr)
            elif ch == "7": self._trim(stdscr)
            elif ch == "8": self._set_id(stdscr)
            elif ch == "9": self._ping(stdscr)

    # ── Drawing ──────────────────────────────────────────────────────────────

    def _draw(self, stdscr):
        try:
            stdscr.erase()
            h, w = stdscr.getmaxyx()
            row = 0

            # ── Header bar ───────────────────────────────────────────────────
            hdr = (f" Aero Open Hand  |  {self.port}  |  {self.hand_type} hand"
                   f"{'  |  [v] verbose ON' if self._verbose else ''}")
            self._put(stdscr, row, 0,
                      hdr.ljust(w - 1)[:w - 1],
                      curses.color_pair(1) | curses.A_BOLD)
            row += 1
            self._put(stdscr, row, 0, "-" * (w - 1))
            row += 1

            # ── Sensor table ─────────────────────────────────────────────────
            self._put(stdscr, row, 0, " LIVE SENSORS", curses.A_BOLD)
            row += 1
            col_hdr = (f"  {'Srv':>3}  {'Position':>8}  {'Velocity':>8}"
                       f"  {'Temp':>6}  {'Current':>8}")
            self._put(stdscr, row, 0, col_hdr[:w - 1])
            row += 1
            self._put(stdscr, row, 0, "-" * (w - 1))
            row += 1

            pos  = self.hand.get_positions()
            vel  = self.hand.get_velocities()
            temp = self.hand.get_temperatures()
            cur  = self.hand.get_currents()

            for i in range(NUM_SERVOS):
                if row >= h - 2:
                    break
                t    = temp[i]
                attr = curses.color_pair(4) | curses.A_BOLD if t >= 70 else 0
                line = (f"  {i:>3}  {pos[i]:>8}  {vel[i]:>8}"
                        f"  {t:>4}C   {cur[i]:>8}")
                self._put(stdscr, row, 0, line[:w - 1], attr)
                row += 1

            self._put(stdscr, row, 0, "-" * (w - 1))
            row += 1

            # ── Serial log panel (only when verbose) ─────────────────────────
            if self._verbose:
                # How many log rows can we fit?
                # Reserve: separator(1) + "COMMANDS"(1) + >=2 menu items +
                #          status(1) = at least 5 below the log panel.
                available_for_log = h - row - 1 - 5
                log_rows = max(0, min(LOG_PANEL_MAX, available_for_log))

                if log_rows > 0:
                    label = " SERIAL LOG" + ("  (logging paused — inject not active)"
                                             if not hasattr(self.hand._protocol._serial, "_on_tx")
                                             else "")
                    self._put(stdscr, row, 0, label, curses.A_BOLD)
                    row += 1

                    with self._log_lock:
                        # Grab the most recent entries that will fit
                        entries = list(self._log_buf)[-(log_rows - 1):]

                    # Pad top with blanks so newest entry is always at bottom
                    pad = (log_rows - 1) - len(entries)
                    row += pad

                    for ts, direction, hexstr in entries:
                        if row >= h - 1:
                            break
                        # Truncate hex if it would overflow the terminal
                        prefix     = f"  {ts}  {direction}  "
                        max_hex    = w - len(prefix) - 1
                        if len(hexstr) > max_hex:
                            # Count how many full "XX " groups fit, then note remainder
                            n_shown   = max(0, (max_hex - 4) // 3)
                            remainder = (len(hexstr) + 1) // 3 - n_shown
                            hexstr    = " ".join(hexstr.split()[:n_shown]) + f" +{remainder}b"
                        line  = prefix + hexstr
                        color = (curses.color_pair(2) if direction == "TX"
                                 else curses.color_pair(3))
                        self._put(stdscr, row, 0, line[:w - 1], color)
                        row += 1

                    self._put(stdscr, row, 0, "-" * (w - 1))
                    row += 1

            # ── Menu ─────────────────────────────────────────────────────────
            menu_space = h - row - 1   # rows available before status bar
            if menu_space > 1:
                self._put(stdscr, row, 0, " COMMANDS", curses.A_BOLD)
                row += 1
                for key_char, label in MENU:
                    if row >= h - 1:
                        break
                    hint_attr = (curses.color_pair(3) if key_char not in ("q", "v")
                                 else 0)
                    self._put(stdscr, row, 0,
                              f"  [{key_char}]  {label}"[:w - 1], hint_attr)
                    row += 1

            # ── Status bar ───────────────────────────────────────────────────
            with self._status_lock:
                msg = self._status
            self._put(stdscr, h - 1, 0,
                      f" {msg} ".ljust(w - 1)[:w - 1], curses.A_REVERSE)

            stdscr.refresh()
        except curses.error:
            pass

    def _put(self, win, y, x, s, attr=0):
        """Safe addstr that silently ignores out-of-bounds writes."""
        h, w = win.getmaxyx()
        if not (0 <= y < h and 0 <= x < w):
            return
        s = s[:w - x - 1]
        try:
            win.addstr(y, x, s, attr)
        except curses.error:
            pass

    def _set_status(self, msg: str):
        with self._status_lock:
            self._status = msg

    # ── Popup helpers ─────────────────────────────────────────────────────────

    def _input_popup(self, stdscr, title: str, prompt: str,
                     default: str = "") -> "str | None":
        """
        Modal popup with a single text-input line.
        Returns the entered string, the default on blank entry, or None
        on empty entry with no default (treat as cancel).
        """
        h, w  = stdscr.getmaxyx()
        box_h = 7
        box_w = min(max(len(title) + 6, len(prompt) + 12, 44), w - 2)
        py    = max(0, (h - box_h) // 2)
        px    = max(0, (w - box_w) // 2)

        win = curses.newwin(box_h, box_w, py, px)
        win.keypad(True)
        win.box()
        self._put(win, 1, 2, title[:box_w - 4],  curses.A_BOLD)
        self._put(win, 3, 2, (prompt + ":")[:box_w - 4])
        if default:
            self._put(win, 4, 4, f"(default: {default})"[:box_w - 6],
                      curses.A_DIM)
        self._put(win, 5, 2, "> ")
        win.refresh()

        curses.echo()
        curses.curs_set(1)
        buf = ""
        try:
            raw = win.getstr(5, 4, box_w - 6)
            buf = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            buf = ""
        finally:
            curses.noecho()
            curses.curs_set(0)

        del win
        stdscr.touchwin()
        stdscr.refresh()

        if buf == "" and default:
            return default
        return buf if buf else None

    def _message_popup(self, stdscr, title: str, lines: list):
        """Modal popup that displays lines of text and waits for a keypress."""
        h, w    = stdscr.getmaxyx()
        inner_w = max((len(l) for l in lines), default=0)
        box_w   = min(max(len(title) + 6, inner_w + 4, 30), w - 2)
        box_h   = min(len(lines) + 5, h - 2)
        py      = max(0, (h - box_h) // 2)
        px      = max(0, (w - box_w) // 2)

        win = curses.newwin(box_h, box_w, py, px)
        win.keypad(True)
        win.box()
        self._put(win, 1, 2, title[:box_w - 4], curses.A_BOLD)
        self._put(win, 2, 2, "-" * (box_w - 4))
        for i, line in enumerate(lines[: box_h - 5]):
            self._put(win, 3 + i, 2, line[:box_w - 4])
        self._put(win, box_h - 1, 2, "[ press any key ]")
        win.refresh()
        win.nodelay(False)
        win.getch()
        del win
        stdscr.touchwin()
        stdscr.refresh()

    def _ask_servo(self, stdscr) -> "int | None":
        raw = self._input_popup(stdscr, "Select servo", "Servo ID (0-6)")
        if raw is None:
            self._set_status("Cancelled.")
            return None
        try:
            sid = int(raw)
            if not 0 <= sid < NUM_SERVOS:
                raise ValueError
            return sid
        except ValueError:
            self._set_status(f"Invalid servo ID '{raw}' — must be 0-6.")
            return None

    # ── Commands ──────────────────────────────────────────────────────────────

    def _set_position_mode(self, stdscr):
        sid = self._ask_servo(stdscr)
        if sid is None:
            return
        raw = self._input_popup(
            stdscr,
            f"Position mode — servo {sid}",
            "Target position (0-65535)",
            default=str(self._cmd_positions[sid]))
        if raw is None:
            self._set_status("Cancelled.")
            return
        try:
            pos = int(raw)
            if not 0 <= pos <= 65535:
                raise ValueError
        except ValueError:
            self._set_status(f"Invalid position '{raw}'.")
            return

        self._cmd_positions[sid] = pos
        try:
            self.hand.set_positions(list(self._cmd_positions))
            self._set_status(f"[POS]  Servo {sid} -> {pos}")
        except Exception as exc:
            self._set_status(f"Error: {exc}")

    def _set_raw_torque(self, stdscr):
        sid = self._ask_servo(stdscr)
        if sid is None:
            return
        raw = self._input_popup(
            stdscr,
            f"Raw torque mode — servo {sid}",
            "Torque magnitude (0-1000)",
            default=str(self._cmd_torques[sid]))
        if raw is None:
            self._set_status("Cancelled.")
            return
        try:
            torque = int(raw)
            if not 0 <= torque <= 1000:
                raise ValueError
        except ValueError:
            self._set_status(f"Invalid torque '{raw}'.")
            return

        self._cmd_torques[sid] = torque
        try:
            self.hand.set_torques(list(self._cmd_torques))
            self._set_status(f"[TORQ] Servo {sid} -> {torque}")
        except Exception as exc:
            self._set_status(f"Error: {exc}")

    def _set_pid_torque(self, stdscr):
        sid = self._ask_servo(stdscr)
        if sid is None:
            return
        raw = self._input_popup(
            stdscr,
            f"PID torque mode — servo {sid}",
            "Target position (0-65535)",
            default=str(self._cmd_pid_pos[sid]))
        if raw is None:
            self._set_status("Cancelled.")
            return
        try:
            pos = int(raw)
            if not 0 <= pos <= 65535:
                raise ValueError
        except ValueError:
            self._set_status(f"Invalid position '{raw}'.")
            return

        self._cmd_pid_pos[sid] = pos
        try:
            self.hand.set_torque_positions(list(self._cmd_pid_pos))
            self._set_status(f"[PID]  Servo {sid} target -> {pos}")
        except Exception as exc:
            self._set_status(f"Error: {exc}")

    def _set_pid_constants(self, stdscr):
        sid = self._ask_servo(stdscr)
        if sid is None:
            return

        current = self.hand.pid_config.get(sid)

        kp_s = self._input_popup(
            stdscr, f"PID constants — servo {sid}", "Kp",
            default=str(current["kp"]))
        if kp_s is None:
            self._set_status("Cancelled.")
            return

        ki_s = self._input_popup(
            stdscr, f"PID constants — servo {sid}", "Ki",
            default=str(current["ki"]))
        if ki_s is None:
            self._set_status("Cancelled.")
            return

        kd_s = self._input_popup(
            stdscr, f"PID constants — servo {sid}", "Kd",
            default=str(current["kd"]))
        if kd_s is None:
            self._set_status("Cancelled.")
            return

        try:
            kp = float(kp_s)
            ki = float(ki_s)
            kd = float(kd_s)
        except ValueError:
            self._set_status("Invalid PID value — must be numeric.")
            return

        self.hand.update_pid(sid, kp, ki, kd)
        self._set_status(
            f"Servo {sid} PID saved:  Kp={kp}  Ki={ki}  Kd={kd}")

    def _show_sensors(self, stdscr):
        pos  = self.hand.get_positions()
        vel  = self.hand.get_velocities()
        temp = self.hand.get_temperatures()
        cur  = self.hand.get_currents()

        hdr   = f"  {'Srv':>3}  {'Position':>8}  {'Velocity':>8}  {'Temp':>5}  {'Current':>8}"
        lines = [hdr, "-" * len(hdr)]
        for i in range(NUM_SERVOS):
            lines.append(
                f"  {i:>3}  {pos[i]:>8}  {vel[i]:>8}"
                f"  {temp[i]:>4}C  {cur[i]:>8}")
        self._message_popup(stdscr, "Sensor Readings", lines)

    def _home(self, stdscr):
        confirm = self._input_popup(
            stdscr, "Home Hand",
            "Type 'yes' to start the homing sequence")
        if confirm != "yes":
            self._set_status("Homing cancelled.")
            return

        self._set_status("Homing in progress — do not touch the hand (~30 s)...")
        self._draw(stdscr)

        exc_holder: list = []

        def _do_home():
            try:
                self.hand.home()
            except Exception as e:
                exc_holder.append(e)

        t = threading.Thread(target=_do_home, daemon=True)
        t.start()

        stdscr.nodelay(True)
        while t.is_alive():
            self._draw(stdscr)
            time.sleep(0.5)
            stdscr.getch()   # discard keypresses during homing
        t.join()

        if exc_holder:
            self._set_status(f"Homing error: {exc_holder[0]}")
        else:
            self._set_status("Homing complete.")

    def _trim(self, stdscr):
        sid = self._ask_servo(stdscr)
        if sid is None:
            return

        ext = self.hand._sd[sid].extend_count
        raw = self._input_popup(
            stdscr,
            f"Trim servo {sid}  (extend_count={ext})",
            "Degrees to add (+ = more extend, - = less extend)")
        if raw is None:
            self._set_status("Cancelled.")
            return
        try:
            degrees = float(raw)
        except ValueError:
            self._set_status(f"Invalid degree value '{raw}'.")
            return

        new_ext = self.hand.trim(sid, degrees)
        self._set_status(
            f"Servo {sid} trimmed {degrees:+.2f} deg  ->  extend_count={new_ext}  (saved)")

    def _set_id(self, stdscr):
        self._message_popup(
            stdscr, "Set Servo ID",
            [
                "Connect ONLY the target servo to the bus before proceeding.",
                "The scan will fail if more than one servo responds.",
                "",
                "Press any key to continue to ID entry.",
            ])

        raw_id = self._input_popup(
            stdscr, "Set Servo ID", "New ID (0-253, not 254)")
        if raw_id is None:
            self._set_status("Cancelled.")
            return
        try:
            new_id = int(raw_id)
            if not (0 <= new_id <= 253 and new_id != 0xFE):
                raise ValueError
        except ValueError:
            self._set_status("Invalid ID (must be 0-253, not 254).")
            return

        raw_cl = self._input_popup(
            stdscr, "Set Servo ID", "Current limit (0-1023)",
            default="1023")
        try:
            current_limit = int(raw_cl) if raw_cl else 1023
            if not 0 <= current_limit <= 1023:
                raise ValueError
        except ValueError:
            self._set_status("Invalid current limit.")
            return

        self._set_status("Scanning bus for a single servo...")
        self._draw(stdscr)

        ok = self.hand.set_servo_id(new_id, current_limit)
        if ok:
            self._set_status(f"Servo ID successfully changed to {new_id}.")
        else:
            self._set_status(
                "Failed: expected exactly one servo, or new ID already taken.")

    def _ping(self, stdscr):
        self._set_status("Pinging all 7 servos...")
        self._draw(stdscr)

        results = self.hand.ping_all()
        lines = []
        for sid, ok in results.items():
            marker = "OK          " if ok else "NO RESPONSE"
            lines.append(f"  Servo {sid}:  {marker}")
        self._message_popup(stdscr, "Ping Results", lines)
        responded = sum(1 for ok in results.values() if ok)
        self._set_status(f"Ping complete — {responded}/{NUM_SERVOS} servos responded.")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Interactive CLI for the Aero Open Hand Controller")
    parser.add_argument(
        "--hand", choices=["left", "right"], default="left",
        metavar="TYPE",
        help="Hand type: 'left' or 'right'  (default: left)")
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Start with verbose serial log enabled")
    parser.add_argument(
        "--simulate", action="store_true",
        help="Use simulated hand (no hardware required, TTL only)")
    parser.add_argument(
        "--protocol", choices=["ttl", "esp32"], default="ttl",
        help="Control protocol: 'ttl' (direct Feetech serial) or 'esp32' (SDK via ESP32). Default: ttl")
    args = parser.parse_args()

    serial_logger = None

    config_dir = os.path.join(os.path.dirname(__file__), "..", "configs")

    if args.protocol == "ttl":
        if args.simulate:
            from sim.hand_simulator import HandSimulator, SERVO_IDS as SIM_IDS
            ser = HandSimulator(os.path.join(_LOG_DIR, "sim"), servo_ids=SIM_IDS)
            print(f"\nUsing simulated hand ({args.hand})...")
            port_label = "SIMULATED"
            try:
                hand = HandTTL(ser, hand_type=args.hand, config_dir=config_dir)
            except Exception as exc:
                print(f"Simulator init failed: {exc}")
                sys.exit(1)
        else:
            port = select_port()
            if port is None:
                print("No port selected.  Exiting.")
                sys.exit(0)
            port_label = port
            print(f"\nConnecting to {port}  ({args.hand} hand, TTL)...")
            try:
                hand = HandTTL(port, hand_type=args.hand, config_dir=config_dir)
            except Exception as exc:
                print(f"Connection failed: {exc}")
                sys.exit(1)
            from hand.ttl.serial_file_logger import SerialFileLogger
            serial_logger = SerialFileLogger(os.path.join(_LOG_DIR, "real"))
            hand._protocol._logger = serial_logger
            print(f"Logging serial traffic to {serial_logger.path}")
    else:  # esp32
        if args.simulate:
            print("--simulate is only supported with --protocol ttl.")
            sys.exit(1)
        port = select_port()
        if port is None:
            print("No port selected.  Exiting.")
            sys.exit(0)
        port_label = port
        print(f"\nConnecting to {port}  ({args.hand} hand, ESP32)...")
        try:
            hand = HandESP32(port, hand_type=args.hand)
        except Exception as exc:
            print(f"Connection failed: {exc}")
            sys.exit(1)

    print("Connected.  Starting UI...")
    time.sleep(0.3)

    cli = HandCLI(hand, port_label, args.hand, verbose=args.verbose)
    # Always inject the logger so [v] can be toggled at runtime without restart
    cli.inject_logger()

    try:
        curses.wrapper(cli.run)
    finally:
        hand.close()
        if serial_logger is not None:
            serial_logger.close()
        print("\nDisconnected.  Goodbye.")


if __name__ == "__main__":
    main()
