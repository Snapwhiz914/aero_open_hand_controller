"""Abstract Hand interface.

All concrete hand implementations (TTL serial, ESP32 SDK, etc.) must
inherit from this class and implement every abstract method.

Position values use a normalised 0–65535 scale throughout:
  0       = fully extended (open)
  65535   = fully grasped (closed)

Torque values use a signed –1000…+1000 scale:
  positive = toward grasp
  negative = toward extend
  0        = no torque / hold
"""

import threading
from abc import ABC, abstractmethod
from collections.abc import Callable


class Hand(ABC):
    """Abstract base class for all Aero Open Hand controller implementations."""

    # ── Context manager ──────────────────────────────────────────────────────

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # ── Lifecycle ────────────────────────────────────────────────────────────

    @abstractmethod
    def close(self) -> None:
        """Release resources (serial port, threads, etc.)."""
        ...

    # ── Control: position mode ───────────────────────────────────────────────

    @abstractmethod
    def set_positions(self, positions: list[int]) -> None:
        """Set target positions for all 7 servos.

        Args:
            positions: 7 normalised values, each 0–65535
                       (0 = extend, 65535 = grasp).
        """
        ...

    # ── Control: raw torque mode ─────────────────────────────────────────────

    @abstractmethod
    def set_torques(self, torques: list[int]) -> None:
        """Set raw torque for all 7 servos.

        Args:
            torques: 7 signed values, each –1000…+1000
                     (positive = toward grasp, negative = toward extend, 0 = off).
        """
        ...

    # ── Control: PID-based torque mode ───────────────────────────────────────

    @abstractmethod
    def set_torque_positions(self, positions: list[int]) -> None:
        """PID-based torque control targeting normalised positions.

        Args:
            positions: 7 target normalised values (0–65535).
        """
        ...

    # ── Per-servo configuration ──────────────────────────────────────────────

    @abstractmethod
    def set_speed(self, channel: int, speed: int) -> None:
        """Set maximum move speed for one servo channel (0–32766)."""
        ...

    @abstractmethod
    def set_torque_limit(self, channel: int, torque: int) -> None:
        """Set torque limit for one servo channel (0–1023)."""
        ...

    # ── PID configuration ────────────────────────────────────────────────────

    @property
    @abstractmethod
    def pid_config(self):
        """Return the PID configuration object for this hand.

        The returned object must support:
            .get(channel) -> dict with keys 'kp', 'ki', 'kd'
            .set(channel, kp, ki, kd)
        """
        ...

    @abstractmethod
    def update_pid(self, channel: int, kp: float, ki: float, kd: float) -> None:
        """Update PID gains for one servo channel and persist them."""
        ...

    # ── Homing ───────────────────────────────────────────────────────────────

    @property
    @abstractmethod
    def is_homing(self) -> bool:
        """True while a homing sequence is running."""
        ...

    @abstractmethod
    def home(self) -> None:
        """Run the automatic homing sequence. Blocks until complete."""
        ...

    @abstractmethod
    def home_manual(self, step_event: threading.Event,
                    on_prompt: Callable[[str], None]) -> None:
        """Interactive manual homing sequence. Blocks until all servos are done.

        Args:
            step_event: Caller sets this event to advance each step.
            on_prompt:  Called with status strings from the homing thread.
        """
        ...

    # ── Grasp ────────────────────────────────────────────────────────────────

    @abstractmethod
    def grasp(self, speed_dps: float = 15.0, torque_limit: int = 500) -> None:
        """Move all servos to their grasp (closed) positions.

        Args:
            speed_dps:    Closing speed in degrees per second.
            torque_limit: Maximum torque during movement (0–1023).
        """
        ...

    # ── Sensor data access ───────────────────────────────────────────────────

    @abstractmethod
    def get_positions(self) -> list[int]:
        """Return cached normalised positions (0–65535) for all 7 servos."""
        ...

    @abstractmethod
    def get_velocities(self) -> list[int]:
        """Return cached velocities for all 7 servos (implementation-defined units)."""
        ...

    @abstractmethod
    def get_currents(self) -> list[int]:
        """Return cached currents for all 7 servos (implementation-defined units)."""
        ...

    @abstractmethod
    def get_temperatures(self) -> list[int]:
        """Return cached temperatures for all 7 servos (°C)."""
        ...

    # ── Trimming ─────────────────────────────────────────────────────────────

    @abstractmethod
    def trim(self, channel: int, degrees: float) -> int:
        """Adjust the extend calibration for one servo.

        Args:
            channel: Servo index 0–6.
            degrees: Delta in degrees (positive = more extended).

        Returns:
            New extend raw count (or equivalent implementation value).
        """
        ...

    # ── Servo ID ─────────────────────────────────────────────────────────────

    @abstractmethod
    def set_servo_id(self, new_id: int, current_limit: int = 1023) -> bool:
        """Change the hardware ID of the single servo connected to the bus.

        Args:
            new_id:        Target ID (0–253, not 254).
            current_limit: Current cap written to the servo (0–1023).

        Returns:
            True on success.
        """
        ...

    # ── Ping ─────────────────────────────────────────────────────────────────

    @abstractmethod
    def ping_all(self) -> dict[int, bool]:
        """Ping all 7 servos. Returns {servo_id: responded}."""
        ...
