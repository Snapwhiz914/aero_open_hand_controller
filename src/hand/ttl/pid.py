import json
from dataclasses import dataclass, field
import os

@dataclass
class PIDController:
    """PID controller for a single servo's torque mode."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = 0.0
    output_max: float = 1000.0
    integral_limit: float = 500.0

    _integral: float = field(default=0.0, init=False, repr=False)
    _prev_error: float = field(default=0.0, init=False, repr=False)
    _first_update: bool = field(default=True, init=False, repr=False)

    def update(self, setpoint: float, current_value: float, dt: float) -> float:
        """Compute PID output.

        Args:
            setpoint: Target position (0-65535 normalized).
            current_value: Current position (0-65535 normalized).
            dt: Time delta in seconds.

        Returns:
            Torque magnitude (clamped to output_min..output_max).
        """
        error = setpoint - current_value

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative (skip on first call)
        if self._first_update:
            d_term = 0.0
            self._first_update = False
        else:
            d_term = self.kd * (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, abs(output)))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_update = True


class PIDConfig:
    """Manages PID constants for all 7 servos, with JSON persistence."""

    DEFAULT_KP = 0.015
    DEFAULT_KI = 0.0
    DEFAULT_KD = 0.0

    def __init__(self, config_path: str):
        self._path = config_path
        self._constants: list[dict[str, float]] = []
        self._load()

    def _load(self):
        if os.path.exists(self._path):
            with open(self._path, "r") as f:
                data = json.load(f)
            self._constants = data.get("servos", [])
        # Ensure we have entries for all 7 servos
        while len(self._constants) < 7:
            self._constants.append({
                "kp": self.DEFAULT_KP,
                "ki": self.DEFAULT_KI,
                "kd": self.DEFAULT_KD,
            })

    def save(self):
        with open(self._path, "w") as f:
            json.dump({"servos": self._constants}, f, indent=2)

    def get(self, channel: int) -> dict[str, float]:
        return dict(self._constants[channel])

    def set(self, channel: int, kp: float, ki: float, kd: float):
        self._constants[channel] = {"kp": kp, "ki": ki, "kd": kd}
        self.save()

    def create_controllers(self) -> list[PIDController]:
        """Create a PIDController instance for each of the 7 servos."""
        controllers = []
        for i in range(7):
            c = self._constants[i]
            controllers.append(PIDController(
                kp=c["kp"], ki=c["ki"], kd=c["kd"]))
        return controllers
