"""
joint_calculator.py — Abstract base class for joint angle calculators.

A JointCalculator takes hand landmark data from a Detector and converts it
into actuatable joint positions.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..detectors.detector import LandmarkData
from ..sources.source import FrameType


class JointCalculator(ABC):
    """Abstract base class for converting landmarks into servo positions.

    Subclasses declare which frame types and detectors they are compatible
    with.  The base class ``calculate`` wrapper validates compatibility before
    delegating to ``_calculate_impl``.
    """

    @abstractmethod
    def supported_frame_types(self) -> frozenset[FrameType]:
        """Frame types this calculator can work with.

        For example, a calculator that relies on depth data would return
        ``frozenset({FrameType.RGBD})``.
        """
        ...

    @abstractmethod
    def supported_detectors(self) -> frozenset[str] | None:
        """Detector names this calculator is compatible with.

        Return ``None`` to accept landmarks from any detector.
        Return a frozenset of detector name strings to restrict.
        """
        ...

    def calculate(
        self,
        landmarks: LandmarkData,
        **kwargs,
    ) -> list[int] | None:
        """Validate compatibility, then compute servo positions.

        Returns:
            A list of normalised servo positions ``[0, 65535]``, or ``None``
            if the requested hand is not present in the landmarks.

        Raises:
            TypeError: If the landmark data came from an unsupported frame
                type or detector.
        """
        # Validate frame type
        if landmarks.frame_type not in self.supported_frame_types():
            supported = ", ".join(ft.value for ft in self.supported_frame_types())
            raise TypeError(
                f"{type(self).__name__} does not support frame type "
                f"'{landmarks.frame_type.value}'. "
                f"Supported frame types: {supported}"
            )

        # Validate detector
        allowed = self.supported_detectors()
        if allowed is not None and landmarks.detector_name not in allowed:
            raise TypeError(
                f"{type(self).__name__} does not support detector "
                f"'{landmarks.detector_name}'. "
                f"Supported detectors: {', '.join(sorted(allowed))}"
            )

        return self._calculate_impl(landmarks, **kwargs)

    @abstractmethod
    def _calculate_impl(
        self,
        landmarks: LandmarkData,
        **kwargs,
    ) -> list[int] | None:
        """Compute servo positions from validated landmark data.

        Subclasses implement this.
        """
        ...
