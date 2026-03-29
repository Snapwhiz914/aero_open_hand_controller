"""
detector.py — Abstract base class for hand landmark detectors.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from ..sources.source import FrameData, FrameType


@dataclass
class LandmarkData:
    """Hand landmarks produced by a detector.

    Attributes:
        hands:         List of hand dicts in the canonical format (see CLAUDE.md).
        timestamp:     Wall-clock time (time.time()) when the frame was processed.
        frame_type:    The FrameType of the source frame that produced these landmarks.
        detector_name: Identifier of the detector that produced these landmarks
                       (e.g. ``"mediapipe"``).
    """
    hands: list[dict]
    timestamp: float
    frame_type: FrameType
    detector_name: str


class Detector(ABC):
    """Abstract base class for hand landmark detectors.

    Subclasses declare which frame types they support.  The base class
    ``detect`` wrapper validates the frame type before delegating to
    ``_detect_impl``.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Short identifier for this detector (e.g. ``"mediapipe"``)."""
        ...

    @abstractmethod
    def supported_frame_types(self) -> frozenset[FrameType]:
        """Frame types this detector can consume."""
        ...

    def detect(self, frame: FrameData) -> LandmarkData | None:
        """Validate the frame type, then run detection.

        Returns ``None`` if no hands were detected.

        Raises:
            TypeError: If the frame type is not supported by this detector.
        """
        if frame.frame_type not in self.supported_frame_types():
            supported = ", ".join(ft.value for ft in self.supported_frame_types())
            raise TypeError(
                f"Detector '{self.name}' does not support frame type "
                f"'{frame.frame_type.value}'. Supported: {supported}"
            )
        return self._detect_impl(frame)

    @abstractmethod
    def _detect_impl(self, frame: FrameData) -> LandmarkData | None:
        """Run detection on a validated frame. Subclasses implement this."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release any resources held by the detector."""
        ...
