"""
source.py — Abstract base class for frame sources (webcams, depth cameras, etc.).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

import numpy as np


class FrameType(Enum):
    """Type of frame data produced by a source."""
    RGB = "rgb"
    RGBD = "rgbd"


@dataclass
class FrameData:
    """A single frame captured from a source.

    Attributes:
        image:        H x W x 3 uint8 RGB array.
        timestamp_ms: Monotonic timestamp in milliseconds.
        frame_type:   Whether this frame includes depth data.
        depth:        H x W float32 depth map (metres). Present only for RGBD.
    """
    image: np.ndarray
    timestamp_ms: int
    frame_type: FrameType
    depth: np.ndarray | None = None


class Source(ABC):
    """Abstract base class for frame sources.

    Subclasses must implement the four lifecycle methods below.
    """

    @property
    @abstractmethod
    def frame_type(self) -> FrameType:
        """The type of frames this source produces."""
        ...

    @abstractmethod
    def open(self) -> None:
        """Acquire the underlying resource (camera, file, etc.)."""
        ...

    @abstractmethod
    def read(self) -> FrameData | None:
        """Return the next frame, or None if unavailable."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release the underlying resource."""
        ...

    @abstractmethod
    def is_open(self) -> bool:
        """Whether the source is currently open and producing frames."""
        ...
