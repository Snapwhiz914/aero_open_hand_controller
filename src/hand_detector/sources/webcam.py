"""
webcam.py — RGB frame source using a standard webcam via OpenCV.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

from .source import FrameData, FrameType, Source


class WebcamSource(Source):
    """Captures RGB frames from a webcam using OpenCV."""

    def __init__(self, camera_index: int = 0) -> None:
        self._camera_index = camera_index
        self._cap: cv2.VideoCapture | None = None
        self._last_timestamp_ms = -1

    @property
    def frame_type(self) -> FrameType:
        return FrameType.RGB

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._camera_index)
        if not self._cap.isOpened():
            self._cap.release()
            self._cap = None
            raise RuntimeError(
                f"Could not open camera index {self._camera_index}"
            )

    def read(self) -> FrameData | None:
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        if not ret:
            return None
        timestamp_ms = max(int(time.monotonic() * 1000), self._last_timestamp_ms + 1)
        self._last_timestamp_ms = timestamp_ms
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return FrameData(
            image=rgb,
            timestamp_ms=timestamp_ms,
            frame_type=FrameType.RGB,
        )

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()
