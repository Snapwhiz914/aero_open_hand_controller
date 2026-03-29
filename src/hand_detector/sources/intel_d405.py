"""
intel_d405.py — RGBD frame source using an Intel RealSense D405 depth camera.

Requires the ``pyrealsense2`` package (``pip install pyrealsense2``).
Produces aligned RGBD frames: an RGB image with a matching float32 depth map
(in metres).
"""

from __future__ import annotations

import time

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as exc:
    raise ImportError(
        "pyrealsense2 is required for the Intel D405 source. "
        "Install it with: pip install pyrealsense2"
    ) from exc

from .source import FrameData, FrameType, Source


class IntelD405Source(Source):
    """Captures aligned RGBD frames from an Intel RealSense D405.

    The depth stream is aligned to the colour stream so that each pixel in the
    RGB image has a corresponding depth value.

    Args:
        width:      Requested frame width (default 640).
        height:     Requested frame height (default 480).
        fps:        Requested framerate (default 30).
        serial:     Device serial number string.  ``None`` selects the first
                    available RealSense device.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        serial: str | None = None,
    ) -> None:
        self._width = width
        self._height = height
        self._fps = fps
        self._serial = serial

        self._pipeline: rs.pipeline | None = None
        self._align: rs.align | None = None
        self._profile: rs.pipeline_profile | None = None
        self._last_timestamp_ms = -1

    @property
    def frame_type(self) -> FrameType:
        return FrameType.RGBD

    @property
    def depth_intrinsics(self) -> rs.intrinsics | None:
        """Camera intrinsics for the depth stream (available after open())."""
        if self._profile is None:
            return None
        depth_stream = self._profile.get_stream(rs.stream.depth)
        return depth_stream.as_video_stream_profile().get_intrinsics()

    @property
    def depth_scale(self) -> float:
        """Conversion factor from raw depth units to metres."""
        if self._profile is None:
            return 0.001  # sensible default for D405
        sensor = self._profile.get_device().first_depth_sensor()
        return sensor.get_depth_scale()

    def open(self) -> None:
        self._pipeline = rs.pipeline()
        config = rs.config()

        if self._serial is not None:
            config.enable_device(self._serial)

        config.enable_stream(
            rs.stream.depth, self._width, self._height, rs.format.z16, self._fps
        )
        config.enable_stream(
            rs.stream.color, self._width, self._height, rs.format.rgb8, self._fps
        )

        try:
            self._profile = self._pipeline.start(config)
        except RuntimeError as exc:
            self._pipeline = None
            raise RuntimeError(
                f"Could not open Intel RealSense device: {exc}"
            ) from exc

        # Align depth frames to the colour frame coordinate space
        self._align = rs.align(rs.stream.color)

    def read(self) -> FrameData | None:
        if self._pipeline is None:
            return None

        frames = self._pipeline.poll_for_frames()
        if not frames:
            return None

        aligned = self._align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return None

        # Monotonic timestamp
        timestamp_ms = max(
            int(time.monotonic() * 1000), self._last_timestamp_ms + 1
        )
        self._last_timestamp_ms = timestamp_ms

        # RGB image (already rgb8 format, no conversion needed)
        rgb = np.asanyarray(color_frame.get_data())

        # Depth map: raw uint16 → float32 metres
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self.depth_scale

        return FrameData(
            image=rgb,
            timestamp_ms=timestamp_ms,
            frame_type=FrameType.RGBD,
            depth=depth_m,
        )

    def close(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
            self._align = None
            self._profile = None

    def is_open(self) -> bool:
        return self._pipeline is not None
