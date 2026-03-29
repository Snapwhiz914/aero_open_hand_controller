from .source import FrameData, FrameType, Source
from .webcam import WebcamSource

__all__ = [
    "FrameData", "FrameType", "Source", "WebcamSource",
    "IntelD405Source", "AVAILABLE_SOURCES",
]

AVAILABLE_SOURCES: dict[str, dict] = {
    "Webcam (RGB)": {
        "class": WebcamSource,
        "frame_type": FrameType.RGB,
        "description": "Standard USB webcam via OpenCV",
    },
}

# RealSense is an optional dependency — register only if available
try:
    from .intel_d405 import IntelD405Source
    AVAILABLE_SOURCES["Intel RealSense D405 (RGBD)"] = {
        "class": IntelD405Source,
        "frame_type": FrameType.RGBD,
        "description": "Intel RealSense D405 depth camera (aligned RGBD)",
    }
except ImportError:
    IntelD405Source = None  # type: ignore[misc,assignment]
