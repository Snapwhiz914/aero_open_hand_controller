from .detector import Detector, LandmarkData
from ..sources.source import FrameType

__all__ = ["Detector", "LandmarkData", "MediaPipeDetector", "AVAILABLE_DETECTORS"]

AVAILABLE_DETECTORS: dict[str, dict] = {
    "MediaPipe Hand Landmarker": {
        "class_path": ".mediapipe.MediaPipeDetector",
        "name": "mediapipe",
        "supported_frame_types": frozenset({FrameType.RGB, FrameType.RGBD}),
        "description": "MediaPipe float16 hand landmarker model",
    },
}


def get_detector_class(key: str):
    """Lazily import and return the detector class for a registry key."""
    entry = AVAILABLE_DETECTORS[key]
    if key == "MediaPipe Hand Landmarker":
        from .mediapipe import MediaPipeDetector
        return MediaPipeDetector
    raise KeyError(f"Unknown detector: {key}")


def __getattr__(name):
    if name == "MediaPipeDetector":
        from .mediapipe import MediaPipeDetector
        return MediaPipeDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
