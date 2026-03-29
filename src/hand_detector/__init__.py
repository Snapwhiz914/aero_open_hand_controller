from .hand_detector import HandDetector
from .joint_calculators import TwoDJointCalculator, hand_data_to_positions
from .sources import FrameData, FrameType, Source, WebcamSource, AVAILABLE_SOURCES
from .sources import IntelD405Source
from .detectors import Detector, LandmarkData, AVAILABLE_DETECTORS, get_detector_class
from .joint_calculators import JointCalculator, ThreeDJointCalculator, AVAILABLE_CALCULATORS

__all__ = [
    "HandDetector",
    "hand_data_to_positions",
    # Sources
    "Source",
    "FrameData",
    "FrameType",
    "WebcamSource",
    "IntelD405Source",
    # Detectors
    "Detector",
    "LandmarkData",
    "MediaPipeDetector",
    # Joint calculators
    "JointCalculator",
    "TwoDJointCalculator",
    "ThreeDJointCalculator",
]


def __getattr__(name):
    if name == "MediaPipeDetector":
        from .detectors.mediapipe import MediaPipeDetector
        return MediaPipeDetector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
