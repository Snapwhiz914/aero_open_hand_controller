import importlib

from .joint_calculator import JointCalculator
from ..sources.source import FrameType

# 2d_joint_calculator.py cannot be imported with a regular import statement
# because the filename starts with a digit.
_2d = importlib.import_module(".2d_joint_calculator", __package__)
TwoDJointCalculator = _2d.TwoDJointCalculator
hand_data_to_positions = _2d.hand_data_to_positions

__all__ = [
    "JointCalculator", "TwoDJointCalculator", "hand_data_to_positions",
    "ThreeDJointCalculator", "AVAILABLE_CALCULATORS",
]

AVAILABLE_CALCULATORS: dict[str, dict] = {
    "2D Projection (MediaPipe)": {
        "class": TwoDJointCalculator,
        "supported_frame_types": frozenset({FrameType.RGB, FrameType.RGBD}),
        "supported_detectors": frozenset({"mediapipe"}),
        "description": "Joint angles from 2D finger projections and 3D thumb data",
    },
}

# pyrealsense2 is an optional dependency — register only if available
try:
    _3d = importlib.import_module(".3d_calculator", __package__)
    ThreeDJointCalculator = _3d.ThreeDJointCalculator
    AVAILABLE_CALCULATORS["3D Depth (RealSense + MediaPipe)"] = {
        "class": ThreeDJointCalculator,
        "supported_frame_types": frozenset({FrameType.RGBD}),
        "supported_detectors": frozenset({"mediapipe"}),
        "description": "Joint angles from depth-enhanced 3D landmark positions",
    }
except ImportError:
    ThreeDJointCalculator = None  # type: ignore[misc,assignment]
