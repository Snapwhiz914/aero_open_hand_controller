"""
mediapipe.py — Hand landmark detector using the MediaPipe Hand Landmarker.
"""

from __future__ import annotations

import os
import urllib.request

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from .._landmark_math import format_result
from ..sources.source import FrameData, FrameType
from .detector import Detector, LandmarkData

_DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "hand_landmarker.task"
)
_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
)


class MediaPipeDetector(Detector):
    """Detects hand landmarks using the MediaPipe Hand Landmarker model.

    Supports RGB and RGBD frames (only the RGB channel is used).
    """

    def __init__(
        self,
        model_path: str | None = None,
        max_hands: int = 2,
        detection_confidence: float = 0.5,
        tracking_confidence: float = 0.5,
    ) -> None:
        self._model_path = model_path or _DEFAULT_MODEL_PATH
        self._max_hands = max_hands
        self._detection_confidence = detection_confidence
        self._tracking_confidence = tracking_confidence
        self._landmarker: mp_vision.HandLandmarker | None = None

    @property
    def name(self) -> str:
        return "mediapipe"

    def supported_frame_types(self) -> frozenset[FrameType]:
        return frozenset({FrameType.RGB, FrameType.RGBD})

    def open(self) -> None:
        """Download the model if needed and create the landmarker."""
        self._ensure_model()
        options = mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=self._model_path),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=self._max_hands,
            min_hand_detection_confidence=self._detection_confidence,
            min_hand_presence_confidence=self._detection_confidence,
            min_tracking_confidence=self._tracking_confidence,
        )
        self._landmarker = mp_vision.HandLandmarker.create_from_options(options)

    @property
    def last_raw_result(self):
        """Expose the raw MediaPipe result for preview drawing (if needed)."""
        return self._last_raw_result if hasattr(self, "_last_raw_result") else None

    def _detect_impl(self, frame: FrameData) -> LandmarkData | None:
        if self._landmarker is None:
            raise RuntimeError("MediaPipeDetector.open() must be called before detect()")

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame.image)
        result = self._landmarker.detect_for_video(mp_image, frame.timestamp_ms)
        self._last_raw_result = result

        if not result.hand_world_landmarks:
            return None

        data = format_result(result)
        return LandmarkData(
            hands=data["hands"],
            timestamp=data["timestamp"],
            frame_type=frame.frame_type,
            detector_name=self.name,
        )

    def close(self) -> None:
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None

    def _ensure_model(self) -> None:
        if os.path.exists(self._model_path):
            return
        print(f"[MediaPipeDetector] Downloading model to {self._model_path} ...")
        try:
            urllib.request.urlretrieve(_MODEL_URL, self._model_path)
            print("[MediaPipeDetector] Model downloaded.")
        except Exception as exc:
            raise RuntimeError(
                f"[MediaPipeDetector] Failed to download model: {exc}"
            ) from exc
