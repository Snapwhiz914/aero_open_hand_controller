"""
hand_detector.py — Threaded hand landmark detection using MediaPipe.

Usage:
    from hand_detector import HandDetector

    detector = HandDetector()
    detector.register_callback(lambda data: print(data["hands"]))
    detector.start()
    ...
    detector.stop()
"""

import os
import threading
import time
import urllib.request
import uuid
from collections.abc import Callable
from typing import Any

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from ._landmark_math import format_result

HAND_PALM_CONNECTIONS = ((0, 1), (0, 5), (9, 13), (13, 17), (5, 9), (0, 17))

HAND_THUMB_CONNECTIONS = ((1, 2), (2, 3), (3, 4))

HAND_INDEX_FINGER_CONNECTIONS = ((5, 6), (6, 7), (7, 8))

HAND_MIDDLE_FINGER_CONNECTIONS = ((9, 10), (10, 11), (11, 12))

HAND_RING_FINGER_CONNECTIONS = ((13, 14), (14, 15), (15, 16))

HAND_PINKY_FINGER_CONNECTIONS = ((17, 18), (18, 19), (19, 20))

_HAND_CONNECTIONS = frozenset().union(*[
    HAND_PALM_CONNECTIONS, HAND_THUMB_CONNECTIONS,
    HAND_INDEX_FINGER_CONNECTIONS, HAND_MIDDLE_FINGER_CONNECTIONS,
    HAND_RING_FINGER_CONNECTIONS, HAND_PINKY_FINGER_CONNECTIONS
])
_PREVIEW_WINDOW = "HandDetector"

_DEFAULT_MODEL_PATH = os.path.join(os.path.dirname(__file__), "hand_landmarker.task")
_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
)


class HandDetector:
    """
    Loads the MediaPipe hand landmarker model, opens a webcam, and runs
    inference on every captured frame inside a background thread.

    Each time hands are detected, all registered callbacks are called with a
    data dict that matches the format the streaming server (server.py) would
    have sent as a JSON line:

        {
            "timestamp": float,
            "hands": [
                {
                    "handedness": "Left" | "Right",
                    "confidence": float,
                    "fingers": {
                        "thumb":  {"cmc": [x,y,z], "mcp": [0,0,0],
                                   "ip": [x,y,z], "tip": [x,y,z]},
                        "index":  {"mcp": [0,0], "pip": [u,v],
                                   "dip": [u,v], "tip": [u,v]},
                        "middle": ...,
                        "ring":   ...,
                        "pinky":  ...,
                    }
                },
                ...
            ]
        }

    Callbacks are invoked on the detector thread. They should return quickly;
    use a queue or signal if heavy processing is needed (e.g. from a Qt GUI).
    """

    def __init__(
        self,
        model_path: str | None = None,
        camera_index: int = 0,
        max_hands: int = 2,
        detection_confidence: float = 0.5,
        tracking_confidence: float = 0.5,
    ) -> None:
        self._model_path = model_path or _DEFAULT_MODEL_PATH
        self._camera_index = camera_index
        self._max_hands = max_hands
        self._detection_confidence = detection_confidence
        self._tracking_confidence = tracking_confidence

        self._callbacks: dict[str, Callable[[dict[str, Any]], None]] = {}
        self._callbacks_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._show_preview = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def register_callback(self, fn: Callable[[dict[str, Any]], None]) -> str:
        """Register a callback invoked on every frame that contains hands.

        Returns a callback ID that can be passed to remove_callback().
        """
        callback_id = str(uuid.uuid4())
        with self._callbacks_lock:
            self._callbacks[callback_id] = fn
        return callback_id

    def remove_callback(self, callback_id: str) -> None:
        """Remove a previously registered callback by its ID."""
        with self._callbacks_lock:
            self._callbacks.pop(callback_id, None)

    def start(self, show_preview: bool = False) -> None:
        """Open the camera, load the model, and start the background thread.

        Args:
            show_preview: If True, open a cv2 window showing the camera feed
                with hand landmarks and connections drawn on each frame.
        """
        if self._thread is not None and self._thread.is_alive():
            return
        self._show_preview = show_preview
        self._stop_event.clear()
        self._ensure_model()
        self._thread = threading.Thread(target=self._run, daemon=True, name="HandDetector")
        self._thread.start()

    def stop(self) -> None:
        """Signal the background thread to stop and wait for it to exit."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _ensure_model(self) -> None:
        if os.path.exists(self._model_path):
            return
        print(f"[HandDetector] Downloading model to {self._model_path} ...")
        try:
            urllib.request.urlretrieve(_MODEL_URL, self._model_path)
            print("[HandDetector] Model downloaded.")
        except Exception as exc:
            raise RuntimeError(f"[HandDetector] Failed to download model: {exc}") from exc

    def _run(self) -> None:
        options = mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=self._model_path),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=self._max_hands,
            min_hand_detection_confidence=self._detection_confidence,
            min_hand_presence_confidence=self._detection_confidence,
            min_tracking_confidence=self._tracking_confidence,
        )

        cap = cv2.VideoCapture(self._camera_index)
        if not cap.isOpened():
            print(f"[HandDetector] Error: could not open camera index {self._camera_index}")
            return

        try:
            with mp_vision.HandLandmarker.create_from_options(options) as landmarker:
                last_timestamp_ms = -1
                while not self._stop_event.is_set():
                    ret, frame = cap.read()
                    if not ret:
                        time.sleep(0.01)
                        continue

                    timestamp_ms = max(int(time.monotonic() * 1000), last_timestamp_ms + 1)
                    last_timestamp_ms = timestamp_ms
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

                    result = landmarker.detect_for_video(mp_image, timestamp_ms)

                    if result.hand_world_landmarks:
                        data = format_result(result)
                        with self._callbacks_lock:
                            callbacks = list(self._callbacks.values())
                        for fn in callbacks:
                            try:
                                fn(data)
                            except Exception as exc:
                                print(f"[HandDetector] Callback error: {exc}")

                    if self._show_preview:
                        self._draw_landmarks(frame, result)
                        cv2.imshow(_PREVIEW_WINDOW, frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            self._stop_event.set()
        finally:
            cap.release()
            if self._show_preview:
                cv2.destroyWindow(_PREVIEW_WINDOW)

    def _draw_landmarks(self, frame, result) -> None:
        """Draw hand landmark dots and connection lines onto frame in-place."""
        h, w = frame.shape[:2]
        for hand_lms in result.hand_landmarks:
            pts = [(int(lm.x * w), int(lm.y * h)) for lm in hand_lms]
            for a, b in _HAND_CONNECTIONS:
                cv2.line(frame, pts[a], pts[b], (0, 255, 0), 2)
            for pt in pts:
                cv2.circle(frame, pt, 4, (0, 0, 255), -1)
