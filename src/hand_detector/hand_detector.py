"""
hand_detector.py — Threaded hand landmark detection pipeline.

Composes a Source (frame provider) and Detector (landmark extractor) and runs
them in a background thread, firing registered callbacks on each frame that
contains detected hands.

Usage:
    from hand_detector import HandDetector

    detector = HandDetector()
    detector.register_callback(lambda data: print(data.hands))
    detector.start()
    ...
    detector.stop()
"""

from __future__ import annotations

import threading
import time
import uuid
from collections.abc import Callable
from typing import Any

import cv2

from .detectors.detector import Detector, LandmarkData
from .sources.source import Source

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


class HandDetector:
    """
    Orchestrates a Source and a Detector in a background thread.

    By default uses a WebcamSource and MediaPipeDetector, but any
    Source/Detector pair can be injected.

    Each time hands are detected, all registered callbacks are called with
    a dict matching the canonical hand data format (see CLAUDE.md).
    """

    def __init__(
        self,
        source: Source | None = None,
        detector: Detector | None = None,
        *,
        # Legacy convenience parameters — used only when source/detector are
        # not provided (i.e. the default WebcamSource + MediaPipeDetector).
        camera_index: int = 0,
        model_path: str | None = None,
        max_hands: int = 2,
        detection_confidence: float = 0.5,
        tracking_confidence: float = 0.5,
    ) -> None:
        if source is None:
            from .sources.webcam import WebcamSource
            source = WebcamSource(camera_index=camera_index)
        if detector is None:
            from .detectors.mediapipe import MediaPipeDetector
            detector = MediaPipeDetector(
                model_path=model_path,
                max_hands=max_hands,
                detection_confidence=detection_confidence,
                tracking_confidence=tracking_confidence,
            )

        self._source = source
        self._detector = detector

        self._callbacks: dict[str, Callable[[dict[str, Any]], None]] = {}
        self._callbacks_lock = threading.Lock()

        self._on_error: Callable[[str], None] | None = None

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

    def set_error_callback(self, fn: Callable[[str], None]) -> None:
        """Register a callback invoked when the detector thread encounters a
        fatal error (e.g. the source fails to open).  The callback receives
        the error message string and is called on the detector thread.
        """
        self._on_error = fn

    def start(self, show_preview: bool = False) -> None:
        """Open the source and detector, then start the background thread.

        Args:
            show_preview: If True, open a cv2 window showing the camera feed
                with hand landmarks and connections drawn on each frame.
        """
        if self._thread is not None and self._thread.is_alive():
            return
        self._show_preview = show_preview
        self._stop_event.clear()
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

    def _fire_error(self, message: str) -> None:
        """Log an error and notify the error callback if one is registered."""
        print(f"[HandDetector] Error: {message}")
        if self._on_error is not None:
            try:
                self._on_error(message)
            except Exception:
                pass

    def _run(self) -> None:
        try:
            self._source.open()
        except RuntimeError as exc:
            self._fire_error(str(exc))
            return

        if not self._source.is_open():
            self._fire_error("Source failed to open.")
            return

        try:
            self._detector.open()
        except Exception as exc:
            self._fire_error(str(exc))
            self._source.close()
            return

        try:
            while not self._stop_event.is_set():
                frame = self._source.read()
                if frame is None:
                    time.sleep(0.01)
                    continue

                landmark_data = self._detector.detect(frame)

                if landmark_data is not None:
                    # Build the legacy callback dict format for compatibility
                    data = {
                        "timestamp": landmark_data.timestamp,
                        "hands": landmark_data.hands,
                    }
                    with self._callbacks_lock:
                        callbacks = list(self._callbacks.values())
                    for fn in callbacks:
                        try:
                            fn(data)
                        except Exception as exc:
                            print(f"[HandDetector] Callback error: {exc}")

                if self._show_preview:
                    self._draw_preview(frame, landmark_data)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self._stop_event.set()
        finally:
            self._detector.close()
            self._source.close()
            if self._show_preview:
                cv2.destroyWindow(_PREVIEW_WINDOW)

    def _draw_preview(self, frame, landmark_data) -> None:
        """Draw hand landmarks onto the frame and show in a cv2 window."""
        # Convert RGB back to BGR for OpenCV display
        bgr = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)

        # Use raw MediaPipe result for drawing if available
        if hasattr(self._detector, "last_raw_result") and self._detector.last_raw_result is not None:
            result = self._detector.last_raw_result
            h, w = bgr.shape[:2]
            for hand_lms in result.hand_landmarks:
                pts = [(int(lm.x * w), int(lm.y * h)) for lm in hand_lms]
                for a, b in _HAND_CONNECTIONS:
                    cv2.line(bgr, pts[a], pts[b], (0, 255, 0), 2)
                for pt in pts:
                    cv2.circle(bgr, pt, 4, (0, 0, 255), -1)

        cv2.imshow(_PREVIEW_WINDOW, bgr)
