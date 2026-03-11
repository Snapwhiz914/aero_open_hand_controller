import json
import socket
import sys
import threading
import time
import urllib.request

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

# --- Constants ---
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
MODEL_PATH = "hand_landmarker.task"
HOST = "0.0.0.0"
PORT = 8765
MAX_HANDS = 2
STREAM_URL = "http://localhost:9999/video"

# Landmark indices
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_PIP = 6
INDEX_DIP = 7
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_PIP = 10
MIDDLE_DIP = 11
MIDDLE_TIP = 12
RING_MCP = 13
RING_PIP = 14
RING_DIP = 15
RING_TIP = 16
PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20


# --- Utility functions ---

def ensure_model(path: str, url: str) -> None:
    import os
    if os.path.exists(path):
        return
    print(f"Downloading model from {url} ...")
    try:
        urllib.request.urlretrieve(url, path)
        print("Model downloaded.")
    except Exception as e:
        print(f"Failed to download model: {e}", file=sys.stderr)
        sys.exit(1)


def vec3(lm) -> np.ndarray:
    return np.array([lm.x, lm.y, lm.z], dtype=np.float64)


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.zeros(3, dtype=np.float64)
    return v / n


def compute_palm_normal(wl) -> np.ndarray:
    wrist = vec3(wl[WRIST])
    index_mcp = vec3(wl[INDEX_MCP])
    pinky_mcp = vec3(wl[PINKY_MCP])
    return normalize(np.cross(index_mcp - wrist, pinky_mcp - wrist))


def project_finger_2d(mcp_lm, pip_lm, dip_lm, tip_lm, palm_normal: np.ndarray) -> dict:
    origin = vec3(mcp_lm)
    pip_rel = vec3(pip_lm) - origin
    dip_rel = vec3(dip_lm) - origin
    tip_rel = vec3(tip_lm) - origin

    u = normalize(pip_rel)

    cross_u_tip = np.cross(u, tip_rel)
    if np.linalg.norm(cross_u_tip) < 1e-6:
        # Degenerate (nearly straight finger): use palm normal
        n = normalize(palm_normal)
    else:
        n = normalize(cross_u_tip)

    v = normalize(np.cross(n, u))

    def proj2d(rel: np.ndarray) -> list:
        return [float(np.dot(rel, u)), float(np.dot(rel, v))]

    return {
        "mcp": [0.0, 0.0],
        "pip": proj2d(pip_rel),
        "dip": proj2d(dip_rel),
        "tip": proj2d(tip_rel),
    }


def format_thumb_3d(wl) -> dict:
    origin = vec3(wl[THUMB_MCP])

    def rel3(idx: int) -> list:
        return (vec3(wl[idx]) - origin).tolist()

    return {
        "cmc": rel3(THUMB_CMC),
        "mcp": [0.0, 0.0, 0.0],
        "ip": rel3(THUMB_IP),
        "tip": rel3(THUMB_TIP),
    }


def format_hand(wl, handedness_cat) -> dict:
    palm_normal = compute_palm_normal(wl)

    fingers = {
        "thumb": format_thumb_3d(wl),
        "index": project_finger_2d(
            wl[INDEX_MCP], wl[INDEX_PIP], wl[INDEX_DIP], wl[INDEX_TIP], palm_normal
        ),
        "middle": project_finger_2d(
            wl[MIDDLE_MCP], wl[MIDDLE_PIP], wl[MIDDLE_DIP], wl[MIDDLE_TIP], palm_normal
        ),
        "ring": project_finger_2d(
            wl[RING_MCP], wl[RING_PIP], wl[RING_DIP], wl[RING_TIP], palm_normal
        ),
        "pinky": project_finger_2d(
            wl[PINKY_MCP], wl[PINKY_PIP], wl[PINKY_DIP], wl[PINKY_TIP], palm_normal
        ),
    }

    return {
        "handedness": handedness_cat.category_name,
        "confidence": round(float(handedness_cat.score), 4),
        "fingers": fingers,
    }


def format_result(result) -> dict:
    hands = []
    for wl, handedness_list in zip(result.hand_world_landmarks, result.handedness):
        hands.append(format_hand(wl, handedness_list[0]))
    return {
        "timestamp": time.time(),
        "hands": hands,
    }


def broadcast(data_bytes: bytes, clients: set, lock: threading.Lock) -> None:
    with lock:
        snapshot = list(clients)
    dead = []
    for conn in snapshot:
        try:
            conn.sendall(data_bytes)
        except (BrokenPipeError, ConnectionResetError, OSError):
            dead.append(conn)
    if dead:
        with lock:
            for conn in dead:
                clients.discard(conn)


def handle_client(conn: socket.socket, addr, clients: set, lock: threading.Lock) -> None:
    print(f"Client connected: {addr}")
    with lock:
        clients.add(conn)
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
    except OSError:
        pass
    finally:
        with lock:
            clients.discard(conn)
        try:
            conn.close()
        except OSError:
            pass
        print(f"Client disconnected: {addr}")


def main() -> None:
    ensure_model(MODEL_PATH, MODEL_URL)

    options = mp_vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_hands=MAX_HANDS,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print(f"Error: could not open stream at {STREAM_URL}", file=sys.stderr)
        sys.exit(1)
    print(f"Stream opened: {STREAM_URL}")

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(16)
    print(f"Listening on {HOST}:{PORT}")

    clients: set = set()
    lock = threading.Lock()
    shutdown = threading.Event()

    def accept_loop() -> None:
        server_sock.settimeout(1.0)
        while not shutdown.is_set():
            try:
                conn, addr = server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            t = threading.Thread(
                target=handle_client, args=(conn, addr, clients, lock), daemon=True
            )
            t.start()

    accept_thread = threading.Thread(target=accept_loop, daemon=True)
    accept_thread.start()

    try:
        with mp_vision.HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Warning: failed to read frame, retrying...")
                    time.sleep(0.01)
                    continue

                timestamp_ms = int(time.monotonic() * 1000)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

                result = landmarker.detect_for_video(mp_image, timestamp_ms)
                frame_data = format_result(result)

                payload = (json.dumps(frame_data, separators=(",", ":")) + "\n").encode()
                broadcast(payload, clients, lock)

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        shutdown.set()
        cap.release()
        server_sock.close()


if __name__ == "__main__":
    main()
