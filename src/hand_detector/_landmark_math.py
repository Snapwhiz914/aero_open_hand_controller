"""
_landmark_math.py — Geometry helpers for converting MediaPipe hand landmarks
into the canonical hand data format.

Extracted verbatim from hand_detector_streaming_server/server.py.
"""

import time

import numpy as np

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
        "wrist": rel3(WRIST),
        "cmc": rel3(THUMB_CMC),
        "mcp": [0.0, 0.0, 0.0],
        "ip": rel3(THUMB_IP),
        "tip": rel3(THUMB_TIP),
    }


def format_hand(wl, handedness_cat) -> dict:
    """
    Convert a single hand's world landmarks and handedness category into the
    canonical dict format.

    Output shape:
        {
            "handedness": "Left" | "Right",
            "confidence": float,
            "palm_normal": [x, y, z],
            "fingers": {
                "thumb":  {"wrist": [x,y,z], "cmc": [x,y,z], "mcp": [0,0,0], "ip": [x,y,z], "tip": [x,y,z]},
                "index":  {"mcp": [0,0], "pip": [u,v], "dip": [u,v], "tip": [u,v]},
                "middle": ...,
                "ring":   ...,
                "pinky":  ...,
            }
        }
    """
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
        "palm_normal": palm_normal.tolist(),
        "fingers": fingers,
    }


def format_result(result) -> dict:
    """
    Convert a MediaPipe HandLandmarkerResult into the canonical payload dict.

    Output shape:
        {
            "timestamp": float,   # time.time()
            "hands": [ <format_hand output>, ... ]
        }

    This is the same structure the streaming server sent as a JSON line per frame.
    Callbacks registered with HandDetector receive this dict directly.
    """
    hands = []
    for wl, handedness_list in zip(result.hand_world_landmarks, result.handedness):
        hands.append(format_hand(wl, handedness_list[0]))
    return {
        "timestamp": time.time(),
        "hands": hands,
    }
