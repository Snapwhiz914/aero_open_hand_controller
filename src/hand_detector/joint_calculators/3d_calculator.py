"""
3d_calculator.py — Convert RGBD-enhanced 3D hand landmarks to servo positions.

Uses depth data from an RGBD source to reconstruct true 3D positions for each
MediaPipe landmark, then computes joint angles from the 3D bone vectors.
This is more accurate than the 2D projection approach because it uses real
metric distances rather than estimated depth from a monocular model.

Requires:
    - RGBD frames (with a depth map aligned to the colour image)
    - The ``"mediapipe"`` detector
    - ``pyrealsense2`` for pixel-to-point deprojection (camera intrinsics)
"""

from __future__ import annotations

import math

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as exc:
    raise ImportError(
        "pyrealsense2 is required for the 3D joint calculator. "
        "Install it with: pip install pyrealsense2"
    ) from exc

from hand.sdk.aero_hand_constants import AeroHandConstants
from hand.sdk.joints_to_actuations import JointsToActuationsModel

from ..detectors.detector import LandmarkData
from ..sources.source import FrameType
from .joint_calculator import JointCalculator

# ── MediaPipe landmark indices ────────────────────────────────────────────────

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

# ── Module-level singletons ──────────────────────────────────────────────────

_model = JointsToActuationsModel()
_consts = AeroHandConstants()

# ── Geometry helpers ─────────────────────────────────────────────────────────


def _unit(v: np.ndarray) -> np.ndarray:
    """Return the unit vector, or zeros if the norm is negligible."""
    n = np.linalg.norm(v)
    return v / n if n >= 1e-9 else np.zeros(3, dtype=np.float64)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _norm_position(val: float, lo: float, hi: float) -> int:
    """Normalise an actuation value to the [0, 65535] servo range."""
    return int(_clamp((val - lo) / (hi - lo), 0.0, 1.0) * 65535)


def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Angle in degrees between two vectors, numerically stable."""
    cross_mag = np.linalg.norm(np.cross(v1, v2))
    dot = float(np.dot(v1, v2))
    return math.degrees(math.atan2(cross_mag, dot))


# ── 3D landmark reconstruction ───────────────────────────────────────────────


def _sample_depth(depth_map: np.ndarray, px: float, py: float) -> float:
    """Bilinearly interpolate the depth map at sub-pixel coordinates.

    Returns 0.0 if any of the four neighbours has an invalid (zero) depth.
    """
    h, w = depth_map.shape[:2]
    x0 = int(math.floor(px))
    y0 = int(math.floor(py))
    x1 = x0 + 1
    y1 = y0 + 1

    if x0 < 0 or y0 < 0 or x1 >= w or y1 >= h:
        return 0.0

    d00 = depth_map[y0, x0]
    d01 = depth_map[y0, x1]
    d10 = depth_map[y1, x0]
    d11 = depth_map[y1, x1]

    # Reject if any neighbour is invalid
    if d00 <= 0 or d01 <= 0 or d10 <= 0 or d11 <= 0:
        return 0.0

    fx = px - x0
    fy = py - y0
    return float(
        d00 * (1 - fx) * (1 - fy)
        + d01 * fx * (1 - fy)
        + d10 * (1 - fx) * fy
        + d11 * fx * fy
    )


def _reconstruct_3d_landmarks(
    hand_landmarks,
    depth_map: np.ndarray,
    intrinsics: rs.intrinsics,
    image_width: int,
    image_height: int,
) -> np.ndarray | None:
    """Deproject 21 MediaPipe landmarks into 3D camera-space points.

    Args:
        hand_landmarks: A single hand's MediaPipe normalised landmarks
                        (list of landmarks with .x, .y attributes in [0,1]).
        depth_map:      H x W float32 aligned depth map in metres.
        intrinsics:     RealSense camera intrinsics for deprojection.
        image_width:    Width of the colour image.
        image_height:   Height of the colour image.

    Returns:
        (21, 3) float64 array of 3D points in metres, or None if too many
        landmarks have invalid depth.
    """
    points = np.zeros((21, 3), dtype=np.float64)
    valid_count = 0

    for i, lm in enumerate(hand_landmarks):
        px = lm.x * image_width
        py = lm.y * image_height

        depth = _sample_depth(depth_map, px, py)
        if depth <= 0:
            # Mark as invalid — will be filled by neighbours later
            points[i] = [float("nan")] * 3
            continue

        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [px, py], depth)
        points[i] = point_3d
        valid_count += 1

    # Need at least 75% of landmarks to have valid depth
    if valid_count < 16:
        return None

    # Fill invalid landmarks by interpolating from neighbours on the hand
    # skeleton graph (parent/child relationships)
    _fill_invalid_landmarks(points)

    return points


# Parent landmark index for each of the 21 landmarks (hand skeleton tree)
_PARENT = [
    -1,  # 0 wrist (root)
    0,   # 1 thumb_cmc <- wrist
    1,   # 2 thumb_mcp <- thumb_cmc
    2,   # 3 thumb_ip  <- thumb_mcp
    3,   # 4 thumb_tip <- thumb_ip
    0,   # 5 index_mcp <- wrist
    5,   # 6 index_pip <- index_mcp
    6,   # 7 index_dip <- index_pip
    7,   # 8 index_tip <- index_dip
    0,   # 9 middle_mcp <- wrist
    9,   # 10 middle_pip
    10,  # 11 middle_dip
    11,  # 12 middle_tip
    0,   # 13 ring_mcp <- wrist
    13,  # 14 ring_pip
    14,  # 15 ring_dip
    15,  # 16 ring_tip
    0,   # 17 pinky_mcp <- wrist
    17,  # 18 pinky_pip
    18,  # 19 pinky_dip
    19,  # 20 pinky_tip
]


def _fill_invalid_landmarks(points: np.ndarray) -> None:
    """Fill NaN landmarks by copying from the nearest valid parent or child."""
    for _ in range(5):  # iterate to propagate through chains
        any_nan = False
        for i in range(21):
            if not np.isnan(points[i, 0]):
                continue
            any_nan = True
            parent = _PARENT[i]
            if parent >= 0 and not np.isnan(points[parent, 0]):
                points[i] = points[parent]
        if not any_nan:
            break


# ── Joint angle extraction from 3D bone vectors ─────────────────────────────


def _flexion_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Compute the flexion angle at joint *b* given points a-b-c.

    Returns degrees in [0, 180].  A straight limb gives ~0 degrees.
    """
    v1 = _unit(a - b)  # proximal bone direction (from joint toward parent)
    v2 = _unit(c - b)  # distal bone direction  (from joint toward child)
    # Angle between the two bones
    raw = _angle_between(v1, v2)
    # 180° = straight, 0° = fully folded → flexion = 180 - raw
    return 180.0 - raw


def _finger_angles_3d(
    pts: np.ndarray,
    mcp_idx: int,
    pip_idx: int,
    dip_idx: int,
    tip_idx: int,
) -> tuple[float, float, float]:
    """Compute MCP-flex, PIP, DIP flexion angles for one finger."""
    mcp_flex = _flexion_angle(pts[WRIST], pts[mcp_idx], pts[pip_idx])
    pip_flex = _flexion_angle(pts[mcp_idx], pts[pip_idx], pts[dip_idx])
    dip_flex = _flexion_angle(pts[pip_idx], pts[dip_idx], pts[tip_idx])
    return (
        _clamp(mcp_flex, 0.0, 90.0),
        _clamp(pip_flex, 0.0, 90.0),
        _clamp(dip_flex, 0.0, 90.0),
    )


def _thumb_angles_3d(pts: np.ndarray) -> tuple[float, float, float, float]:
    """Compute CMC abduction, CMC flexion, MCP, IP angles for the thumb.

    Uses a palm reference frame built from the wrist, index MCP, and pinky MCP
    to decompose thumb CMC motion into abduction (out of palm plane) and
    flexion (within the palm plane).
    """
    wrist = pts[WRIST]
    cmc = pts[THUMB_CMC]
    mcp = pts[THUMB_MCP]
    ip = pts[THUMB_IP]
    tip = pts[THUMB_TIP]

    # --- Build palm coordinate frame ---
    v_middle = pts[MIDDLE_MCP] - wrist
    v_index = pts[INDEX_MCP] - wrist
    palm_normal = _unit(np.cross(v_index, v_middle))
    palm_forward = _unit(v_middle)
    palm_lateral = _unit(np.cross(palm_forward, palm_normal))

    # --- Thumb metacarpal vector (CMC → MCP) ---
    thumb_meta = mcp - cmc
    thumb_meta_norm = np.linalg.norm(thumb_meta)

    if thumb_meta_norm < 1e-9:
        cmc_abd_deg = 0.0
        cmc_flex_deg = 0.0
    else:
        # Out-of-plane component = abduction
        out_of_plane = float(np.dot(thumb_meta, palm_normal))
        in_plane = thumb_meta - out_of_plane * palm_normal
        in_plane_mag = np.linalg.norm(in_plane)

        cmc_abd_deg = abs(math.degrees(math.atan2(out_of_plane, in_plane_mag)))

        # In-plane rotation relative to wrist-CMC direction = flexion
        ref_vec = cmc - wrist
        ref_in_plane = ref_vec - float(np.dot(ref_vec, palm_normal)) * palm_normal
        ref_in_plane = _unit(ref_in_plane)
        meta_in_plane = _unit(in_plane)

        cos_angle = _clamp(float(np.dot(ref_in_plane, meta_in_plane)), -1.0, 1.0)
        sin_angle = float(np.dot(np.cross(ref_in_plane, meta_in_plane), palm_normal))
        cmc_flex_deg = abs(math.degrees(math.atan2(sin_angle, cos_angle)))

    # --- Simple three-point angles for MCP and IP ---
    thumb_mcp_deg = _flexion_angle(cmc, mcp, ip)
    thumb_ip_deg = _flexion_angle(mcp, ip, tip)

    return (
        _clamp(cmc_abd_deg, 0.0, 100.0),
        _clamp(cmc_flex_deg, 0.0, 55.0),
        _clamp(thumb_mcp_deg, 0.0, 90.0),
        _clamp(thumb_ip_deg, 0.0, 90.0),
    )


# ── JointCalculator implementation ──────────────────────────────────────────


class ThreeDJointCalculator(JointCalculator):
    """Computes servo positions from depth-enhanced 3D hand landmarks.

    Requires RGBD frames and RealSense camera intrinsics to deproject
    MediaPipe 2D landmarks into true 3D metric positions, then computes
    joint angles from the resulting bone vectors.

    Args:
        intrinsics: ``pyrealsense2.intrinsics`` from the depth/colour stream.
                    Must be set (via constructor or ``set_intrinsics``) before
                    the first call to ``calculate``.
    """

    def __init__(self, intrinsics: rs.intrinsics | None = None) -> None:
        self._intrinsics = intrinsics

    def set_intrinsics(self, intrinsics: rs.intrinsics) -> None:
        """Update the camera intrinsics (e.g. after opening a new stream)."""
        self._intrinsics = intrinsics

    def supported_frame_types(self) -> frozenset[FrameType]:
        return frozenset({FrameType.RGBD})

    def supported_detectors(self) -> frozenset[str] | None:
        return frozenset({"mediapipe"})

    def _calculate_impl(
        self,
        landmarks: LandmarkData,
        *,
        hand_side: str = "Right",
        depth_map: np.ndarray | None = None,
        image_width: int = 640,
        image_height: int = 480,
        raw_hand_landmarks=None,
    ) -> list[int] | None:
        """Convert RGBD landmarks to 7 normalised servo positions.

        Args:
            landmarks:          LandmarkData from a detector.
            hand_side:          ``"Left"`` or ``"Right"`` — which hand to use.
            depth_map:          H x W float32 depth in metres (from FrameData.depth).
            image_width:        Width of the source colour image.
            image_height:       Height of the source colour image.
            raw_hand_landmarks: The raw MediaPipe ``hand_landmarks`` list entry
                                for the selected hand (normalised pixel coords).
                                Required for depth lookup.

        Returns:
            List of 7 ints in ``[0, 65535]``, or ``None`` if the hand is
            missing or depth data is insufficient.
        """
        if self._intrinsics is None:
            raise RuntimeError(
                "ThreeDJointCalculator requires camera intrinsics. "
                "Call set_intrinsics() before calculate()."
            )
        if depth_map is None or raw_hand_landmarks is None:
            return None

        # Find the matching hand in canonical data (to confirm it exists)
        hand = next(
            (h for h in landmarks.hands if h["handedness"] == hand_side),
            None,
        )
        if hand is None:
            return None

        # Reconstruct 3D landmark positions from depth
        pts = _reconstruct_3d_landmarks(
            raw_hand_landmarks,
            depth_map,
            self._intrinsics,
            image_width,
            image_height,
        )
        if pts is None:
            return None

        # Extract joint angles from 3D positions
        cmc_abd, cmc_flex, thumb_mcp, thumb_ip = _thumb_angles_3d(pts)

        idx_mcp, idx_pip, idx_dip = _finger_angles_3d(
            pts, INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP
        )
        mid_mcp, mid_pip, mid_dip = _finger_angles_3d(
            pts, MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP
        )
        rng_mcp, rng_pip, rng_dip = _finger_angles_3d(
            pts, RING_MCP, RING_PIP, RING_DIP, RING_TIP
        )
        pnk_mcp, pnk_pip, pnk_dip = _finger_angles_3d(
            pts, PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP
        )

        joint_angles = [
            cmc_abd, cmc_flex, thumb_mcp, thumb_ip,
            idx_mcp, idx_pip, idx_dip,
            mid_mcp, mid_pip, mid_dip,
            rng_mcp, rng_pip, rng_dip,
            pnk_mcp, pnk_pip, pnk_dip,
        ]

        # Clamp to anatomical limits
        joint_angles = [
            _clamp(a, _consts.joint_lower_limits[i], _consts.joint_upper_limits[i])
            for i, a in enumerate(joint_angles)
        ]

        # Map through kinematic model to actuations
        actuations = _model.hand_actuations(joint_angles)

        return [
            _norm_position(
                actuations[i],
                _consts.actuation_lower_limits[i],
                _consts.actuation_upper_limits[i],
            )
            for i in range(7)
        ]
