"""
2d_joint_calculator.py — Convert 2D-projected hand landmarks to servo positions.

Uses the same algorithm as the original detector_to_positions.py: extracts
joint angles from the 2D finger projections and 3D thumb data produced by
MediaPipe, then maps them through the kinematic model to servo positions.

This calculator works with any frame type (it only uses the landmark
projections, not depth data) and with the ``"mediapipe"`` detector.
"""

from __future__ import annotations

import math

import numpy as np

from hand.sdk.aero_hand_constants import AeroHandConstants
from hand.sdk.joints_to_actuations import JointsToActuationsModel

from ..detectors.detector import LandmarkData
from ..sources.source import FrameType
from .joint_calculator import JointCalculator

# ── Module-level singletons ─────────────────────────────────────────────────

_model = JointsToActuationsModel()
_consts = AeroHandConstants()


# ── Internal helpers ─────────────────────────────────────────────────────────

def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n >= 1e-9 else np.zeros(3, dtype=np.float64)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _norm_position(val: float, lo: float, hi: float) -> int:
    return int(_clamp((val - lo) / (hi - lo), 0.0, 1.0) * 65535)


# ── Finger angle extraction (2-D projection data) ───────────────────────────

def _finger_angles(finger: dict, mcp_pip_coupling: float) -> tuple[float, float, float]:
    u_pip, _     = finger["pip"]
    u_dip, v_dip = finger["dip"]
    u_tip, v_tip = finger["tip"]

    pip_angle = math.degrees(math.atan2(abs(v_dip), u_dip - u_pip))
    dip_angle = math.degrees(math.atan2(abs(v_tip - v_dip), u_tip - u_dip))
    mcp_flex  = mcp_pip_coupling * pip_angle

    return (
        _clamp(mcp_flex,  0.0, 90.0),
        _clamp(pip_angle, 0.0, 90.0),
        _clamp(dip_angle, 0.0, 90.0),
    )


# ── Thumb angle extraction (3-D world-space data) ───────────────────────────

def _thumb_angles(
    thumb: dict,
    palm_normal: np.ndarray,
    cmc_abd_scale: float,
    cmc_abd_offset: float,
) -> tuple[float, float, float, float]:
    wrist_vec = np.array(thumb.get("wrist", [0.0, 0.0, 0.0]), dtype=np.float64)
    cmc_vec   = np.array(thumb["cmc"], dtype=np.float64)
    ip_vec    = np.array(thumb["ip"],  dtype=np.float64)
    tip_vec   = np.array(thumb["tip"], dtype=np.float64)

    meta_axis = _unit(-cmc_vec)
    pha_axis  = _unit(ip_vec)

    mcp_deg = math.degrees(math.acos(_clamp(float(np.dot(meta_axis, pha_axis)), -1.0, 1.0)))

    ip_to_tip = _unit(tip_vec - ip_vec)
    ip_deg = math.degrees(math.acos(_clamp(float(np.dot(pha_axis, ip_to_tip)), -1.0, 1.0)))

    joint_vec  = cmc_vec - wrist_vec
    joint_norm = float(np.linalg.norm(joint_vec))

    if joint_norm > 1e-6:
        cmc_joint_axis = joint_vec / joint_norm

        abd_ref      = np.cross(cmc_joint_axis, palm_normal)
        abd_ref_norm = float(np.linalg.norm(abd_ref))

        if abd_ref_norm > 1e-6:
            abd_unit  = abd_ref / abd_ref_norm
            flex_unit = _unit(np.cross(abd_unit, cmc_joint_axis))

            cmc_abd_raw  = math.degrees(math.asin(
                _clamp(abs(float(np.dot(meta_axis, abd_unit))),  0.0, 1.0)))
            cmc_flex_raw = math.degrees(math.asin(
                _clamp(abs(float(np.dot(meta_axis, flex_unit))), 0.0, 1.0)))
        else:
            cmc_abd_raw = cmc_flex_raw = 0.0
    else:
        cmc_abd_raw = cmc_flex_raw = 0.0

    cmc_abd_deg  = _clamp(cmc_abd_raw * cmc_abd_scale + cmc_abd_offset, 0.0, 100.0)
    cmc_flex_deg = _clamp(cmc_flex_raw, 0.0, 55.0)

    return (
        cmc_abd_deg,
        cmc_flex_deg,
        _clamp(mcp_deg, 0.0, 90.0),
        _clamp(ip_deg,  0.0, 90.0),
    )


# ── JointCalculator implementation ──────────────────────────────────────────

class TwoDJointCalculator(JointCalculator):
    """Computes servo positions from 2D-projected hand landmark data.

    Compatible with any frame type (RGB or RGBD) and the ``"mediapipe"``
    detector.
    """

    def supported_frame_types(self) -> frozenset[FrameType]:
        return frozenset({FrameType.RGB, FrameType.RGBD})

    def supported_detectors(self) -> frozenset[str] | None:
        return frozenset({"mediapipe"})

    def _calculate_impl(
        self,
        landmarks: LandmarkData,
        *,
        hand_side: str = "Right",
        mcp_pip_coupling: float = 0.5,
        cmc_abd_scale: float = 1.0,
        cmc_abd_offset: float = 0.0,
    ) -> list[int] | None:
        """Convert landmarks to 7 normalised servo positions.

        Args:
            landmarks:        LandmarkData from a detector.
            hand_side:        ``"Left"`` or ``"Right"`` — which hand to extract.
            mcp_pip_coupling: MCP flex estimated as this fraction of PIP angle.
            cmc_abd_scale:    Gain applied to the raw CMC abduction proxy.
            cmc_abd_offset:   Offset added after scaling the CMC abduction proxy.

        Returns:
            List of 7 ints in ``[0, 65535]``, or ``None`` if the requested
            hand is not present.
        """
        hand = next(
            (h for h in landmarks.hands if h["handedness"] == hand_side),
            None,
        )
        if hand is None:
            return None

        fingers = hand["fingers"]
        palm_normal = np.array(
            hand.get("palm_normal", [0.0, 0.0, -1.0]), dtype=np.float64
        )

        cmc_abd, cmc_flex, thumb_mcp, thumb_ip = _thumb_angles(
            fingers["thumb"], palm_normal, cmc_abd_scale, cmc_abd_offset
        )
        idx_mcp, idx_pip, idx_dip  = _finger_angles(fingers["index"],  mcp_pip_coupling)
        mid_mcp, mid_pip, mid_dip  = _finger_angles(fingers["middle"], mcp_pip_coupling)
        rng_mcp, rng_pip, rng_dip  = _finger_angles(fingers["ring"],   mcp_pip_coupling)
        pnk_mcp, pnk_pip, pnk_dip = _finger_angles(fingers["pinky"],  mcp_pip_coupling)

        joint_angles = [
            cmc_abd, cmc_flex, thumb_mcp, thumb_ip,
            idx_mcp, idx_pip,  idx_dip,
            mid_mcp, mid_pip,  mid_dip,
            rng_mcp, rng_pip,  rng_dip,
            pnk_mcp, pnk_pip,  pnk_dip,
        ]

        joint_angles = [
            _clamp(a, _consts.joint_lower_limits[i], _consts.joint_upper_limits[i])
            for i, a in enumerate(joint_angles)
        ]

        actuations = _model.hand_actuations(joint_angles)

        return [
            _norm_position(
                actuations[i],
                _consts.actuation_lower_limits[i],
                _consts.actuation_upper_limits[i],
            )
            for i in range(7)
        ]


# ── Backward-compatible function wrapper ─────────────────────────────────────

_default_calculator = TwoDJointCalculator()


def hand_data_to_positions(
    hand_data: dict,
    hand_side: str = "Right",
    mcp_pip_coupling: float = 0.5,
    cmc_abd_scale: float = 1.0,
    cmc_abd_offset: float = 0.0,
) -> list[int] | None:
    """Backward-compatible wrapper around TwoDJointCalculator.

    Accepts the raw hand-data dict (as produced by the old HandDetector
    callbacks) without requiring a LandmarkData object.
    """
    landmarks = LandmarkData(
        hands=hand_data.get("hands", []),
        timestamp=hand_data.get("timestamp", 0.0),
        frame_type=FrameType.RGB,
        detector_name="mediapipe",
    )
    return _default_calculator.calculate(
        landmarks,
        hand_side=hand_side,
        mcp_pip_coupling=mcp_pip_coupling,
        cmc_abd_scale=cmc_abd_scale,
        cmc_abd_offset=cmc_abd_offset,
    )
