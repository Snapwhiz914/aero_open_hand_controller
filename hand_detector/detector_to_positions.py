"""
detector_to_positions.py — Convert HandDetector callback data to servo positions.

Pipeline:
    hand_data (dict) → 16 joint angles (°) → 7 actuations → 7 normalized positions [0, 65535]
"""

from __future__ import annotations

import math

import numpy as np

from hand.sdk.aero_hand_constants import AeroHandConstants
from hand.sdk.joints_to_actuations import JointsToActuationsModel

# ── Module-level singletons ───────────────────────────────────────────────────

_model = JointsToActuationsModel()
_consts = AeroHandConstants()


# ── Internal helpers ──────────────────────────────────────────────────────────

def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n >= 1e-9 else np.zeros(3, dtype=np.float64)


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _norm_position(val: float, lo: float, hi: float) -> int:
    return int(_clamp((val - lo) / (hi - lo), 0.0, 1.0) * 65535)


# ── Finger angle extraction (2-D projection data) ─────────────────────────────

def _finger_angles(finger: dict, mcp_pip_coupling: float) -> tuple[float, float, float]:
    """
    Extract (mcp_flex, pip_angle, dip_angle) in degrees from a projected-finger dict.

    finger: {"mcp": [0,0], "pip": [u,v], "dip": [u,v], "tip": [u,v]}

    By construction of project_finger_2d, pip lies exactly on the u-axis (v_pip=0).
    PIP flex = angle that the PIP→DIP vector makes with the u-axis.
    DIP flex = angle that the DIP→TIP vector makes with the u-axis.
    MCP flex is estimated via natural coupling (mcp_pip_coupling × pip_angle).
    """
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


# ── Thumb angle extraction (3-D world-space data) ────────────────────────────

def _thumb_angles(
    thumb: dict,
    palm_normal: np.ndarray,
    cmc_abd_scale: float,
    cmc_abd_offset: float,
) -> tuple[float, float, float, float]:
    """
    Extract (cmc_abd, cmc_flex, mcp, ip) in degrees from the 3-D thumb dict.

    thumb: {"wrist": [x,y,z], "cmc": [x,y,z], "mcp": [0,0,0], "ip": [x,y,z], "tip": [x,y,z]}
    All coordinates are relative to THUMB_MCP (the dict origin).

    palm_normal: unit palm-plane normal from compute_palm_normal.

    CMC angles are computed by decomposing the bend at the CMC joint (WRIST→CMC→MCP
    angle) into abduction and flexion components using palm_normal as the reference:
      - abd_unit  = cross(cmc_joint_axis, palm_normal)  — lateral/abduction direction
      - flex_unit = cross(abd_unit, cmc_joint_axis)     — palmar/flexion direction
      - cmc_abd  = asin(|meta_axis · abd_unit|)
      - cmc_flex = asin(|meta_axis · flex_unit|)
    """
    wrist_vec = np.array(thumb.get("wrist", [0.0, 0.0, 0.0]), dtype=np.float64)
    cmc_vec   = np.array(thumb["cmc"], dtype=np.float64)
    ip_vec    = np.array(thumb["ip"],  dtype=np.float64)
    tip_vec   = np.array(thumb["tip"], dtype=np.float64)

    meta_axis = _unit(-cmc_vec)   # CMC→MCP direction (metacarpal "forward")
    pha_axis  = _unit(ip_vec)     # MCP→IP direction (proximal phalanx)

    # MCP flex: angle between metacarpal axis and proximal phalanx axis
    mcp_deg = math.degrees(math.acos(_clamp(float(np.dot(meta_axis, pha_axis)), -1.0, 1.0)))

    # IP flex: angle at IP joint between proximal and distal phalanx
    ip_to_tip = _unit(tip_vec - ip_vec)
    ip_deg = math.degrees(math.acos(_clamp(float(np.dot(pha_axis, ip_to_tip)), -1.0, 1.0)))

    # CMC joint angles — require the wrist→CMC bone direction as the proximal reference.
    # joint_vec = WRIST→THUMB_CMC expressed relative to THUMB_MCP.
    joint_vec  = cmc_vec - wrist_vec
    joint_norm = float(np.linalg.norm(joint_vec))

    if joint_norm > 1e-6:
        cmc_joint_axis = joint_vec / joint_norm

        # Build an orthonormal abduction/flexion basis relative to palm_normal:
        #   abd_unit  ⊥ cmc_joint_axis, ⊥ palm_normal  → lateral (abduction) axis
        #   flex_unit ⊥ abd_unit, ⊥ cmc_joint_axis     → palmar  (flexion)   axis
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


# ── Public API ────────────────────────────────────────────────────────────────

def hand_data_to_positions(
    hand_data: dict,
    hand_side: str = "Right",
    mcp_pip_coupling: float = 0.5,
    cmc_abd_scale: float = 1.0,
    cmc_abd_offset: float = 0.0,
) -> list[int] | None:
    """
    Convert a HandDetector callback payload to 7 normalised servo positions.

    Args:
        hand_data:        dict from HandDetector callback  {"timestamp": ..., "hands": [...]}
        hand_side:        "Left" or "Right" — which hand to extract
        mcp_pip_coupling: MCP flex estimated as this fraction of PIP angle (natural coupling).
                          Default 0.5; increase if base of finger under-closes, decrease if it
                          over-drives.
        cmc_abd_scale:    Gain applied to the raw CMC abduction proxy (degrees).
                          Default 1.0; calibrate against a fully-abducted thumb.
        cmc_abd_offset:   Offset (degrees) added after scaling the CMC abduction proxy.
                          Default 0.0; calibrate against a fully-adducted thumb.

    Returns:
        list of 7 ints in [0, 65535] (0=extend, 65535=grasp), or None if the
        requested hand is not present in this frame.
    """
    hand = next(
        (h for h in hand_data.get("hands", []) if h["handedness"] == hand_side),
        None,
    )
    if hand is None:
        return None

    fingers = hand["fingers"]
    palm_normal = np.array(hand.get("palm_normal", [0.0, 0.0, -1.0]), dtype=np.float64)

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

    # Clamp to joint limits before passing to kinematic model
    joint_angles = [
        _clamp(a, _consts.joint_lower_limits[i], _consts.joint_upper_limits[i])
        for i, a in enumerate(joint_angles)
    ]

    # Joint angles → 7 actuations (degrees, kinematic model)
    actuations = _model.hand_actuations(joint_angles)

    # Normalize actuations → [0, 65535]
    return [
        _norm_position(
            actuations[i],
            _consts.actuation_lower_limits[i],
            _consts.actuation_upper_limits[i],
        )
        for i in range(7)
    ]
