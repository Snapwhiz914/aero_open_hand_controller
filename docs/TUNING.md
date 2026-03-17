# Hand Detector → Servo Position Tuning Guide

## 1. Overview

The conversion pipeline in `detector_to_positions.py` has three stages:

1. **Joint angles** — raw MediaPipe landmark geometry is converted to 16 joint angles (degrees).
2. **Actuations** — `JointsToActuationsModel.hand_actuations()` maps 16 joint angles to 7 actuator values (degrees of motor rotation).
3. **Normalized positions** — actuations are linearly mapped to [0, 65535] using `AeroHandConstants` limits (0 = fully extended, 65535 = fully grasped).

Tunable constants live at the top of `detector_to_positions.py`.

---

## 2. Finger Joint Coupling (`mcp_flex` estimate)

MediaPipe's 2-D projected finger data gives PIP and DIP angles directly. MCP flex is not independently observable from the projection, so it is estimated:

```
mcp_flex = MCP_PIP_COUPLING × pip_angle
```

**Default:** `MCP_PIP_COUPLING = 0.5`

**Adjusting:**
- Change `MCP_PIP_COUPLING` in `detector_to_positions.py`.
- If the finger doesn't close/open far enough at the base, **increase** the constant.
- If the MCP joint over-drives (base of finger bends too aggressively), **decrease** it.

**Diagnostic:** add a debug print to `_finger_angles()` and log `pip_angle` alongside the observed servo position for actuators 3–6. Ideal coupling produces natural-looking closing without the base joint hitting its limit before the fingertip closes.

---

## 3. Thumb CMC Abduction

The CMC abduction proxy captures how far the thumb metacarpal is displaced perpendicular to the MCP flex plane.

**Primary (normal case):**
```
n_flex      = normalize(cross(meta_axis, pha_axis))
cmc_abd_raw = degrees(asin(abs(dot(cmc_unit, n_flex))))
```

**Fallback (when MCP is near straight, n_flex ill-conditioned):**
```
cmc_abd_raw = degrees(asin(abs(cmc_unit.z)))
```
MediaPipe world-space Z correlates with radial abduction (thumb spreading away from palm).

**Scale/offset calibration:**
```
cmc_abd_deg = clip(cmc_abd_raw * CMC_ABD_SCALE + CMC_ABD_OFFSET, 0, 100)
```

**Calibration procedure:**
1. Hold hand flat, thumb fully adducted (closed against palm) — note `cmc_abd_raw`. Set `CMC_ABD_OFFSET = -cmc_abd_raw` so this reads 0°.
2. Spread thumb maximally — note `cmc_abd_raw`. Set `CMC_ABD_SCALE = 100 / (cmc_abd_raw + CMC_ABD_OFFSET)`.
3. Re-run and verify actuator 0 (`thumb_cmc_abd_act`) moves from 0 to 65535 across that range.

---

## 4. Thumb CMC Flexion

```
cmc_in_plane  = dot(cmc_unit, meta_axis)
cmc_flex_deg  = clip(degrees(acos(abs(cmc_in_plane))), 0, 55)
```

When `cmc_vec` and `meta_axis` are antiparallel (thumb straight), `dot = -1`, `acos(1) = 0°`. As the metacarpal rotates away from alignment, flexion increases toward 55°.

**If flexion feels inverted** (robot thumb flexes when human extends): negate the dot product:
```python
cmc_in_plane = -float(np.dot(cmc_unit, meta_axis))
```
This swaps the 0° and 90° ends.

**Upper limit:** 55° per `AeroHandConstants.joint_upper_limits[1]`.

---

## 5. Output Range Sanity Check

Before a hardware run, add a debug loop:

```python
for i, (act, lo, hi) in enumerate(zip(actuations, _consts.actuation_lower_limits, _consts.actuation_upper_limits)):
    print(f"  act[{i}]={act:.1f}  limits=[{lo:.1f}, {hi:.1f}]")
```

All actuations should stay within their limits during a live demo. If any frequently exceed the bounds, the corresponding joint angle estimate is likely over-scaled.

---

## 6. Jitter Suppression (optional)

MediaPipe landmark positions can jitter by a few pixels per frame, causing small oscillations in servo commands. A simple exponential moving average (EMA) suppresses this:

```python
_smoothed = None

def smoothed_positions(hand_data, hand_side="Right", alpha=0.3):
    global _smoothed
    raw = hand_data_to_positions(hand_data, hand_side)
    if raw is None:
        return None
    if _smoothed is None:
        _smoothed = raw
    else:
        _smoothed = [int(alpha * r + (1 - alpha) * s) for r, s in zip(raw, _smoothed)]
    return _smoothed
```

**Recommended α:** 0.2–0.5. Lower α = smoother but more lag; higher α = more responsive but jitterier.

---

## 7. Camera / Handedness Convention

MediaPipe reports handedness from the model's perspective (mirrored from a front-facing camera):
- A human right hand viewed in a mirror is reported as `"Left"`.
- Pass `hand_side="Right"` when you want to mirror a right human hand onto the robot right hand.
- Swap to `"Left"` if the detected label is inverted for your camera setup.

Check `HandDetector` callback output directly to confirm which label your physical hand receives before wiring up `hand_data_to_positions`.
