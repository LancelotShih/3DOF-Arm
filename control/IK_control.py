"""
EEZYbotARM MK1 — Inverse Kinematics Solver
============================================
Computes (base, shoulder, elbow) servo angles from a target (x, y, z) in mm,
enforces per-joint limits and the coupled shoulder/elbow constraint, then
sends the result over UART in the format your STM32 expects: S1:90\n etc.

Coordinate frame (looking down from above):
  - Origin: base servo axis, at ground level
  - x: forward (straight ahead = +x)
  - y: left/right
  - z: up

Servo mapping (from your GUI):
  Ch1 = S1 = Base
  Ch2 = S2 = Shoulder
  Ch3 = S3 = Elbow
  Ch4 = S4 = Claw (not solved by IK — pass through manually)

CALIBRATION NOTE:
  Servo zero ≠ geometric zero. The offsets in SERVO_OFFSET below convert
  geometric angles (from the IK math) to the degree values your STM32 servo
  driver expects. Tune these by commanding a known pose, measuring the actual
  joint angles with a protractor, and adjusting until they match.
"""

import math
import time
import serial
from dataclasses import dataclass
from typing import Optional


# ---------------------------------------------------------------------------
# Physical constants — EEZYbotARM MK1 defaults
# ---------------------------------------------------------------------------
L1_MM = 80.0   # upper arm link length (mm)
L2_MM = 80.0   # forearm link length   (mm)

# Height of the shoulder pivot above the base reference plane (mm).
# Measure from the centre of the base servo to the centre of the shoulder servo.
BASE_TO_SHOULDER_Z_MM = 75.0   # <-- measure and update for your build


# ---------------------------------------------------------------------------
# Joint limits (servo degrees, as enforced by the STM32 / your GUI)
# ---------------------------------------------------------------------------
BASE_LIM     = (0,   180)
SHOULDER_LIM = (40,  120)
ELBOW_LIM    = (55,  150)
CLAW_LIM     = (0,   180)


# ---------------------------------------------------------------------------
# Coupled constraint: shoulder_max decreases as elbow increases
#
# Observed: at elbow=150° the shoulder must stay ≤ 100° to avoid self-collision.
# Modelled as a linear slope from (elbow=55, sh_max=120) to (elbow=150, sh_max=100).
# Adjust SHOULDER_MAX_AT_ELBOW_MAX if you observe the cliff is sharper/softer.
# ---------------------------------------------------------------------------
COUPLED_ELBOW_LO         = 55    # elbow angle where coupling starts
COUPLED_ELBOW_HI         = 150   # elbow angle at worst-case interference
COUPLED_SHOULDER_MAX_LO  = 120   # shoulder max when elbow = COUPLED_ELBOW_LO
COUPLED_SHOULDER_MAX_HI  = 100   # shoulder max when elbow = COUPLED_ELBOW_HI


# ---------------------------------------------------------------------------
# Servo calibration offsets
#
# servo_deg = geo_deg + SERVO_OFFSET[joint]
#
# How to calibrate:
#   1. Command the arm to a known neutral pose via the GUI (e.g. arm pointing
#      straight up: shoulder=90°, elbow=90° geometrically).
#   2. Note the servo degree values shown in the GUI for that pose.
#   3. offset = servo_gui_value - geometric_value_for_that_pose
#
# These are PLACEHOLDER values — you must tune them on your physical arm.
# ---------------------------------------------------------------------------
SERVO_OFFSET = {
    'base':     90.0,   # geometric 0° (pointing right) → servo 90° (centre)
    'shoulder': 90.0,   # geometric 0° (horizontal)     → servo 90°
    'elbow':    90.0,   # geometric 0° (straight)        → servo 90°
}


# ---------------------------------------------------------------------------
# UART settings
# ---------------------------------------------------------------------------
BAUD_RATE    = 115200
SEND_DELAY_S = 0.05   # pause between each S1/S2/S3 command (seconds)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------
@dataclass
class IKResult:
    base_deg:     float
    shoulder_deg: float
    elbow_deg:    float
    reachable:    bool
    clamped:      bool          # True if any joint was pulled back by safety logic
    clamp_notes:  list[str]     # human-readable list of what was clamped and why


# ---------------------------------------------------------------------------
# Core IK solver
# ---------------------------------------------------------------------------
def solve_ik(x: float, y: float, z: float,
             elbow_up: bool = False) -> IKResult:
    """
    Solve inverse kinematics for a target point (x, y, z) in mm.

    Parameters
    ----------
    x, y, z   : target position in mm (origin = base servo axis at ground)
    elbow_up  : choose elbow-up configuration instead of elbow-down

    Returns
    -------
    IKResult with servo angles in degrees, safety flags, and clamp notes.
    Raises ValueError if target is geometrically unreachable before clamping.
    """
    notes = []

    # --- Base angle ---------------------------------------------------
    base_geo = math.degrees(math.atan2(y, x))          # -180 to +180
    base_servo = base_geo + SERVO_OFFSET['base']

    # --- Project into shoulder-plane (r, h) ---------------------------
    r = math.sqrt(x**2 + y**2)                         # horizontal reach
    h = z - BASE_TO_SHOULDER_Z_MM                       # height rel. to shoulder

    d = math.sqrt(r**2 + h**2)                         # distance shoulder → target
    max_reach = L1_MM + L2_MM
    min_reach = abs(L1_MM - L2_MM)

    if d > max_reach:
        raise ValueError(
            f"Target unreachable: distance {d:.1f}mm exceeds max reach "
            f"{max_reach:.1f}mm. Move target closer."
        )
    if d < min_reach and min_reach > 0:
        raise ValueError(
            f"Target too close: distance {d:.1f}mm below minimum reach "
            f"{min_reach:.1f}mm. Move target further away."
        )

    # --- Elbow angle (law of cosines) ---------------------------------
    cos_t2 = (d**2 - L1_MM**2 - L2_MM**2) / (2 * L1_MM * L2_MM)
    cos_t2 = max(-1.0, min(1.0, cos_t2))               # numerical guard
    t2 = math.acos(cos_t2)
    if elbow_up:
        t2 = -t2

    elbow_geo   = math.degrees(t2)
    elbow_servo = elbow_geo + SERVO_OFFSET['elbow']

    # --- Shoulder angle -----------------------------------------------
    alpha   = math.atan2(h, r)
    cos_b   = (d**2 + L1_MM**2 - L2_MM**2) / (2 * d * L1_MM)
    cos_b   = max(-1.0, min(1.0, cos_b))
    beta    = math.acos(cos_b)
    t1      = (alpha - beta) if not elbow_up else (alpha + beta)

    shoulder_geo   = math.degrees(t1)
    shoulder_servo = shoulder_geo + SERVO_OFFSET['shoulder']

    # --- Safety layer -------------------------------------------------
    clamped = False

    base_servo, clamped, notes = _clamp(
        base_servo, *BASE_LIM, 'Base', clamped, notes)
    shoulder_servo, clamped, notes = _clamp(
        shoulder_servo, *SHOULDER_LIM, 'Shoulder', clamped, notes)
    elbow_servo, clamped, notes = _clamp(
        elbow_servo, *ELBOW_LIM, 'Elbow', clamped, notes)

    # Coupled constraint — must be checked AFTER individual clamps
    sh_max = _coupled_shoulder_max(elbow_servo)
    if shoulder_servo > sh_max:
        notes.append(
            f"Shoulder clamped {shoulder_servo:.1f}° → {sh_max:.1f}° "
            f"(coupled constraint with elbow at {elbow_servo:.1f}°)"
        )
        shoulder_servo = sh_max
        clamped = True

    return IKResult(
        base_deg=round(base_servo, 1),
        shoulder_deg=round(shoulder_servo, 1),
        elbow_deg=round(elbow_servo, 1),
        reachable=True,
        clamped=clamped,
        clamp_notes=notes,
    )


# ---------------------------------------------------------------------------
# UART interface
# ---------------------------------------------------------------------------
def send_angles(port: str, result: IKResult,
                claw_deg: Optional[float] = None,
                verbose: bool = True) -> None:
    """
    Send solved angles to the STM32 over UART.

    Commands sent (one per line):
        S1:<base>\\n
        S2:<shoulder>\\n
        S3:<elbow>\\n
        S4:<claw>\\n   (only if claw_deg is provided)

    Parameters
    ----------
    port      : serial port string, e.g. 'COM3' or '/dev/ttyACM0'
    result    : IKResult from solve_ik()
    claw_deg  : optional claw angle in degrees (0–180). If None, claw is not moved.
    verbose   : print sent commands to stdout
    """
    if not result.reachable:
        raise RuntimeError("Cannot send — IKResult is not reachable.")

    commands = [
        f"S1:{int(round(result.base_deg))}",
        f"S2:{int(round(result.shoulder_deg))}",
        f"S3:{int(round(result.elbow_deg))}",
    ]

    if claw_deg is not None:
        claw_safe = max(CLAW_LIM[0], min(CLAW_LIM[1], claw_deg))
        commands.append(f"S4:{int(round(claw_safe))}")

    with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
        for cmd in commands:
            line = cmd + "\n"
            ser.write(line.encode("ascii"))
            if verbose:
                print(f"  → {cmd}")
            time.sleep(SEND_DELAY_S)


def move_to(port: str, x: float, y: float, z: float,
            claw_deg: Optional[float] = None,
            elbow_up: bool = False,
            verbose: bool = True) -> IKResult:
    """
    Convenience one-liner: solve IK and send in one call.

    Example
    -------
        result = move_to('COM3', x=100, y=0, z=80)
    """
    result = solve_ik(x, y, z, elbow_up=elbow_up)

    if verbose:
        print(f"Target: ({x}, {y}, {z}) mm")
        print(f"Angles: base={result.base_deg}°  "
              f"shoulder={result.shoulder_deg}°  "
              f"elbow={result.elbow_deg}°")
        if result.clamped:
            print("  ⚠ Safety clamps applied:")
            for note in result.clamp_notes:
                print(f"    • {note}")

    send_angles(port, result, claw_deg=claw_deg, verbose=verbose)
    return result


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------
def _coupled_shoulder_max(elbow_deg: float) -> float:
    """Linear interpolation: shoulder ceiling as a function of elbow angle."""
    t = (elbow_deg - COUPLED_ELBOW_LO) / (COUPLED_ELBOW_HI - COUPLED_ELBOW_LO)
    t = max(0.0, min(1.0, t))
    return COUPLED_SHOULDER_MAX_LO + t * (COUPLED_SHOULDER_MAX_HI - COUPLED_SHOULDER_MAX_LO)


def _clamp(value: float, lo: float, hi: float,
           name: str, already_clamped: bool,
           notes: list) -> tuple[float, bool, list]:
    """Clamp value to [lo, hi], appending a note if it was out of range."""
    if value < lo:
        notes.append(f"{name} clamped {value:.1f}° → {lo:.1f}° (below minimum)")
        return lo, True, notes
    if value > hi:
        notes.append(f"{name} clamped {value:.1f}° → {hi:.1f}° (above maximum)")
        return hi, True, notes
    return value, already_clamped, notes


# ---------------------------------------------------------------------------
# Quick sanity check — run this file directly to test without hardware
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=== EEZYbotARM MK1 IK Solver — self-test ===\n")

    test_cases = [
        ("Straight ahead, mid height",  100,  0,  80),
        ("Slight left",                  80,  40, 60),
        ("Low pose",                    110,  0,  20),
        ("Near max reach",              150,  0,  80),
        ("Too far (expect error)",      200,  0,   0),
    ]

    for label, x, y, z in test_cases:
        print(f"[{label}]  target=({x}, {y}, {z})")
        try:
            r = solve_ik(x, y, z)
            print(f"  base={r.base_deg}°  shoulder={r.shoulder_deg}°  elbow={r.elbow_deg}°")
            if r.clamped:
                for note in r.clamp_notes:
                    print(f"  ⚠ {note}")
        except ValueError as e:
            print(f"  ✗ {e}")
        print()