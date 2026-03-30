"""
EEZYbotARM MK1 — Inverse Kinematics Solver
============================================
Calibrated from physical measurements. Accurate to ~0.4cm.

Coordinate system  (units: cm)
───────────────────────────────
  Origin : base servo rotation axis at table level
  x      : left / right  (positive = left when facing the arm)
  y      : forward        (positive = away from the arm)
  z      : up             (positive = above the table)

  S1=90 means the arm points straight forward (+y direction).

Servo mapping
─────────────
  S1 = Ch1 = Base     (0–180°)
  S2 = Ch2 = Shoulder (40–120°)
  S3 = Ch3 = Depth    (55–150°)
  S4 = Ch4 = Claw     (0–180°, not solved by IK — pass through manually)

Arm geometry (EEZYbotARM MK1, parallelogram linkage)
─────────────────────────────────────────────────────
  L1  = 8.0 cm  upper arm  (shoulder pivot → elbow pivot)
  L2  = 8.0 cm  forearm    (elbow pivot → gripper base)
  BSZ = 7.04 cm shoulder pivot height above table

  IMPORTANT: S3 drives the forearm via a 4-bar parallelogram linkage.
  This means S3 controls the ABSOLUTE angle of the forearm (not relative
  to the upper arm). The FK/IK account for this.

Calibration data used
─────────────────────
  S1:90 S2:100 S3:150  →  (0, 15, 10) cm   Far Center
  S1:90 S2:70  S3:115  →  (0, 10, 10) cm   Mid Center
  S1:90 S2:70  S3:80   →  (0,  5, 10) cm   Near Center
  S1:90 S2:50  S3:130  →  (0, 10,  7) cm   Lower Center
  RMS residual: ~0.4 cm

Adding more calibration poses (especially with S1 != 90 and different z)
will improve accuracy. Call calibrate() with expanded CALIBRATION_POSES.

Requires: pip install pyserial scipy
"""

import math
import time
import serial
from dataclasses import dataclass, field
from typing import Optional


# ── Physical constants ────────────────────────────────────────────────────────

L1_CM  = 8.0      # upper arm length (cm)
L2_CM  = 8.0      # forearm length (cm)
BSZ_CM = 7.037    # shoulder pivot height above table (cm)


# ── Servo limits ──────────────────────────────────────────────────────────────

BASE_LIM     = (0,   180)
SHOULDER_LIM = (40,  120)
ELBOW_LIM    = (55,  150)
CLAW_LIM     = (0,   180)


# ── Coupled constraint ────────────────────────────────────────────────────────

COUPLED_ELBOW_LO        = 55
COUPLED_ELBOW_HI        = 150
COUPLED_SHOULDER_MAX_LO = 120
COUPLED_SHOULDER_MAX_HI = 100


# ── Calibrated servo offsets ──────────────────────────────────────────────────
# s2 = t1_degrees + SHOULDER_OFFSET
# s3 = ELBOW_OFFSET - phi2_degrees   <-- note the minus sign

BASE_OFFSET     = 90.0
SHOULDER_OFFSET = 104.94
ELBOW_OFFSET    = 179.975


# ── UART ──────────────────────────────────────────────────────────────────────

BAUD_RATE    = 115200
SEND_DELAY_S = 0.05


# ── Calibration data ──────────────────────────────────────────────────────────

CALIBRATION_POSES = [
    (90, 100, 150,  0, 15, 10),
    (90,  70, 115,  0, 10, 10),
    (90,  70,  80,  0,  5, 10),
    (90,  50, 130,  0, 10,  7),
]


# ─────────────────────────────────────────────────────────────────────────────
# Data types
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class IKResult:
    base_deg:     float
    shoulder_deg: float
    elbow_deg:    float
    reachable:    bool
    clamped:      bool
    clamp_notes:  list


# ─────────────────────────────────────────────────────────────────────────────
# Forward kinematics  —  (S1, S2, S3) → (x, y, z) in cm
# ─────────────────────────────────────────────────────────────────────────────

def forward_kinematics(s1: float, s2: float, s3: float) -> tuple:
    """Given servo angles (degrees), return (x, y, z) in cm."""
    geo_base = math.radians(s1 - BASE_OFFSET)
    t1       = math.radians(s2 - SHOULDER_OFFSET)
    phi2     = math.radians(-(s3 - ELBOW_OFFSET))   # absolute forearm angle

    r     = L1_CM * math.cos(t1) + L2_CM * math.cos(phi2)
    z_rel = L1_CM * math.sin(t1) + L2_CM * math.sin(phi2)

    x = -r * math.sin(geo_base)
    y =  r * math.cos(geo_base)
    z =  BSZ_CM + z_rel
    return round(x, 3), round(y, 3), round(z, 3)


# ─────────────────────────────────────────────────────────────────────────────
# Inverse kinematics  —  (x, y, z) → IKResult
# ─────────────────────────────────────────────────────────────────────────────

def solve_ik(x: float, y: float, z: float, elbow_up: bool = False) -> IKResult:
    """
    Solve IK for target (x, y, z) in cm.
    Returns IKResult with servo angles, safety flags, and clamp notes.
    Raises ValueError if target is outside the geometric reach envelope.
    """
    notes   = []
    clamped = False

    # Base
    geo_base   = math.atan2(-x, y)
    base_servo = math.degrees(geo_base) + BASE_OFFSET

    # Project into shoulder plane
    r = math.sqrt(x**2 + y**2)
    h = z - BSZ_CM

    d = math.sqrt(r**2 + h**2)
    if d > L1_CM + L2_CM:
        raise ValueError(
            f"Target unreachable — {d:.2f}cm from shoulder exceeds max {L1_CM+L2_CM:.1f}cm."
        )
    if d < abs(L1_CM - L2_CM):
        raise ValueError(
            f"Target too close — {d:.2f}cm from shoulder below minimum {abs(L1_CM-L2_CM):.1f}cm."
        )

    # Elbow relative angle (law of cosines)
    cos_t2_rel = max(-1.0, min(1.0,
        (d**2 - L1_CM**2 - L2_CM**2) / (2 * L1_CM * L2_CM)
    ))
    t2_rel = math.acos(cos_t2_rel)
    if elbow_up:
        t2_rel = -t2_rel

    # Shoulder angle
    alpha = math.atan2(h, r)
    cos_b = max(-1.0, min(1.0,
        (d**2 + L1_CM**2 - L2_CM**2) / (2 * d * L1_CM)
    ))
    beta = math.acos(cos_b)
    t1   = (alpha - beta) if not elbow_up else (alpha + beta)

    # Forearm absolute angle
    phi2 = t1 + t2_rel

    # Convert to servo degrees
    shoulder_servo = math.degrees(t1)   + SHOULDER_OFFSET
    elbow_servo    = ELBOW_OFFSET - math.degrees(phi2)

    # Safety clamps
    base_servo,     clamped, notes = _clamp(base_servo,     *BASE_LIM,     'Base',     clamped, notes)
    shoulder_servo, clamped, notes = _clamp(shoulder_servo, *SHOULDER_LIM, 'Shoulder', clamped, notes)
    elbow_servo,    clamped, notes = _clamp(elbow_servo,    *ELBOW_LIM,    'Elbow',    clamped, notes)

    # Coupled constraint
    sh_max = _coupled_shoulder_max(elbow_servo)
    if shoulder_servo > sh_max:
        notes.append(
            f"Shoulder clamped {shoulder_servo:.1f} -> {sh_max:.1f} deg "
            f"(coupled with elbow at {elbow_servo:.1f} deg)"
        )
        shoulder_servo = sh_max
        clamped = True

    return IKResult(
        base_deg     = round(base_servo,     1),
        shoulder_deg = round(shoulder_servo, 1),
        elbow_deg    = round(elbow_servo,    1),
        reachable    = True,
        clamped      = clamped,
        clamp_notes  = notes,
    )


# ─────────────────────────────────────────────────────────────────────────────
# UART interface
# ─────────────────────────────────────────────────────────────────────────────

def send_angles(port: str, result: IKResult,
                claw_deg: Optional[float] = None,
                verbose: bool = True) -> None:
    """Send IK result to STM32 over UART (format: S1:90\\n etc.)"""
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
            ser.write((cmd + "\n").encode("ascii"))
            if verbose:
                print(f"  -> {cmd}")
            time.sleep(SEND_DELAY_S)


def move_to(port: str, x: float, y: float, z: float,
            claw_deg: Optional[float] = None,
            elbow_up: bool = False,
            verbose: bool = True) -> IKResult:
    """
    Solve IK and send to the arm in one call.

    Example
    -------
        result = move_to('COM3', x=0, y=10, z=10)
    """
    result = solve_ik(x, y, z, elbow_up=elbow_up)

    if verbose:
        print(f"Target : ({x}, {y}, {z}) cm")
        print(f"Angles : S1={result.base_deg}  S2={result.shoulder_deg}  S3={result.elbow_deg}")
        if result.clamped:
            print("  Warning - safety clamps applied:")
            for note in result.clamp_notes:
                print(f"    - {note}")

    send_angles(port, result, claw_deg=claw_deg, verbose=verbose)
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Calibration
# ─────────────────────────────────────────────────────────────────────────────

def calibrate(poses=None, verbose=True):
    """
    Refit SHOULDER_OFFSET, ELBOW_OFFSET, and BSZ_CM from calibration poses.
    Add more (S1, S2, S3, x, y, z) tuples to CALIBRATION_POSES for better accuracy.
    """
    from scipy.optimize import differential_evolution, minimize
    global SHOULDER_OFFSET, ELBOW_OFFSET, BSZ_CM

    if poses is None:
        poses = CALIBRATION_POSES

    def _fk(s1, s2, s3, os, oe, bsz):
        gb   = math.radians(s1 - BASE_OFFSET)
        t1   = math.radians(s2 - os)
        phi2 = math.radians(-(s3 - oe))
        r  = L1_CM*math.cos(t1) + L2_CM*math.cos(phi2)
        zr = L1_CM*math.sin(t1) + L2_CM*math.sin(phi2)
        return -r*math.sin(gb), r*math.cos(gb), bsz+zr

    def error(p):
        os, oe, bsz = p
        return sum(
            (px-tx)**2 + (py-ty)**2 + (pz-tz)**2
            for s1,s2,s3,tx,ty,tz in poses
            for px,py,pz in [_fk(s1,s2,s3,os,oe,bsz)]
        )

    r1 = differential_evolution(error, [(60,150),(100,260),(3,15)],
                                 seed=42, maxiter=20000, tol=1e-14, popsize=40)
    r2 = minimize(error, r1.x, method='Nelder-Mead',
                  options={'xatol':1e-10, 'fatol':1e-12, 'maxiter':100000})

    SHOULDER_OFFSET, ELBOW_OFFSET, BSZ_CM = r2.x
    rms = math.sqrt(r2.fun / len(poses))

    if verbose:
        print(f"Calibration complete  RMS: {rms:.4f} cm")
        print(f"  SHOULDER_OFFSET = {SHOULDER_OFFSET:.4f}")
        print(f"  ELBOW_OFFSET    = {ELBOW_OFFSET:.4f}")
        print(f"  BSZ_CM          = {BSZ_CM:.4f}")
    return {'shoulder_offset': SHOULDER_OFFSET,
            'elbow_offset':    ELBOW_OFFSET,
            'bsz_cm':          BSZ_CM,
            'rms_cm':          rms}


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _coupled_shoulder_max(elbow_deg):
    t = (elbow_deg - COUPLED_ELBOW_LO) / (COUPLED_ELBOW_HI - COUPLED_ELBOW_LO)
    t = max(0.0, min(1.0, t))
    return COUPLED_SHOULDER_MAX_LO + t * (COUPLED_SHOULDER_MAX_HI - COUPLED_SHOULDER_MAX_LO)


def _clamp(value, lo, hi, name, already_clamped, notes):
    if value < lo:
        notes.append(f"{name} clamped {value:.1f} -> {lo:.1f} deg (below minimum)")
        return lo, True, notes
    if value > hi:
        notes.append(f"{name} clamped {value:.1f} -> {hi:.1f} deg (above maximum)")
        return hi, True, notes
    return value, already_clamped, notes


# ─────────────────────────────────────────────────────────────────────────────
# Self-test
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=== FK verification ===\n")
    calib  = [(90,100,150,0,15,10),(90,70,115,0,10,10),(90,70,80,0,5,10),(90,50,130,0,10,7)]
    labels = ["Far Center  ","Mid Center  ","Near Center ","Lower Center"]
    for lbl,(s1,s2,s3,tx,ty,tz) in zip(labels,calib):
        px,py,pz = forward_kinematics(s1,s2,s3)
        err = math.sqrt((px-tx)**2+(py-ty)**2+(pz-tz)**2)
        print(f"  {lbl}: pred=({px:.2f},{py:.2f},{pz:.2f})  meas=({tx},{ty},{tz})  err={err:.3f}cm")

    print("\n=== IK round-trip test ===\n")
    targets = [
        ("Far Center  ", 0, 15, 10),
        ("Mid Center  ", 0, 10, 10),
        ("Near Center ", 0,  5, 10),
        ("Lower Center", 0, 10,  7),
        ("Left offset ", -5, 10, 10),
        ("Right offset",  5, 10, 10),
        ("Too far     ", 0, 20,  0),
    ]
    for lbl,x,y,z in targets:
        try:
            r = solve_ik(x,y,z)
            px,py,pz = forward_kinematics(r.base_deg, r.shoulder_deg, r.elbow_deg)
            err = math.sqrt((px-x)**2+(py-y)**2+(pz-z)**2)
            flag = "  CLAMPED" if r.clamped else ""
            print(f"  {lbl}: S1={r.base_deg:5.1f} S2={r.shoulder_deg:5.1f} S3={r.elbow_deg:5.1f}  err={err:.3f}cm{flag}")
        except ValueError as e:
            print(f"  {lbl}: ERROR - {e}")