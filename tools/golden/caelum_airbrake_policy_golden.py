#!/usr/bin/env python3
"""Generate CaelumSufflamen airbrake-policy golden vectors.

The constants and semantics mirror the verified Arduino C++ policy. The output
CSV is meant to seed a later RTL-vs-golden self-checking flow; it is not tied to
any simulator-specific file format.
"""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass

G = 9.80665
POLICY_TARGET_APOGEE_M = 3048.0
POLICY_MIN_ALT_M = 30.0
POLICY_MIN_VZ_MPS = 15.0
POLICY_APOGEE_DEADBAND_M = 5.0
POLICY_VEHICLE_MASS_KG = 2.50
POLICY_RHO_KGPM3 = 1.225
POLICY_CDA_BODY_M2 = 0.0040
POLICY_CDA_BRAKE_M2 = 0.0200
POLICY_MAX_COMMAND01 = 1.0
POLICY_SLEW_PER_SEC = 1.5
POLICY_MAX_EST_AGE_MS = 200
POLICY_BISECTION_STEPS = 18
POLICY_SIGMA_MARGIN_N = 1.0
POLICY_MAX_UNCERTAINTY_MARGIN_M = 20.0

PHASE_IDLE = 0
PHASE_BOOST = 1
PHASE_COAST = 2
PHASE_BRAKE = 3
PHASE_DESCENT = 4
ARM_DISARMED = 0
ARM_SAFE = 1
ARM_ARMED = 2


@dataclass(frozen=True)
class Case:
    name: str
    h_m: float
    v_mps: float
    p00: float
    est_valid: bool = True
    est_age_ms: int = 0
    runtime_enable: bool = True
    arm_state: int = ARM_ARMED
    software_arm_token: bool = True
    phase: int = PHASE_COAST
    dt_s: float = 0.02
    prev_cmd: float = 0.0


def clamp01(x: float) -> float:
    if not math.isfinite(x):
        return 0.0
    return min(1.0, max(0.0, x))


def drag_k(command01: float) -> float:
    u = clamp01(command01)
    cda = POLICY_CDA_BODY_M2 + u * POLICY_CDA_BRAKE_M2
    if POLICY_RHO_KGPM3 <= 0 or POLICY_VEHICLE_MASS_KG <= 0 or cda < 0:
        return 0.0
    return POLICY_RHO_KGPM3 * cda / (2.0 * POLICY_VEHICLE_MASS_KG)


def predict_apogee_m(h_m: float, v_mps: float, command01: float) -> float:
    if not math.isfinite(h_m) or not math.isfinite(v_mps):
        return math.nan
    if v_mps <= 0.0:
        return h_m
    k = drag_k(command01)
    v2 = v_mps * v_mps
    if not math.isfinite(k) or k < 1.0e-7:
        return h_m + v2 / (2.0 * G)
    arg = 1.0 + (k * v2) / G
    if not math.isfinite(arg) or arg <= 0.0:
        return math.nan
    return h_m + math.log(arg) / (2.0 * k)


def uncertainty_margin_m(p00: float) -> float:
    if not math.isfinite(p00) or p00 < 0.0:
        return 0.0
    margin = POLICY_SIGMA_MARGIN_N * math.sqrt(p00)
    if not math.isfinite(margin) or margin < 0.0:
        return 0.0
    return min(POLICY_MAX_UNCERTAINTY_MARGIN_M, margin)


def solve_command01(h_m: float, v_mps: float, target_m: float) -> float:
    if not all(math.isfinite(x) for x in (h_m, v_mps, target_m)):
        return 0.0
    if v_mps <= 0.0:
        return 0.0
    u_max = clamp01(POLICY_MAX_COMMAND01)
    ap0 = predict_apogee_m(h_m, v_mps, 0.0)
    if not math.isfinite(ap0) or ap0 <= target_m + POLICY_APOGEE_DEADBAND_M:
        return 0.0
    apmax = predict_apogee_m(h_m, v_mps, u_max)
    if not math.isfinite(apmax):
        return 0.0
    if apmax > target_m:
        return u_max
    lo, hi = 0.0, u_max
    for _ in range(POLICY_BISECTION_STEPS):
        mid = 0.5 * (lo + hi)
        apmid = predict_apogee_m(h_m, v_mps, mid)
        if not math.isfinite(apmid):
            return 0.0
        if apmid > target_m:
            lo = mid
        else:
            hi = mid
    return clamp01(0.5 * (lo + hi))


def apply_slew(desired: float, prev: float, dt_s: float) -> float:
    desired = clamp01(desired)
    if not math.isfinite(dt_s) or dt_s < 0.0 or dt_s > 1.0:
        dt_s = 0.0
    max_step = POLICY_SLEW_PER_SEC * dt_s
    return clamp01(min(max(desired, prev - max_step), prev + max_step))


def compute(case: Case) -> dict[str, float | int | str]:
    gate = (
        case.runtime_enable
        and case.arm_state == ARM_ARMED
        and case.software_arm_token
        and case.phase in (PHASE_COAST, PHASE_BRAKE)
        and case.est_valid
        and math.isfinite(case.h_m)
        and math.isfinite(case.v_mps)
        and case.est_age_ms <= POLICY_MAX_EST_AGE_MS
        and case.h_m >= POLICY_MIN_ALT_M
        and case.v_mps >= POLICY_MIN_VZ_MPS
    )

    margin = uncertainty_margin_m(case.p00)
    target_eff = max(0.0, POLICY_TARGET_APOGEE_M - margin)
    ap0 = predict_apogee_m(case.h_m, case.v_mps, 0.0)
    apmax = predict_apogee_m(case.h_m, case.v_mps, POLICY_MAX_COMMAND01)
    desired = solve_command01(case.h_m, case.v_mps, target_eff) if gate else 0.0
    cmd = apply_slew(desired, case.prev_cmd, case.dt_s) if gate else 0.0
    valid = gate and cmd > 0.0
    servo_us = 1000 + round(cmd * 1000.0) if valid else 1000

    return {
        "name": case.name,
        "h_m": case.h_m,
        "v_mps": case.v_mps,
        "p00": case.p00,
        "phase": case.phase,
        "arm_state": case.arm_state,
        "runtime_enable": int(case.runtime_enable),
        "software_arm_token": int(case.software_arm_token),
        "est_valid": int(case.est_valid),
        "est_age_ms": case.est_age_ms,
        "gate": int(gate),
        "target_nominal_m": POLICY_TARGET_APOGEE_M,
        "uncertainty_margin_m": margin,
        "target_effective_m": target_eff,
        "apogee_no_brake_m": ap0,
        "apogee_full_brake_m": apmax,
        "apogee_error_m": ap0 - target_eff if math.isfinite(ap0) else math.nan,
        "desired_command01": desired,
        "slewed_command01": cmd,
        "policy_valid": int(valid),
        "servo_us": servo_us,
    }


def default_cases() -> list[Case]:
    return [
        Case("policy_disabled", 500.0, 80.0, 1.0, runtime_enable=False),
        Case("disarmed", 500.0, 80.0, 1.0, arm_state=ARM_DISARMED),
        Case("missing_token", 500.0, 80.0, 1.0, software_arm_token=False),
        Case("idle_phase", 500.0, 80.0, 1.0, phase=PHASE_IDLE),
        Case("boost_phase", 500.0, 80.0, 1.0, phase=PHASE_BOOST),
        Case("descent_phase", 500.0, 80.0, 1.0, phase=PHASE_DESCENT),
        Case("invalid_estimator", 500.0, 80.0, 1.0, est_valid=False),
        Case("stale_estimator", 500.0, 80.0, 1.0, est_age_ms=201),
        Case("below_altitude_gate", 10.0, 80.0, 1.0),
        Case("below_speed_gate", 500.0, 5.0, 1.0),
        Case("negative_speed", 500.0, -5.0, 1.0),
        Case("below_target", 500.0, 25.0, 1.0),
        Case("deadband", 3040.0, 5.0, 1.0),
        Case("reachable_coast", 2500.0, 120.0, 1.0),
        Case("full_brake_saturates", 3000.0, 180.0, 1.0),
        Case("brake_phase_valid", 2500.0, 120.0, 4.0, phase=PHASE_BRAKE),
        Case("large_uncertainty_clamped", 2500.0, 120.0, 10000.0),
        Case("slew_from_zero", 2500.0, 120.0, 1.0, dt_s=0.02, prev_cmd=0.0),
        Case("slew_after_gap", 2500.0, 120.0, 1.0, dt_s=2.0, prev_cmd=0.5),
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", default="caelum_airbrake_policy_golden.csv")
    args = parser.parse_args()

    rows = [compute(c) for c in default_cases()]
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} cases to {args.output}")


if __name__ == "__main__":
    main()
