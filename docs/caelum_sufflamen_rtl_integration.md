# CaelumSufflamen RTL Integration Notes

This document records the first CaelumSufflamen-to-CaelumFusion RTL integration pass.
It is intentionally explicit about what is now implemented, what remains an
approximation, and what must still be verified before this branch can be described
as a complete firmware-equivalent flight-control implementation.

## Source of Truth

The behavioral reference is the verified Arduino C++ CaelumSufflamen firmware:

- `airbrake_policy.cpp/.h`
- `attitude.cpp/.h`
- `estimation.cpp/.h`
- `kalman_alt2.cpp/.h`
- `flight_phase.cpp/.h`
- `config.h`
- `data_types.h`

The architecture references are:

- `Caelum_Sufflamen_IEEE_Report (13).pdf`
- `CaelumFusion_Flight_Control_System (2).pdf`

Version rule: when a report describes an older placeholder or simplified model
that conflicts with the current verified Arduino source, preserve the current
Arduino behavior and document the delta.

## Current RTL Baseline Reviewed

The current CaelumFusion RTL is a Basys 3 FPGA telemetry and visualization
platform built around:

- SYS-domain sensor acquisition and derived-state publication;
- committed snapshots with validity, status, sequence, and age metadata;
- explicit SYS-to-PIX visualization bundle CDC;
- VGA/HUD rendering; and
- a simplified apogee-authority model intended for display, telemetry, and gated
  actuator demand.

The main RTL files reviewed for this pass were:

- `apogee_authority_policy_sys.v`
- `authority_gate_phase_sys.v`
- `flight_attitude_math_sys.v`
- `flight_viz_model_sys.v`
- `flight_viz_bundle_defs.vh`
- `telemetry_defs_vh.vh`
- top-level integration paths through `caelumfusion_top_vga.v`

## Firmware Behavior Summary

### Airbrake Policy

The verified firmware policy is a drag-aware coast-phase controller. It rejects
deployment unless all policy gates pass:

- runtime policy enable;
- arming state is `ARMED`;
- software arm token present;
- phase is `COAST` or `BRAKE`;
- estimator valid, finite, and fresh;
- altitude is above `POLICY_MIN_ALT_M`;
- vertical speed is above `POLICY_MIN_VZ_MPS`.

The firmware computes:

```text
k(u) = rho * (CDA_body + u*CDA_brake) / (2*m)
```

and predicts apogee as either the ballistic limit or the quadratic-drag closed
form:

```text
h_ap = h + v^2/(2g)                       when k is near zero
h_ap = h + ln(1 + k*v^2/g) / (2k)         otherwise
```

It then solves for command `u` in `[0, POLICY_MAX_COMMAND01]` using deterministic
bisection, applies a covariance-aware target reduction from `sqrt(P00)`, applies
slew limiting, and publishes `valid=true` only when the command is positive and
authorized for consideration. Final actuator motion remains owned by safety and
actuator gates.

### Estimator and Attitude

The verified estimator pipeline is:

```text
IMU update ->
Madgwick quaternion update ->
quaternion vertical acceleration projection ->
Kalman predict using measured IMU dt ->
barometric altitude update ->
published estimator state
```

The Kalman state is `[h_m, v_mps]`, uses measured vertical acceleration and
measured `dt_s`, seeds from the first trusted relative altitude, updates from
barometric altitude, and publishes covariance terms `P00/P01/P10/P11` for
observability and policy uncertainty margin.

### Flight Phase

The verified firmware phase detector is stateful:

```text
IDLE -> BOOST -> COAST -> BRAKE -> DESCENT
```

It uses launch, burnout, and descent latches with dwell timers to prevent phase
chatter. After launch, transient invalid data must not erase flight history.
`DESCENT` remains latched until reset. Phase classification is advisory; actuation
permission remains owned by safety gates.

## Changes in This Branch

### `apogee_authority_policy_sys.v`

Implemented in this pass:

- Added firmware-aligned coast policy gates:
  - minimum altitude gate: 30 m (`3000 cm`);
  - minimum upward vertical-speed gate: 15 m/s (`1500 cm/s`);
  - 5 m no-command deadband (`500 cm`).
- Changed policy validity semantics:
  - `auth_valid` now asserts only when all available authority gates pass and the
    computed command is positive;
  - denied gates force `auth_brake_cmd_u8=0` and `auth_servo_us=1000`.
- Preserved separation between policy demand and final servo command:
  - command selection is computed only after authority gates pass;
  - servo output remains idle unless the final safety predicate passes.
- Changed the target output to the effective target (`target_nominal - margin`)
  instead of always publishing the nominal target.
- Replaced the previous age-growing uncertainty band defaults with a conservative
  fixed margin until the RTL estimator exposes `P00`:
  - default `UNC_BASE_CM = 100`;
  - default `UNC_MAX_CM = 2000`;
  - default `UNC_AGE_CM_PER_MS = 0`.

Still approximate/deferred:

- The current RTL still uses a resource-safe ballistic/half-energy authority
  approximation rather than the C++ logarithmic drag model.
- The current RTL does not yet publish estimator covariance `P00` into this
  module, so the uncertainty margin is not yet `sqrt(P00)`.
- The current RTL does not yet implement firmware-equivalent slew limiting
  because the policy block does not receive a semantic update pulse or measured
  policy `dt`.
- Command solve remains quantized into five servo levels rather than the C++
  fixed-count bisection result.

### `authority_gate_phase_sys.v`

Implemented in this pass:

- Increased the default BOOST dwell from 50 ms to 250 ms at 100 MHz, matching the
  firmware default dwell order.
- Preserved phase history through transient invalid data after launch instead of
  collapsing the state machine to `UNKNOWN`.
- Kept pre-launch invalid data fail-safe by publishing `IDLE`.
- Made `DESCENT` latched until reset instead of returning to `IDLE` near ground.
- Published phase as meaningful after launch even if runtime freshness gates
  temporarily fail.

Still approximate/deferred:

- The RTL phase block does not receive IMU acceleration norm directly, so launch
  and burnout confirmation use altitude/vertical-speed evidence only.
- The RTL phase block does not receive previous-pass policy command directly;
  BRAKE state therefore remains a hardware approximation rather than a full copy
  of the firmware `brake_active` diagnostic.
- Full firmware phase diagnostics (`launch_candidate`, `burnout_candidate`, dwell
  timer values, and `since_*_ms`) are not yet exposed in the visualization bundle.

## C++ to RTL Contract Mapping

| Firmware field / concept | RTL signal or field | Units / scale | Status |
| --- | --- | --- | --- |
| `SystemState.est.valid` | `der_valid` and `der_status == ST_OK` | boolean/status | approximated through derived-state validity |
| `SystemState.est.h_m` | `altitude_cm` / `der_altitude_cm_q` | centimeters | matched by unit conversion |
| `SystemState.est.v_mps` | `vertical_speed_cms` / `der_vertical_speed_cms_q` | centimeters per second | matched by unit conversion |
| `SystemState.est.a_mps2` | not yet exposed | m/s^2 | missing/deferred |
| `SystemState.est.P00` | not yet exposed | variance | missing/deferred; required for firmware-equivalent uncertainty margin |
| `SystemState.est.P01/P10/P11` | not yet exposed | covariance | missing/deferred |
| `SystemState.attitude.q0..q3` | not yet exposed | quaternion | missing/deferred; current RTL uses display CORDIC roll/heading |
| `SystemState.auxvz.a_vertical` | not yet exposed | m/s^2 | missing/deferred |
| `SystemState.phase` | `auth_phase_code` | enum | approximated; IDLE/BOOST/COAST/BRAKE/DESCENT preserved |
| `SystemState.phase_diag` | not yet exposed | diagnostic flags/timers | missing/deferred |
| `SystemState.policy.valid` | `auth_valid` | boolean | now closer: true only for positive authorized command |
| `SystemState.policy.command01` | `auth_brake_cmd_u8 / 255` | normalized u8 | approximated; quantized five-level command |
| `predicted_apogee_no_brake_m` | `auth_pred_no_cm` | centimeters | approximated ballistic prediction |
| `predicted_apogee_full_brake_m` | `auth_pred_full_cm` | centimeters | approximated half-energy authority proxy |
| `target_nominal_m` | `TARGET_APOGEE_CM` parameter | centimeters | present as parameter |
| `target_effective_m` | `auth_target_cm` | centimeters | now published as effective target |
| `uncertainty_margin_m` | `auth_uncertainty_cm` | centimeters | approximated fixed margin until P00 exists |
| `apogee_error_m` | not directly bundled | centimeters | derivable off-chip from `auth_pred_no_cm - auth_target_cm` |
| `policy_runtime_enabled` | `policy_runtime_enable` | boolean | present |
| `software_arm_token` | `software_armed` | boolean | present |
| `arm_state == ARMED` | folded into `software_armed` and `safety_allows_actuation` | boolean | approximated |
| actuator idle/min/max | `auth_servo_us` | microseconds | present; idle forced to 1000 us |

## Fixed-Point Notes

| Quantity | RTL representation | Notes |
| --- | --- | --- |
| altitude | unsigned 32-bit centimeters | saturating addition used for apogee predictions |
| vertical speed | signed 32-bit centimeters/second | negative speeds clamp to zero for upward coast prediction |
| positive speed | unsigned 16-bit centimeters/second | saturated at 65535 cm/s before squaring |
| speed squared | unsigned 32-bit `(cm/s)^2` | derived from 16-bit positive speed |
| ballistic height gain | `v^2 >> 11` centimeters | approximates `v^2/(2g)` with `g~=981 cm/s^2` |
| full-brake height gain | half of no-brake gain | existing resource-safe authority approximation |
| command | unsigned 8-bit | quantized to `0,64,128,192,255` |
| servo | unsigned 12-bit microseconds | `1000,1250,1500,1750,2000`, gated to idle when denied |

## Verification Status

Added verification artifacts:

- `CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_apogee_authority_policy_sys.v`
- `tools/golden/caelum_airbrake_policy_golden.py`

The testbench is intended to check reset, disabled/denied gates, minimum altitude,
minimum vertical-speed, valid coast command generation, and stale-estimator
rejection.

The Python script is a standalone C++-semantics golden-vector generator for the
airbrake policy math. It includes the drag-aware prediction, uncertainty margin,
bisection command solve, and gate-denial cases. It is not yet wired into an
RTL-vs-golden self-checking flow.

Commands not run in this environment:

- Vivado `xvlog/xelab/xsim`
- Vivado synthesis/implementation/timing
- formal/property checks

## Remaining Work for Complete Integration

1. Add a fixed-point two-state Kalman estimator that publishes `P00/P01/P10/P11`.
2. Route `P00` or a scaled covariance field into `apogee_authority_policy_sys`.
3. Replace the ballistic/half-energy authority approximation with a verified
   fixed-point drag model or a bounded LUT/CORDIC approximation of the C++
   logarithmic predictor.
4. Replace five-level command quantization with a deterministic bisection or
   verified equivalent command solve.
5. Add policy update timing and firmware-equivalent slew limiting.
6. Add quaternion/Madgwick attitude and vertical acceleration projection, or
   connect an external estimator source with explicit validity/freshness fields.
7. Extend visualization/telemetry bundles for covariance, target nominal/effective
   separation, uncertainty margin provenance, apogee error, and phase diagnostics.
8. Run RTL simulation, lint, synthesis, implementation, and timing.
9. Compare RTL outputs against C++ golden vectors and record fixed-point errors.

## Integration Status

`PARTIAL` - this branch implements Stage-A safety and policy-gate alignment and
corrects key policy-valid/phase-latching mismatches. It does not yet implement a
complete fixed-point clone of the verified Arduino estimator, Madgwick attitude
path, covariance-aware uncertainty margin, or logarithmic drag/bisection solver.
