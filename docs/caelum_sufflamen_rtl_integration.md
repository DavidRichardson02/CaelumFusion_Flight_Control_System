# CaelumSufflamen RTL Integration Notes

This document records the staged CaelumSufflamen-to-CaelumFusion RTL integration work on this branch. It is intentionally explicit about what is implemented, what remains approximate, and what must still be verified before the branch can be described as a complete firmware-equivalent flight-control implementation.

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

Version rule: when a report describes an older placeholder or simplified model that conflicts with the current verified Arduino source, preserve the current Arduino behavior and document the delta.

## Current RTL Baseline Reviewed

The current CaelumFusion RTL is a Basys 3 FPGA telemetry and visualization platform built around:

- SYS-domain sensor acquisition and derived-state publication;
- committed snapshots with validity, status, sequence, and age metadata;
- explicit SYS-to-PIX visualization bundle CDC;
- VGA/HUD rendering; and
- a simplified apogee-authority model intended for display, telemetry, and gated actuator demand.

The main RTL files reviewed for this branch were:

- `apogee_authority_policy_sys.v`
- `authority_gate_phase_sys.v`
- `flight_attitude_math_sys.v`
- `flight_viz_model_sys.v`
- `flight_viz_bundle_defs.vh`
- `telemetry_defs_vh.vh`
- top-level integration paths through `caelumfusion_top_vga.v`

## Firmware Behavior Summary

### Airbrake Policy

The verified firmware policy is a drag-aware coast-phase controller. It rejects deployment unless all policy gates pass:

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

and predicts apogee as either the ballistic limit or the quadratic-drag closed form:

```text
h_ap = h + v^2/(2g)                       when k is near zero
h_ap = h + ln(1 + k*v^2/g) / (2k)         otherwise
```

It then solves for command `u` in `[0, POLICY_MAX_COMMAND01]` using deterministic bisection, applies a covariance-aware target reduction from `sqrt(P00)`, applies slew limiting, and publishes `valid=true` only when the command is positive and authorized for consideration. Final actuator motion remains owned by safety and actuator gates.

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

The Kalman state is `[h_m, v_mps]`, uses measured vertical acceleration and measured `dt_s`, seeds from the first trusted relative altitude, updates from barometric altitude, and publishes covariance terms `P00/P01/P10/P11` for observability and policy uncertainty margin.

### Flight Phase

The verified firmware phase detector is stateful:

```text
IDLE -> BOOST -> COAST -> BRAKE -> DESCENT
```

It uses launch, burnout, and descent latches with dwell timers to prevent phase chatter. After launch, transient invalid data must not erase flight history. `DESCENT` remains latched until reset. Phase classification is advisory; actuation permission remains owned by safety gates.

## Changes in This Branch

### `apogee_authority_policy_sys.v`

Implemented:

- Added firmware-aligned coast policy gates:
  - minimum altitude gate: 30 m (`3000 cm`);
  - minimum upward vertical-speed gate: 15 m/s (`1500 cm/s`);
  - 5 m no-command deadband (`500 cm`).
- Changed policy validity semantics:
  - `auth_valid` now asserts only when all available authority gates pass and the slew-limited command is positive;
  - denied gates force `auth_brake_cmd_u8=0` and `auth_servo_us=1000`.
- Preserved separation between policy demand and final servo command:
  - command selection is computed only after authority gates pass;
  - servo output remains idle unless the final safety predicate passes.
- Changed target output to the effective target (`target_nominal - margin`) instead of always publishing the nominal target.
- Began covariance-aware uncertainty integration:
  - computes `sigma_h_cm = sqrt(P00_cm2)` using a fixed-point integer square root;
  - applies `POLICY_SIGMA_MARGIN_Q8`, defaulting to 1.0 sigma;
  - clamps the margin to `UNC_MAX_CM`.
- Added firmware-style command-memory behavior:
  - private previous-command memory resets when authority gates fail;
  - command changes are slew-limited at a bounded `POLICY_UPDATE_HZ` cadence;
  - the servo mapping now follows the slew-limited command rather than the unslewed demand.

Still approximate/deferred:

- The current apogee predictor still uses a resource-safe ballistic/full-brake authority approximation rather than the C++ logarithmic drag model.
- `P00` is currently supplied through `POLICY_P00_FALLBACK_CM2` until the live fixed-point Kalman publisher is wired into the derived-state path.
- Command solve remains a five-region monotonic approximation before slew limiting rather than the C++ fixed-count bisection result.

### `kalman_alt2_fixed_sys.v`

Implemented:

- Added a standalone synthesizable two-state altitude Kalman core in integer project units:
  - `h_cm` altitude;
  - `v_cms` vertical speed;
  - `a_cms2` acceleration input;
  - `P00/P01/P10/P11` covariance publication.
- Supports explicit seed, predict, and update operations.
- Predicts state using measured `dt_ms` and constant-acceleration kinematics.
- Propagates covariance for the two-state model.
- Updates from scalar barometric altitude using Q16.16 Kalman gains.
- Applies a Joseph-form covariance correction and symmetrizes `P01/P10`.
- Rejects invalid or excessive `dt_ms` without silently producing fresh valid output.

Still approximate/deferred:

- The core is not yet wired into `flight_viz_model_sys` or top-level derived-state production.
- The division/Joseph datapath is not pipelined yet; timing must be checked before claiming 100 MHz closure.
- Relative altitude baseline capture is not yet implemented in RTL.
- The live barometric pressure-to-altitude converter is still outside this core.

### `authority_gate_phase_sys.v`

Implemented:

- Increased the default BOOST dwell from 50 ms to 250 ms at 100 MHz, matching the firmware default dwell order.
- Preserved phase history through transient invalid data after launch instead of collapsing the state machine to `UNKNOWN`.
- Kept pre-launch invalid data fail-safe by publishing `IDLE`.
- Made `DESCENT` latched until reset instead of returning to `IDLE` near ground.
- Published phase as meaningful after launch even if runtime freshness gates temporarily fail.

Still approximate/deferred:

- The RTL phase block does not receive IMU acceleration norm directly, so launch and burnout confirmation use altitude/vertical-speed evidence only.
- The RTL phase block does not receive previous-pass policy command directly; BRAKE state therefore remains a hardware approximation rather than a full copy of the firmware `brake_active` diagnostic.
- Full firmware phase diagnostics (`launch_candidate`, `burnout_candidate`, dwell timer values, and `since_*_ms`) are not yet exposed in the visualization bundle.

## C++ to RTL Contract Mapping

| Firmware field / concept | RTL signal or field | Units / scale | Status |
| --- | --- | --- | --- |
| `SystemState.est.valid` | `der_valid` and `der_status == ST_OK`; `kalman_alt2_fixed_sys.est_valid` | boolean/status | partially implemented |
| `SystemState.est.seeded` | `kalman_alt2_fixed_sys.est_seeded` | boolean | implemented in standalone core |
| `SystemState.est.h_m` | `der_altitude_cm`; `kalman_alt2_fixed_sys.est_h_cm` | centimeters | partially implemented |
| `SystemState.est.v_mps` | `der_vertical_speed_cms`; `kalman_alt2_fixed_sys.est_v_cms` | centimeters per second | partially implemented |
| `SystemState.est.a_mps2` | `kalman_alt2_fixed_sys.est_a_cms2` | centimeters per second squared | standalone core implemented; not wired |
| `SystemState.est.P00` | `kalman_alt2_fixed_sys.est_P00_cm2`; policy fallback parameter | centimeters squared | standalone core implemented; not wired to policy |
| `SystemState.est.P01/P10/P11` | `kalman_alt2_fixed_sys.est_P01/est_P10/est_P11` | fixed integer covariance units | standalone core implemented |
| `SystemState.attitude.q0..q3` | not yet exposed | quaternion | missing/deferred |
| `SystemState.auxvz.a_vertical` | not yet exposed | m/s^2 or cm/s^2 | missing/deferred |
| `SystemState.phase` | `auth_phase_code` | enum | approximated; IDLE/BOOST/COAST/BRAKE/DESCENT preserved |
| `SystemState.phase_diag` | not yet exposed | diagnostic flags/timers | missing/deferred |
| `SystemState.policy.valid` | `auth_valid` | boolean | closer: true only for positive authorized slew-limited command |
| `SystemState.policy.command01` | `auth_brake_cmd_u8 / 255` | normalized u8 | partially implemented with slew limiting |
| `predicted_apogee_no_brake_m` | `auth_pred_no_cm` | centimeters | approximated ballistic prediction |
| `predicted_apogee_full_brake_m` | `auth_pred_full_cm` | centimeters | approximated full-brake authority proxy |
| `target_nominal_m` | `TARGET_APOGEE_CM` parameter | centimeters | present as parameter |
| `target_effective_m` | `auth_target_cm` | centimeters | implemented |
| `uncertainty_margin_m` | `auth_uncertainty_cm` | centimeters | integer sqrt(P00 fallback) implemented; live P00 wiring deferred |
| `apogee_error_m` | not directly bundled | centimeters | derivable off-chip from `auth_pred_no_cm - auth_target_cm` |
| `policy_runtime_enabled` | `policy_runtime_enable` | boolean | present |
| `software_arm_token` | `software_armed` | boolean | present |
| `arm_state == ARMED` | folded into `software_armed` and `safety_allows_actuation` | boolean | approximated |
| actuator idle/min/max | `auth_servo_us` | microseconds | present; idle forced to 1000 us |

## Fixed-Point Notes

| Quantity | RTL representation | Notes |
| --- | --- | --- |
| altitude | signed/unsigned 32-bit centimeters | policy consumes unsigned; Kalman core publishes signed |
| vertical speed | signed 32-bit centimeters/second | negative speeds clamp to zero for upward coast prediction |
| acceleration | signed 32-bit centimeters/second^2 | Kalman predictor input |
| covariance P00 | signed 32-bit centimeters^2 | policy uses integer sqrt to produce cm margin |
| covariance P01/P10 | signed 32-bit centimeters^2/second | Kalman core symmetrizes off-diagonal terms |
| covariance P11 | signed 32-bit centimeters^2/second^2 | velocity variance |
| ballistic height gain | `v^2 >> 11` centimeters | approximates `v^2/(2g)` with `g~=981 cm/s^2` |
| command | unsigned 8-bit | solver demand is quantized; output is slew-limited u8 |
| servo | unsigned 12-bit microseconds | `1000 + round(cmd_u8*1000/255)`, gated to idle when denied |

## Verification Status

Added verification artifacts:

- `CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_apogee_authority_policy_sys.v`
- `CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_kalman_alt2_fixed_sys.v`
- `tools/golden/caelum_airbrake_policy_golden.py`

The apogee testbench checks reset, disabled/denied gates, minimum altitude, minimum vertical-speed, stale-estimator rejection, P00-derived margin from the fallback covariance, valid command generation, and slew-memory reset after gate failure.

The Kalman testbench checks reset, seeding, predict with measured `dt_ms`, positive covariance propagation, Joseph-update symmetry, and invalid-`dt` rejection.

The Python script is a standalone C++-semantics golden-vector generator for the airbrake policy math. It includes the drag-aware prediction, uncertainty margin, bisection command solve, and gate-denial cases. It is not yet wired into an RTL-vs-golden self-checking flow.

Commands not run in this environment:

- Vivado `xvlog/xelab/xsim`
- Vivado synthesis/implementation/timing
- formal/property checks

## Remaining Work for Complete Integration

1. Wire `kalman_alt2_fixed_sys` into the live derived-state path.
2. Add pressure-to-relative-altitude conversion and baseline/reference-frame handling in RTL.
3. Route live `P00` into `apogee_authority_policy_sys` instead of using `POLICY_P00_FALLBACK_CM2`.
4. Replace the ballistic/full-brake authority approximation with a verified fixed-point drag model or bounded LUT/CORDIC approximation of the C++ logarithmic predictor.
5. Replace five-region command approximation with deterministic bisection or a verified equivalent command solve.
6. Add quaternion/Madgwick attitude and vertical acceleration projection, or connect an external estimator source with explicit validity/freshness fields.
7. Extend visualization/telemetry bundles for covariance, target nominal/effective separation, uncertainty margin provenance, apogee error, and phase diagnostics.
8. Run RTL simulation, lint, synthesis, implementation, and timing.
9. Compare RTL outputs against C++ golden vectors and record fixed-point errors.

## Integration Status

`PARTIAL` - this branch now implements Stage-A safety/policy-gate alignment and begins Stage-B/Stage-C estimator integration with a standalone fixed-point Kalman core, P00-style margin plumbing inside the policy block, and firmware-style slew-memory behavior. It does not yet implement a complete fixed-point clone of the verified Arduino estimator, Madgwick attitude path, live covariance wiring, or logarithmic drag/bisection solver.
