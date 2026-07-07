# CaelumSufflamen Validation Status

Current status: `PARTIAL`.

This integration is not yet complete firmware-equivalent RTL. The current branch contains Stage-A policy/safety gate alignment and early Stage-B/Stage-C estimator scaffolding, but it still requires RTL simulation, synthesis, implementation, timing, golden-vector comparison, CDC review, and hardware evidence before it can be marked complete.

## Implemented and Ready for Validation

- Stage-A policy/safety gate alignment.
- Stage-B/Stage-C standalone fixed-point Kalman core.
- P00-style uncertainty-margin computation inside the apogee policy block using fallback covariance.
- Firmware-style command memory reset and slew limiting.
- Focused apogee-policy and Kalman testbenches.
- Python airbrake-policy golden-vector generator.

## Not Yet Validated

- RTL simulations have not been run.
- Vivado synthesis, implementation, timing, and DRC have not been run.
- Formal/assertion checks have not been run.
- RTL-vs-C++ golden-vector comparison has not been completed.
- Live Kalman P00 is not yet wired into the apogee policy.
- Madgwick quaternion attitude and vertical acceleration RTL are not yet implemented.
- Visualization/telemetry expansion for covariance, apogee error, and phase diagnostics is not yet complete.
- Hardware WaveForms captures have not been collected for this integration.

## Required Before Marking Complete

- `tb_apogee_authority_policy_sys` passes.
- `tb_kalman_alt2_fixed_sys` passes.
- Golden-vector comparison against the verified Arduino C++ behavior passes within documented fixed-point tolerances.
- Vivado synthesis and implementation complete with acceptable warnings, resource use, and timing slack.
- CDC review confirms visualization bundle integrity.
- Any hardware claims are backed by archived WaveForms/CSV/MATLAB evidence.

## Suggested Focused Simulation Commands

Run from the repository root with a Vivado environment available. Adjust the Vivado path and shell syntax as needed for the local machine.

```powershell
$SRC = "CaelumFusion_Flight_Control_System.srcs/sources_1/new"
$SIM = "CaelumFusion_Flight_Control_System.srcs/sim_1/new"

xvlog -sv -i $SRC `
  "$SRC/apogee_authority_policy_sys.v" `
  "$SIM/tb_apogee_authority_policy_sys.v"

xelab tb_apogee_authority_policy_sys -s sim_apogee_policy
xsim sim_apogee_policy -runall
```

```powershell
$SRC = "CaelumFusion_Flight_Control_System.srcs/sources_1/new"
$SIM = "CaelumFusion_Flight_Control_System.srcs/sim_1/new"

xvlog -sv -i $SRC `
  "$SRC/kalman_alt2_fixed_sys.v" `
  "$SIM/tb_kalman_alt2_fixed_sys.v"

xelab tb_kalman_alt2_fixed_sys -s sim_kalman_alt2
xsim sim_kalman_alt2 -runall
```

## Suggested Golden-Vector Command

```powershell
python tools/golden/caelum_airbrake_policy_golden.py `
  -o artifacts/caelum_airbrake_policy_golden.csv
```

The next validation step is to consume these vectors in an RTL self-checking comparison flow for policy validity, command output, target-effective computation, uncertainty margin, apogee prediction fields, gate-denial behavior, and slew behavior.

## Evidence Boundary

Do not upgrade this integration from `PARTIAL` to `COMPLETE` until tool output and hardware evidence support the claim. In particular, a visually plausible VGA HUD is not sufficient evidence by itself; the numeric bundle fields, CDC transfer, simulator outputs, synthesis/timing reports, and any physical captures must all be traceable to the design contract.
