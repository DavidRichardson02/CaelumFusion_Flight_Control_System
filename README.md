# CaelumFusion Flight Control System

CaelumFusion Flight Control System is a Vivado/RTL FPGA project for a Basys 3
flight-control visualization and sensor-integration stack. The project combines
Verilog RTL, VGA rendering, sensor-bus jobs, CDC snapshot logic, focused
testbenches, host-side validation tools, firmware bridge examples, constraints,
and engineering documentation.

## Target Hardware and Toolchain

- FPGA board: Digilent Basys 3 class target
- FPGA part: `xc7a35tcpg236-3`
- Primary top module: `caelumfusion_top_vga`
- Primary constraint file:
  `CaelumFusion_Flight_Control_System.srcs/constrs_1/new/Basys-3-Master.xdc`
- Vivado project:
  `CaelumFusion_Flight_Control_System.xpr`
- Toolchain note: this checkout is stored under a Vivado 2023.2 workspace path,
  while the current `.xpr` header reports Vivado v2025.1.1. Reconfirm the
  intended Vivado version before regenerating project metadata or publishing
  synthesis/timing claims.
- Display path: VGA at 640 x 480 timing through the board-facing top

## Source Tree

- `CaelumFusion_Flight_Control_System.xpr` - Vivado project metadata and file-set
  contract.
- `CaelumFusion_Flight_Control_System.srcs/sources_1/new/` - synthesizable
  Verilog RTL and include files.
- `CaelumFusion_Flight_Control_System.srcs/sim_1/new/` - simulation benches and
  test models.
- `CaelumFusion_Flight_Control_System.srcs/constrs_1/new/` - Basys 3 XDC
  constraints.
- `tools/` - Python, Tcl, and WaveForms helper scripts for analysis, synthesis,
  and bench capture.
- `firmware/` - external MCU bridge producer examples.
- `docs/` - hand-authored engineering notes, bring-up guides, control maps, and
  LaTeX sources.

## RTL and Simulation Entry Points

The canonical board-facing VGA integration top is:

```text
CaelumFusion_Flight_Control_System.srcs/sources_1/new/caelumfusion_top_vga.v
```

Representative focused benches include:

```text
CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_flight_visualizer_pix.v
CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_i2c_suite_regression_all3_real_engine.v
CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_landing_nav_wind_observer.v
CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_mag1_bench_snapshot_source.v
CaelumFusion_Flight_Control_System.srcs/sim_1/new/tb_teensy_uart_range_bridge.v
```

The Vivado `.xpr` is treated as part of the build contract. If new RTL or bench
files are added, register them in the appropriate Vivado file set or document why
they are intentionally kept outside the project.

## Build and Verification Notes

Open the project in Vivado 2023.2 with:

```text
vivado CaelumFusion_Flight_Control_System.xpr
```

Repository scripts under `tools/vivado/` provide narrower synthesis and
implementation flows for the active top-level design. Focused simulation should
prefer explicit `xvlog`, `xelab`, and `xsim` invocations or the project file-set
configuration so that bench coverage remains reproducible.

Before publishing hardware claims, keep the evidence boundary explicit:

- RTL and XDC source presence is not the same as synthesis or timing closure.
- Generated bitstreams, checkpoints, WDBs, logs, and Vivado run directories are
  intentionally excluded from source control.
- Bench images and generated PDFs should be regenerated or archived separately
  unless they are explicitly approved as deliverables.

## Generated Artifacts Excluded From Git

The repository `.gitignore` excludes local or generated Vivado state, including
`.Xil/`, `*.runs/`, `*.sim/`, `*.cache/`, `*.hw/`, `*.ip_user_files/`,
`xsim.dir/`, WDB files, logs, journals, bitstreams, checkpoints, generated
documentation builds, Python caches, local Codex metadata, and bulky vendor
reference archives.

The initial publication stance is to include source, constraints, scripts,
firmware, and hand-authored documentation while excluding generated build
products and third-party reference PDFs unless explicitly approved.
