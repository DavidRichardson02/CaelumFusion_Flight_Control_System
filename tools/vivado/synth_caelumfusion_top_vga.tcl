# Reproducible synthesis baseline for the CaelumFusion VGA/I2C top.
#
# Usage from the project root:
#   vivado -mode batch -source tools/vivado/synth_caelumfusion_top_vga.tcl

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set ROOT_DIR   [file normalize [file join $SCRIPT_DIR .. ..]]
cd $ROOT_DIR

if {![info exists OUT_DIR]} {
    set OUT_DIR .codex_build/synth_baseline
}
file mkdir $OUT_DIR

# Vivado 2025.1.1 can fail before read_verilog when the user Tcl Store cache is
# corrupt and the install-area Tcl Store support subdirectories are not on
# auto_path. Add the read-only install support paths for this batch run only.
if {[info exists ::env(XILINX_VIVADO)]} {
    set TCLSTORE_ROOT [file normalize [file join $::env(XILINX_VIVADO) data XilinxTclStore]]
    foreach pkgdir [list \
        [file join $TCLSTORE_ROOT support] \
        [file join $TCLSTORE_ROOT support appinit] \
        [file join $TCLSTORE_ROOT support args] \
        [file join $TCLSTORE_ROOT tclapp] \
        [file join $TCLSTORE_ROOT tclapp xilinx] \
        [file join $TCLSTORE_ROOT tclapp xilinx xsim] \
    ] {
        if {[file isdirectory $pkgdir] && ([lsearch -exact $::auto_path $pkgdir] == -1)} {
            lappend ::auto_path $pkgdir
        }
    }
}

set PART xc7a35tcpg236-3
set TOP  caelumfusion_top_vga
if {![info exists SYNTH_DESIGN_ARGS]} {
    set SYNTH_DESIGN_ARGS [list]
}

set sources [list \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/apogee_authority_policy_sys.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/authority_gate_phase_sys.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/adxl362_spi_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/adxl_irq_sample_event.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/sync_bit_3ff.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/bmp585_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/bmp5xx_spi_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/caelumfusion_science_page_vga.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/caelumfusion_vga_render_control.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/caelumfusion_top_vga.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/caelumfusion_top_vga_i2c.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/caelumfusion_top_vga_spi.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cdc_bundle_toggle_2way.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cdc_bus_toggle_1way.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cdc_word_toggle.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/clk_div_4.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/clk_wiz_0.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cls_console_top_min.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cls_page_formatter_min.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cls_page_scheduler_min.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cls_refresh_fsm.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cls_uart_tx_9600.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/derived_snapshot_regs.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/derived_state_producer.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/altitude_lut_rom_u8_to_u16.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_attitude_math_sys.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/cordic_atan2_q12.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/angle_wrap_0_2pi.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/dp_bram_1024x16.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/epoch_scheduler.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_visualizer_pix.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_vga_page_mux_pix.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_base_layer.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_bundle_cdc.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_model_sys.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_suite_top.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_telemetry_compositor.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_telemetry_textgen.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/flight_viz_vga_timing.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/i2c_job_arbiter.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/i2c_job_arbiter7.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/i2c_job_mux.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/i2c_job_mux7.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/i2c_master_engine.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/l3g4200d_i2c_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/mmc3416_i2c_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/pmod_dpot_spi_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/pmod_gpio_capture.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/pmod_hygro_i2c_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/pmon1_i2c_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/blackbox_frame_packer.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/sensor_extension_hub.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/snapshot_fault_injector.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/gnss_bridge_snapshot_source.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/uart_rx_8n1.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/teensy_bridge_packet_ingress.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/teensy_uart_range_bridge.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/nav_wind_snapshot_producer.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/landing_nav_wind_observer.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/mag1_bench_snapshot_source.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/planar_compass_truth_page_vga.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/lis2mdl_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/lis2mdl_spi_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/lis3dh_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/lis3dh_spi_job.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/rocket_i2c_suite_top.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/rocket_spi_suite_top.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/simple_dp_ram.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/snapshot_cdc_bundle.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/snapshot_regs.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/spi_job_arbiter.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/spi_job_mux.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/spi_master_engine.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/spi_master_engine_mode0.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/telemetry_pix_snapshot_bank.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/timebase_us.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/vga_char_glyph_3x5.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/vga_textline_3x5.v \
    CaelumFusion_Flight_Control_System.srcs/sources_1/new/vga_timing_640x480_60.v \
]

foreach src $sources {
    read_verilog $src
}

read_xdc CaelumFusion_Flight_Control_System.srcs/constrs_1/new/Basys-3-Master.xdc
synth_design -top $TOP -part $PART {*}$SYNTH_DESIGN_ARGS

report_drc -file [file join $OUT_DIR caelumfusion_top_vga_drc_synth.rpt]
report_utilization -file [file join $OUT_DIR caelumfusion_top_vga_utilization_synth.rpt]
report_timing_summary -file [file join $OUT_DIR caelumfusion_top_vga_timing_synth.rpt] -max_paths 10 -warn_on_violation
write_checkpoint -force [file join $OUT_DIR caelumfusion_top_vga_synth.dcp]
