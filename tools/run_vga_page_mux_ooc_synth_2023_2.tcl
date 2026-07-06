set repo_root [file normalize [pwd]]
set src_dir   [file join $repo_root "CaelumFusion_Flight_Control.srcs" "sources_1" "new"]
set out_dir   [file join $repo_root "tmp"]

file mkdir $out_dir

create_project -in_memory -part xc7a35tcpg236-3
set_property target_language Verilog [current_project]
set_property include_dirs [list $src_dir] [current_fileset]

read_verilog -library xil_defaultlib [list \
    [file join $src_dir "vga_timing_640x480_60.v"] \
    [file join $src_dir "flight_viz_telemetry_textgen.v"] \
    [file join $src_dir "flight_viz_telemetry_compositor.v"] \
    [file join $src_dir "flight_vga_page_mux_pix.v"] \
    [file join $src_dir "flight_visualizer_pix.v"] \
]

synth_design -top flight_visualizer_pix -part xc7a35tcpg236-3 -mode out_of_context
create_clock -name pix_clk_25m -period 40.000 [get_ports pix_clk]

report_timing_summary -delay_type max -max_paths 20 \
    -file [file join $out_dir "vga_page_mux_ooc_timing_summary.rpt"]
report_utilization \
    -file [file join $out_dir "vga_page_mux_ooc_utilization.rpt"]
write_checkpoint -force [file join $out_dir "vga_page_mux_ooc.dcp"]

puts "OOC_SYNTH_PASS flight_visualizer_pix"
exit 0
