open_project ./CaelumFusion_Flight_Control.xpr

set_param general.maxThreads 6

reset_run synth_1
launch_runs synth_1 -jobs 6
wait_on_run synth_1

set synth_status [get_property STATUS [get_runs synth_1]]
puts "SYNTH_STATUS=$synth_status"

if {[regexp -nocase {fail|error} $synth_status]} {
    exit 1
}

file mkdir ./tmp
open_run synth_1
report_timing_summary -delay_type max -max_paths 20 \
    -file ./tmp/vga_page_mux_synth_timing_summary.rpt
report_utilization \
    -file ./tmp/vga_page_mux_synth_utilization.rpt

exit 0
