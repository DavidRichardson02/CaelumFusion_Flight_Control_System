## ============================================================================
## Basys 3 constraints for: caelumfusion_top_vga
## Active top-level ports assumed here:
##   clk, rst,
##   sw_arm_raw, sw_policy_enable_raw,
##   scl, sda,
##   adxl362_cs_n, adxl362_mosi, adxl362_miso, adxl362_sclk,
##   adxl362_int1, adxl362_int2,
##   btn_page_raw, btn_prev_raw, btn_direct_compass_raw,
##   sw_selftest_raw, sw_mag1_bench_raw, sw_compass_page_raw,
##   sw_history_freeze_raw, sw_log_diag_raw, sw_lis3dh_i2c_acc_raw,
##   sw_adxl362_spi_acc_raw, sw_cmps2_mmc3416_mag_raw, sw_pmon1_pwr_raw,
##   sw_mag1_offset_x_raw, sw_mag1_offset_y_raw, sw_mag1_offset_z_raw,
##   sw_compass_default_raw, sw_ext_i2c_raw,
##   ls1_s_raw[3:0], pir_motion_raw,
##   dpot_cs_n, dpot_mosi, dpot_sclk, cls_tx,
##   teensy_uart_rx_raw, teensy_uart_tx,
##   vga_hsync, vga_vsync, vga_rgb[11:0]
##
## Notes:
##   - Pmod JA3/JA4 carry the shared I2C SCL/SDA sensor bus.
##   - Pmod CMPS2/MMC34160PJ remains the magnetometer/heading source at 7'h30.
##   - LIS3DH, when fitted to the same I2C bus, is an auxiliary accelerometer;
##     its 7-bit address is 7'h18 when SA0=0 or 7'h19 when SA0=1.
##   - Pmod ACL2 is allocated to Pmod JB for its dedicated mode-0 SPI path.
##   - Basys-3 has JA/JB/JC plus JXADC in this canonical constraint file; there
##     is no ordinary JD Pmod header to allocate for the external-MCU bridge.
##   - Pmod power pins (3.3V/GND) are board pins, not FPGA pins, so they do not
##     appear in the XDC.
## ============================================================================

## Basys 3 configuration bank metadata. These properties close Vivado CFGBVS-1
## and make configuration-voltage assumptions explicit in implementation reports.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

## ----------------------------
## 100 MHz clock
## ----------------------------
set_property PACKAGE_PIN W5        [get_ports clk]
set_property IOSTANDARD LVCMOS33   [get_ports clk]
create_clock -name clk -period 10.000 [get_ports clk]

## clock_gen_xilinx_7series derives the 25 MHz VGA pixel clock through an
## MMCME2_BASE and BUFG. Vivado should infer that generated clock from the MMCM
## primitive. If a different flow does not, add a scoped create_generated_clock
## on u_clock_gen/u_bufg_clk_25m/O rather than timing the pixel domain as an
## unconstrained fabric-derived clock.

## ----------------------------
## SYS -> PIX visualization snapshot CDC
## ----------------------------
## flight_viz_bundle_cdc transfers a wide, low-rate semantic snapshot by:
##   1) holding src_bundle_hold stable in clk domain,
##   2) synchronizing a toggle event into the generated 25 MHz pixel domain, and
##   3) sampling the held bundle into dst_bundle_shadow after the toggle arrives.
##
## The wide held data bus is therefore a CDC snapshot bus, not a single-cycle
## timed datapath. Keep this exception scoped to that bus only; do not clock-
## group the whole SYS and PIX domains because other generated-clock timing
## relationships should remain visible to implementation.
set_false_path \
    -from [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/u_viz_bundle_cdc/src_bundle_hold_reg.*}] \
    -to   [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/u_viz_bundle_cdc/dst_bundle_shadow_reg.*}]

set_false_path \
    -from [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/u_viz_bundle_cdc/src_toggle_reg.*}] \
    -to   [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/u_viz_bundle_cdc/dst_toggle_sync_ff_reg\[0\].*}]

## ----------------------------
## SYS -> PIX render-control status-strip CDC
## ----------------------------
## u_render_status_cdc publishes the effective view ID and direct-select
## diagnostics into the pixel domain for the lower-right VGA status strip.
## The held status bundle is a coherent low-rate snapshot, not a single-cycle
## timed datapath.
set_false_path \
    -from [get_cells -quiet -hier -regexp {.*u_vga_render_control/u_render_status_cdc/src_bundle_hold_reg.*}] \
    -to   [get_cells -quiet -hier -regexp {.*u_vga_render_control/u_render_status_cdc/dst_bundle_reg.*}]

set_false_path \
    -from [get_cells -quiet -hier -regexp {.*u_vga_render_control/u_render_status_cdc/src_evt_toggle_reg.*}] \
    -to   [get_cells -quiet -hier -regexp {.*u_vga_render_control/u_render_status_cdc/dst_sync_ff1_reg.*}]

## ----------------------------
## ADXL362 interrupt CDC
## ----------------------------
## INT1/INT2 are asynchronous board inputs from the Pmod ACL2. Only the path
## into the first synchronizer stage is false-pathed; the synchronizer chain
## itself remains normally timed for setup/hold.
set_false_path \
    -from [get_ports {adxl362_int1 adxl362_int2}] \
    -to   [get_cells -quiet -hier -regexp {.*u_sys_sensors/u_adxl362_int[12]_sync/sync_ff_reg\[0\].*}]

## ----------------------------
## Software-arm switch CDC
## ----------------------------
## SW0 is an asynchronous human-operated arm input. Only the path into the
## first synchronizer stage is false-pathed; the synchronizer chain remains
## normally timed inside the SYS clock domain.
set_false_path \
    -from [get_ports sw_arm_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_authority_gate_phase/u_sw_arm_sync/sync_ff_reg\[0\].*}]

## ----------------------------
## Policy-enable switch CDC
## ----------------------------
## SW1 is an asynchronous human-operated policy-enable input. It is separate
## from SW0 arming so the display can distinguish armed from policy-allowed.
set_false_path \
    -from [get_ports sw_policy_enable_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_authority_gate_phase/u_sw_policy_enable_sync/sync_ff_reg\[0\].*}]

## ----------------------------
## Live visualization / sensor-control CDC
## ----------------------------
## Human switches and buttons are asynchronous to clk. Only the path into each
## first synchronizer stage is false-pathed; synchronized logic remains timed.
set_false_path \
    -from [get_ports btn_page_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_btn_next_page_ctrl/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports btn_prev_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_btn_prev_page_ctrl/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports btn_direct_compass_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_btn_direct_compass_ctrl/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_selftest_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_selftest_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_mag1_bench_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_mag1_bench_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_compass_page_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_compass_page_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_history_freeze_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_history_freeze_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_log_diag_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_log_diag_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_lis3dh_i2c_acc_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_lis3dh_i2c_acc_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_adxl362_spi_acc_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_adxl362_spi_acc_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_cmps2_mmc3416_mag_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_cmps2_mmc3416_mag_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_pmon1_pwr_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_pmon1_pwr_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_mag1_offset_x_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_mag1_offset_x_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_mag1_offset_y_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_mag1_offset_y_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_mag1_offset_z_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_mag1_offset_z_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_compass_default_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_compass_default_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports sw_ext_i2c_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_sw_ext_i2c_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports teensy_uart_rx_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_teensy_uart_range_bridge/u_uart_rx_8n1/rx_meta_r_reg.*}]

set_false_path \
    -from [get_ports {ls1_s_raw[0]}] \
    -to   [get_cells -quiet -hier -regexp {.*u_pmod_gpio_capture/u_ls1_s0_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports {ls1_s_raw[1]}] \
    -to   [get_cells -quiet -hier -regexp {.*u_pmod_gpio_capture/u_ls1_s1_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports {ls1_s_raw[2]}] \
    -to   [get_cells -quiet -hier -regexp {.*u_pmod_gpio_capture/u_ls1_s2_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports {ls1_s_raw[3]}] \
    -to   [get_cells -quiet -hier -regexp {.*u_pmod_gpio_capture/u_ls1_s3_sync/sync_ff_reg\[0\].*}]
set_false_path \
    -from [get_ports pir_motion_raw] \
    -to   [get_cells -quiet -hier -regexp {.*u_pmod_gpio_capture/u_pir_motion_sync/sync_ff_reg\[0\].*}]

set_false_path \
    -from [get_cells -quiet -hier -regexp {.*u_sw_history_freeze_sync/sync_ff_reg\[2\].*}] \
    -to   [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/history_freeze_pix_ff_reg\[0\].*}]

## The diagnostic-page selector is a low-rate SYS-domain control bit sampled by
## a dedicated PIX-domain synchronizer.  Keep the exception scoped to the first
## synchronizer stage; the remaining PIX-domain synchronizer flops stay timed.
set_false_path \
    -from [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/vga_sensor_diag_page_sys_r_reg.*}] \
    -to   [get_cells -quiet -hier -regexp {.*u_flight_viz_suite_top/vga_diag_page_pix_ff_reg\[0\].*}]

## ----------------------------
## Reset button (BTNC)
## ----------------------------
set_property PACKAGE_PIN U18       [get_ports rst]
set_property IOSTANDARD LVCMOS33   [get_ports rst]

## ----------------------------
## Software-arm switch (SW0)
## ----------------------------
set_property PACKAGE_PIN V17       [get_ports sw_arm_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_arm_raw]

## ----------------------------
## Policy-enable switch (SW1)
## ----------------------------
set_property PACKAGE_PIN V16       [get_ports sw_policy_enable_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_policy_enable_raw]

## ----------------------------
## Live visualization / sensor-control switches
##   SW2  = self-test HUD hold
##   SW3  = MAG1 synthetic bench source enable
##   SW4  = compass/MAG evidence page hold
##   SW5  = HUD history freeze
##   SW6  = black-box/log diagnostics enable
##   SW7  = LIS3DH I2C accelerometer path enable
##   SW8  = ADXL362 SPI accelerometer path enable
##   SW9  = CMPS2/MMC34160PJ magnetometer path enable
##   SW10 = PMON1 power telemetry path enable
##   SW11 = MAG1 bench X-offset apply; fault-class bit 0;
##          encoded view ID bit 0 only when SW3 and SW2+SW6 fault injection are inactive
##   SW12 = MAG1 bench Y-offset apply; fault-class bit 1;
##          encoded view ID bit 1 only when SW3 and SW2+SW6 fault injection are inactive
##   SW13 = MAG1 bench Z-offset apply; fault-class bit 2;
##          encoded view ID bit 2 only when SW3 and SW2+SW6 fault injection are inactive
##   SW14 = compass/MAG evidence default/hold companion
##   SW15 = optional shared-I2C extension group enable
##          HYGRO, GYRO I2C, and LIS2MDL/MAG1 stay quiescent when low.
## ----------------------------
set_property PACKAGE_PIN W16       [get_ports sw_selftest_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_selftest_raw]

set_property PACKAGE_PIN W17       [get_ports sw_mag1_bench_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_mag1_bench_raw]

set_property PACKAGE_PIN W15       [get_ports sw_compass_page_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_compass_page_raw]

set_property PACKAGE_PIN V15       [get_ports sw_history_freeze_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_history_freeze_raw]

set_property PACKAGE_PIN W14       [get_ports sw_log_diag_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_log_diag_raw]

set_property PACKAGE_PIN W13       [get_ports sw_lis3dh_i2c_acc_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_lis3dh_i2c_acc_raw]

set_property PACKAGE_PIN V2        [get_ports sw_adxl362_spi_acc_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_adxl362_spi_acc_raw]

set_property PACKAGE_PIN T3        [get_ports sw_cmps2_mmc3416_mag_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_cmps2_mmc3416_mag_raw]

set_property PACKAGE_PIN T2        [get_ports sw_pmon1_pwr_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_pmon1_pwr_raw]

set_property PACKAGE_PIN R3        [get_ports sw_mag1_offset_x_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_mag1_offset_x_raw]

set_property PACKAGE_PIN W2        [get_ports sw_mag1_offset_y_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_mag1_offset_y_raw]

set_property PACKAGE_PIN U1        [get_ports sw_mag1_offset_z_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_mag1_offset_z_raw]

set_property PACKAGE_PIN T1        [get_ports sw_compass_default_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_compass_default_raw]

set_property PACKAGE_PIN R2        [get_ports sw_ext_i2c_raw]
set_property IOSTANDARD LVCMOS33   [get_ports sw_ext_i2c_raw]



## ----------------------------
## Shared I2C sensor bus on Pmod JA
##   Basys 3 JA3 = SCL -> shared I2C SCL
##   Basys 3 JA4 = SDA -> shared I2C SDA
##
## Devices on this bus:
##   Pmod CMPS2 / MMC34160PJ 3-axis magnetometer
##     - Magnetic-field and derived-heading source for the RTL
##     - 7-bit addr: 0b0110000 = 7'h30 = 0x30
##     - Range: approximately +/-16 gauss magnetic field
##
##   LIS3DH 3-axis accelerometer
##     - Auxiliary acceleration / redundancy / vibration / shock source
##     - Not a magnetometer; do not route it into mag_* or heading logic
##     - 7-bit addr: 7'h18 when SA0=0, 7'h19 when SA0=1
##
## No additional FPGA pins are required for LIS3DH when it shares this I2C bus.
## Pmod power pins, including 3.3V and GND, are board pins and do not appear in
## the XDC.
##
## The current I2C master does not support clock stretching, so SCL is an
## open-drain output-only FPGA port. SDA remains bidirectional for ACK and
## read-data sampling.
## ----------------------------
set_property PACKAGE_PIN J2        [get_ports scl]
set_property IOSTANDARD LVCMOS33   [get_ports scl]
set_property PULLUP true           [get_ports scl]
set_property SLEW SLOW             [get_ports scl]

set_property PACKAGE_PIN G2        [get_ports sda]
set_property IOSTANDARD LVCMOS33   [get_ports sda]
set_property PULLUP true           [get_ports sda]
set_property SLEW SLOW             [get_ports sda]



## ----------------------------
## Pmod ACL2 allocation on Pmod JB
##   ACL2 J1 pin 1 = CS_N  -> Basys 3 JB1
##   ACL2 J1 pin 2 = MOSI  -> Basys 3 JB2
##   ACL2 J1 pin 3 = MISO  -> Basys 3 JB3
##   ACL2 J1 pin 4 = SCLK  -> Basys 3 JB4
##   ACL2 J1 pin 7 = INT2  -> Basys 3 JB7  (optional)
##   ACL2 J1 pin 8 = INT1  -> Basys 3 JB8  (optional)
##
## Active after the ADXL362/ACL2 RTL exposes these exact top-level ports.
## ----------------------------
set_property PACKAGE_PIN A14      [get_ports adxl362_cs_n]
set_property IOSTANDARD LVCMOS33  [get_ports adxl362_cs_n]
set_property SLEW SLOW            [get_ports adxl362_cs_n]

set_property PACKAGE_PIN A16      [get_ports adxl362_mosi]
set_property IOSTANDARD LVCMOS33  [get_ports adxl362_mosi]
set_property SLEW SLOW            [get_ports adxl362_mosi]

set_property PACKAGE_PIN B15      [get_ports adxl362_miso]
set_property IOSTANDARD LVCMOS33  [get_ports adxl362_miso]

set_property PACKAGE_PIN B16      [get_ports adxl362_sclk]
set_property IOSTANDARD LVCMOS33  [get_ports adxl362_sclk]
set_property SLEW SLOW            [get_ports adxl362_sclk]

set_property PACKAGE_PIN A15      [get_ports adxl362_int2]
set_property IOSTANDARD LVCMOS33  [get_ports adxl362_int2]

set_property PACKAGE_PIN A17      [get_ports adxl362_int1]
set_property IOSTANDARD LVCMOS33  [get_ports adxl362_int1]

## ----------------------------
## Pmod JC slow GPIO / DPOT allocation
##   JC1..JC4 = Pmod LS1 S1..S4 detector outputs
##   JC7      = Pmod PIR motion output
##   JC8..JC10 are reserved for DPOT mode-0 SPI, but the current top holds
##              DPOT CS_N high until a command-authority path is implemented.
## ----------------------------
set_property PACKAGE_PIN K17       [get_ports {ls1_s_raw[0]}]
set_property IOSTANDARD LVCMOS33   [get_ports {ls1_s_raw[0]}]

set_property PACKAGE_PIN M18       [get_ports {ls1_s_raw[1]}]
set_property IOSTANDARD LVCMOS33   [get_ports {ls1_s_raw[1]}]

set_property PACKAGE_PIN N17       [get_ports {ls1_s_raw[2]}]
set_property IOSTANDARD LVCMOS33   [get_ports {ls1_s_raw[2]}]

set_property PACKAGE_PIN P18       [get_ports {ls1_s_raw[3]}]
set_property IOSTANDARD LVCMOS33   [get_ports {ls1_s_raw[3]}]

set_property PACKAGE_PIN L17       [get_ports pir_motion_raw]
set_property IOSTANDARD LVCMOS33   [get_ports pir_motion_raw]

set_property PACKAGE_PIN M19       [get_ports dpot_cs_n]
set_property IOSTANDARD LVCMOS33   [get_ports dpot_cs_n]
set_property SLEW SLOW             [get_ports dpot_cs_n]

set_property PACKAGE_PIN P17       [get_ports dpot_mosi]
set_property IOSTANDARD LVCMOS33   [get_ports dpot_mosi]
set_property SLEW SLOW             [get_ports dpot_mosi]

set_property PACKAGE_PIN R18       [get_ports dpot_sclk]
set_property IOSTANDARD LVCMOS33   [get_ports dpot_sclk]
set_property SLEW SLOW             [get_ports dpot_sclk]

## ----------------------------
## VGA
## vga_rgb[11:8] = R
## vga_rgb[7:4]  = G
## vga_rgb[3:0]  = B
## ----------------------------

## Red -> vga_rgb[11:8]
set_property PACKAGE_PIN G19       [get_ports {vga_rgb[8]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[8]}]
set_property PACKAGE_PIN H19       [get_ports {vga_rgb[9]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[9]}]
set_property PACKAGE_PIN J19       [get_ports {vga_rgb[10]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[10]}]
set_property PACKAGE_PIN N19       [get_ports {vga_rgb[11]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[11]}]

## Green -> vga_rgb[7:4]
set_property PACKAGE_PIN J17       [get_ports {vga_rgb[4]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[4]}]
set_property PACKAGE_PIN H17       [get_ports {vga_rgb[5]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[5]}]
set_property PACKAGE_PIN G17       [get_ports {vga_rgb[6]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[6]}]
set_property PACKAGE_PIN D17       [get_ports {vga_rgb[7]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[7]}]

## Blue -> vga_rgb[3:0]
set_property PACKAGE_PIN N18       [get_ports {vga_rgb[0]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[0]}]
set_property PACKAGE_PIN L18       [get_ports {vga_rgb[1]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[1]}]
set_property PACKAGE_PIN K18       [get_ports {vga_rgb[2]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[2]}]
set_property PACKAGE_PIN J18       [get_ports {vga_rgb[3]}]
set_property IOSTANDARD LVCMOS33   [get_ports {vga_rgb[3]}]


## Visualization page buttons
##   BTNU = next page
##   BTND = previous page
##   BTNR = direct select latch; default direct compass/MAG evidence page
##          or optional SW13:SW11 encoded view ID load outside MAG1/fault ownership
##   BTNL is intentionally unused by this top-level port list.
set_property PACKAGE_PIN T18      [get_ports btn_page_raw]
set_property IOSTANDARD LVCMOS33  [get_ports btn_page_raw]

set_property PACKAGE_PIN U17      [get_ports btn_prev_raw]
set_property IOSTANDARD LVCMOS33  [get_ports btn_prev_raw]

set_property PACKAGE_PIN T17      [get_ports btn_direct_compass_raw]
set_property IOSTANDARD LVCMOS33  [get_ports btn_direct_compass_raw]

## CLS UART TX
set_property PACKAGE_PIN A18  [get_ports cls_tx]
set_property IOSTANDARD LVCMOS33        [get_ports cls_tx]

# set_property PACKAGE_PIN L2       [get_ports cls_rx]
# set_property IOSTANDARD LVCMOS33  [get_ports cls_rx]

## External-MCU fixed-packet UART bridge on Basys-3 JXADC.
## Board wiring contract:
##   EK-TM4C123GXL PC5/U1TX/J4.05 -> JXADC pin 1 / XA1_P /
##     teensy_uart_rx_raw / FPGA input
##   JXADC pin 2 / XA2_P / teensy_uart_tx / FPGA output -> optional
##     EK-TM4C123GXL PC4/U1RX/J4.04
##   JXADC pin 5 or 11 GND -> LaunchPad GND
##   Do not backfeed power between boards. Use 3.3 V logic only; do not connect
##   LaunchPad J3.01 5.0 V to any FPGA signal. TX currently idles high for
##   future ACK/debug.
set_property PACKAGE_PIN J3       [get_ports teensy_uart_rx_raw]
set_property IOSTANDARD LVCMOS33  [get_ports teensy_uart_rx_raw]
set_property PULLUP true          [get_ports teensy_uart_rx_raw]
set_property PACKAGE_PIN L3       [get_ports teensy_uart_tx]
set_property IOSTANDARD LVCMOS33  [get_ports teensy_uart_tx]
set_property DRIVE 8              [get_ports teensy_uart_tx]
set_property SLEW SLOW            [get_ports teensy_uart_tx]

## Sync
set_property PACKAGE_PIN P19       [get_ports vga_hsync]
set_property IOSTANDARD LVCMOS33   [get_ports vga_hsync]
set_property PACKAGE_PIN R19       [get_ports vga_vsync]
set_property IOSTANDARD LVCMOS33   [get_ports vga_vsync]

## ----------------------------
## SYS -> PIX Planar Compass Truth page overlay CDC
## ----------------------------
## planar_compass_truth_page_vga transfers a low-rate diagnostic telemetry
## snapshot by holding src_bundle_hold stable in the SYS clock domain,
## synchronizing a toggle into the 25 MHz pixel domain, and then sampling the
## held bundle into dst_bundle_shadow. Keep this exception scoped to the overlay
## snapshot bus only; do not clock-group the whole SYS and PIX domains.
## The Basys-3 default build compiles this diagnostic page out to stay within
## xc7a35t LUT limits. Leave the page-specific CDC exceptions out of the active
## XDC; reintroduce them with the diagnostic page build if the overlay is enabled.
