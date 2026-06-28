set_property SEVERITY {Warning} [get_drc_checks LUTOI-1]

set_property -dict { PACKAGE_PIN K17 IOSTANDARD LVCMOS33 } [get_ports { sysclk }]
create_clock -add -name sys_clk_pin -period 8.000 -waveform {0 4} [get_ports { sysclk }]
# dphy_hs_clock = chip MIPI clock lane. 30fps (2026-06-22): chip PLL mult=96 ->
# link_freq = VCO(768)/mipi_div(2) = 384MHz (768Mbps/lane) -> period 2.604ns.
# byte_clk = 384/4 = 96MHz (BUFR/4). (was 2.976ns/336MHz for the 17fps build.)
create_clock -period 2.604 -name dphy_hs_clock_clk_p -waveform {0.000 1.302} [get_ports { dphy_hs_clock_clk_p }]

# -include_generated_clocks pulls refclk_200 (PLLE2 CLKOUT0, the D-PHY lane
# supervisor ctl_clk) and every other sysclk-derived clock into the sys group,
# so all supervisor refclk_200 <-> phy_byte_clk CDCs are declared asynchronous.
set_clock_groups -asynchronous -quiet \
	-group [get_clocks -quiet -include_generated_clocks sys_clk_pin] \
	-group [get_clocks -quiet phy_byte_clk]

set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN M15 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN G14 IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN D18 IOSTANDARD LVCMOS33 } [get_ports { led[3] }]

set_property INTERNAL_VREF 0.6 [get_iobanks 35]

set_property -dict { PACKAGE_PIN J19 IOSTANDARD HSUL_12 } [get_ports { dphy_clk_lp_n }]
set_property -dict { PACKAGE_PIN H20 IOSTANDARD HSUL_12 } [get_ports { dphy_clk_lp_p }]
set_property -dict { PACKAGE_PIN M18 IOSTANDARD HSUL_12 } [get_ports { dphy_data_lp_n[0] }]
set_property -dict { PACKAGE_PIN L19 IOSTANDARD HSUL_12 } [get_ports { dphy_data_lp_p[0] }]
set_property -dict { PACKAGE_PIN L20 IOSTANDARD HSUL_12 } [get_ports { dphy_data_lp_n[1] }]
set_property -dict { PACKAGE_PIN J20 IOSTANDARD HSUL_12 } [get_ports { dphy_data_lp_p[1] }]

set_property -dict { PACKAGE_PIN H18 IOSTANDARD LVDS_25 } [get_ports { dphy_hs_clock_clk_n }]
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVDS_25 } [get_ports { dphy_hs_clock_clk_p }]
set_property -dict { PACKAGE_PIN M20 IOSTANDARD LVDS_25 } [get_ports { dphy_data_hs_n[0] }]
set_property -dict { PACKAGE_PIN M19 IOSTANDARD LVDS_25 } [get_ports { dphy_data_hs_p[0] }]
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVDS_25 } [get_ports { dphy_data_hs_n[1] }]
set_property -dict { PACKAGE_PIN L16 IOSTANDARD LVDS_25 } [get_ports { dphy_data_hs_p[1] }]

set_property -dict { PACKAGE_PIN G19 IOSTANDARD LVCMOS33 } [get_ports { cam_clk }]
set_property -dict { PACKAGE_PIN G20 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { cam_gpio }]
set_property -dict { PACKAGE_PIN F20 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { cam_scl }]
set_property -dict { PACKAGE_PIN F19 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { cam_sda }]

set_property -dict { PACKAGE_PIN E18 IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_hpd }]
set_property -dict { PACKAGE_PIN G17 IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_scl }]
set_property -dict { PACKAGE_PIN G18 IOSTANDARD LVCMOS33 PULLUP true } [get_ports { hdmi_tx_sda }]
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_cec }]

set_property -dict { PACKAGE_PIN H16 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_clk_p }]
set_property -dict { PACKAGE_PIN H17 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_clk_n }]
set_property -dict { PACKAGE_PIN D19 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_p[0] }]
set_property -dict { PACKAGE_PIN D20 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_n[0] }]
set_property -dict { PACKAGE_PIN C20 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_p[1] }]
set_property -dict { PACKAGE_PIN B20 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_n[1] }]
set_property -dict { PACKAGE_PIN B19 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_p[2] }]
set_property -dict { PACKAGE_PIN A20 IOSTANDARD TMDS_33 } [get_ports { hdmi_tx_n[2] }]

set_false_path -from [get_ports hdmi_tx_hpd]

# === D-PHY lane supervisor (dphy_lane_supervisor.sv + dphy_cdc_prims.sv) ===
# refclk_200_unbuf (PLLE2 CLKOUT0 = supervisor ctl_clk + IDELAY refclk) shares
# the sysclk PLL source, so Vivado treats it as SYNCHRONOUS to sys_clk_pin and
# applies a 1 ns setup requirement to the cross-domain paths. The only such
# crossings are the supervisor reset (rst_n: sys -> refclk, 80 reset endpoints)
# and the diagnostic status bundle (refclk -> sys); both are CDC-safe (reset
# asserts asynchronously, the status word is read-only diagnostics). Waive both
# directions. The refclk_200_unbuf <-> phy_byte_clk crossings are already async
# via the set_clock_groups at the top (refclk_200_unbuf is a sys generated clk).
set_false_path -from [get_clocks -quiet sys_clk_pin]      -to [get_clocks -quiet refclk_200_unbuf]
set_false_path -from [get_clocks -quiet refclk_200_unbuf] -to [get_clocks -quiet sys_clk_pin]

# === D-PHY frontend placement pins (2026-06-22, build reproducibility) ===
# The 30fps build sits at WNS=+0.112 (byte_clk 96MHz), a thinner margin than the
# 17fps +0.333, so build-to-build placement drift of the byte_clk source could push
# it negative. Pin the byte_clk generation chain (BUFIO -> BUFR/4 -> phy_byte_clk)
# and the IDELAY reference (IDELAYCTRL) to the exact sites of the verified
# WNS=+0.112 routed checkpoint, so RTL edits elsewhere can't move them.
#   - IODELAY_GROUP "mipi_dphy_idelay" + REFCLK_FREQUENCY=200 are already RTL
#     attributes on the IDELAYCTRL/IDELAYE2 (dphy_hs_byte_probe.sv).
#   - ISERDESE2/IDELAYE2 are pin-tied to the data-lane input ILOGIC/IDELAY sites
#     (M20/M19, L17/L16) by the PACKAGE_PIN assignments, so they need no LOC.
# REF_NAME filters (each primitive is unique in the design) avoid hard-coding the
# hierarchical instance path. If a future build adds another BUFR/BUFIO/IDELAYCTRL
# the filter would match >1 and error -- that is the intended early warning.
set_property LOC BUFIO_X1Y8      [get_cells -hier -filter {REF_NAME == BUFIO}]
set_property LOC BUFR_X1Y9       [get_cells -hier -filter {REF_NAME == BUFR}]
set_property LOC IDELAYCTRL_X1Y2 [get_cells -hier -filter {REF_NAME == IDELAYCTRL}]
