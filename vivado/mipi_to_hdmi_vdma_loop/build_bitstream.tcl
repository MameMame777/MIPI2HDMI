set script_dir [file normalize [file dirname [info script]]]
set repo_dir   [file normalize [file join $script_dir ../..]]
set PROJECT_DIR_NAME vloop

if {[llength $argv] >= 8} {
    set PROJECT_DIR_NAME [lindex $argv 7]
}

if {[llength $argv] >= 9} {
    set CAPTURE_RAW_PAYLOAD [lindex $argv 8]
} elseif {![info exists CAPTURE_RAW_PAYLOAD]} {
    set CAPTURE_RAW_PAYLOAD 0
}

if {[llength $argv] >= 10} {
    set USE_RGB565_GRAY [lindex $argv 9]
} elseif {![info exists USE_RGB565_GRAY]} {
    set USE_RGB565_GRAY 0
}

if {[llength $argv] >= 11} {
    set PROBE_LANE1_BITSLIP_SWEEP [lindex $argv 10]
} elseif {![info exists PROBE_LANE1_BITSLIP_SWEEP]} {
    set PROBE_LANE1_BITSLIP_SWEEP 0
}

# IMAGE_FORMAT: 0=YUV422 (default), 1=RGB565, 2=RAW8, 3=RAW10
if {[llength $argv] >= 12} {
    set IMAGE_FORMAT [lindex $argv 11]
} elseif {![info exists IMAGE_FORMAT]} {
    set IMAGE_FORMAT 0
}

set proj_dir   [file join $repo_dir $PROJECT_DIR_NAME]

if {[llength $argv] >= 1} {
    set PROBE_IDELAY_TAP [lindex $argv 0]
} elseif {![info exists PROBE_IDELAY_TAP]} {
    set PROBE_IDELAY_TAP 8
}

if {[llength $argv] >= 2} {
    set STREAM_PAIRING [lindex $argv 1]
} elseif {![info exists STREAM_PAIRING]} {
    set STREAM_PAIRING 0
}

if {[llength $argv] >= 3} {
    set OV5640_MIPI_CTRL_4800 [lindex $argv 2]
} elseif {![info exists OV5640_MIPI_CTRL_4800]} {
    set OV5640_MIPI_CTRL_4800 36
}

if {[llength $argv] >= 4} {
    set OV5640_TEST_PATTERN_ENABLE [lindex $argv 3]
} elseif {![info exists OV5640_TEST_PATTERN_ENABLE]} {
    set OV5640_TEST_PATTERN_ENABLE 0
}

if {[llength $argv] >= 5} {
    set OV5640_FORMAT_CTRL_4300 [lindex $argv 4]
} elseif {![info exists OV5640_FORMAT_CTRL_4300]} {
    set OV5640_FORMAT_CTRL_4300 48
}

if {[llength $argv] >= 6} {
    set OV5640_ISP_CTRL_5000 [lindex $argv 5]
} elseif {![info exists OV5640_ISP_CTRL_5000]} {
    set OV5640_ISP_CTRL_5000 167
}

if {[llength $argv] >= 7} {
    set OV5640_ISP_CTRL_5001 [lindex $argv 6]
} elseif {![info exists OV5640_ISP_CTRL_5001]} {
    set OV5640_ISP_CTRL_5001 131
}

if {![info exists OV5640_ISP_FORMAT_501F]} {
    set OV5640_ISP_FORMAT_501F 0
}

puts "PROBE_IDELAY_TAP: $PROBE_IDELAY_TAP"
puts "STREAM_PAIRING: $STREAM_PAIRING"
puts "OV5640_MIPI_CTRL_4800: $OV5640_MIPI_CTRL_4800"
puts "OV5640_TEST_PATTERN_ENABLE: $OV5640_TEST_PATTERN_ENABLE"
puts "OV5640_FORMAT_CTRL_4300: $OV5640_FORMAT_CTRL_4300"
puts "OV5640_ISP_FORMAT_501F: $OV5640_ISP_FORMAT_501F"
puts "OV5640_ISP_CTRL_5000: $OV5640_ISP_CTRL_5000"
puts "OV5640_ISP_CTRL_5001: $OV5640_ISP_CTRL_5001"
puts "CAPTURE_RAW_PAYLOAD: $CAPTURE_RAW_PAYLOAD"
puts "USE_RGB565_GRAY: $USE_RGB565_GRAY"
puts "IMAGE_FORMAT: $IMAGE_FORMAT (0=YUV422, 1=RGB565, 2=RAW8, 3=RAW10)"
puts "PROBE_LANE1_BITSLIP_SWEEP: $PROBE_LANE1_BITSLIP_SWEEP"
puts "PROJECT_DIR_NAME: $PROJECT_DIR_NAME"

create_project vloop $proj_dir -part xc7z020clg400-1 -force

set_property target_language Verilog [current_project]
set_property simulator_language Verilog [current_project]

add_files -norecurse [list \
    [file join $repo_dir rtl/prototype/ov5640_sccb_init_probe.sv] \
    [file join $repo_dir rtl/prototype/dphy_hs_byte_probe.sv] \
    [file join $repo_dir rtl/prototype/dphy_raw_byte_ringbuf.sv] \
    [file join $repo_dir rtl/mipi_rx/byte_to_core_cdc.sv] \
    [file join $repo_dir rtl/mipi_rx/csi2_packet_parser.sv] \
    [file join $repo_dir rtl/mipi_rx/csi2_header_ecc.sv] \
    [file join $repo_dir rtl/mipi_rx/csi2_payload_crc.sv] \
    [file join $repo_dir rtl/mipi_rx/csi2_vcdt_filter.sv] \
    [file join $repo_dir rtl/mipi_rx/csi2_frame_state.sv] \
    [file join $repo_dir rtl/mipi_rx/axis_video_bridge.sv] \
    [file join $repo_dir rtl/img_proc/yuv422_gray_unpack.sv] \
    [file join $repo_dir rtl/img_proc/rgb565_gray_unpack.sv] \
    [file join $repo_dir rtl/img_proc/yuv422_crc_framebuffer_axis.sv] \
    [file join $repo_dir rtl/img_proc/axis_y8_to_vdma32.sv] \
    [file join $repo_dir rtl/img_proc/axis_y8_to_vdma32_ref.v] \
    [file join $repo_dir rtl/img_proc/axis_vdma32_to_y8.sv] \
    [file join $repo_dir rtl/img_proc/axis_vdma32_to_y8_ref.v] \
    [file join $repo_dir rtl/img_proc/ob_row_masker.sv] \
    [file join $repo_dir rtl/img_proc/raw8_passthrough.sv] \
    [file join $repo_dir rtl/img_proc/raw10_unpack.sv] \
    [file join $repo_dir rtl/hdmi/hdmi_output.sv] \
    [file join $repo_dir rtl/hdmi/hdmi_tpg_top.sv] \
    [file join $repo_dir rtl/prototype/mipi_to_hdmi_probe_top.sv] \
    [file join $repo_dir rtl/prototype/mipi_to_hdmi_vdma_loop_ref.v] \
]

foreach source_file [get_files *.sv] {
    set_property file_type SystemVerilog $source_file
}

set_property verilog_define {MIPI_CAPTURE_PORTS MIPI_VDMA_LOOP_PORTS} [current_fileset]
set_property generic [list \
    PROBE_IDELAY_TAP=$PROBE_IDELAY_TAP \
    PROBE_LANE1_BITSLIP_SWEEP=$PROBE_LANE1_BITSLIP_SWEEP \
    STREAM_PAIRING=$STREAM_PAIRING \
    OV5640_MIPI_CTRL_4800=$OV5640_MIPI_CTRL_4800 \
    OV5640_FORMAT_CTRL_4300=$OV5640_FORMAT_CTRL_4300 \
    OV5640_ISP_FORMAT_501F=$OV5640_ISP_FORMAT_501F \
    OV5640_ISP_CTRL_5000=$OV5640_ISP_CTRL_5000 \
    OV5640_ISP_CTRL_5001=$OV5640_ISP_CTRL_5001 \
    OV5640_TEST_PATTERN_ENABLE=$OV5640_TEST_PATTERN_ENABLE \
    CAPTURE_RAW_PAYLOAD=$CAPTURE_RAW_PAYLOAD \
    USE_RGB565_GRAY=$USE_RGB565_GRAY \
    IMAGE_FORMAT=$IMAGE_FORMAT \
] [current_fileset]

add_files -fileset constrs_1 -norecurse [file join $repo_dir vivado/constraints/mipi_to_hdmi_probe.xdc]
add_files -fileset constrs_1 -norecurse [file join $repo_dir vivado/constraints/vdma_loop_clock_groups.xdc]
set_property used_in_synthesis false [get_files [file join $repo_dir vivado/constraints/vdma_loop_clock_groups.xdc]]
set_property SEVERITY {Warning} [get_drc_checks LUTOI-1]

update_compile_order -fileset sources_1

proc connect_pin_if_exists {from_pin to_pin} {
    if {[llength [get_bd_pins -quiet $from_pin]] && [llength [get_bd_pins -quiet $to_pin]]} {
        connect_bd_net [get_bd_pins $from_pin] [get_bd_pins $to_pin]
    }
}

proc create_and_connect_port {name dir cell_pin args} {
    set cmd [list create_bd_port -dir $dir]
    foreach arg $args { lappend cmd $arg }
    lappend cmd $name
    set port [eval $cmd]
    connect_bd_net $port [get_bd_pins $cell_pin]
    return $port
}

# Suppress AV-scanner-induced BD rule init errors (Ip 78-90 fires when Vivado
# can't read install-dir TCL rule files; the BD design is still created correctly).
foreach msg_id {{Ip 78-90} {BD 41-69} {Common 17-232} {IP_Flow 19-883} {IP_Flow 19-3428}} {
    catch {set_msg_config -id $msg_id -new_severity {WARNING}}
}
if {[catch {create_bd_design bd} err]} {
    puts "  create_bd_design note: $err"
}
if {[catch {current_bd_design} _cbd] || [string length [current_bd_design]] == 0} {
    puts "ERROR: BD design not available after create_bd_design"
    exit 1
}
puts "BD design ready: [current_bd_design]"

create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_EN_RST0_PORT {1} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100.000000} \
    CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125} \
    CONFIG.PCW_DDR_RAM_HIGHADDR {0x3FFFFFFF} \
] [get_bd_cells ps7]

make_bd_intf_pins_external [get_bd_intf_pins ps7/DDR]
make_bd_intf_pins_external [get_bd_intf_pins ps7/FIXED_IO]
set_property name DDR [get_bd_intf_ports DDR_0]
set_property name FIXED_IO [get_bd_intf_ports FIXED_IO_0]

create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst0
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins rst0/slowest_sync_clk]
connect_bd_net [get_bd_pins ps7/FCLK_RESET0_N] [get_bd_pins rst0/ext_reset_in]
connect_pin_if_exists ps7/FCLK_CLK0 ps7/S_AXI_HP0_ACLK

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma:6.3 axi_vdma_0
set_property -dict [list \
    CONFIG.c_include_s2mm {1} \
    CONFIG.c_include_mm2s {1} \
    CONFIG.c_m_axi_s2mm_data_width {64} \
    CONFIG.c_m_axi_mm2s_data_width {64} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
    CONFIG.c_num_fstores {3} \
    CONFIG.c_s2mm_linebuffer_depth {1024} \
    CONFIG.c_mm2s_linebuffer_depth {1024} \
    CONFIG.c_include_s2mm_dre {0} \
    CONFIG.c_include_mm2s_dre {0} \
    CONFIG.c_use_s2mm_fsync {0} \
    CONFIG.c_use_mm2s_fsync {0} \
    CONFIG.c_s2mm_genlock_mode {0} \
    CONFIG.c_s2mm_genlock_repeat_en {0} \
    CONFIG.c_mm2s_genlock_mode {0} \
    CONFIG.c_mm2s_genlock_repeat_en {0} \
] [get_bd_cells axi_vdma_0]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 ic0
set_property -dict [list \
    CONFIG.NUM_SI {2} \
    CONFIG.NUM_MI {1} \
] [get_bd_cells ic0]

create_bd_cell -type module -reference axis_y8_to_vdma32_ref pack_s2mm

create_bd_cell -type module -reference axis_vdma32_to_y8_ref unpack_mm2s

create_bd_cell -type ip -vlnv xilinx.com:ip:axis_clock_converter:1.1 cc_mm2s
set_property -dict [list \
    CONFIG.TDATA_NUM_BYTES {1} \
    CONFIG.HAS_TLAST {1} \
    CONFIG.TUSER_WIDTH {1} \
] [get_bd_cells cc_mm2s]

create_bd_cell -type ip -vlnv xilinx.com:ip:axis_subset_converter:1.1 sub_y_to_rgb
set_property -dict [list \
    CONFIG.S_TDATA_NUM_BYTES {1} \
    CONFIG.M_TDATA_NUM_BYTES {3} \
    CONFIG.S_HAS_TLAST {1} \
    CONFIG.M_HAS_TLAST {1} \
    CONFIG.S_TUSER_WIDTH {1} \
    CONFIG.M_TUSER_WIDTH {1} \
    CONFIG.TDATA_REMAP {tdata[7:0],tdata[7:0],tdata[7:0]} \
    CONFIG.TUSER_REMAP {tuser[0:0]} \
    CONFIG.TLAST_REMAP {tlast[0]} \
] [get_bd_cells sub_y_to_rgb]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 dbg_gpio
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_INPUTS {1} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_ALL_OUTPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {8} \
] [get_bd_cells dbg_gpio]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 sccb_gpio
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
] [get_bd_cells sccb_gpio]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 idelay_gpio
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
] [get_bd_cells idelay_gpio]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 bitslip_gpio
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
] [get_bd_cells bitslip_gpio]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 frame_lines_gpio
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
    CONFIG.C_DOUT_DEFAULT {0x02000000} \
] [get_bd_cells frame_lines_gpio]

create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 rawcap_gpio
set_property -dict [list \
    CONFIG.C_IS_DUAL {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
] [get_bd_cells rawcap_gpio]

create_bd_cell -type module -reference mipi_to_hdmi_vdma_loop_ref core0
set_property -dict [list \
    CONFIG.PROBE_IDELAY_TAP $PROBE_IDELAY_TAP \
    CONFIG.PROBE_LANE1_BITSLIP_SWEEP $PROBE_LANE1_BITSLIP_SWEEP \
    CONFIG.STREAM_PAIRING $STREAM_PAIRING \
    CONFIG.OV5640_MIPI_CTRL_4800 $OV5640_MIPI_CTRL_4800 \
    CONFIG.OV5640_FORMAT_CTRL_4300 $OV5640_FORMAT_CTRL_4300 \
    CONFIG.OV5640_ISP_FORMAT_501F $OV5640_ISP_FORMAT_501F \
    CONFIG.OV5640_ISP_CTRL_5000 $OV5640_ISP_CTRL_5000 \
    CONFIG.OV5640_ISP_CTRL_5001 $OV5640_ISP_CTRL_5001 \
    CONFIG.OV5640_TEST_PATTERN_ENABLE $OV5640_TEST_PATTERN_ENABLE \
    CONFIG.CAPTURE_RAW_PAYLOAD $CAPTURE_RAW_PAYLOAD \
    CONFIG.USE_RGB565_GRAY $USE_RGB565_GRAY \
] [get_bd_cells core0]

connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins core0/capture_aclk]
connect_bd_net [get_bd_pins rst0/peripheral_aresetn] [get_bd_pins core0/capture_aresetn]

# Capture (S2MM) path: core0 -> pack_s2mm -> VDMA S2MM -> DDR
connect_bd_intf_net [get_bd_intf_pins core0/m_axis_capture] [get_bd_intf_pins pack_s2mm/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins pack_s2mm/M_AXIS] [get_bd_intf_pins axi_vdma_0/S_AXIS_S2MM]
connect_bd_net [get_bd_pins core0/capture_debug] [get_bd_pins dbg_gpio/gpio_io_i]
connect_bd_net [get_bd_pins dbg_gpio/gpio2_io_o] [get_bd_pins core0/debug_page_sel]
connect_bd_net [get_bd_pins sccb_gpio/gpio_io_o] [get_bd_pins core0/sccb_rt_write_word_in]
connect_bd_net [get_bd_pins core0/sccb_rt_write_status_out] [get_bd_pins sccb_gpio/gpio2_io_i]
connect_bd_net [get_bd_pins idelay_gpio/gpio_io_o] [get_bd_pins core0/idelay_runtime_word_in]
connect_bd_net [get_bd_pins core0/idelay_runtime_status_out] [get_bd_pins idelay_gpio/gpio2_io_i]
connect_bd_net [get_bd_pins bitslip_gpio/gpio_io_o] [get_bd_pins core0/bitslip_runtime_word_in]
connect_bd_net [get_bd_pins core0/bitslip_runtime_status_out] [get_bd_pins bitslip_gpio/gpio2_io_i]
connect_bd_net [get_bd_pins frame_lines_gpio/gpio_io_o] [get_bd_pins core0/frame_lines_runtime_word_in]
connect_bd_net [get_bd_pins core0/frame_lines_runtime_status_out] [get_bd_pins frame_lines_gpio/gpio2_io_i]
connect_bd_net [get_bd_pins rawcap_gpio/gpio_io_o] [get_bd_pins core0/rawcap_word_in]
connect_bd_net [get_bd_pins core0/rawcap_status_out] [get_bd_pins rawcap_gpio/gpio2_io_i]

# Display (MM2S) path: VDMA MM2S -> unpack_mm2s -> cc_mm2s -> sub_y_to_rgb -> core0/s_axis_hdmi
connect_bd_intf_net [get_bd_intf_pins axi_vdma_0/M_AXIS_MM2S] [get_bd_intf_pins unpack_mm2s/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins unpack_mm2s/M_AXIS] [get_bd_intf_pins cc_mm2s/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins cc_mm2s/M_AXIS] [get_bd_intf_pins sub_y_to_rgb/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins sub_y_to_rgb/M_AXIS] [get_bd_intf_pins core0/s_axis_hdmi]

# Clock + reset wiring for the MM2S display path output side and HDMI s_axis
connect_bd_net [get_bd_pins core0/pix_clk_out] [get_bd_pins cc_mm2s/m_axis_aclk]
connect_bd_net [get_bd_pins core0/pix_aresetn_out] [get_bd_pins cc_mm2s/m_axis_aresetn]
connect_bd_net [get_bd_pins core0/pix_clk_out] [get_bd_pins sub_y_to_rgb/aclk]
connect_bd_net [get_bd_pins core0/pix_aresetn_out] [get_bd_pins sub_y_to_rgb/aresetn]

# FCLK_CLK0 (100MHz) wiring for AXI/AXIS infra
connect_pin_if_exists ps7/FCLK_CLK0 axi_vdma_0/s_axi_lite_aclk
connect_pin_if_exists ps7/FCLK_CLK0 axi_vdma_0/m_axi_s2mm_aclk
connect_pin_if_exists ps7/FCLK_CLK0 axi_vdma_0/m_axi_mm2s_aclk
connect_pin_if_exists ps7/FCLK_CLK0 axi_vdma_0/s_axis_s2mm_aclk
connect_pin_if_exists ps7/FCLK_CLK0 axi_vdma_0/m_axis_mm2s_aclk
connect_pin_if_exists ps7/FCLK_CLK0 pack_s2mm/aclk
connect_pin_if_exists ps7/FCLK_CLK0 unpack_mm2s/aclk
connect_pin_if_exists ps7/FCLK_CLK0 cc_mm2s/s_axis_aclk
connect_pin_if_exists ps7/FCLK_CLK0 dbg_gpio/s_axi_aclk
connect_pin_if_exists ps7/FCLK_CLK0 sccb_gpio/s_axi_aclk
connect_pin_if_exists ps7/FCLK_CLK0 idelay_gpio/s_axi_aclk
connect_pin_if_exists ps7/FCLK_CLK0 bitslip_gpio/s_axi_aclk
connect_pin_if_exists ps7/FCLK_CLK0 frame_lines_gpio/s_axi_aclk
connect_pin_if_exists ps7/FCLK_CLK0 rawcap_gpio/s_axi_aclk

connect_pin_if_exists rst0/peripheral_aresetn axi_vdma_0/axi_resetn
connect_pin_if_exists rst0/peripheral_aresetn axi_vdma_0/s_axis_s2mm_aresetn
connect_pin_if_exists rst0/peripheral_aresetn axi_vdma_0/m_axis_mm2s_aresetn
connect_pin_if_exists rst0/peripheral_aresetn axi_vdma_0/s2mm_prmry_resetn
connect_pin_if_exists rst0/peripheral_aresetn axi_vdma_0/mm2s_prmry_resetn
connect_pin_if_exists rst0/peripheral_aresetn pack_s2mm/aresetn
connect_pin_if_exists rst0/peripheral_aresetn unpack_mm2s/aresetn
connect_pin_if_exists rst0/peripheral_aresetn cc_mm2s/s_axis_aresetn
connect_pin_if_exists rst0/peripheral_aresetn dbg_gpio/s_axi_aresetn
connect_pin_if_exists rst0/peripheral_aresetn sccb_gpio/s_axi_aresetn
connect_pin_if_exists rst0/peripheral_aresetn idelay_gpio/s_axi_aresetn
connect_pin_if_exists rst0/peripheral_aresetn bitslip_gpio/s_axi_aresetn
connect_pin_if_exists rst0/peripheral_aresetn frame_lines_gpio/s_axi_aresetn
connect_pin_if_exists rst0/peripheral_aresetn rawcap_gpio/s_axi_aresetn

apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/axi_vdma_0/S_AXI_LITE"} [get_bd_intf_pins axi_vdma_0/S_AXI_LITE]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/dbg_gpio/S_AXI"} [get_bd_intf_pins dbg_gpio/S_AXI]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/sccb_gpio/S_AXI"} [get_bd_intf_pins sccb_gpio/S_AXI]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/idelay_gpio/S_AXI"} [get_bd_intf_pins idelay_gpio/S_AXI]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/bitslip_gpio/S_AXI"} [get_bd_intf_pins bitslip_gpio/S_AXI]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/frame_lines_gpio/S_AXI"} [get_bd_intf_pins frame_lines_gpio/S_AXI]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 -config {Master "/ps7/M_AXI_GP0" Clk "/ps7/FCLK_CLK0" Slave "/rawcap_gpio/S_AXI"} [get_bd_intf_pins rawcap_gpio/S_AXI]

# VDMA MM2S and S2MM both go through ic0 -> S_AXI_HP0
connect_bd_intf_net [get_bd_intf_pins axi_vdma_0/M_AXI_S2MM] [get_bd_intf_pins ic0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_vdma_0/M_AXI_MM2S] [get_bd_intf_pins ic0/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins ic0/M00_AXI] [get_bd_intf_pins ps7/S_AXI_HP0]
connect_pin_if_exists ps7/FCLK_CLK0 ic0/ACLK
connect_pin_if_exists ps7/FCLK_CLK0 ic0/S00_ACLK
connect_pin_if_exists ps7/FCLK_CLK0 ic0/S01_ACLK
connect_pin_if_exists ps7/FCLK_CLK0 ic0/M00_ACLK
connect_pin_if_exists rst0/peripheral_aresetn ic0/ARESETN
connect_pin_if_exists rst0/peripheral_aresetn ic0/S00_ARESETN
connect_pin_if_exists rst0/peripheral_aresetn ic0/S01_ARESETN
connect_pin_if_exists rst0/peripheral_aresetn ic0/M00_ARESETN

create_and_connect_port sysclk I core0/sysclk -type clk
set_property CONFIG.FREQ_HZ 125000000 [get_bd_ports sysclk]
create_and_connect_port led O core0/led -from 3 -to 0

create_and_connect_port dphy_hs_clock_clk_p I core0/dphy_hs_clock_clk_p
create_and_connect_port dphy_hs_clock_clk_n I core0/dphy_hs_clock_clk_n
create_and_connect_port dphy_data_hs_p I core0/dphy_data_hs_p -from 1 -to 0
create_and_connect_port dphy_data_hs_n I core0/dphy_data_hs_n -from 1 -to 0
create_and_connect_port dphy_clk_lp_p I core0/dphy_clk_lp_p
create_and_connect_port dphy_clk_lp_n I core0/dphy_clk_lp_n
create_and_connect_port dphy_data_lp_p I core0/dphy_data_lp_p -from 1 -to 0
create_and_connect_port dphy_data_lp_n I core0/dphy_data_lp_n -from 1 -to 0

create_and_connect_port cam_clk O core0/cam_clk
create_and_connect_port cam_gpio O core0/cam_gpio
create_and_connect_port cam_scl IO core0/cam_scl
create_and_connect_port cam_sda IO core0/cam_sda

create_and_connect_port hdmi_tx_hpd I core0/hdmi_tx_hpd
create_and_connect_port hdmi_tx_clk_p O core0/hdmi_tx_clk_p
create_and_connect_port hdmi_tx_clk_n O core0/hdmi_tx_clk_n
create_and_connect_port hdmi_tx_p O core0/hdmi_tx_p -from 2 -to 0
create_and_connect_port hdmi_tx_n O core0/hdmi_tx_n -from 2 -to 0
create_and_connect_port hdmi_tx_scl O core0/hdmi_tx_scl
create_and_connect_port hdmi_tx_sda IO core0/hdmi_tx_sda
create_and_connect_port hdmi_tx_cec O core0/hdmi_tx_cec

assign_bd_address
# Ensure both VDMA channels can reach the full HP0 DDR range
if {[llength [get_bd_addr_segs -quiet ps7/S_AXI_HP0/HP0_DDR_LOWOCM]]} {
    catch { assign_bd_address -target_address_space /axi_vdma_0/Data_S2MM [get_bd_addr_segs ps7/S_AXI_HP0/HP0_DDR_LOWOCM] -force }
    catch { assign_bd_address -target_address_space /axi_vdma_0/Data_MM2S [get_bd_addr_segs ps7/S_AXI_HP0/HP0_DDR_LOWOCM] -force }
}
validate_bd_design
save_bd_design

set wrapper_file [make_wrapper -files [get_files [file join $proj_dir vloop.srcs sources_1 bd bd bd.bd]] -top]
add_files -norecurse $wrapper_file
set_property top bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

generate_target all [get_files [file join $proj_dir vloop.srcs sources_1 bd bd bd.bd]]

set ooc_synth_runs [list]
foreach run [get_runs *_synth_1] {
    if {[get_property NAME $run] ne "synth_1"} {
        lappend ooc_synth_runs $run
    }
}

if {[llength $ooc_synth_runs] > 0} {
    launch_runs $ooc_synth_runs -jobs 4
    foreach run $ooc_synth_runs {
        wait_on_run $run
        if {[get_property PROGRESS $run] != "100%"} {
            puts "ERROR: OOC synthesis failed: [get_property NAME $run]"
            exit 1
        }
        set run_name [get_property NAME $run]
        regsub {_synth_1$} $run_name "" ip_name
        set run_dcp [file join $proj_dir vloop.runs $run_name ${ip_name}.dcp]
        set ip_dcp [file join $proj_dir vloop.gen sources_1 bd bd ip $ip_name ${ip_name}.dcp]
        if {[file exists $run_dcp] && [file isdirectory [file dirname $ip_dcp]]} {
            file copy -force $run_dcp $ip_dcp
        }
    }
}

launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed"
    exit 1
}

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    puts "ERROR: Implementation failed"
    exit 1
}

set bit_file [file join $proj_dir vloop.runs impl_1 bd_wrapper.bit]
if {![file exists $bit_file]} {
    puts "ERROR: Bitstream file not found at $bit_file"
    exit 1
}

set hwh_src [file join $proj_dir vloop.gen sources_1 bd bd hw_handoff bd.hwh]
if {[file exists $hwh_src]} {
    file copy -force $hwh_src [file join [file dirname $bit_file] bd_wrapper.hwh]
    puts "HWH ready: [file join [file dirname $bit_file] bd_wrapper.hwh]"
} else {
    puts "WARNING: HWH file not found at $hwh_src"
}

puts "Bitstream ready: $bit_file"
