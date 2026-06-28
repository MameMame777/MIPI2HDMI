
################################################################
# This is a generated script based on design: bd
#
# Though there are limitations about the generated script,
# the main purpose of this utility is to make learning
# IP Integrator Tcl commands easier.
################################################################

namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]

################################################################
# Check if script is running in correct Vivado version.
################################################################
set scripts_vivado_version 2024.2
set current_vivado_version [version -short]

if { [string first $scripts_vivado_version $current_vivado_version] == -1 } {
   puts ""
   if { [string compare $scripts_vivado_version $current_vivado_version] > 0 } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2042 -severity "ERROR" " This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Sourcing the script failed since it was created with a future version of Vivado."}

   } else {
     catch {common::send_gid_msg -ssname BD::TCL -id 2041 -severity "ERROR" "This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Please run the script in Vivado <$scripts_vivado_version> then open the design in Vivado <$current_vivado_version>. Upgrade the design by running \"Tools => Report => Report IP Status...\", then run write_bd_tcl to create an updated script."}

   }

   return 1
}

################################################################
# START
################################################################

# To test this script, run the following commands from Vivado Tcl console:
# source bd_script.tcl


# The design that will be created by this Tcl script contains the following 
# module references:
# axis_y8_to_vdma32_ref, axis_vdma32_to_y8_ref, mipi_to_hdmi_vdma_loop_ref

# Please add the sources of those modules before sourcing this Tcl script.

# If there is no project opened, this script will create a
# project, but make sure you do not have an existing project
# <./myproj/project_1.xpr> in the current working folder.

set list_projs [get_projects -quiet]
if { $list_projs eq "" } {
   create_project project_1 myproj -part xc7z020clg400-1
}


# CHANGE DESIGN NAME HERE
variable design_name
set design_name bd

# If you do not already have an existing IP Integrator design open,
# you can create a design using the following command:
#    create_bd_design $design_name

# Creating design if needed
set errMsg ""
set nRet 0

set cur_design [current_bd_design -quiet]
set list_cells [get_bd_cells -quiet]

if { ${design_name} eq "" } {
   # USE CASES:
   #    1) Design_name not set

   set errMsg "Please set the variable <design_name> to a non-empty value."
   set nRet 1

} elseif { ${cur_design} ne "" && ${list_cells} eq "" } {
   # USE CASES:
   #    2): Current design opened AND is empty AND names same.
   #    3): Current design opened AND is empty AND names diff; design_name NOT in project.
   #    4): Current design opened AND is empty AND names diff; design_name exists in project.

   if { $cur_design ne $design_name } {
      common::send_gid_msg -ssname BD::TCL -id 2001 -severity "INFO" "Changing value of <design_name> from <$design_name> to <$cur_design> since current design is empty."
      set design_name [get_property NAME $cur_design]
   }
   common::send_gid_msg -ssname BD::TCL -id 2002 -severity "INFO" "Constructing design in IPI design <$cur_design>..."

} elseif { ${cur_design} ne "" && $list_cells ne "" && $cur_design eq $design_name } {
   # USE CASES:
   #    5) Current design opened AND has components AND same names.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 1
} elseif { [get_files -quiet ${design_name}.bd] ne "" } {
   # USE CASES: 
   #    6) Current opened design, has components, but diff names, design_name exists in project.
   #    7) No opened design, design_name exists in project.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 2

} else {
   # USE CASES:
   #    8) No opened design, design_name not in project.
   #    9) Current opened design, has components, but diff names, design_name not in project.

   common::send_gid_msg -ssname BD::TCL -id 2003 -severity "INFO" "Currently there is no design <$design_name> in project, so creating one..."

   create_bd_design $design_name

   common::send_gid_msg -ssname BD::TCL -id 2004 -severity "INFO" "Making design <$design_name> as current_bd_design."
   current_bd_design $design_name

}

common::send_gid_msg -ssname BD::TCL -id 2005 -severity "INFO" "Currently the variable <design_name> is equal to \"$design_name\"."

if { $nRet != 0 } {
   catch {common::send_gid_msg -ssname BD::TCL -id 2006 -severity "ERROR" $errMsg}
   return $nRet
}

set bCheckIPsPassed 1
##################################################################
# CHECK IPs
##################################################################
set bCheckIPs 1
if { $bCheckIPs == 1 } {
   set list_check_ips "\ 
xilinx.com:ip:processing_system7:5.5\
xilinx.com:ip:proc_sys_reset:5.0\
xilinx.com:ip:axi_vdma:6.3\
xilinx.com:ip:axis_clock_converter:1.1\
xilinx.com:ip:axis_subset_converter:1.1\
xilinx.com:ip:axi_gpio:2.0\
"

   set list_ips_missing ""
   common::send_gid_msg -ssname BD::TCL -id 2011 -severity "INFO" "Checking if the following IPs exist in the project's IP catalog: $list_check_ips ."

   foreach ip_vlnv $list_check_ips {
      set ip_obj [get_ipdefs -all $ip_vlnv]
      if { $ip_obj eq "" } {
         lappend list_ips_missing $ip_vlnv
      }
   }

   if { $list_ips_missing ne "" } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2012 -severity "ERROR" "The following IPs are not found in the IP Catalog:\n  $list_ips_missing\n\nResolution: Please add the repository containing the IP(s) to the project." }
      set bCheckIPsPassed 0
   }

}

##################################################################
# CHECK Modules
##################################################################
set bCheckModules 1
if { $bCheckModules == 1 } {
   set list_check_mods "\ 
axis_y8_to_vdma32_ref\
axis_vdma32_to_y8_ref\
mipi_to_hdmi_vdma_loop_ref\
"

   set list_mods_missing ""
   common::send_gid_msg -ssname BD::TCL -id 2020 -severity "INFO" "Checking if the following modules exist in the project's sources: $list_check_mods ."

   foreach mod_vlnv $list_check_mods {
      if { [can_resolve_reference $mod_vlnv] == 0 } {
         lappend list_mods_missing $mod_vlnv
      }
   }

   if { $list_mods_missing ne "" } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2021 -severity "ERROR" "The following module(s) are not found in the project: $list_mods_missing" }
      common::send_gid_msg -ssname BD::TCL -id 2022 -severity "INFO" "Please add source files for the missing module(s) above."
      set bCheckIPsPassed 0
   }
}

if { $bCheckIPsPassed != 1 } {
  common::send_gid_msg -ssname BD::TCL -id 2023 -severity "WARNING" "Will not continue with creation of design due to the error(s) above."
  return 3
}

##################################################################
# DESIGN PROCs
##################################################################



# Procedure to create entire design; Provide argument to make
# procedure reusable. If parentCell is "", will use root.
proc create_root_design { parentCell } {

  variable script_folder
  variable design_name

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj


  # Create interface ports
  set DDR [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 DDR ]

  set FIXED_IO [ create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 FIXED_IO ]


  # Create ports
  set sysclk [ create_bd_port -dir I -type clk -freq_hz 125000000 sysclk ]
  set led [ create_bd_port -dir O -from 3 -to 0 led ]
  set dphy_hs_clock_clk_p [ create_bd_port -dir I dphy_hs_clock_clk_p ]
  set dphy_hs_clock_clk_n [ create_bd_port -dir I dphy_hs_clock_clk_n ]
  set dphy_data_hs_p [ create_bd_port -dir I -from 1 -to 0 dphy_data_hs_p ]
  set dphy_data_hs_n [ create_bd_port -dir I -from 1 -to 0 dphy_data_hs_n ]
  set dphy_clk_lp_p [ create_bd_port -dir I dphy_clk_lp_p ]
  set dphy_clk_lp_n [ create_bd_port -dir I dphy_clk_lp_n ]
  set dphy_data_lp_p [ create_bd_port -dir I -from 1 -to 0 dphy_data_lp_p ]
  set dphy_data_lp_n [ create_bd_port -dir I -from 1 -to 0 dphy_data_lp_n ]
  set cam_clk [ create_bd_port -dir O cam_clk ]
  set cam_gpio [ create_bd_port -dir O cam_gpio ]
  set cam_scl [ create_bd_port -dir IO cam_scl ]
  set cam_sda [ create_bd_port -dir IO cam_sda ]
  set hdmi_tx_hpd [ create_bd_port -dir I hdmi_tx_hpd ]
  set hdmi_tx_clk_p [ create_bd_port -dir O hdmi_tx_clk_p ]
  set hdmi_tx_clk_n [ create_bd_port -dir O hdmi_tx_clk_n ]
  set hdmi_tx_p [ create_bd_port -dir O -from 2 -to 0 hdmi_tx_p ]
  set hdmi_tx_n [ create_bd_port -dir O -from 2 -to 0 hdmi_tx_n ]
  set hdmi_tx_scl [ create_bd_port -dir O hdmi_tx_scl ]
  set hdmi_tx_sda [ create_bd_port -dir IO hdmi_tx_sda ]
  set hdmi_tx_cec [ create_bd_port -dir O hdmi_tx_cec ]

  # Create instance: ps7, and set properties
  set ps7 [ create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7 ]
  set_property -dict [list \
    CONFIG.PCW_ACT_APU_PERIPHERAL_FREQMHZ {666.666687} \
    CONFIG.PCW_ACT_CAN_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_DCI_PERIPHERAL_FREQMHZ {10.158730} \
    CONFIG.PCW_ACT_ENET0_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_ENET1_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ {100.000000} \
    CONFIG.PCW_ACT_FPGA1_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_FPGA2_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_FPGA3_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_PCAP_PERIPHERAL_FREQMHZ {200.000000} \
    CONFIG.PCW_ACT_QSPI_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_SDIO_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_SMC_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_SPI_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_TPIU_PERIPHERAL_FREQMHZ {200.000000} \
    CONFIG.PCW_ACT_TTC0_CLK0_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_ACT_TTC0_CLK1_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_ACT_TTC0_CLK2_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_ACT_TTC1_CLK0_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_ACT_TTC1_CLK1_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_ACT_TTC1_CLK2_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_ACT_UART_PERIPHERAL_FREQMHZ {10.000000} \
    CONFIG.PCW_ACT_WDT_PERIPHERAL_FREQMHZ {111.111115} \
    CONFIG.PCW_CLK0_FREQ {100000000} \
    CONFIG.PCW_CLK1_FREQ {10000000} \
    CONFIG.PCW_CLK2_FREQ {10000000} \
    CONFIG.PCW_CLK3_FREQ {10000000} \
    CONFIG.PCW_DDR_RAM_HIGHADDR {0x3FFFFFFF} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_EN_RST0_PORT {1} \
    CONFIG.PCW_FCLK_CLK0_BUF {TRUE} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100.000000} \
    CONFIG.PCW_FPGA_FCLK0_ENABLE {1} \
    CONFIG.PCW_UIPARAM_ACT_DDR_FREQ_MHZ {533.333374} \
    CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125} \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
  ] $ps7


  # Create instance: rst0, and set properties
  set rst0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst0 ]

  # Create instance: axi_vdma_0, and set properties
  set axi_vdma_0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_vdma:6.3 axi_vdma_0 ]
  set_property -dict [list \
    CONFIG.c_include_mm2s {1} \
    CONFIG.c_include_mm2s_dre {0} \
    CONFIG.c_include_s2mm {1} \
    CONFIG.c_include_s2mm_dre {0} \
    CONFIG.c_m_axi_mm2s_data_width {64} \
    CONFIG.c_m_axi_s2mm_data_width {64} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_mm2s_genlock_mode {0} \
    CONFIG.c_mm2s_genlock_repeat_en {0} \
    CONFIG.c_mm2s_linebuffer_depth {1024} \
    CONFIG.c_num_fstores {3} \
    CONFIG.c_s2mm_genlock_mode {2} \
    CONFIG.c_s2mm_genlock_repeat_en {0} \
    CONFIG.c_s2mm_linebuffer_depth {1024} \
    CONFIG.c_use_mm2s_fsync {0} \
    CONFIG.c_use_s2mm_fsync {2} \
  ] $axi_vdma_0


  # Create instance: ic0, and set properties
  set ic0 [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 ic0 ]
  set_property -dict [list \
    CONFIG.NUM_MI {1} \
    CONFIG.NUM_SI {2} \
  ] $ic0


  # Create instance: pack_s2mm, and set properties
  set block_name axis_y8_to_vdma32_ref
  set block_cell_name pack_s2mm
  if { [catch {set pack_s2mm [create_bd_cell -type module -reference $block_name $block_cell_name] } errmsg] } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2095 -severity "ERROR" "Unable to add referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   } elseif { $pack_s2mm eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2096 -severity "ERROR" "Unable to referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   }
  
  # Create instance: unpack_mm2s, and set properties
  set block_name axis_vdma32_to_y8_ref
  set block_cell_name unpack_mm2s
  if { [catch {set unpack_mm2s [create_bd_cell -type module -reference $block_name $block_cell_name] } errmsg] } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2095 -severity "ERROR" "Unable to add referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   } elseif { $unpack_mm2s eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2096 -severity "ERROR" "Unable to referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   }
  
  # Create instance: cc_mm2s, and set properties
  set cc_mm2s [ create_bd_cell -type ip -vlnv xilinx.com:ip:axis_clock_converter:1.1 cc_mm2s ]
  set_property -dict [list \
    CONFIG.HAS_TLAST {1} \
    CONFIG.TDATA_NUM_BYTES {1} \
    CONFIG.TUSER_WIDTH {1} \
  ] $cc_mm2s


  # Create instance: sub_y_to_rgb, and set properties
  set sub_y_to_rgb [ create_bd_cell -type ip -vlnv xilinx.com:ip:axis_subset_converter:1.1 sub_y_to_rgb ]
  set_property -dict [list \
    CONFIG.M_HAS_TKEEP {0} \
    CONFIG.M_HAS_TLAST {1} \
    CONFIG.M_HAS_TREADY {1} \
    CONFIG.M_HAS_TSTRB {0} \
    CONFIG.M_TDATA_NUM_BYTES {3} \
    CONFIG.M_TDEST_WIDTH {0} \
    CONFIG.M_TID_WIDTH {0} \
    CONFIG.M_TUSER_WIDTH {1} \
    CONFIG.S_HAS_TKEEP {0} \
    CONFIG.S_HAS_TLAST {1} \
    CONFIG.S_HAS_TREADY {1} \
    CONFIG.S_HAS_TSTRB {0} \
    CONFIG.S_TDATA_NUM_BYTES {1} \
    CONFIG.S_TDEST_WIDTH {0} \
    CONFIG.S_TID_WIDTH {0} \
    CONFIG.S_TUSER_WIDTH {1} \
    CONFIG.TDATA_REMAP {tdata[7:0],tdata[7:0],tdata[7:0]} \
    CONFIG.TLAST_REMAP {tlast[0]} \
    CONFIG.TUSER_REMAP {tuser[0:0]} \
  ] $sub_y_to_rgb


  # Create instance: dbg_gpio, and set properties
  set dbg_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 dbg_gpio ]
  set_property -dict [list \
    CONFIG.C_ALL_INPUTS {1} \
    CONFIG.C_ALL_OUTPUTS_2 {1} \
    CONFIG.C_GPIO2_WIDTH {8} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_IS_DUAL {1} \
  ] $dbg_gpio


  # Create instance: sccb_gpio, and set properties
  set sccb_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 sccb_gpio ]
  set_property -dict [list \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_IS_DUAL {1} \
  ] $sccb_gpio


  # Create instance: idelay_gpio, and set properties
  set idelay_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 idelay_gpio ]
  set_property -dict [list \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_DOUT_DEFAULT {0x40000000} \
    CONFIG.C_GPIO2_WIDTH {32} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_IS_DUAL {1} \
  ] $idelay_gpio


  # Create instance: bitslip_gpio, and set properties
  set bitslip_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 bitslip_gpio ]
  set_property -dict [list \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_IS_DUAL {1} \
  ] $bitslip_gpio


  # Create instance: frame_lines_gpio, and set properties
  set frame_lines_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 frame_lines_gpio ]
  set_property -dict [list \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_DOUT_DEFAULT {0xC24501E0} \
    CONFIG.C_GPIO2_WIDTH {32} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_IS_DUAL {1} \
  ] $frame_lines_gpio


  # Create instance: rawcap_gpio, and set properties
  set rawcap_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 rawcap_gpio ]
  set_property -dict [list \
    CONFIG.C_ALL_INPUTS_2 {1} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_GPIO2_WIDTH {32} \
    CONFIG.C_GPIO_WIDTH {32} \
    CONFIG.C_IS_DUAL {1} \
  ] $rawcap_gpio


  # Create instance: core0, and set properties
  set block_name mipi_to_hdmi_vdma_loop_ref
  set block_cell_name core0
  if { [catch {set core0 [create_bd_cell -type module -reference $block_name $block_cell_name] } errmsg] } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2095 -severity "ERROR" "Unable to add referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   } elseif { $core0 eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2096 -severity "ERROR" "Unable to referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   }
    set_property -dict [list \
    CONFIG.CAPTURE_RAW_PAYLOAD {0} \
    CONFIG.OV5640_FORMAT_CTRL_4300 {01101111} \
    CONFIG.OV5640_ISP_CTRL_5000 {"10100111"} \
    CONFIG.OV5640_ISP_CTRL_5001 {"10000011"} \
    CONFIG.OV5640_MIPI_CTRL_4800 {00010100} \
    CONFIG.OV5640_TEST_PATTERN_ENABLE {0} \
    CONFIG.PROBE_IDELAY_TAP {16} \
    CONFIG.PROBE_LANE1_BITSLIP_SWEEP {0} \
    CONFIG.STREAM_PAIRING {0} \
    CONFIG.USE_RGB565_GRAY {0} \
  ] $core0


  # Create instance: ps7_axi_periph, and set properties
  set ps7_axi_periph [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 ps7_axi_periph ]
  set_property CONFIG.NUM_MI {7} $ps7_axi_periph


  # Create interface connections
  connect_bd_intf_net -intf_net axi_vdma_0_M_AXIS_MM2S [get_bd_intf_pins axi_vdma_0/M_AXIS_MM2S] [get_bd_intf_pins unpack_mm2s/S_AXIS]
  connect_bd_intf_net -intf_net axi_vdma_0_M_AXI_MM2S [get_bd_intf_pins axi_vdma_0/M_AXI_MM2S] [get_bd_intf_pins ic0/S01_AXI]
  connect_bd_intf_net -intf_net axi_vdma_0_M_AXI_S2MM [get_bd_intf_pins axi_vdma_0/M_AXI_S2MM] [get_bd_intf_pins ic0/S00_AXI]
  connect_bd_intf_net -intf_net cc_mm2s_M_AXIS [get_bd_intf_pins cc_mm2s/M_AXIS] [get_bd_intf_pins sub_y_to_rgb/S_AXIS]
  connect_bd_intf_net -intf_net core0_m_axis_capture [get_bd_intf_pins core0/m_axis_capture] [get_bd_intf_pins pack_s2mm/S_AXIS]
  connect_bd_intf_net -intf_net ic0_M00_AXI [get_bd_intf_pins ic0/M00_AXI] [get_bd_intf_pins ps7/S_AXI_HP0]
  connect_bd_intf_net -intf_net pack_s2mm_M_AXIS [get_bd_intf_pins pack_s2mm/M_AXIS] [get_bd_intf_pins axi_vdma_0/S_AXIS_S2MM]
  connect_bd_intf_net -intf_net ps7_DDR [get_bd_intf_ports DDR] [get_bd_intf_pins ps7/DDR]
  connect_bd_intf_net -intf_net ps7_FIXED_IO [get_bd_intf_ports FIXED_IO] [get_bd_intf_pins ps7/FIXED_IO]
  connect_bd_intf_net -intf_net ps7_M_AXI_GP0 [get_bd_intf_pins ps7/M_AXI_GP0] [get_bd_intf_pins ps7_axi_periph/S00_AXI]
  connect_bd_intf_net -intf_net ps7_axi_periph_M00_AXI [get_bd_intf_pins ps7_axi_periph/M00_AXI] [get_bd_intf_pins axi_vdma_0/S_AXI_LITE]
  connect_bd_intf_net -intf_net ps7_axi_periph_M01_AXI [get_bd_intf_pins ps7_axi_periph/M01_AXI] [get_bd_intf_pins dbg_gpio/S_AXI]
  connect_bd_intf_net -intf_net ps7_axi_periph_M02_AXI [get_bd_intf_pins ps7_axi_periph/M02_AXI] [get_bd_intf_pins sccb_gpio/S_AXI]
  connect_bd_intf_net -intf_net ps7_axi_periph_M03_AXI [get_bd_intf_pins ps7_axi_periph/M03_AXI] [get_bd_intf_pins idelay_gpio/S_AXI]
  connect_bd_intf_net -intf_net ps7_axi_periph_M04_AXI [get_bd_intf_pins ps7_axi_periph/M04_AXI] [get_bd_intf_pins bitslip_gpio/S_AXI]
  connect_bd_intf_net -intf_net ps7_axi_periph_M05_AXI [get_bd_intf_pins ps7_axi_periph/M05_AXI] [get_bd_intf_pins frame_lines_gpio/S_AXI]
  connect_bd_intf_net -intf_net ps7_axi_periph_M06_AXI [get_bd_intf_pins ps7_axi_periph/M06_AXI] [get_bd_intf_pins rawcap_gpio/S_AXI]
  connect_bd_intf_net -intf_net sub_y_to_rgb_M_AXIS [get_bd_intf_pins sub_y_to_rgb/M_AXIS] [get_bd_intf_pins core0/s_axis_hdmi]
  connect_bd_intf_net -intf_net unpack_mm2s_M_AXIS [get_bd_intf_pins unpack_mm2s/M_AXIS] [get_bd_intf_pins cc_mm2s/S_AXIS]

  # Create port connections
  connect_bd_net -net Net  [get_bd_ports cam_scl] \
  [get_bd_pins core0/cam_scl]
  connect_bd_net -net Net1  [get_bd_ports cam_sda] \
  [get_bd_pins core0/cam_sda]
  connect_bd_net -net Net2  [get_bd_ports hdmi_tx_sda] \
  [get_bd_pins core0/hdmi_tx_sda]
  connect_bd_net -net bitslip_gpio_gpio_io_o  [get_bd_pins bitslip_gpio/gpio_io_o] \
  [get_bd_pins core0/bitslip_runtime_word_in]
  connect_bd_net -net core0_bitslip_runtime_status_out  [get_bd_pins core0/bitslip_runtime_status_out] \
  [get_bd_pins bitslip_gpio/gpio2_io_i]
  connect_bd_net -net core0_cam_clk  [get_bd_pins core0/cam_clk] \
  [get_bd_ports cam_clk]
  connect_bd_net -net core0_cam_gpio  [get_bd_pins core0/cam_gpio] \
  [get_bd_ports cam_gpio]
  connect_bd_net -net core0_capture_debug  [get_bd_pins core0/capture_debug] \
  [get_bd_pins dbg_gpio/gpio_io_i]
  connect_bd_net -net core0_frame_lines_runtime_status_out  [get_bd_pins core0/frame_lines_runtime_status_out] \
  [get_bd_pins frame_lines_gpio/gpio2_io_i]
  connect_bd_net -net core0_hdmi_tx_cec  [get_bd_pins core0/hdmi_tx_cec] \
  [get_bd_ports hdmi_tx_cec]
  connect_bd_net -net core0_hdmi_tx_clk_n  [get_bd_pins core0/hdmi_tx_clk_n] \
  [get_bd_ports hdmi_tx_clk_n]
  connect_bd_net -net core0_hdmi_tx_clk_p  [get_bd_pins core0/hdmi_tx_clk_p] \
  [get_bd_ports hdmi_tx_clk_p]
  connect_bd_net -net core0_hdmi_tx_n  [get_bd_pins core0/hdmi_tx_n] \
  [get_bd_ports hdmi_tx_n]
  connect_bd_net -net core0_hdmi_tx_p  [get_bd_pins core0/hdmi_tx_p] \
  [get_bd_ports hdmi_tx_p]
  connect_bd_net -net core0_hdmi_tx_scl  [get_bd_pins core0/hdmi_tx_scl] \
  [get_bd_ports hdmi_tx_scl]
  connect_bd_net -net core0_idelay_runtime_status_out  [get_bd_pins core0/idelay_runtime_status_out] \
  [get_bd_pins idelay_gpio/gpio2_io_i]
  connect_bd_net -net core0_led  [get_bd_pins core0/led] \
  [get_bd_ports led]
  connect_bd_net -net core0_pix_aresetn_out  [get_bd_pins core0/pix_aresetn_out] \
  [get_bd_pins cc_mm2s/m_axis_aresetn] \
  [get_bd_pins sub_y_to_rgb/aresetn]
  connect_bd_net -net core0_pix_clk_out  [get_bd_pins core0/pix_clk_out] \
  [get_bd_pins cc_mm2s/m_axis_aclk] \
  [get_bd_pins sub_y_to_rgb/aclk]
  connect_bd_net -net core0_rawcap_status_out  [get_bd_pins core0/rawcap_status_out] \
  [get_bd_pins rawcap_gpio/gpio2_io_i]
  connect_bd_net -net core0_sccb_rt_write_status_out  [get_bd_pins core0/sccb_rt_write_status_out] \
  [get_bd_pins sccb_gpio/gpio2_io_i]
  connect_bd_net -net dbg_gpio_gpio2_io_o  [get_bd_pins dbg_gpio/gpio2_io_o] \
  [get_bd_pins core0/debug_page_sel]
  connect_bd_net -net dphy_clk_lp_n_1  [get_bd_ports dphy_clk_lp_n] \
  [get_bd_pins core0/dphy_clk_lp_n]
  connect_bd_net -net dphy_clk_lp_p_1  [get_bd_ports dphy_clk_lp_p] \
  [get_bd_pins core0/dphy_clk_lp_p]
  connect_bd_net -net dphy_data_hs_n_1  [get_bd_ports dphy_data_hs_n] \
  [get_bd_pins core0/dphy_data_hs_n]
  connect_bd_net -net dphy_data_hs_p_1  [get_bd_ports dphy_data_hs_p] \
  [get_bd_pins core0/dphy_data_hs_p]
  connect_bd_net -net dphy_data_lp_n_1  [get_bd_ports dphy_data_lp_n] \
  [get_bd_pins core0/dphy_data_lp_n]
  connect_bd_net -net dphy_data_lp_p_1  [get_bd_ports dphy_data_lp_p] \
  [get_bd_pins core0/dphy_data_lp_p]
  connect_bd_net -net dphy_hs_clock_clk_n_1  [get_bd_ports dphy_hs_clock_clk_n] \
  [get_bd_pins core0/dphy_hs_clock_clk_n]
  connect_bd_net -net dphy_hs_clock_clk_p_1  [get_bd_ports dphy_hs_clock_clk_p] \
  [get_bd_pins core0/dphy_hs_clock_clk_p]
  connect_bd_net -net frame_lines_gpio_gpio_io_o  [get_bd_pins frame_lines_gpio/gpio_io_o] \
  [get_bd_pins core0/frame_lines_runtime_word_in]
  connect_bd_net -net hdmi_tx_hpd_1  [get_bd_ports hdmi_tx_hpd] \
  [get_bd_pins core0/hdmi_tx_hpd]
  connect_bd_net -net idelay_gpio_gpio_io_o  [get_bd_pins idelay_gpio/gpio_io_o] \
  [get_bd_pins core0/idelay_runtime_word_in]
  connect_bd_net -net ps7_FCLK_CLK0  [get_bd_pins ps7/FCLK_CLK0] \
  [get_bd_pins rst0/slowest_sync_clk] \
  [get_bd_pins ps7/S_AXI_HP0_ACLK] \
  [get_bd_pins core0/capture_aclk] \
  [get_bd_pins axi_vdma_0/s_axi_lite_aclk] \
  [get_bd_pins axi_vdma_0/m_axi_s2mm_aclk] \
  [get_bd_pins axi_vdma_0/m_axi_mm2s_aclk] \
  [get_bd_pins axi_vdma_0/s_axis_s2mm_aclk] \
  [get_bd_pins axi_vdma_0/m_axis_mm2s_aclk] \
  [get_bd_pins pack_s2mm/aclk] \
  [get_bd_pins unpack_mm2s/aclk] \
  [get_bd_pins cc_mm2s/s_axis_aclk] \
  [get_bd_pins dbg_gpio/s_axi_aclk] \
  [get_bd_pins sccb_gpio/s_axi_aclk] \
  [get_bd_pins idelay_gpio/s_axi_aclk] \
  [get_bd_pins bitslip_gpio/s_axi_aclk] \
  [get_bd_pins frame_lines_gpio/s_axi_aclk] \
  [get_bd_pins rawcap_gpio/s_axi_aclk] \
  [get_bd_pins ps7_axi_periph/M00_ACLK] \
  [get_bd_pins ps7/M_AXI_GP0_ACLK] \
  [get_bd_pins ps7_axi_periph/S00_ACLK] \
  [get_bd_pins ps7_axi_periph/ACLK] \
  [get_bd_pins ps7_axi_periph/M01_ACLK] \
  [get_bd_pins ps7_axi_periph/M02_ACLK] \
  [get_bd_pins ps7_axi_periph/M03_ACLK] \
  [get_bd_pins ps7_axi_periph/M04_ACLK] \
  [get_bd_pins ps7_axi_periph/M05_ACLK] \
  [get_bd_pins ps7_axi_periph/M06_ACLK] \
  [get_bd_pins ic0/ACLK] \
  [get_bd_pins ic0/S00_ACLK] \
  [get_bd_pins ic0/S01_ACLK] \
  [get_bd_pins ic0/M00_ACLK]
  connect_bd_net -net ps7_FCLK_RESET0_N  [get_bd_pins ps7/FCLK_RESET0_N] \
  [get_bd_pins rst0/ext_reset_in]
  connect_bd_net -net rawcap_gpio_gpio_io_o  [get_bd_pins rawcap_gpio/gpio_io_o] \
  [get_bd_pins core0/rawcap_word_in]
  connect_bd_net -net rst0_peripheral_aresetn  [get_bd_pins rst0/peripheral_aresetn] \
  [get_bd_pins core0/capture_aresetn] \
  [get_bd_pins axi_vdma_0/axi_resetn] \
  [get_bd_pins pack_s2mm/aresetn] \
  [get_bd_pins unpack_mm2s/aresetn] \
  [get_bd_pins cc_mm2s/s_axis_aresetn] \
  [get_bd_pins dbg_gpio/s_axi_aresetn] \
  [get_bd_pins sccb_gpio/s_axi_aresetn] \
  [get_bd_pins idelay_gpio/s_axi_aresetn] \
  [get_bd_pins bitslip_gpio/s_axi_aresetn] \
  [get_bd_pins frame_lines_gpio/s_axi_aresetn] \
  [get_bd_pins rawcap_gpio/s_axi_aresetn] \
  [get_bd_pins ps7_axi_periph/M00_ARESETN] \
  [get_bd_pins ps7_axi_periph/S00_ARESETN] \
  [get_bd_pins ps7_axi_periph/ARESETN] \
  [get_bd_pins ps7_axi_periph/M01_ARESETN] \
  [get_bd_pins ps7_axi_periph/M02_ARESETN] \
  [get_bd_pins ps7_axi_periph/M03_ARESETN] \
  [get_bd_pins ps7_axi_periph/M04_ARESETN] \
  [get_bd_pins ps7_axi_periph/M05_ARESETN] \
  [get_bd_pins ps7_axi_periph/M06_ARESETN] \
  [get_bd_pins ic0/ARESETN] \
  [get_bd_pins ic0/S00_ARESETN] \
  [get_bd_pins ic0/S01_ARESETN] \
  [get_bd_pins ic0/M00_ARESETN]
  connect_bd_net -net sccb_gpio_gpio_io_o  [get_bd_pins sccb_gpio/gpio_io_o] \
  [get_bd_pins core0/sccb_rt_write_word_in]
  connect_bd_net -net sysclk_1  [get_bd_ports sysclk] \
  [get_bd_pins core0/sysclk]

  # Create address segments
  assign_bd_address -offset 0x43000000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs axi_vdma_0/S_AXI_LITE/Reg] -force
  assign_bd_address -offset 0x41230000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs bitslip_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x41200000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs dbg_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x41240000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs frame_lines_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x41220000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs idelay_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x41250000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs rawcap_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x41210000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps7/Data] [get_bd_addr_segs sccb_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x00000000 -range 0x40000000 -target_address_space [get_bd_addr_spaces axi_vdma_0/Data_MM2S] [get_bd_addr_segs ps7/S_AXI_HP0/HP0_DDR_LOWOCM] -force
  assign_bd_address -offset 0x00000000 -range 0x40000000 -target_address_space [get_bd_addr_spaces axi_vdma_0/Data_S2MM] [get_bd_addr_segs ps7/S_AXI_HP0/HP0_DDR_LOWOCM] -force


  # Restore current instance
  current_bd_instance $oldCurInst

  validate_bd_design
  save_bd_design
}
# End of create_root_design()


##################################################################
# MAIN FLOW
##################################################################

create_root_design ""


