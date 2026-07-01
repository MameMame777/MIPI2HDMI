`timescale 1ns / 1ps
// Clean Verilator-compatible behavioral stubs for the Xilinx 7-series primitives
// used by dphy_hs_byte_probe. Local copy for this block because the shared
// lib/verilator_unisim_stubs.sv header comment trips Verilator's BADVLTPRAGMA
// check when the DUT is built with --public-flat-rw (needed here because the DSim
// TB drives DUT-internal signals hierarchically -- serdes_byte_sample, trace_*).
// Connectivity-only stubs (no serialization/bitslip); the TB never relies on the
// ISERDES gearbox output -- it forces serdes_byte_sample directly.

module IBUFDS #(
    parameter DIFF_TERM = "FALSE",
    parameter IBUF_LOW_PWR = "TRUE",
    parameter IOSTANDARD = "DEFAULT"
) (
    input  wire I,
    input  wire IB,
    output wire O
);
    assign O = I;
endmodule

module BUFIO (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule

module BUFR #(
    parameter BUFR_DIVIDE = "BYPASS",
    parameter SIM_DEVICE = "7SERIES"
) (
    input  wire I,
    input  wire CE,
    input  wire CLR,
    output wire O
);
    assign O = CLR ? 1'b0 : (CE ? I : 1'b0);
endmodule

module IDELAYCTRL (
    input  wire REFCLK,
    input  wire RST,
    output wire RDY
);
    assign RDY = !RST;
endmodule

module IDELAYE2 #(
    parameter CINVCTRL_SEL = "FALSE",
    parameter DELAY_SRC = "IDATAIN",
    parameter HIGH_PERFORMANCE_MODE = "TRUE",
    parameter IDELAY_TYPE = "FIXED",
    parameter IDELAY_VALUE = 0,
    parameter PIPE_SEL = "FALSE",
    parameter real REFCLK_FREQUENCY = 200.0,
    parameter SIGNAL_PATTERN = "DATA"
) (
    input  wire C,
    input  wire REGRST,
    input  wire LD,
    input  wire CE,
    input  wire INC,
    input  wire LDPIPEEN,
    input  wire CINVCTRL,
    input  wire [4:0] CNTVALUEIN,
    input  wire IDATAIN,
    input  wire DATAIN,
    output wire DATAOUT,
    output wire [4:0] CNTVALUEOUT
);
    assign DATAOUT = IDATAIN;
    assign CNTVALUEOUT = CNTVALUEIN;
endmodule

module ISERDESE2 #(
    parameter DATA_RATE = "DDR",
    parameter DATA_WIDTH = 8,
    parameter DYN_CLKDIV_INV_EN = "FALSE",
    parameter DYN_CLK_INV_EN = "FALSE",
    parameter INTERFACE_TYPE = "NETWORKING",
    parameter IOBDELAY = "IFD",
    parameter NUM_CE = 1,
    parameter OFB_USED = "FALSE",
    parameter SERDES_MODE = "MASTER"
) (
    output wire Q1,
    output wire Q2,
    output wire Q3,
    output wire Q4,
    output wire Q5,
    output wire Q6,
    output wire Q7,
    output wire Q8,
    output wire SHIFTOUT1,
    output wire SHIFTOUT2,
    input  wire BITSLIP,
    input  wire CE1,
    input  wire CE2,
    input  wire CLK,
    input  wire CLKB,
    input  wire CLKDIV,
    input  wire CLKDIVP,
    input  wire D,
    input  wire DDLY,
    input  wire DYNCLKDIVSEL,
    input  wire DYNCLKSEL,
    input  wire OCLK,
    input  wire OCLKB,
    input  wire OFB,
    input  wire RST,
    input  wire SHIFTIN1,
    input  wire SHIFTIN2,
    output wire O
);
    assign {Q8, Q7, Q6, Q5, Q4, Q3, Q2, Q1} = 8'h00;
    assign SHIFTOUT1 = 1'b0;
    assign SHIFTOUT2 = 1'b0;
    assign O = DDLY;
endmodule
