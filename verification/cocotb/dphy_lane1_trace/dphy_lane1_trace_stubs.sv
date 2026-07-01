`timescale 1ns / 1ps
//
// Behavioral Xilinx 7-series primitive stubs for the dphy_lane1_trace port.
//
// Byte-for-byte the same pass-through / assign models the DSim TB
// (verification/tb/tb_dphy_lane1_trace.sv) defined inline, and the same as
// verification/cocotb/lib/verilator_unisim_stubs.sv -- reproduced here (with no
// header text containing the token that Verilator misreads as a metacomment
// pragma) so this block can build under Verilator without editing the shared lib.
//
// These reproduce connectivity, NOT serialization/bitslip timing. The test does not
// depend on real ISERDES deserialization: stimulus is injected by force-writing the
// DUT's internal serdes_byte_sample register, so an ISERDESE2 that drives Q1..Q8=0 is
// exactly what the DSim TB relied on.
//

module IBUFDS #(
    parameter string DIFF_TERM = "FALSE",
    parameter string IBUF_LOW_PWR = "TRUE",
    parameter string IOSTANDARD = "DEFAULT"
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
    parameter string BUFR_DIVIDE = "BYPASS",
    parameter string SIM_DEVICE = "7SERIES"
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
    parameter string CINVCTRL_SEL = "FALSE",
    parameter string DELAY_SRC = "IDATAIN",
    parameter string HIGH_PERFORMANCE_MODE = "TRUE",
    parameter string IDELAY_TYPE = "FIXED",
    parameter int IDELAY_VALUE = 0,
    parameter string PIPE_SEL = "FALSE",
    parameter real REFCLK_FREQUENCY = 200.0,
    parameter string SIGNAL_PATTERN = "DATA"
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
    parameter string DATA_RATE = "DDR",
    parameter int DATA_WIDTH = 8,
    parameter string DYN_CLKDIV_INV_EN = "FALSE",
    parameter string DYN_CLK_INV_EN = "FALSE",
    parameter string INTERFACE_TYPE = "NETWORKING",
    parameter string IOBDELAY = "IFD",
    parameter int NUM_CE = 1,
    parameter string OFB_USED = "FALSE",
    parameter string SERDES_MODE = "MASTER"
) (
    output wire Q1, output wire Q2, output wire Q3, output wire Q4,
    output wire Q5, output wire Q6, output wire Q7, output wire Q8,
    output wire SHIFTOUT1, output wire SHIFTOUT2,
    input  wire BITSLIP, input  wire CE1, input  wire CE2,
    input  wire CLK, input  wire CLKB, input  wire CLKDIV, input  wire CLKDIVP,
    input  wire D, input  wire DDLY, input  wire DYNCLKDIVSEL, input  wire DYNCLKSEL,
    input  wire OCLK, input  wire OCLKB, input  wire OFB, input  wire RST,
    input  wire SHIFTIN1, input  wire SHIFTIN2,
    output wire O
);
    assign {Q8, Q7, Q6, Q5, Q4, Q3, Q2, Q1} = 8'h00;
    assign SHIFTOUT1 = 1'b0;
    assign SHIFTOUT2 = 1'b0;
    assign O = DDLY;
endmodule
