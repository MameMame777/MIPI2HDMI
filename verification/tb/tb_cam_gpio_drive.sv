`timescale 1ns / 1ps
//
// tb_cam_gpio_drive
//
// Verifies that cam_gpio output of mipi_to_hdmi_probe_top follows
// frame_lines_runtime_word_in[25] correctly (after 2FF CDC sync).
//
// Test cases:
//   S0) Default frame_lines_word = 0 → cam_gpio = 0 (chip in RESETB low / reset)
//   S1) Set bit 25 = 1 → cam_gpio = 1 within a few cycles (RESETB high / chip running)
//   S2) Clear bit 25 → cam_gpio = 0 (RESETB asserted → reset)
//   S3) Toggle pulse: 1 → 0 (≥1ms) → 1 → verify cam_gpio follows
//
// This is a focused unit test — we only stimulate the frame_lines GPIO and
// check cam_gpio. Other top-level ports get tied off / left dangling.
//

// === Xilinx primitive stubs (sim-only) ===
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

module BUFG (input wire I, output wire O);
    assign O = I;
endmodule

module IBUFG (input wire I, output wire O);
    assign O = I;
endmodule

module IDELAYCTRL (
    input  wire REFCLK,
    input  wire RST,
    output wire RDY
);
    assign RDY = !RST;
endmodule

module IDELAYE2 #(
    parameter string IDELAY_TYPE = "VAR_LOAD",
    parameter int    IDELAY_VALUE = 0,
    parameter int    DELAY_SRC = 0,
    parameter int    REFCLK_FREQUENCY = 200,
    parameter int    PIPE_SEL = 0,
    parameter int    CINVCTRL_SEL = 0,
    parameter int    HIGH_PERFORMANCE_MODE = 0,
    parameter int    SIGNAL_PATTERN = 0
) (
    input  wire C,
    input  wire CE,
    input  wire INC,
    input  wire LD,
    input  wire CINVCTRL,
    input  wire CNTVALUEIN,
    input  wire DATAIN,
    input  wire IDATAIN,
    input  wire LDPIPEEN,
    input  wire REGRST,
    output wire CNTVALUEOUT,
    output wire DATAOUT
);
    assign DATAOUT = IDATAIN;
    assign CNTVALUEOUT = '0;
endmodule

module ISERDESE2 #(
    parameter string DATA_RATE = "DDR",
    parameter int    DATA_WIDTH = 8,
    parameter string DYN_CLKDIV_INV_EN = "FALSE",
    parameter string DYN_CLK_INV_EN = "FALSE",
    parameter string INTERFACE_TYPE = "NETWORKING",
    parameter string IOBDELAY = "IFD",
    parameter int    NUM_CE = 1,
    parameter string OFB_USED = "FALSE",
    parameter string SERDES_MODE = "MASTER"
) (
    output wire O,
    output wire Q1, Q2, Q3, Q4, Q5, Q6, Q7, Q8,
    output wire SHIFTOUT1, SHIFTOUT2,
    input  wire BITSLIP,
    input  wire CE1, CE2,
    input  wire CLK, CLKB,
    input  wire CLKDIV, CLKDIVP,
    input  wire D,
    input  wire DDLY,
    input  wire DYNCLKDIVSEL, DYNCLKSEL,
    input  wire OCLK, OCLKB,
    input  wire OFB,
    input  wire RST,
    input  wire SHIFTIN1, SHIFTIN2
);
    assign {Q8, Q7, Q6, Q5, Q4, Q3, Q2, Q1} = 8'h0;
    assign O = 0;
    assign SHIFTOUT1 = 0; assign SHIFTOUT2 = 0;
endmodule

module PLLE2_ADV #(
    parameter int CLKFBOUT_MULT = 8,
    parameter real CLKFBOUT_PHASE = 0.0,
    parameter int CLKIN1_PERIOD = 8,
    parameter int CLKOUT0_DIVIDE = 8,
    parameter real CLKOUT0_PHASE = 0.0,
    parameter int CLKOUT1_DIVIDE = 8,
    parameter int CLKOUT2_DIVIDE = 8,
    parameter int CLKOUT3_DIVIDE = 8,
    parameter int CLKOUT4_DIVIDE = 8,
    parameter int CLKOUT5_DIVIDE = 8,
    parameter int COMPENSATION = 0,
    parameter int DIVCLK_DIVIDE = 1,
    parameter real REF_JITTER1 = 0.010
) (
    output wire CLKOUT0, CLKOUT1, CLKOUT2, CLKOUT3, CLKOUT4, CLKOUT5,
    output wire CLKFBOUT,
    output wire LOCKED,
    input  wire CLKIN1, CLKIN2, CLKINSEL,
    input  wire CLKFBIN,
    input  wire PWRDWN, RST,
    input  wire [6:0] DADDR,
    input  wire DCLK, DEN, DWE,
    input  wire [15:0] DI,
    output wire [15:0] DO,
    output wire DRDY
);
    assign CLKOUT0 = CLKIN1; assign CLKOUT1 = CLKIN1; assign CLKOUT2 = CLKIN1;
    assign CLKOUT3 = CLKIN1; assign CLKOUT4 = CLKIN1; assign CLKOUT5 = CLKIN1;
    assign CLKFBOUT = CLKIN1;
    assign LOCKED = !RST;
    assign DO = '0; assign DRDY = 0;
endmodule

module IOBUF (
    inout wire IO,
    output wire O,
    input  wire I,
    input  wire T
);
    assign IO = T ? 1'bz : I;
    assign O  = IO;
endmodule

module IBUF (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule

module OBUF (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule

module OBUFT (
    input  wire I,
    input  wire T,
    output wire O
);
    assign O = T ? 1'bz : I;
endmodule

// === Testbench ===
module tb_cam_gpio_drive;
    logic        sysclk = 0;
    logic        ref_clk_125 = 0;
    logic        rst_n_ext = 0;
    logic        rst_n;
    logic        hs_clk_p, hs_clk_n;
    logic [1:0]  hs_data_p, hs_data_n;

    logic [31:0] frame_lines_word_in;
    logic [31:0] frame_lines_status_out;

    logic [31:0] dbg_gpio_addr_in;
    logic [31:0] dbg_gpio_data_out;
    logic [31:0] sccb_gpio_word_in;
    logic [31:0] sccb_gpio_status_out;
    logic [31:0] bitslip_word_in;
    logic [31:0] bitslip_status_out;
    logic [31:0] idelay_word_in;
    logic [31:0] idelay_status_out;
    logic [31:0] rawcap_word_in;
    logic [31:0] rawcap_status_out;

    wire         cam_clk;
    wire         cam_gpio;
    wire         cam_scl;
    wire         cam_sda;

    // Clocks: sysclk 100 MHz, ref_clk_125 125 MHz
    always #5 sysclk = ~sysclk;
    always #4 ref_clk_125 = ~ref_clk_125;

    // Tie hs lanes to LP-11 (both high)
    assign hs_clk_p = 1'b1; assign hs_clk_n = 1'b0;
    assign hs_data_p = 2'b11; assign hs_data_n = 2'b00;

    // rst_n delayed
    assign rst_n = rst_n_ext;

    mipi_to_hdmi_probe_top dut (
        .ref_clk_125(ref_clk_125),
        .sysclk(sysclk),
        .rst_n(rst_n),
        .hs_clk_p(hs_clk_p), .hs_clk_n(hs_clk_n),
        .hs_data_p(hs_data_p), .hs_data_n(hs_data_n),
        .frame_lines_runtime_word_in(frame_lines_word_in),
        .frame_lines_runtime_status_out(frame_lines_status_out),
        .dbg_gpio_addr_in(dbg_gpio_addr_in),
        .dbg_gpio_data_out(dbg_gpio_data_out),
        .sccb_gpio_word_in(sccb_gpio_word_in),
        .sccb_gpio_status_out(sccb_gpio_status_out),
        .bitslip_runtime_word_in(bitslip_word_in),
        .bitslip_runtime_status_out(bitslip_status_out),
        .idelay_runtime_word_in(idelay_word_in),
        .idelay_runtime_status_out(idelay_status_out),
        .rawcap_word_in(rawcap_word_in),
        .rawcap_status_out(rawcap_status_out),
        .cam_clk(cam_clk),
        .cam_gpio(cam_gpio),
        .cam_scl(cam_scl),
        .cam_sda(cam_sda)
    );

    int errors = 0;
    task expect_eq(input string label, input int got, input int want);
        if (got !== want) begin
            $display("[FAIL] %s: got=%0d want=%0d", label, got, want);
            errors++;
        end else begin
            $display("[ OK ] %s: %0d", label, got);
        end
    endtask

    initial begin
        frame_lines_word_in = 32'h0;
        dbg_gpio_addr_in = 0;
        sccb_gpio_word_in = 0;
        bitslip_word_in = 0;
        idelay_word_in = 0;
        rawcap_word_in = 0;

        // Reset
        rst_n_ext = 0;
        repeat (10) @(posedge sysclk);
        rst_n_ext = 1;
        repeat (20) @(posedge sysclk);

        // S0: default frame_lines_word = 0 → cam_gpio should be 0 (chip in RESETB low)
        $display("--- S0: default (frame_lines_word=0) ---");
        expect_eq("S0: cam_gpio = 0", cam_gpio, 0);

        // S1: set bit 25 = 1 → cam_gpio should go high (after CDC ~2 cycles)
        $display("--- S1: set bit 25 = 1 (RESETB release) ---");
        @(negedge sysclk);
        frame_lines_word_in = 32'h02000000;  // bit 25 = 1
        repeat (10) @(posedge sysclk);  // wait for CDC sync
        expect_eq("S1: cam_gpio = 1", cam_gpio, 1);

        // S2: clear bit 25 → cam_gpio should go low
        $display("--- S2: clear bit 25 (RESETB assert) ---");
        @(negedge sysclk);
        frame_lines_word_in = 32'h0;
        repeat (10) @(posedge sysclk);
        expect_eq("S2: cam_gpio = 0", cam_gpio, 0);

        // S3: HW reset pulse — release → assert → release
        $display("--- S3: HW reset pulse ---");
        @(negedge sysclk);
        frame_lines_word_in = 32'h02000000;  // bit 25 = 1 (RESETB high)
        repeat (10) @(posedge sysclk);
        expect_eq("S3.a: cam_gpio = 1 (chip released)", cam_gpio, 1);
        @(negedge sysclk);
        frame_lines_word_in = 32'h0;  // bit 25 = 0 (RESETB low / reset)
        repeat (10) @(posedge sysclk);
        expect_eq("S3.b: cam_gpio = 0 (chip in reset)", cam_gpio, 0);
        @(negedge sysclk);
        frame_lines_word_in = 32'h02000000;  // bit 25 = 1 again
        repeat (10) @(posedge sysclk);
        expect_eq("S3.c: cam_gpio = 1 (chip released again)", cam_gpio, 1);

        // S4: bit 25 = 1 with other bits set (apply, value, etc.) — cam_gpio unchanged
        $display("--- S4: bit 25 with other bits ---");
        @(negedge sysclk);
        frame_lines_word_in = 32'h024401E0;  // bit 25 + bit 24 (apply) + bit 22-17 + value 480
        repeat (10) @(posedge sysclk);
        expect_eq("S4: cam_gpio = 1 with apply pulse", cam_gpio, 1);

        $display("\n=========================================");
        if (errors == 0) $display("[PASS] All tests passed");
        else             $display("[FAIL] %0d errors", errors);
        $display("=========================================");
        $finish;
    end

    initial begin
        #1_000_000;
        $display("[FAIL] Timeout");
        $finish;
    end
endmodule
