// SPDX-License-Identifier: MIT
// Portions derived from the Digilent MIPI D-PHY Receiver IP (SyncAsync /
// GlitchFilter / ResetBridge), Copyright (c) 2016 Digilent, MIT License
// (Author: Elod Gyorgy). Full notice: THIRD_PARTY_NOTICES.md.
`timescale 1ns / 1ps
`default_nettype none

// CDC primitives for the D-PHY lane supervisor (Digilent SyncAsync /
// GlitchFilter / ResetBridge equivalents). Instance names must keep the
// sync_* / bridge_* prefixes: the XDC false-path filters match on them.

// 2FF synchronizer with asynchronous reset to RESET_VAL.
module dphy_sync_2ff #(
    parameter logic RESET_VAL = 1'b0
) (
    input  wire  clk,
    input  wire  arst,
    input  wire  d,
    output logic q
);
    (* ASYNC_REG = "TRUE" *) logic sync_meta;
    (* ASYNC_REG = "TRUE" *) logic sync_out;

    always_ff @(posedge clk or posedge arst) begin
        if (arst) begin
            sync_meta <= RESET_VAL;
            sync_out  <= RESET_VAL;
        end else begin
            sync_meta <= d;
            sync_out  <= sync_meta;
        end
    end
    assign q = sync_out;
endmodule

// Output follows input only after STABLE_CYCLES of unchanged input
// (Digilent GlitchFilter; D-PHY kTMinRx = 20 ns).
module dphy_glitch_filter #(
    parameter int STABLE_CYCLES = 4,
    parameter logic RESET_VAL = 1'b0
) (
    input  wire  clk,
    input  wire  rst,
    input  wire  d,
    output logic q
);
    localparam int CW = (STABLE_CYCLES <= 2) ? 1 : $clog2(STABLE_CYCLES);
    logic [CW-1:0] stable_cnt;
    logic          last_d;

    always_ff @(posedge clk) begin
        if (rst) begin
            stable_cnt <= '0;
            last_d     <= RESET_VAL;
            q          <= RESET_VAL;
        end else if (d != last_d) begin
            stable_cnt <= '0;
            last_d     <= d;
        end else if (stable_cnt == CW'(STABLE_CYCLES - 1)) begin
            q <= last_d;
        end else begin
            stable_cnt <= stable_cnt + 1'b1;
        end
    end
endmodule

// Reset bridge: asynchronous assert, synchronous (2FF) release
// (Digilent ResetBridge, kPolarity='1').
module dphy_reset_bridge (
    input  wire  clk,
    input  wire  arst,
    output logic rst_out
);
    (* ASYNC_REG = "TRUE" *) logic bridge_meta;
    (* ASYNC_REG = "TRUE" *) logic bridge_out;

    always_ff @(posedge clk or posedge arst) begin
        if (arst) begin
            bridge_meta <= 1'b1;
            bridge_out  <= 1'b1;
        end else begin
            bridge_meta <= 1'b0;
            bridge_out  <= bridge_meta;
        end
    end
    assign rst_out = bridge_out;
endmodule

`default_nettype wire
