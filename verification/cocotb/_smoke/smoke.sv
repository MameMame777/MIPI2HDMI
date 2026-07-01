`timescale 1ns / 1ps

// Proof-of-life DUT for the cocotb + Verilator toolchain bootstrap (Phase 0 gate).
// A single flop with active-low synchronous reset, matching the project reset
// convention. If cocotb can clock it, reset it, drive `d`, and observe `q`, the whole
// native-Windows toolchain (perl verilator wrapper, make shim, VPI link) is working.
module smoke (
    input  logic clk,
    input  logic rst_n,
    input  logic d,
    output logic q
);
    always_ff @(posedge clk) begin
        if (!rst_n) q <= 1'b0;
        else        q <= d;
    end
endmodule
