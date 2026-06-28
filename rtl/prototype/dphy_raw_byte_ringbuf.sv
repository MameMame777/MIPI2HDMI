`timescale 1ns / 1ps
`default_nettype none

// Raw byte ring buffer for chip side observation.
// Captures serdes_byte_sample (post-ISERDES + post-BITSLIP, pre-decode) to BRAM.
// PYNQ arms via GPIO, reads 512 entries via address-mapped read.
//
// Word layout (32 bits per entry):
//   [31:24] reserved (zero)
//   [23]    sync_header_valid at this byte_clk
//   [22]    sot_l1 (lane1_byte_in == 0xB8 this cycle)
//   [21]    sot_l0 (lane0_byte_in == 0xB8 this cycle)
//   [20]    first_entry marker (1 only on the word at wp=0 of this capture)
//   [19:16] reserved
//   [15:8]  lane1_byte_in
//   [7:0]   lane0_byte_in
//
// Trigger modes (trigger_mode_in selects):
//   0: free-run — on arm rising edge, capture starts immediately (legacy)
//   1: SoT-triggered — on arm, wait for sync_trigger_byte; capture starts on its rising edge
//
// PYNQ read protocol (32-bit data, dual half via rd_addr[9]):
//   rd_addr[8:0] = entry index 0..511
//   rd_addr[9]   = 0 → low 16 bits, 1 → high 16 bits
//
// BRAM is inferred dual-port: write port = byte_clk, read port = rd_clk
module dphy_raw_byte_ringbuf #(
    parameter int DEPTH = 512
) (
    // Capture side (byte_clk domain)
    input  wire        byte_clk,
    input  wire        rst_n_byte,
    input  wire [7:0]  lane0_byte_in,
    input  wire [7:0]  lane1_byte_in,
    input  wire        sync_header_valid_byte,  // byte_clk-domain sync_header_valid
    input  wire        sync_trigger_byte,       // byte_clk-domain trigger (SoT/sync edge); rising = trigger
    input  wire        arm_trigger_byte,        // pre-synced rising-edge pulse trigger
    input  wire        trigger_mode_byte,       // 0=free-run, 1=wait for sync_trigger after arm

    // Read side (rd_clk domain — typically sysclk)
    input  wire        rd_clk,
    input  wire [9:0]  rd_addr,                 // [8:0]=index, [9]=hi/lo select
    output logic [15:0] rd_data,

    // Status (synced to rd_clk)
    output logic [9:0]  last_write_addr_sync,
    output logic        full_sync,
    output logic        armed_sync,
    output logic        waiting_sync             // armed but not yet capturing (trigger_mode=1, no trigger yet)
);

    (* ram_style = "block" *) logic [31:0] mem [0:DEPTH-1];

    logic [9:0] wp;
    logic       armed;
    logic       full;
    logic       waiting;       // armed && trigger_mode==1 && no trigger seen yet
    logic       arm_trigger_d;
    logic       sync_trigger_d;
    logic [31:0] write_word;

    logic sot_l0;
    logic sot_l1;
    assign sot_l0 = (lane0_byte_in == 8'hB8);
    assign sot_l1 = (lane1_byte_in == 8'hB8);

    logic first_entry_marker;
    assign first_entry_marker = (wp == 10'd0);

    always_comb begin
        write_word          = 32'h0;
        write_word[7:0]     = lane0_byte_in;
        write_word[15:8]    = lane1_byte_in;
        write_word[20]      = first_entry_marker;
        write_word[21]      = sot_l0;
        write_word[22]      = sot_l1;
        write_word[23]      = sync_header_valid_byte;
    end

    always_ff @(posedge byte_clk or negedge rst_n_byte) begin
        if (!rst_n_byte) begin
            wp <= 10'd0;
            armed <= 1'b0;
            full <= 1'b0;
            waiting <= 1'b0;
            arm_trigger_d <= 1'b0;
            sync_trigger_d <= 1'b0;
        end else begin
            arm_trigger_d  <= arm_trigger_byte;
            sync_trigger_d <= sync_trigger_byte;

            // rising-edge of arm: reset, set armed; if trigger_mode==1 enter waiting
            if (arm_trigger_byte && !arm_trigger_d) begin
                wp      <= 10'd0;
                full    <= 1'b0;
                armed   <= 1'b1;
                waiting <= trigger_mode_byte;
            end

            // exit waiting on sync_trigger rising edge
            if (armed && waiting && sync_trigger_byte && !sync_trigger_d) begin
                waiting <= 1'b0;
            end

            // capture while armed, not waiting, not full
            if (armed && !waiting && !full) begin
                mem[wp[8:0]] <= write_word;
                if (wp == DEPTH[9:0] - 10'd1) begin
                    full  <= 1'b1;
                    armed <= 1'b0;
                end else begin
                    wp <= wp + 10'd1;
                end
            end
        end
    end

    // Read side: 1-cycle latency BRAM read with hi/lo selector
    logic [31:0] mem_rd_q;
    always_ff @(posedge rd_clk) begin
        mem_rd_q <= mem[rd_addr[8:0]];
        rd_data  <= rd_addr[9] ? mem_rd_q[31:16] : mem_rd_q[15:0];
    end

    // 2FF synchronizer for status flags (byte_clk → rd_clk)
    (* ASYNC_REG = "TRUE" *) logic        full_meta, full_sync_r;
    (* ASYNC_REG = "TRUE" *) logic        armed_meta, armed_sync_r;
    (* ASYNC_REG = "TRUE" *) logic        waiting_meta, waiting_sync_r;
    (* ASYNC_REG = "TRUE" *) logic [9:0]  wp_meta, wp_sync_r;

    always_ff @(posedge rd_clk) begin
        full_meta     <= full;
        full_sync_r   <= full_meta;
        armed_meta    <= armed;
        armed_sync_r  <= armed_meta;
        waiting_meta  <= waiting;
        waiting_sync_r<= waiting_meta;
        wp_meta       <= wp;
        wp_sync_r     <= wp_meta;
    end

    assign full_sync            = full_sync_r;
    assign armed_sync           = armed_sync_r;
    assign waiting_sync         = waiting_sync_r;
    assign last_write_addr_sync = wp_sync_r;

endmodule

`default_nettype wire
