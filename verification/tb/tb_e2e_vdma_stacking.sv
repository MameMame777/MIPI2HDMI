`timescale 1ns / 1ps
`default_nettype none
`ifndef NORMALIZE_EN
`define NORMALIZE_EN 1
`endif
`ifndef SPURIOUS_FS
`define SPURIOUS_FS 0
`endif

// E2E frame-assembly / VDMA-stacking test (2026-06-04).
//
// Question: does the FPGA assembly chain (csi2_frame_state -> yuv422_gray_unpack
// -> axis_video_bridge -> VDMA write) replicate ("stack") a frame in the
// VSIZE-line DDR buffer, or reproduce it cleanly? Lighting/flicker cannot be a
// factor here -- the stimulus is a deterministic, vertically-UNIQUE frame
// (line k carries Y = BASE + k, so any vertical repetition is visible).
//
// The bridge AXIS output (tuser=SOF, tlast=EOL) is consumed by a behavioural
// AXI-VDMA S2MM write model: on SOF reset line_ptr=0; each pixel writes
// buf[line_ptr][col]; on EOL advance line_ptr (capped at VSIZE). We then check
// whether buf[k] == the unique value of source line k for the first FRAME_LINES
// rows, and whether the SOF count per frame is exactly 1.
//
// Config mirrors hardware (FE_DELIMITS=1, GUARD=1, FS_MIN floor) at small scale.

module tb_e2e_vdma_stacking;
    localparam int LINE_PIXELS  = 8;          // Y pixels per line
    localparam int LINE_BYTES   = LINE_PIXELS * 2;  // YUV422: 2 bytes / Y
    // MISMATCH case mirroring hardware: chip frame (FRAME_LINES) shorter than the
    // VDMA frame height (VSIZE) -> a free-running VDMA tiles ~VSIZE/FRAME_LINES
    // copies into one buffer. 11 lines into a 44-line buffer => ~4 tiles.
    localparam int FRAME_LINES  = 11;         // source frame height (chip)
    localparam int VSIZE        = 44;         // VDMA frame height
    localparam int N_FRAMES     = 8;          // drive several frames back-to-back
    localparam logic [5:0] DT_FS = 6'h00, DT_FE = 6'h01, DT_LS = 6'h02,
                           DT_LE = 6'h03, DT_YUV = 6'h1e;

    logic core_clk = 0, aclk = 0, core_aresetn, aresetn, cfg_use_lsle;
    always #5  core_clk = ~core_clk;
    always #7  aclk     = ~aclk;

    // frame_state I/O
    logic [7:0] in_pkt_di; logic [15:0] in_pkt_wc;
    logic in_pkt_is_short, in_pkt_is_long, in_pkt_start, in_pkt_end, in_pkt_err;
    logic [7:0] in_payload_data; logic in_payload_valid, in_payload_first, in_payload_last;
    logic fs_sof, fs_eof, fs_sol, fs_eol; logic [15:0] fs_line_idx;
    logic [7:0] fs_pd; logic fs_pv, fs_pf, fs_pl, fs_ferr;
    logic [31:0] fs_fcnt, fs_lcnt; logic [15:0] fs_lastlines, fs_syncerr;

    // unpack I/O
    logic [23:0] up_pixel; logic up_v, up_sof, up_eol, up_eof, up_err; logic [15:0] up_ppl;

    // bridge I/O (aclk side)
    logic [7:0] br_tdata; logic br_tvalid, br_tready, br_tlast; logic [0:0] br_tuser;
    logic [15:0] br_ovf, br_bp;

    csi2_frame_state #(
        .MAX_LINES(64), .GUARD_FRAME_LINES(1'b1), .EXPECTED_FRAME_LINES(FRAME_LINES),
        .EXPECTED_LINE_WC(16'(LINE_BYTES)), .FS_MIN_LINES(4), .FE_DELIMITS(1'b1)
    ) u_fs (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .cfg_use_lsle(cfg_use_lsle),
        .cfg_expected_frame_lines(16'd0),
        .in_pkt_di(in_pkt_di), .in_pkt_wc(in_pkt_wc), .in_pkt_is_short(in_pkt_is_short),
        .in_pkt_is_long(in_pkt_is_long), .in_pkt_start(in_pkt_start), .in_pkt_end(in_pkt_end),
        .in_pkt_err(in_pkt_err), .in_payload_data(in_payload_data), .in_payload_valid(in_payload_valid),
        .in_payload_first(in_payload_first), .in_payload_last(in_payload_last),
        .out_sof(fs_sof), .out_eof(fs_eof), .out_sol(fs_sol), .out_eol(fs_eol),
        .out_line_idx(fs_line_idx), .out_payload_data(fs_pd), .out_payload_valid(fs_pv),
        .out_payload_first(fs_pf), .out_payload_last(fs_pl), .out_frame_err(fs_ferr),
        .sts_frame_count(fs_fcnt), .sts_line_count(fs_lcnt),
        .sts_last_frame_lines(fs_lastlines), .sts_frame_sync_err_cnt(fs_syncerr)
    );

    yuv422_gray_unpack #(.LINE_PIXELS(0)) u_up (
        .core_clk(core_clk), .core_aresetn(core_aresetn),
        .in_sof(fs_sof), .in_eof(fs_eof), .in_eol(fs_eol),
        .in_payload_data(fs_pd), .in_payload_valid(fs_pv),
        .in_payload_first(fs_pf), .in_payload_last(fs_pl), .in_frame_err(fs_ferr),
        .out_pixel(up_pixel), .out_pixel_valid(up_v), .out_pixel_sof(up_sof),
        .out_pixel_eol(up_eol), .out_pixel_eof(up_eof), .out_pixel_err(up_err),
        .sts_pixel_per_line(up_ppl)
    );

    // frame normalizer: pin every frame to exactly VSIZE x LINE_PIXELS
    logic [7:0] nm_data; logic nm_v, nm_sof, nm_eol, nm_eof, nm_err;
    video_frame_normalizer #(.OUT_LINES(VSIZE), .OUT_PIXELS(LINE_PIXELS),
                             .FILL(8'h00), .NORMALIZE(`NORMALIZE_EN)) u_norm (
        .clk(core_clk), .aresetn(core_aresetn),
        .in_data(up_pixel[7:0]), .in_valid(up_v), .in_sof(up_sof),
        .in_eol(up_eol), .in_eof(up_eof), .in_err(up_err),
        .out_data(nm_data), .out_valid(nm_v), .out_sof(nm_sof),
        .out_eol(nm_eol), .out_eof(nm_eof), .out_err(nm_err)
    );

    axis_video_bridge #(.TDATA_WIDTH(8), .TUSER_WIDTH(1), .FIFO_DEPTH(4096),
                        .AXIS_TUSER_ERR_DEBUG(1'b0)) u_br (
        .core_clk(core_clk), .core_aresetn(core_aresetn), .aclk(aclk), .aresetn(aresetn),
        .in_pixel(nm_data), .in_pixel_valid(nm_v), .in_pixel_sof(nm_sof),
        .in_pixel_eol(nm_eol), .in_pixel_eof(nm_eof), .in_pixel_err(nm_err),
        .m_axis_tdata(br_tdata), .m_axis_tvalid(br_tvalid), .m_axis_tready(br_tready),
        .m_axis_tlast(br_tlast), .m_axis_tuser(br_tuser),
        .sts_fifo_overflow_cnt(br_ovf), .sts_back_pressure_cnt(br_bp)
    );

    assign br_tready = 1'b1;   // VDMA always ready (low pixel rate)

    // instrumentation: count frame markers at unpack out and normalizer out
    int up_sof_n, up_eof_n, nm_sof_n, nm_eof_n;
    always_ff @(posedge core_clk or negedge core_aresetn) begin
        if (!core_aresetn) begin up_sof_n<=0; up_eof_n<=0; nm_sof_n<=0; nm_eof_n<=0; end
        else begin
            if (up_v && up_sof) up_sof_n<=up_sof_n+1;
            if (up_eof)         up_eof_n<=up_eof_n+1;
            if (nm_v && nm_sof) nm_sof_n<=nm_sof_n+1;
            if (nm_v && nm_eof) nm_eof_n<=nm_eof_n+1;
        end
    end

    // ---- behavioural AXI-VDMA S2MM write model ----
    logic [7:0] vbuf [0:VSIZE-1][0:LINE_PIXELS-1];
    int line_ptr, col_ptr, sof_pulses, eol_pulses, total_px;
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            line_ptr <= 0; col_ptr <= 0; sof_pulses <= 0; eol_pulses <= 0; total_px <= 0;
        end else if (br_tvalid && br_tready) begin
            if (br_tuser[0]) begin       // SOF: VDMA frame sync -> top of buffer
                line_ptr <= 0; col_ptr <= 0; sof_pulses <= sof_pulses + 1;
            end
            if (line_ptr < VSIZE && col_ptr < LINE_PIXELS)
                vbuf[line_ptr][col_ptr] <= br_tdata;
            total_px <= total_px + 1;
            if (br_tlast) begin          // EOL: next line
                eol_pulses <= eol_pulses + 1;
                col_ptr <= 0;
                if (line_ptr < VSIZE-1) line_ptr <= line_ptr + 1;
            end else begin
                if (col_ptr < LINE_PIXELS-1) col_ptr <= col_ptr + 1;
            end
        end
    end

    // ---- FREE-RUN VDMA write model (no SOF resync; counts VSIZE lines, wraps).
    //      This is what an un-genlocked AXI-VDMA S2MM does: it writes one line
    //      per TLAST into the buffer and wraps at VSIZE regardless of SOF, so a
    //      source frame shorter than VSIZE leaves several frames tiled. ----
    logic [7:0] fbuf [0:VSIZE-1][0:LINE_PIXELS-1];
    int fline, fcol;
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            fline <= 0; fcol <= 0;
        end else if (br_tvalid && br_tready) begin
            if (fline < VSIZE && fcol < LINE_PIXELS) fbuf[fline][fcol] <= br_tdata;
            if (br_tlast) begin
                fcol <= 0;
                fline <= (fline == VSIZE-1) ? 0 : fline + 1;   // wrap, ignore SOF
            end else if (fcol < LINE_PIXELS-1) begin
                fcol <= fcol + 1;
            end
        end
    end

    // ---- stimulus ----
    task automatic drv_short(input logic [5:0] dt);
        @(posedge core_clk); in_pkt_di<={2'b00,dt}; in_pkt_wc<=0;
        in_pkt_is_short<=1; in_pkt_is_long<=0; in_pkt_start<=1; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_start<=0; in_pkt_end<=0; in_pkt_is_short<=0;
    endtask
    // one YUV422 line: long packet, bytes = U,Y,V,Y,... ; all Y in this line = yval
    task automatic drv_line(input logic [7:0] yval);
        drv_short(DT_LS);
        @(posedge core_clk); in_pkt_di<=8'h1e; in_pkt_wc<=16'(LINE_BYTES);
        in_pkt_is_short<=0; in_pkt_is_long<=1; in_pkt_start<=1;
        @(posedge core_clk); in_pkt_start<=0;
        for (int b=0;b<LINE_BYTES;b++) begin
            in_payload_data <= (b[0]==1'b1) ? yval : 8'h80;  // odd byte = Y, even = chroma 0x80
            in_payload_valid<=1; in_payload_first<=(b==0); in_payload_last<=(b==LINE_BYTES-1);
            @(posedge core_clk);
        end
        in_payload_valid<=0; in_payload_first<=0; in_payload_last<=0; in_pkt_end<=1;
        @(posedge core_clk); in_pkt_end<=0; in_pkt_is_long<=0;
        drv_short(DT_LE);
    endtask
    task automatic drv_frame(input int nlines, input logic [7:0] base);
        drv_short(DT_FS);
        for (int k=0;k<nlines;k++) begin
            // SPURIOUS_FS: inject a false in-frame FS mid-frame (models a
            // payload-0x00 black run decoding as a 00 00 00 00 FS short packet
            // on the open-loop aligner). With FE_DELIMITS this MUST be ignored
            // (no extra SOF, no VDMA re-tile).
            if (`SPURIOUS_FS && k == nlines/2) drv_short(DT_FS);
            drv_line(8'(base + k));
        end
        drv_short(DT_FE);
    endtask

    initial begin
        core_aresetn=0; aresetn=0; cfg_use_lsle=1;
        in_pkt_di=0; in_pkt_wc=0; in_pkt_is_short=0; in_pkt_is_long=0;
        in_pkt_start=0; in_pkt_end=0; in_pkt_err=0;
        in_payload_data=0; in_payload_valid=0; in_payload_first=0; in_payload_last=0;
        repeat(10) @(posedge core_clk); core_aresetn=1; aresetn=1; repeat(4) @(posedge core_clk);

        // drive N_FRAMES frames, each FRAME_LINES lines, line k -> Y=0x10+k
        // (identical ramp each frame so a tiled buffer shows repeated ramps).
        // inter-frame gap must exceed the normalizer's tail-pad burst
        // ((VSIZE-FRAME_LINES)*LINE_PIXELS pixels) -- on hardware this is the
        // ~0.4 s blanking interval; in sim we make it generous.
        for (int f=0; f<N_FRAMES; f++) begin
            drv_frame(FRAME_LINES, 8'h10);
            repeat(1600) @(posedge core_clk);
        end
        repeat(4000) @(posedge core_clk);

        $display("frame_state: frames=%0d last_lines=%0d sync_err=%0d  (each source frame=%0d lines, VSIZE=%0d)",
                 fs_fcnt, fs_lastlines, fs_syncerr, FRAME_LINES, VSIZE);
        $display("bridge AXIS: total sof_pulses=%0d eol_pulses=%0d  (=> %0.1f EOL per SOF; clean RTL framing => %0d)",
                 sof_pulses, eol_pulses, (sof_pulses>0)?real'(eol_pulses)/real'(sof_pulses):0.0, FRAME_LINES);
        $display("markers: unpack_out sof=%0d eof=%0d | normalizer_out sof=%0d eof=%0d (frame_state frames=%0d)",
                 up_sof_n, up_eof_n, nm_sof_n, nm_eof_n, fs_fcnt);

        // Count occurrences of the frame-top marker (line-0 value 0x10) in
        // each VDMA buffer = number of tiled frame copies.
        begin
            automatic int gl_tiles = 0, fr_tiles = 0;
            for (int k=0;k<VSIZE;k++) if (vbuf[k][1] === 8'h10) gl_tiles++;
            for (int k=0;k<VSIZE;k++) if (fbuf[k][1] === 8'h10) fr_tiles++;
            $display("NORMALIZE_EN=%b  source frame=%0d lines, VSIZE=%0d", `NORMALIZE_EN, FRAME_LINES, VSIZE);
            $display("GENLOCK  VDMA (resets on SOF): frame-top copies in buffer = %0d", gl_tiles);
            $display("FREE-RUN VDMA (VSIZE wrap)   : frame-top copies in buffer = %0d", fr_tiles);
            $display("");
            $display("DIAGNOSIS:");
            if (sof_pulses == fs_fcnt)
                $display(" - RTL framing CLEAN: exactly 1 SOF per frame (%0d SOF / %0d frames).", sof_pulses, fs_fcnt);
            if (fr_tiles <= 1)
                $display(" => FREE-RUN VDMA shows %0d frame copy => NO stacking (normalizer pins frame==VSIZE).", fr_tiles);
            else
                $display(" => FREE-RUN VDMA TILES %0d copies => stacking (frame %0d != VSIZE %0d).", fr_tiles, FRAME_LINES, VSIZE);
        end
        $finish;
    end
    initial begin #5ms; $fatal(1,"timeout"); end
endmodule
`default_nettype wire
