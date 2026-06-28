`timescale 1ns / 1ps
`default_nettype none

// Unit test for video_frame_normalizer: output must be EXACTLY OUT_LINES x
// OUT_PIXELS per frame regardless of input geometry (short/long line/frame).
module tb_video_frame_normalizer;
    localparam int OL = 4, OP = 4;
    logic clk = 0, aresetn;
    always #5 clk = ~clk;

    logic [7:0] in_d; logic in_v, in_sof, in_eol, in_eof, in_err;
    logic [7:0] out_d; logic out_v, out_sof, out_eol, out_eof, out_err;

    video_frame_normalizer #(.OUT_LINES(OL), .OUT_PIXELS(OP), .FILL(8'hEE), .NORMALIZE(1'b1)) dut (
        .clk(clk), .aresetn(aresetn),
        .in_data(in_d), .in_valid(in_v), .in_sof(in_sof), .in_eol(in_eol), .in_eof(in_eof), .in_err(in_err),
        .out_data(out_d), .out_valid(out_v), .out_sof(out_sof), .out_eol(out_eol), .out_eof(out_eof), .out_err(out_err)
    );

    // scoreboard for one output frame
    int o_px, o_lines, o_px_this_line, sof_seen, eof_seen;
    int line_px [0:63];
    bit capturing;
    always_ff @(posedge clk) begin
        if (!aresetn) begin
            o_px<=0; o_lines<=0; o_px_this_line<=0; sof_seen<=0; eof_seen<=0; capturing<=0;
        end else if (out_v) begin
            if (out_sof) begin sof_seen<=sof_seen+1; end
            o_px <= o_px + 1;
            o_px_this_line <= o_px_this_line + 1;
            if (out_eol) begin
                if (o_lines < 64) line_px[o_lines] <= o_px_this_line + 1;
                o_lines <= o_lines + 1;
                o_px_this_line <= 0;
            end
            if (out_eof) eof_seen <= eof_seen + 1;
        end
    end

    // drive one input line of `npx` pixels (value=val), last pixel carries eol,
    // and if `eof` the last pixel also carries eof.
    task automatic drv_line(input int npx, input logic [7:0] val, input bit eof);
        for (int i=0;i<npx;i++) begin
            @(posedge clk);
            in_v<=1; in_d<=val; in_sof<=1'b0; in_eol<=(i==npx-1); in_eof<=(eof && i==npx-1);
        end
        @(posedge clk); in_v<=0; in_eol<=0; in_eof<=0;
        // inter-line gap
        repeat(3) @(posedge clk);
    endtask
    // drive a frame: nlines lines, line k has linepx[k] pixels
    task automatic drv_frame(input int nlines, input int npx);
        // first pixel of frame carries sof
        @(posedge clk); in_v<=1; in_d<=8'h10; in_sof<=1'b1; in_eol<=(npx==1); in_eof<=1'b0;
        for (int i=1;i<npx;i++) begin @(posedge clk); in_v<=1; in_sof<=0; in_d<=8'h10; in_eol<=(i==npx-1); end
        @(posedge clk); in_v<=0; in_sof<=0; in_eol<=0; repeat(3) @(posedge clk);
        for (int k=1;k<nlines;k++) drv_line(npx, 8'h10+8'(k), (k==nlines-1));
    endtask

    task automatic reset_sb(); o_px=0; o_lines=0; o_px_this_line=0; sof_seen=0; eof_seen=0; endtask
    task automatic chk(input bit c, input string m); if(!c) $fatal(1,"FAIL: %s",m); endtask

    task automatic settle(); repeat(60) @(posedge clk); endtask

    initial begin
        aresetn=0; in_v=0; in_d=0; in_sof=0; in_eol=0; in_eof=0; in_err=0;
        repeat(8) @(posedge clk); aresetn=1; repeat(4) @(posedge clk);

        // A: exact frame 4 lines x 4 px
        reset_sb(); drv_frame(4,4); settle();
        $display("[A exact 4x4] sof=%0d lines=%0d px=%0d eof=%0d", sof_seen,o_lines,o_px,eof_seen);
        chk(sof_seen==1,"A sof==1"); chk(o_lines==OL,"A lines==4"); chk(o_px==OL*OP,"A px==16"); chk(eof_seen==1,"A eof==1");
        for(int k=0;k<OL;k++) chk(line_px[k]==OP,"A each line 4px");

        // B: short lines (2 px) -> pad to 4 ; 4 lines
        reset_sb(); drv_frame(4,2); settle();
        $display("[B short-line 4x2] sof=%0d lines=%0d px=%0d", sof_seen,o_lines,o_px);
        chk(o_lines==OL,"B lines==4"); chk(o_px==OL*OP,"B px==16"); chk(sof_seen==1,"B sof==1");
        for(int k=0;k<OL;k++) chk(line_px[k]==OP,"B each padded to 4px");

        // C: long lines (6 px) -> truncate to 4 ; 4 lines
        reset_sb(); drv_frame(4,6); settle();
        $display("[C long-line 4x6] lines=%0d px=%0d", o_lines,o_px);
        chk(o_lines==OL,"C lines==4"); chk(o_px==OL*OP,"C px==16");
        for(int k=0;k<OL;k++) chk(line_px[k]==OP,"C each truncated to 4px");

        // D: short frame (2 lines) -> pad to 4 lines
        reset_sb(); drv_frame(2,4); settle();
        $display("[D short-frame 2x4] lines=%0d px=%0d eof=%0d", o_lines,o_px,eof_seen);
        chk(o_lines==OL,"D lines padded to 4"); chk(o_px==OL*OP,"D px==16"); chk(eof_seen==1,"D eof==1");

        // E: long frame (7 lines) -> truncate to 4 lines
        reset_sb(); drv_frame(7,4); settle();
        $display("[E long-frame 7x4] lines=%0d px=%0d eof=%0d", o_lines,o_px,eof_seen);
        chk(o_lines==OL,"E lines truncated to 4"); chk(o_px==OL*OP,"E px==16"); chk(eof_seen==1,"E eof==1");

        // F: mismatched both (3 lines x 5 px) -> 4x4
        reset_sb(); drv_frame(3,5); settle();
        $display("[F 3x5] lines=%0d px=%0d", o_lines,o_px);
        chk(o_lines==OL,"F lines==4"); chk(o_px==OL*OP,"F px==16");

        $display("TEST PASSED: tb_video_frame_normalizer (output always %0dx%0d)", OL, OP);
        $finish;
    end
    initial begin #2ms; $fatal(1,"timeout"); end
endmodule
`default_nettype wire
