"""Reusable cocotb helpers for the MIPI2HDMI RTL verification environment.

Three DUT interface families are modelled here (the project's ``axis_`` filename prefix is
misleading -- most img_proc blocks are valid-only, not AXI4-Stream):

* ``byte_beat``    -- ``s_byte_{data,keep,valid,sop,eop}`` (no tready); csi2 byte path.
* ``pixel_stream`` -- ``in_{pixel,valid,sof,eol,eof,err}`` -> ``out_*`` (no tready); img_proc.
* ``axis``         -- true AXI4-Stream ``*_t{valid,ready,data,last,user}``; bridges, HDMI.

Plus ``clkreset`` (clock + active-low synchronous reset, the project convention) and
``scoreboard`` (``check()`` with a log token compatible with the old DSim flow).
"""
