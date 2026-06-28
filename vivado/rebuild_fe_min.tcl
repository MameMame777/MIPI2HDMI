## rebuild_fe_min.tcl (2026-06-15)
## Rebuild the BD-based MIPI pipeline (vloop_probes2) with the csi2_frame_state
## spurious-early-FE windowing fix. (vloop_probes2 project, bd_core0_0_synth_1
## OOC, pre_synth_tpg.tcl hook).
##
## RTL change in this build (2026-06-19c, branch feat/gated-supervisor-480):
##   - BAKE HWLOCK_DEFAULT_ON via the RTL module DEFAULT (=1'b1) in
##       mipi_to_hdmi_probe_top -- the HW lock FSM is ON at power-up -> auto-locks
##       as soon as the chip streams, NO PYNQ lock step. bitslip_word[26] inhibits
##       it at runtime (software lock_mode fallback). NB: the pre_synth fileset
##       generics are all "Unused" in this BD flow (verified 19b: the FSM stayed
##       IDLE when set only via the generic) -> the binding path is the RTL default,
##       exactly like IMAGE_FORMAT=1.
##   - dphy_hwlock_fsm.sv : S_FAILED now auto-retries the whole sweep after
##       RETRY_CYC (~10ms) so a boot-enable BEFORE the chip streams does not stick
##       in FAILED -- it keeps trying and locks the moment the stream is up.
##       DSim tb_dphy_hwlock_fsm 18/18 (adds T5 retry).
##   - mipi_to_hdmi_probe_top.sv : NEW param HWLOCK_DEFAULT_ON; cfg_hw_lock =
##       (bitslip[25] | HWLOCK_DEFAULT_ON) & ~bitslip[26]. default-off + bit26=0 is
##       bit-identical to the prior build.
##   Caveat: not zero-PYNQ -- the chip SCCB init still NACKs at boot (AXI GPIO
##   RESETB race); this bake only drops the *lock* step once PYNQ inits the chip.
##
## (earlier 2026-06-19) HW deterministic-lock FSM (E2):
##   - rtl/mipi_rx/dphy_hwlock_fsm.sv : NEW HW deterministic-lock FSM (E2). Ports
##       the software lock_mode (8x8 bitslip sweep + /4 BUFR.CLR re-roll + hold)
##       into RTL on refclk_200 (survives the re-roll) so a bare bitstream
##       auto-locks on power-up (continuous only). Opt-in via bitslip_word[25].
##   - rtl/prototype/dphy_hs_byte_probe.sv : NEW input hwlock_bufr_clr, ORed with
##       rt_bufr_clr_ctl into one bufr_reroll net (BUFR.CLR + ISERDES re-roll).
##   - rtl/prototype/mipi_to_hdmi_probe_top.sv : byte_clk windowed sync-header
##       detector (hdr_ok) -> refclk_200; dphy_hwlock_fsm instance; bitslip target
##       muxed FSM-vs-GPIO on cfg_hw_lock; status on debug page 0x2e (ctrl 0x8e).
##   - DSim: tb_dphy_hwlock_fsm (sweep->lock->hold, never-active->reroll->fail) +
##       tb_dphy_probe_supervised regress (new probe input default 0 = unchanged).
##   - cfg_hw_lock=0 -> bit-identical to the prior build (GPIO/lock_mode fallback).
##
## (earlier 2026-06-18c)
##   - rtl/mipi_rx/dphy_lane_supervisor.sv : NEW runtime input cfg_clk_settle_cyc
##       (clock-lane settle count = when byte_clk starts after a clock-lane restart).
##       0 = build-time default. Gated FS-recovery: sweep it to catch the vblank-exit
##       FS (fs=0 today). Wired from bitslip_word[23:17] -> refclk_200 2FF -> sup.
##   - DSim: tb_dphy_lane_supervisor + tb_dphy_probe_supervised 26/26 (default 0 =
##       build-time, unchanged).
##
## (earlier 2026-06-18b)
##   - rtl/prototype/dphy_hs_byte_probe.sv : DECOUPLE the sup HS-SETTLE SoT gate from
##   - rtl/prototype/dphy_hs_byte_probe.sv : DECOUPLE the sup HS-SETTLE SoT gate from
##       the sup BUFR/ISERDES per-gate management (settle_gate_en). In sup mode, if
##       cfg_settle_blank_k>0 the sup SoT gate turns OFF and the byte-domain
##       settle-blank (the proven continuous burst-head fix) handles the burst head
##       -> the two no longer stack/over-blank in gated. blank=0 = old behaviour
##       exactly (continuous on main unchanged). Goal: gated (0x34) + sup + blank=8
##       -> last_frame_lines 480. DSim tb_dphy_probe_supervised T9 + 26/26.
##   - (also) vblank-exit RE-LOCK latency counter
##       (byte_clk cycles from ISERDES-reset release to the first accepted SoT) ->
##       output dbg_relock_latency/dbg_relock_max, exposed on debug page 0x2d
##       (ctrl 0x8D). Diagnostic-only (no behaviour change). Pins the gated
##       top-loss mechanism (is the ~36-line loss a long re-lock latency?).
##   - rtl/prototype/mipi_to_hdmi_probe_top.sv : wire + CDC + page 0x2d.
##
## (earlier, carried) 2026-06-17 settle-blank + SoT-miss diagnostics:
##   - rtl/prototype/dphy_hs_byte_probe.sv : NEW cfg_settle_blank_k (byte_clk
##       domain): hold the SoT window CLOSED K byte_clk after a data-lane LP-exit,
##       so the per-line SoT search skips the HS-prepare/settle garbage at the
##       burst head (correctly-timed byte-domain version of cfg_hs_settle_gate,
##       which used ctl_clk sup_hs_settled -> too late). Plus SoT-miss diagnostics:
##       dbg_burst_count (LP-exit edges), dbg_sot_burst_count (bursts with a stream
##       SoT), dbg_missed_burst (head bytes of the last no-SoT burst). Default K=0.
##   - rtl/prototype/mipi_to_hdmi_probe_top.sv : idelay_word[30:27] ->
##       cfg_settle_blank_k_byte (2FF); diagnostics on debug pages 0x2b/0x2c.
##   - (carried) cfg_hs_settle_gate (bit28), csi2_frame_state cfg_long_as_line +
##       FE_MIN windowing.
## DSim-validated: tb_dphy_probe_supervised T8 (blank=4 still locks + burst/sot
##   counters advance) + 25/25; tb_csi2_frame_state* regress.
##
## IMPORTANT: this MUST build vloop_probes2 (NOT vloop). The deployed bitstream
## (supervisor bit29 + VDMA C_USE_S2MM_FSYNC=2/GENLOCK=2) lives in vloop_probes2;
## rebuild_core0_fsmin.tcl targets the OLD vloop project and would regress both.
## Only bd_core0_0_synth_1 is reset here, so the VDMA OOC (fsync=2) is preserved.

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set xpr      [file join $repo_dir vloop_probes2 vloop.xpr]
set hook_tcl [file normalize [file join [file dirname [info script]] pre_synth_tpg.tcl]]

puts "Opening $xpr"
open_project $xpr

proc ensure_source {repo_dir rel} {
    set f [file normalize [file join $repo_dir {*}$rel]]
    set base [file tail $f]
    if {[get_files -quiet $f] eq "" && [get_files -quiet *$base] eq ""} {
        puts "INFO: Adding $f"
        add_files -norecurse -fileset [get_filesets sources_1] $f
        set_property file_type SystemVerilog [get_files $f]
    } else {
        puts "INFO: $base already in project"
    }
}

# Keep the full design source set present (idempotent; same as the supervisor build).
ensure_source $repo_dir {rtl mipi_rx dphy_cdc_prims.sv}
ensure_source $repo_dir {rtl mipi_rx dphy_lane_supervisor.sv}
ensure_source $repo_dir {rtl mipi_rx dphy_hwlock_fsm.sv}
ensure_source $repo_dir {rtl prototype csi2_tpg.sv}
ensure_source $repo_dir {rtl img_proc video_frame_normalizer.sv}
ensure_source $repo_dir {rtl img_proc median9.sv}
ensure_source $repo_dir {rtl img_proc axis_rgb_prefilter.sv}
ensure_source $repo_dir {rtl img_proc axis_rgb_dither.sv}
ensure_source $repo_dir {rtl mipi_rx csi2_frame_state.sv}
ensure_source $repo_dir {rtl prototype mipi_to_hdmi_probe_top.sv}

update_compile_order -fileset sources_1

# Pre-synth hook on bd_core0_0_synth_1 (keeps IMAGE_FORMAT=1, as the deployed build).
puts "INFO: Installing pre-synth hook on bd_core0_0_synth_1: $hook_tcl"
set_property -name {STEPS.SYNTH_DESIGN.TCL.PRE} \
             -value $hook_tcl \
             -objects [get_runs bd_core0_0_synth_1]

# Disable incremental synthesis so the frame_state edit definitely enters the bitstream.
set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
set_property INCREMENTAL_CHECKPOINT       "" [get_runs synth_1]

# Reset core0 OOC (mandatory — feedback_rtl_edits_need_core0_ooc_resynth) + synth + impl.
# Do NOT reset bd_axi_vdma_0_0_synth_1, so the VDMA fsync=2 OOC netlist is preserved.
puts "Resetting OOC core0 + synth_1 + impl_1 ..."
reset_run bd_core0_0_synth_1
reset_run synth_1
reset_run impl_1
# Performance_ExplorePostRoutePhysOpt (2026-06-16): the design sits at the timing
# edge (WNS ~0); the force-480/open-at-top logic + placement noise pushed a
# Congestion_SpreadLogic_high run to WNS=-0.278. This strategy optimises for
# timing and runs post-route phys_opt to close small remaining violations. The
# u_rawcap congestion that originally needed the Congestion strategy is fixed
# (DEPTH 64), so a Performance strategy is safe here.
set_property strategy "Performance_ExplorePostRoutePhysOpt" [get_runs impl_1]
# 2026-06-26: the PRE-stage prefilter (median9 x3 + line buffers) added congestion that left
# conv3x3/conv5x5 acc paths ~ -0.03 ns (net-delay bound; phys_opt exhausted logic opts at -0.029).
# Strengthen the post-route phys_opt (the step already closing -0.069 -> -0.031) to AggressiveExplore
# to recover the last few ps without touching the verified RTL.
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
# Timing-driven placement: the residual -0.031 is route delay (59%) on conv3x3's sat path,
# congestion from the adjacent prefilter. A timing-driven place directive re-places it tighter
# (LUT util is only 27.5%, so headroom exists) without touching the verified RTL.
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraNetDelay_high [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

puts "Launching impl_1 -to_step write_bitstream -jobs 6 ..."
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1

set prog [get_property PROGRESS [get_runs impl_1]]
set wns  [get_property STATS.WNS [get_runs impl_1]]
puts "impl_1 progress=$prog  WNS=$wns"

if {$prog ne "100%"} {
    puts "IMPL_FAILED prog=$prog"
    exit 1
}

set bit [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]
set hwh [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.hwh]
puts "BITSTREAM_OK $bit"
puts "HWH_OK $hwh"
