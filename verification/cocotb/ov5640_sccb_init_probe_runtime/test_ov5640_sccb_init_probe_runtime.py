"""cocotb port of verification/tb/tb_ov5640_sccb_init_probe_runtime.sv.

The DUT (``ov5640_sccb_init_probe``) is a bit-banged SCCB/I2C *master*: on reset it walks a
261-entry init ROM, then services runtime requests (test-pattern toggle 0x503d, arbitrary
register write, register read). The DSim TB models the OV5640 side of the two-wire bus with
four ``always`` blocks:

  * an I2C *slave* that ACKs every byte (drives ``cam_sda`` low on the 9th SCL),
  * a bus *monitor* that reassembles each 4-byte write and classifies it (test-pattern write
    to 0x503d, or an arbitrary runtime write flagged by ``dut.runtime_active`` /
    ``dut.runtime_kind``),
  * plus an ``always @(posedge clk)`` scoreboard that records that the seven AEC reference
    writes were emitted during init.

Those become coroutines here. The two-wire bus is tri-state (``tri1`` with pull-ups in the
TB). Verilator is 2-state and cannot resolve two tri-state drivers on an ``inout`` from
Python, so the DUT is built with ``USE_EXTERNAL_IOBUF=1`` (its internal open-drain drivers
are elided) and this testbench owns the wired-AND resolution in ``Bus.update``:
``cam_x = 0`` iff the master pulls it low (``*_drive_low_o``) or the slave pulls SDA low.
That is functionally identical to the TB's ``tri1`` net, just resolved in Python.

The whole init sequence (incl. the ROM's 1000 ms / 300 ms power-up + stream-on delays at
CLK_HZ=1 MHz) runs entirely inside the RTL counters, so the Python models only wake on real
bus edges -- the idle delay windows cost nothing. All TB ``$fatal`` checks are replicated
1:1 as ``check()`` calls; ``$display("TEST PASSED")``/``$finish`` -> return.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cocotb
from cocotb.triggers import Edge, RisingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lib.clkreset import start_clock  # noqa: E402
from lib.scoreboard import check  # noqa: E402

CLK_HZ = 1_000_000
I2C_HZ = 100_000
CLK_PERIOD_NS = 1000.0  # always #500 clk = ~clk


# --------------------------------------------------------------------------------------
# Two-wire bus model (replaces the TB's `tri1 cam_scl/cam_sda` + slave `assign`).
# --------------------------------------------------------------------------------------
class Bus:
    """Wired-AND resolver for cam_scl / cam_sda.

    The master (DUT) pulls low via scl_drive_low_o / sda_drive_low_o; the slave pulls sda
    low via slave_sda_drive_low. Otherwise the pull-ups win (logic 1).
    """

    def __init__(self, dut):
        self.dut = dut
        self.slave_sda_drive_low = 0

    def update(self):
        scl = 0 if int(self.dut.scl_drive_low_o.value) else 1
        sda = 0 if (int(self.dut.sda_drive_low_o.value) or self.slave_sda_drive_low) else 1
        self.dut.cam_scl.value = scl
        self.dut.cam_sda.value = sda

    def set_slave_low(self, low: int):
        self.slave_sda_drive_low = 1 if low else 0
        self.update()

    async def run(self):
        """React to master drive changes; keep cam_scl/cam_sda resolved."""
        self.update()
        while True:
            await Edge(self.dut.scl_drive_low_o)  # any master drive change
            self.update()

    async def run_sda(self):
        self.update()
        while True:
            await Edge(self.dut.sda_drive_low_o)
            self.update()


# --------------------------------------------------------------------------------------
# I2C slave: ACK every byte (mirrors the TB slave always blocks).
# --------------------------------------------------------------------------------------
class Slave:
    def __init__(self, dut, bus: Bus):
        self.dut = dut
        self.bus = bus
        self.in_transfer = False
        self.pending_ack = False
        self.ack_phase = False
        self.bit_count = 0

    async def on_start(self):
        """negedge cam_sda while cam_scl high -> START."""
        self.in_transfer = True
        self.pending_ack = False
        self.ack_phase = False
        self.bit_count = 0
        self.bus.set_slave_low(0)

    def on_stop(self):
        self.in_transfer = False
        self.pending_ack = False
        self.ack_phase = False
        self.bus.set_slave_low(0)

    def on_scl_rise(self):
        if self.in_transfer and not self.ack_phase:
            if self.bit_count == 7:
                self.pending_ack = True
                self.bit_count = 0
            else:
                self.bit_count += 1

    def on_scl_fall(self):
        if self.in_transfer:
            if self.ack_phase:
                self.bus.set_slave_low(0)
                self.ack_phase = False
            elif self.pending_ack:
                self.bus.set_slave_low(1)
                self.pending_ack = False
                self.ack_phase = True


# --------------------------------------------------------------------------------------
# Bus monitor: reassemble writes, classify (mirrors the TB monitor always blocks + task).
# --------------------------------------------------------------------------------------
class Monitor:
    def __init__(self, dut):
        self.dut = dut
        self.in_transfer = False
        self.skip_ack = False
        self.bit_count = 0
        self.byte_count = 0
        self.byte = 0
        self.bytes = [0] * 8
        self.runtime_active_latched = 0
        self.runtime_kind_latched = 0  # bit[0] of dut.runtime_kind

        # scoreboard state (matches the TB's module-scope vars)
        self.test_pattern_write_count = 0
        self.last_test_pattern_value = 0
        self.arbitrary_write_count = 0
        self.last_arbitrary_addr = 0
        self.last_arbitrary_value = 0

    def on_start(self):
        self.in_transfer = True
        self.skip_ack = False
        self.bit_count = 0
        self.byte_count = 0
        self.byte = 0
        self.runtime_active_latched = int(self.dut.runtime_active.value)
        self.runtime_kind_latched = int(self.dut.runtime_kind.value) & 0x1

    def on_scl_rise(self, sda: int):
        if not self.in_transfer:
            return
        if self.skip_ack:
            self.skip_ack = False
        else:
            self.byte = ((self.byte << 1) | (sda & 0x1)) & 0xFF
            if self.bit_count == 7:
                if self.byte_count < 8:
                    self.bytes[self.byte_count] = self.byte
                self.byte_count += 1
                self.bit_count = 0
                self.skip_ack = True
            else:
                self.bit_count += 1

    def on_stop(self):
        if self.in_transfer:
            self._process()
        self.in_transfer = False

    def _process(self):
        addr = (self.bytes[1] << 8) | self.bytes[2]
        value = self.bytes[3]

        if self.byte_count >= 4 and self.bytes[0] == 0x78 and addr == 0x503D:
            self.test_pattern_write_count += 1
            self.last_test_pattern_value = value

        if (self.byte_count >= 4 and self.bytes[0] == 0x78
                and self.runtime_active_latched and self.runtime_kind_latched == 1):
            self.arbitrary_write_count += 1
            self.last_arbitrary_addr = addr
            self.last_arbitrary_value = value


# --------------------------------------------------------------------------------------
# Bus-edge dispatcher: fan out cam_sda / cam_scl edges to slave + monitor exactly like the
# TB's `always @(negedge/posedge cam_sda)` and `always @(posedge/negedge cam_scl)`.
# --------------------------------------------------------------------------------------
async def sda_edges(dut, slave: Slave, monitor: Monitor):
    while True:
        await Edge(dut.cam_sda)
        # Let the resolver settle any simultaneous master drive change first.
        await Timer(1, unit="ns")
        scl = int(dut.cam_scl.value)
        sda = int(dut.cam_sda.value)
        if sda == 0:
            # negedge cam_sda
            if scl:  # START
                await slave.on_start()
                monitor.on_start()
        else:
            # posedge cam_sda (#1 in TB before checking scl)
            if scl:  # STOP
                monitor.on_stop()
                slave.on_stop()


async def scl_edges(dut, slave: Slave, monitor: Monitor):
    while True:
        await Edge(dut.cam_scl)
        scl = int(dut.cam_scl.value)
        if scl:
            # posedge cam_scl: sample sda for the slave counter + monitor shift
            sda = int(dut.cam_sda.value)
            slave.on_scl_rise()
            monitor.on_scl_rise(sda)
        else:
            slave.on_scl_fall()


# --------------------------------------------------------------------------------------
# Helpers replicating the TB tasks.
# --------------------------------------------------------------------------------------
async def wait_ready(dut):
    """wait (rt_test_pattern_ready) -- edge/level wait without per-cycle polling."""
    if int(dut.rt_test_pattern_ready.value) == 1:
        return
    await RisingEdge(dut.rt_test_pattern_ready)


async def wait_write_ready(dut):
    if int(dut.rt_reg_write_ready.value) == 1:
        return
    await RisingEdge(dut.rt_reg_write_ready)


async def issue_test_pattern_request(dut, mon: Monitor, enable: int, expected_value: int):
    await wait_ready(dut)
    dut.rt_test_pattern_enable.value = enable
    dut.rt_test_pattern_valid.value = 1
    await RisingEdge(dut.clk)
    dut.rt_test_pattern_valid.value = 0
    await RisingEdge(dut.clk)
    # wait (!rt_test_pattern_done); wait (rt_test_pattern_done);
    while int(dut.rt_test_pattern_done.value) != 0:
        await RisingEdge(dut.clk)
    while int(dut.rt_test_pattern_done.value) != 1:
        await RisingEdge(dut.clk)
    check(int(dut.rt_test_pattern_error.value) == 0,
          f"runtime SCCB error for 0x503d=0x{expected_value:02x}")
    check(int(dut.rt_ack_error_count.value) == 0,
          f"runtime ACK error count is {int(dut.rt_ack_error_count.value)}")
    check(int(dut.rt_test_pattern_value.value) == expected_value,
          f"status value 0x{int(dut.rt_test_pattern_value.value):02x}, "
          f"expected 0x{expected_value:02x}")
    check(int(dut.reg_addr.value) == 0x503D and int(dut.reg_value.value) == expected_value,
          f"final SCCB command addr=0x{int(dut.reg_addr.value):04x} "
          f"value=0x{int(dut.reg_value.value):02x}, expected 0x503d=0x{expected_value:02x}")


async def issue_arbitrary_reg_write(dut, mon: Monitor, addr: int, value: int):
    before_count = mon.arbitrary_write_count
    await wait_write_ready(dut)
    dut.rt_reg_write_addr.value = addr
    dut.rt_reg_write_value.value = value
    dut.rt_reg_write_valid.value = 1
    await RisingEdge(dut.clk)
    dut.rt_reg_write_valid.value = 0
    await RisingEdge(dut.clk)
    while int(dut.rt_reg_write_done.value) != 0:
        await RisingEdge(dut.clk)
    while int(dut.rt_reg_write_done.value) != 1:
        await RisingEdge(dut.clk)
    for _ in range(4):
        await RisingEdge(dut.clk)
    check(int(dut.rt_reg_write_error.value) == 0,
          f"arbitrary write error for 0x{addr:04x}=0x{value:02x}")
    check(int(dut.rt_reg_write_last_addr.value) == addr,
          f"rt_reg_write_last_addr 0x{int(dut.rt_reg_write_last_addr.value):04x}, "
          f"expected 0x{addr:04x}")
    check(mon.arbitrary_write_count == before_count + 1,
          f"monitor saw {mon.arbitrary_write_count - before_count} arbitrary writes, "
          f"expected 1")
    check(mon.last_arbitrary_addr == addr and mon.last_arbitrary_value == value,
          f"monitor last 0x{mon.last_arbitrary_addr:04x}=0x{mon.last_arbitrary_value:02x}, "
          f"expected 0x{addr:04x}=0x{value:02x}")


# --------------------------------------------------------------------------------------
# The single TB scenario, ported 1:1.
# --------------------------------------------------------------------------------------
@cocotb.test(timeout_time=3000, timeout_unit="ms")
async def sccb_init_and_runtime(dut):
    # --- wiring: bus, slave, monitor, AEC scoreboard ---
    bus = Bus(dut)
    slave = Slave(dut, bus)
    monitor = Monitor(dut)

    dut.rt_test_pattern_valid.value = 0
    dut.rt_test_pattern_enable.value = 0
    dut.rt_reg_write_valid.value = 0
    dut.rt_reg_write_addr.value = 0
    dut.rt_reg_write_value.value = 0
    dut.rt_reg_read_valid.value = 0
    dut.rt_reg_read_addr.value = 0
    dut.rst_n.value = 0
    dut.cam_scl.value = 1
    dut.cam_sda.value = 1

    # AEC reference-write mask, updated by an inline monitor coroutine (closure over `state`).
    aec = {"mask": 0}

    async def aec_watch():
        # Mirrors `always @(posedge clk) case({reg_addr,reg_value})`. reg_addr/reg_value
        # change together only ~261 times (once per init step) plus during runtime, so
        # triggering on their value_change is equivalent to per-cycle sampling but avoids
        # 1.5M Python callbacks over the ROM's multi-ms power-up delays.
        targets = {
            0x350300: 0, 0x3A0078: 1, 0x3A0101: 2, 0x3A1343: 3,
            0x3A1800: 4, 0x3A19F8: 5, 0x3A1A04: 6,
        }

        def sample():
            if int(dut.rst_n.value) == 1:
                key = (int(dut.reg_addr.value) << 8) | int(dut.reg_value.value)
                if key in targets:
                    aec["mask"] |= (1 << targets[key])

        while True:
            await Edge(dut.reg_addr)
            sample()

    start_clock(dut.clk, CLK_PERIOD_NS)
    cocotb.start_soon(bus.run())
    cocotb.start_soon(bus.run_sda())
    cocotb.start_soon(sda_edges(dut, slave, monitor))
    cocotb.start_soon(scl_edges(dut, slave, monitor))
    cocotb.start_soon(aec_watch())

    # repeat (10) @(posedge clk); rst_n = 1;
    for _ in range(10):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1

    # wait (rt_test_pattern_ready);  (init sequence completes here)
    await wait_ready(dut)

    # TB literal is `sccb_step_index !== 8'd255`, written for the pre-2026-05-30 RTL whose
    # init ROM ended at step 255. The current flat-ROM RTL has 261 entries (0..260),
    # LAST_STEP=260, so init settles at step_index=260 (step 255 is now just the mid-block
    # 0x503d write). The check's intent -- "init ran to its final ROM entry" -- is preserved
    # against the DUT's actual terminal step. (The stale TB also declared sccb_step_index as
    # 8-bit vs the RTL's 9-bit step_index output, so the DSim TB literal is doubly outdated.)
    LAST_STEP = 260
    check(int(dut.step_index.value) == LAST_STEP,
          f"init ended at step {int(dut.step_index.value)}, expected {LAST_STEP}")
    check(aec["mask"] == 0x7F,
          f"missing AEC reference writes mask=0x{aec['mask']:02x}")

    await issue_test_pattern_request(dut, monitor, 1, 0x80)
    await issue_test_pattern_request(dut, monitor, 0, 0x00)

    await issue_arbitrary_reg_write(dut, monitor, 0x380C, 0x07)
    await issue_arbitrary_reg_write(dut, monitor, 0x380D, 0x68)
    await issue_arbitrary_reg_write(dut, monitor, 0x380E, 0x03)

    check(monitor.arbitrary_write_count == 3,
          f"arbitrary write count {monitor.arbitrary_write_count}, expected 3")
    check(monitor.last_arbitrary_addr == 0x380E and monitor.last_arbitrary_value == 0x03,
          f"last arbitrary write 0x{monitor.last_arbitrary_addr:04x}="
          f"0x{monitor.last_arbitrary_value:02x}, expected 0x380e=0x03")
    check(int(dut.rt_reg_write_ack_err_count.value) == 0,
          f"arbitrary write ACK error count {int(dut.rt_reg_write_ack_err_count.value)}")
    check(int(dut.rt_ack_error_count.value) == 0,
          f"test-pattern ACK error count {int(dut.rt_ack_error_count.value)} "
          f"after mixed sequence")

    await issue_test_pattern_request(dut, monitor, 1, 0x80)
    # TB literal expects 3 (the three runtime 0x503d requests above). The monitor's
    # test-pattern counter has NO runtime_active guard (unlike the arbitrary-write counter),
    # so it also counts init-time 0x503d writes. The pre-2026-05-30 RTL emitted no 0x503d
    # during init, so 3 was correct then. The current flat-ROM RTL writes 0x503d once during
    # init at step 255 (value TEST_PATTERN_ENABLE?0x80:0x00 = 0x00, runtime_active=0 -- the
    # "orig inline 228..232" post-init block folded into the ROM). Verified via the monitored
    # stream: #1 init value=0x00 runtime=0, then #2 0x80, #3 0x00, #4 0x80 all runtime=1. So
    # the faithful total against this DUT is 1 (init) + 3 (runtime) = 4; the monitor logic is
    # unchanged from the SV.
    RUNTIME_TP_WRITES = 3
    INIT_TP_WRITES = 1  # ROM step 255: 0x503d <= 0x00
    check(monitor.test_pattern_write_count == INIT_TP_WRITES + RUNTIME_TP_WRITES,
          f"test-pattern write count {monitor.test_pattern_write_count} after arbitrary "
          f"writes, expected {INIT_TP_WRITES + RUNTIME_TP_WRITES} "
          f"(1 init-time 0x503d + 3 runtime)")

    for _ in range(20):
        await RisingEdge(dut.clk)
    # $display("TEST PASSED"); $finish;


def test_ov5640_sccb_init_probe_runtime():
    from runner_support import build_and_test

    build_and_test(
        block="ov5640_sccb_init_probe_runtime",
        sources=["rtl/prototype/ov5640_sccb_init_probe.sv"],
        toplevel="ov5640_sccb_init_probe",
        test_module="test_ov5640_sccb_init_probe_runtime",
        test_dir=Path(__file__).resolve().parent,
        parameters={
            "CLK_HZ": CLK_HZ,
            "I2C_HZ": I2C_HZ,
            "POWERUP_DELAY_MS": 1,
            "TEST_PATTERN_ENABLE": 0,
            "USE_EXTERNAL_IOBUF": 1,
        },
        engine="verilator",
    )
