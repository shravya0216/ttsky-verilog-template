"""
Tiny Tapeout required cocotb test.

This drives the design through its ACTUAL fabricated pins (ui_in/uio_*),
the same way the Vivado/Icarus testbenches in this project did, just in
cocotb's Python style since that's what Tiny Tapeout's CI gate requires.

Everything is checked through the pins only (I2C readback + LED), never
through internal signals, so the same test runs on the RTL and on the
gate-level netlist (GATES=yes, used by the gl_test CI job).

Note: regfile/IMEM/DMEM readback is only valid while the CPU is halted
(RUN=0 or DONE=1); the readback shares the CPU's read ports.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

SDA_BIT = 0   # uio[0]
SCL_BIT = 0   # ui_in[0]
HALF = 1000   # ns per SCL half-period (100 clk cycles at 10 ns clk)


class I2CDriver:
    """Models the open-drain SDA bus: two independent drivers (this 'MCU'
    model and the DUT itself via uio_oe/uio_out) resolved the same way a
    real pull-up resistor resolves a shared open-drain line."""

    def __init__(self, dut):
        self.dut = dut
        self.mcu_release = True
        cocotb.start_soon(self._resolve_loop())

    async def _resolve_loop(self):
        while True:
            await Timer(10, unit="ns")
            dut_oe = (int(self.dut.uio_oe.value) >> SDA_BIT) & 1
            bus_high = self.mcu_release and not dut_oe
            val = int(self.dut.uio_in.value)
            if bus_high:
                val |= (1 << SDA_BIT)
            else:
                val &= ~(1 << SDA_BIT)
            self.dut.uio_in.value = val

    def set_scl(self, level):
        val = int(self.dut.ui_in.value)
        if level:
            val |= (1 << SCL_BIT)
        else:
            val &= ~(1 << SCL_BIT)
        self.dut.ui_in.value = val

    async def start(self):
        self.mcu_release = True
        self.set_scl(1)
        await Timer(HALF, unit="ns")
        self.mcu_release = False
        await Timer(HALF, unit="ns")
        self.set_scl(0)
        await Timer(HALF, unit="ns")

    async def stop(self):
        self.mcu_release = False
        self.set_scl(0)
        await Timer(HALF, unit="ns")
        self.set_scl(1)
        await Timer(HALF, unit="ns")
        self.mcu_release = True
        await Timer(HALF, unit="ns")

    async def write_bit(self, b):
        self.set_scl(0)
        self.mcu_release = bool(b)
        await Timer(HALF, unit="ns")
        self.set_scl(1)
        await Timer(HALF, unit="ns")

    async def ack_bit(self):
        self.set_scl(0)
        self.mcu_release = True
        await Timer(HALF, unit="ns")
        self.set_scl(1)
        await Timer(HALF, unit="ns")
        self.set_scl(0)
        await Timer(HALF, unit="ns")

    async def write_byte(self, byte_val):
        for k in range(7, -1, -1):
            await self.write_bit((byte_val >> k) & 1)
        await self.ack_bit()

    async def mmio_write(self, addr, data):
        await self.start()
        await self.write_byte(0x84)  # 0x42 << 1 | write
        await self.write_byte((addr >> 8) & 0xFF)
        await self.write_byte(addr & 0xFF)
        await self.write_byte((data >> 24) & 0xFF)
        await self.write_byte((data >> 16) & 0xFF)
        await self.write_byte((data >> 8) & 0xFF)
        await self.write_byte(data & 0xFF)
        await self.stop()
        await Timer(100, unit="ns")

    async def read_byte(self, ack):
        val = 0
        for _ in range(8):
            self.set_scl(0)
            self.mcu_release = True
            await Timer(HALF, unit="ns")
            self.set_scl(1)
            await Timer(HALF, unit="ns")
            val = (val << 1) | ((int(self.dut.uio_in.value) >> SDA_BIT) & 1)
        self.set_scl(0)
        self.mcu_release = not ack   # ACK = pull low, NACK = release
        await Timer(HALF, unit="ns")
        self.set_scl(1)
        await Timer(HALF, unit="ns")
        return val

    async def mmio_read(self, addr):
        await self.start()
        await self.write_byte(0x84)  # 0x42 << 1 | write
        await self.write_byte((addr >> 8) & 0xFF)
        await self.write_byte(addr & 0xFF)
        await self.start()           # repeated START
        await self.write_byte(0x85)  # 0x42 << 1 | read
        val = 0
        for i in range(4):
            val = (val << 8) | await self.read_byte(ack=(i < 3))
        await self.stop()
        await Timer(100, unit="ns")
        return val


async def reset(dut):
    dut.rst_n.value = 0
    dut.ena.value = 1
    dut.ui_in.value = 1  # scl idle high
    dut.uio_in.value = 0
    await Timer(100, unit="ns")
    dut.rst_n.value = 1
    await Timer(100, unit="ns")


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)
    return I2CDriver(dut)


@cocotb.test()
async def test_reset_sanity(dut):
    """After reset: LED off, CSR=0, PC=0, R0=0, unmapped reads 0xDEADBEEF."""
    i2c = await setup(dut)

    assert int(dut.uo_out.value) == 0, "led should be 0 after reset"
    assert await i2c.mmio_read(0x3000) == 0, "CSR should be 0 after reset"
    assert await i2c.mmio_read(0x3008) == 0, "PC should be 0 after reset"
    assert await i2c.mmio_read(0x1000) == 0, "R0 should read 0"
    assert await i2c.mmio_read(0x4000) == 0xDEADBEEF, "unmapped read"
    assert await i2c.mmio_read(0x1040) == 0xDEADBEEF, "only 16 registers are mapped"


@cocotb.test()
async def test_readback_while_halted(dut):
    """Write then read back one word of every region while RUN=0."""
    i2c = await setup(dut)

    for addr, data in [(0x000C, 0x12345678),   # IMEM[3]
                       (0x101C, 0xCAFEF00D),   # R7
                       (0x2014, 0xA5A5A5A5),   # DMEM[5]
                       (0x3004, 0x00000020)]:  # TARGET_PC
        await i2c.mmio_write(addr, data)
        got = await i2c.mmio_read(addr)
        assert got == data, f"readback {addr:#06x}: got {got:#010x}, want {data:#010x}"

    await i2c.mmio_write(0x1000, 0xFFFFFFFF)
    assert await i2c.mmio_read(0x1000) == 0, "R0 must stay 0"


@cocotb.test()
async def test_load_run_halt(dut):
    """End-to-end: load a 2-instruction program over I2C, run it, check
    the results by I2C readback after DONE."""
    i2c = await setup(dut)

    # ADDI R1,$0,5   -> R1 = 5
    # SW   R1,0($0)  -> DM[0] = 5
    await i2c.mmio_write(0x0000, 0x08010005)
    await i2c.mmio_write(0x0004, 0x10010000)
    await i2c.mmio_write(0x3004, 0x00000008)  # target_pc
    await i2c.mmio_write(0x3000, 0x00000001)  # RUN=1

    max_cycles = 2000
    n = 0
    while (int(dut.uo_out.value) & 1) == 0 and n < max_cycles:
        await ClockCycles(dut.clk, 1)
        n += 1

    assert int(dut.uo_out.value) & 1 == 1, f"led (DONE) never went high within {max_cycles} cycles"
    assert await i2c.mmio_read(0x3000) == 0x3, "CSR should be RUN|DONE"
    assert await i2c.mmio_read(0x3008) == 0x8, "PC should be held at TARGET_PC"
    assert await i2c.mmio_read(0x1004) == 5, "R1 should be 5"
    assert await i2c.mmio_read(0x2000) == 5, "DM[0] should be 5"


@cocotb.test()
async def test_branch_loop(dut):
    """Countdown loop: taken and not-taken beq, forwarding, R2 = 3+2+1."""
    i2c = await setup(dut)

    prog = [
        0x08010003,  # 0x00 ADDI R1,R0,3
        0x08020000,  # 0x04 ADDI R2,R0,0
        0x00411000,  # 0x08 loop: ADD R2,R2,R1
        0x0821FFFF,  # 0x0C ADDI R1,R1,-1
        0x14200001,  # 0x10 BEQ R1,R0,+1  -> 0x18 when R1==0
        0x1400FFFC,  # 0x14 BEQ R0,R0,-4  -> 0x08 (always taken)
        0x10020001,  # 0x18 SW R2,1(R0)   -> DM[1] = R2
    ]
    for k, word in enumerate(prog):
        await i2c.mmio_write(4 * k, word)
    await i2c.mmio_write(0x3004, 4 * len(prog))  # target_pc
    await i2c.mmio_write(0x3000, 0x00000001)     # RUN=1

    n = 0
    while (int(dut.uo_out.value) & 1) == 0 and n < 2000:
        await ClockCycles(dut.clk, 1)
        n += 1

    assert int(dut.uo_out.value) & 1 == 1, "led (DONE) never went high"
    assert await i2c.mmio_read(0x1004) == 0, "R1 should count down to 0"
    assert await i2c.mmio_read(0x1008) == 6, "R2 should be 3+2+1"
    assert await i2c.mmio_read(0x2004) == 6, "DM[1] should be 6"
