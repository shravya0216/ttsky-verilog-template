# 5-Stage Pipelined MIPS with I2C/MMIO Loader

## How it works 

This is a 5-stage pipelined MIPS-style CPU (fetch, decode, execute, memory,
writeback) with:
- Full data-hazard forwarding (EX/MEM and MEM/WB paths, including a
  write-first bypass in the register file for the one same-cycle
  WB-write/ID-read case not covered by pipeline forwarding)
- Load-use hazard detection with automatic pipeline stalling
- Static not-taken branch prediction; a taken `beq` is resolved in EX and
  the pipeline is flushed and redirected

Because the on-chip instruction memory, data memory, and register file are
all normally fixed at synthesis time, this design adds a memory-mapped I2C
loader so the entire program state can be reprogrammed after the chip is
fabricated:

| MMIO Address Range | Region |
|---|---|
| 0x0000 - 0x003C | Instruction memory (16 words) |
| 0x1000 - 0x103C | Register file (16 registers, R0 reads 0) |
| 0x2000 - 0x201C | Data memory (8 words) |
| 0x3000 | CSR (bit 0 = RUN, bit 1 = DONE) |
| 0x3004 | TARGET_PC (halt address) |
| 0x3008 | Live PC (read-only) |

Any other address reads 0xDEADBEEF. Instruction memory, register file and
data memory read back correctly only while halted (`RUN=0` or `DONE=1`).
Register fields are still 5 bits wide, but only the low 4 bits are used:
r16-r31 alias r0-r15. Registers are not reset, so a program must initialise
every register it reads.

The pipeline is held frozen while `RUN=0`. Writing `RUN=1` releases it to
execute from address 0. Once the program counter reaches `TARGET_PC`, fetch
is blocked, in-flight instructions are allowed to drain to completion, and
`DONE` asserts (driven out on the `led` pin).

All memories are plain flip-flop arrays (there are no SRAM macros that fit
a Tiny Tapeout tile), sized down from the original 256-word version to fit
the tile area budget.

## How to test

1. Hold reset (`rst_n` low), then release it.
2. Over I2C (7-bit slave address `0x42`), send a sequence of 7-byte write
   frames -- `[CMD][ADDR_HI][ADDR_LO][D3][D2][D1][D0]` -- to load your
   program into instruction memory, optionally preload data memory or
   register file values, and set `TARGET_PC`.
3. Write `CSR = 0x00000001` to release the pipeline and start execution.
4. Poll the `led` pin (or watch for it going high) to know when execution
   has completed.

A single 32-bit MMIO write takes 63 SCL clocks (9 for the address+ACK, 18
for the 16-bit MMIO address, 36 for the 32-bit data, all including ACKs).

Note: `TARGET_PC` must be an address your program's real (taken) control
flow actually revisits -- if a branch skips over it entirely, the design
will wait indefinitely for `DONE` rather than corrupting anything.

## External hardware

None required beyond an I2C master (e.g. a microcontroller) wired to
`ui_in[0]` (SCL) and `uio[0]` (SDA, open-drain, needs a pull-up if your
setup doesn't already provide one).
