`timescale 1ns / 1ps

// Word-organised instruction memory: 2**IM_AW 32-bit words, plain flip-flops.
// (Tiny Tapeout sky130 has no SRAM macro, so every bit costs area; the depth
// is a parameter so it can be sized to the tile. Default 16 words = 512 bits.)
//
// CPU fetch keeps the byte-addressed PC convention: the word index is
// PC[IM_AW+1:2] (PC is always word aligned). PC bits above that are ignored,
// so a program must not run past the last word without TARGET_PC stopping it
// first (the address wraps otherwise).
// Programming writes one 32-bit instruction at a time (MSB byte first on the
// I2C wire, stored as a single word here). No reset: contents are undefined
// until programmed over I2C; fetch is frozen until RUN=1.
module instruction_memory (
    input        rst,
    input        clk,
    input [31:0] PC_out,
    output [31:0] instruction_code,
    input        halted,     // RUN=0 or DONE: fetch port serves I2C readback
    input        prog_we,
    input [5:0]  prog_addr,
    input [31:0] prog_wdata,
    output [31:0] prog_rdata
);
    localparam IM_AW    = 4;    // 2**IM_AW = 16 words. Must match mmio_decoder.v's IM_AW.
    localparam IM_WORDS = 1 << IM_AW;

    reg [31:0] IM [IM_WORDS-1:0];

    // While halted, PC and IF/ID are held, so the fetched word is ignored and
    // the fetch port can read prog_addr for I2C readback (valid only while halted).
    assign instruction_code = IM[halted ? prog_addr[IM_AW-1:0] : PC_out[IM_AW+1:2]];

    assign prog_rdata = instruction_code;

    always @(posedge clk) begin
        if (prog_we)
            IM[prog_addr[IM_AW-1:0]] <= prog_wdata;
    end
endmodule
