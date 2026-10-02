`timescale 1ns / 1ps

// 32 x 32 register file. Single shared write port (I2C programming wins over
// the CPU write-back when both fire in the same cycle; r0 is never written by
// either). Sharing one port instead of two saves a 2:1 mux per stored bit.
// Only r1-r31 are stored; r0 always reads 0. No reset (saves a gate per flop):
// registers are undefined until written, so programs must initialise every
// register they read (over I2C or by an instruction). rst is unused.
module reg_file(
    input        clk,
    input        rst,
    input [4:0]  rs,
    input [4:0]  rt,
    input [4:0]  rd,
    input [31:0] write_data,
    input        Reg_write,
    output [31:0] D_1,
    output [31:0] D_2,
    input        halted,     // RUN=0 or DONE: rs port serves I2C readback
    input        prog_we,
    input [4:0]  prog_addr,
    input [31:0] prog_wdata,
    output [31:0] prog_rdata
);
    reg [31:0] RF [31:1];

    // While halted the pipeline is frozen, so the rs port is free: it reads
    // prog_addr for I2C readback instead of a third read-mux tree. Readback
    // is only valid while halted.
    wire [4:0] ra = halted ? prog_addr : rs;

    assign D_1 = (ra == 5'd0) ? 32'd0 :
                 (Reg_write && (rd == ra)) ? write_data : RF[ra];
    assign D_2 = (rt == 5'd0) ? 32'd0 :
                 (Reg_write && (rd == rt)) ? write_data : RF[rt];

    assign prog_rdata = D_1;

    wire        sel_prog = prog_we && (prog_addr != 5'd0);
    wire        wr_en    = sel_prog || (Reg_write && (rd != 5'd0));
    wire [4:0]  wr_addr  = sel_prog ? prog_addr  : rd;
    wire [31:0] wr_data  = sel_prog ? prog_wdata : write_data;

    always @(posedge clk) begin
        if (wr_en)
            RF[wr_addr] <= wr_data;
    end
endmodule
