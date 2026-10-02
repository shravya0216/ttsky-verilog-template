`timescale 1ns / 1ps

// 32 x 32 register file. Single shared write port (I2C programming wins over
// the CPU write-back when both fire in the same cycle; r0 is never written by
// either). Sharing one port instead of two saves a 2:1 mux per stored bit.
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
    integer i;
    reg [31:0] RF [31:0];

    // While halted the pipeline is frozen, so the rs port is free: it reads
    // prog_addr for I2C readback instead of a third read-mux tree. Readback
    // is only valid while halted.
    wire [4:0] ra = halted ? prog_addr : rs;

    assign D_1 = (Reg_write && (rd == ra) && (rd != 5'd0)) ? write_data : RF[ra];
    assign D_2 = (Reg_write && (rd == rt) && (rd != 5'd0)) ? write_data : RF[rt];

    assign prog_rdata = D_1;

    wire        sel_prog = prog_we && (prog_addr != 5'd0);
    wire        wr_en    = sel_prog || (Reg_write && (rd != 5'd0));
    wire [4:0]  wr_addr  = sel_prog ? prog_addr  : rd;
    wire [31:0] wr_data  = sel_prog ? prog_wdata : write_data;

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i <= 31; i = i + 1)
                RF[i] <= i;
        end else if (wr_en) begin
            RF[wr_addr] <= wr_data;
        end
    end
endmodule
