`timescale 1ns / 1ps

// 2**DM_AW x 32-bit data memory, plain flip-flops (default 8 words = 256 bits).
// CPU addressing is WORD-indexed: LW/SW address 3 accesses DM[3].
// Only the low DM_AW address bits are used, so LW/SW addresses must stay in
// 0 .. 2**DM_AW-1 (higher addresses alias/wrap).
module data_memory (
    input        clk,
    input        rst,
    input        Mem_rd,
    input        Mem_write,
    input [31:0] rd_addr,
    input [31:0] write_data,
    output reg [31:0] rd_data,
    input        halted,     // RUN=0 or DONE: read port serves I2C readback
    input        prog_we,
    input [5:0]  prog_addr,
    input [31:0] prog_wdata,
    output [31:0] prog_rdata
);
    localparam DM_AW     = 3;    // 2**DM_AW = 8 words. Must match mmio_decoder.v's DM_AW.
    localparam DM_WORDS = 1 << DM_AW;

    integer i;
    reg [31:0] DM [DM_WORDS-1:0];

    // While halted the pipeline is frozen, so the CPU read port reads
    // prog_addr for I2C readback (valid only while halted). Only the READ
    // address is switched: a frozen EX/MEM can still hold Mem_write=1.
    assign prog_rdata = DM[halted ? prog_addr[DM_AW-1:0] : rd_addr[DM_AW-1:0]];

    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DM_WORDS; i = i + 1)
                DM[i] <= 32'd0;
        end else begin
            if (Mem_write)
                DM[rd_addr[DM_AW-1:0]] <= write_data;
            if (prog_we)
                DM[prog_addr[DM_AW-1:0]] <= prog_wdata;
        end
    end

    always @(*) begin
        if (Mem_rd)
            rd_data = prog_rdata;
        else
            rd_data = 32'b0;
    end
endmodule
