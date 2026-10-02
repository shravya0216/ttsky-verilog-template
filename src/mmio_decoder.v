`timescale 1ns / 1ps

// MMIO map (IMEM/DMEM depth set by IM_AW / DM_AW; defaults 16 / 8 words):
//   0x0000 .. 0x0000+4*(2**IM_AW)-4 : IMEM,    2**IM_AW x 32-bit words
//   0x1000-0x103C : REGFILE, R0-R15
//   0x2000 .. 0x2000+4*(2**DM_AW)-4 : DMEM,    2**DM_AW x 32-bit words
// Word addresses beyond the implemented depth are UNMAPPED: writes are
// ignored and reads return 0xDEADBEEF (they never alias onto real words).
//   0x3000        : CSR      (bit0 RUN, bit1 DONE)
//   0x3004        : TARGET_PC
//   0x3008        : live PC (read-only)
module mmio_decoder (
    input        clk,
    input        rst,
    input        mmio_wr,
    input [15:0] mmio_addr,
    input [31:0] mmio_wdata,
    input        done,
    output       imem_prog_we,
    output [5:0] imem_prog_addr,
    output [31:0] imem_prog_wdata,
    output       regfile_prog_we,
    output [4:0] regfile_prog_addr,
    output [31:0] regfile_prog_wdata,
    output       dmem_prog_we,
    output [5:0] dmem_prog_addr,
    output [31:0] dmem_prog_wdata,
    output reg [7:0] csr,
    output reg [31:0] target_pc,
    // I2C readback: every writable address reads back its current value,
    // plus the live PC at 0x3008. Unmapped addresses read 0xDEADBEEF.
    input [31:0] imem_rdata,
    input [31:0] regfile_rdata,
    input [31:0] dmem_rdata,
    input [31:0] pc,
    output reg [31:0] mmio_rdata
);

    // Region hits (word index must be inside the implemented depth).
    localparam IM_AW = 4;   // must match instruction_memory.v's IM_AW
    localparam DM_AW = 3;   // must match data_memory.v's DM_AW
    wire imem_hit = (mmio_addr[15:8] == 8'h00) && (mmio_addr[7:2] < (1 << IM_AW));
    wire dmem_hit = (mmio_addr[15:8] == 8'h20) && (mmio_addr[7:2] < (1 << DM_AW));

    // IMEM
    assign imem_prog_we    = mmio_wr && imem_hit &&
                             (mmio_addr[1:0] == 2'b00);
    assign imem_prog_addr  = mmio_addr[7:2];
    assign imem_prog_wdata = mmio_wdata;

    // REGFILE: 0x1000-0x103C (16 x 4 bytes = 64 bytes)
    assign regfile_prog_we    = mmio_wr && (mmio_addr >= 16'h1000) &&
                                (mmio_addr <= 16'h103C) &&
                                (mmio_addr[1:0] == 2'b00);
    assign regfile_prog_addr  = mmio_addr[6:2];
    assign regfile_prog_wdata = mmio_wdata;

    // DMEM
    assign dmem_prog_we    = mmio_wr && dmem_hit &&
                             (mmio_addr[1:0] == 2'b00);
    assign dmem_prog_addr  = mmio_addr[7:2];
    assign dmem_prog_wdata = mmio_wdata;

    // The *_prog_addr outputs above already index the memories, so each
    // memory's rdata is the word at mmio_addr.
    always @(*) begin
        if (imem_hit)
            mmio_rdata = imem_rdata;
        else if ((mmio_addr >= 16'h1000) && (mmio_addr <= 16'h103C))
            mmio_rdata = regfile_rdata;
        else if (dmem_hit)
            mmio_rdata = dmem_rdata;
        else if (mmio_addr == 16'h3000)
            mmio_rdata = {30'd0, csr[1:0]};
        else if (mmio_addr == 16'h3004)
            mmio_rdata = target_pc;
        else if (mmio_addr == 16'h3008)
            mmio_rdata = pc;
        else
            mmio_rdata = 32'hDEADBEEF;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            csr       <= 8'h00;
            target_pc <= 32'h00000000;
        end else begin
            csr[1] <= done;
            if (mmio_wr && (mmio_addr == 16'h3000))
                csr[0] <= mmio_wdata[0];
            if (mmio_wr && (mmio_addr == 16'h3004))
                target_pc <= mmio_wdata;
        end
    end
endmodule
