`timescale 1ns / 1ps

// MIPS + I2C programming wrapper.
// External protocol:
//   MCU writes IMEM/REGFILE/DMEM/TARGET_PC while CSR.RUN=0.
//   MCU writes CSR.RUN=1.
//   CPU executes normally until the next PC reaches TARGET_PC.
//   Fetch is then blocked, the existing pipeline drains, and DONE goes high.
//   MCU can read back any of those addresses at any time, plus the live PC
//   at 0x3008, with a repeated-START I2C read.
//
// Memories sized for the Tiny Tapeout sky130 area budget (flip-flop arrays,
// no SRAM macros). Depths are log2 parameters:
//   IMEM: 2**IM_AW 32-bit words  (default IM_AW=4 -> 16 words = 512 bits)
//   DMEM: 2**DM_AW 32-bit words  (default DM_AW=3 ->  8 words = 256 bits)
//   REGFILE: 16 x 32-bit words (r0 hardwired 0; r16-r31 alias r0-r15)
// (IM_AW=6, DM_AW=6 restores the old 64/64 sizes but does NOT fit any TT tile.)
module top (
    input  clk,
    input  rst,
    input  scl,
    input  sda_in,
    output sda_out,
    output sda_oe,
    output reg led
);
    wire PC_flush;
    wire [31:0] PC_out;
    wire [31:0] D_1, D_2;
    wire [3:0] EX_w;
    wire [1:0] M_w, WB_w;
    wire [31:0] immediate_w;

    wire [3:0] ID_EX_EX;
    wire [1:0] ID_EX_M, ID_EX_WB;
    wire [4:0] ID_EX_rs, ID_EX_rt;
    wire [31:0] ID_EX_D_1, ID_EX_D_2;
    wire ID_EX_comparator;
    wire [4:0] ID_EX_rd_maybe;
    wire [31:0] ID_EX_immediate;
    wire [31:0] ID_EX_PC_out;
    wire [3:0] ID_EX_BHT_rd_addr;
    wire ID_EX_flush, ID_EX_stall;

    wire [1:0] forward_rs_w, forward_rt_w;
    wire [31:0] operand_1_w, rt_f_w, operand_2_w;
    wire [31:0] R_w;
    wire zero_w;
    wire [4:0] rd_w;

    wire [1:0] EX_MEM_M, EX_MEM_WB;
    wire [4:0] EX_MEM_rs, EX_MEM_rt;
    wire [31:0] EX_MEM_R, EX_MEM_rt_f;
    wire [4:0] EX_MEM_rd;
    wire [31:0] mem_rd_data_w;
    wire [1:0] MEM_WB_WB;
    wire [31:0] MEM_WB_Rd_data, MEM_WB_R;
    wire [4:0] MEM_WB_rd;
    wire [31:0] write_data_w;

    // ================================================================
    // I2C / MMIO
    // ================================================================
    wire        mmio_wr;
    wire [15:0] mmio_addr;
    wire [31:0] mmio_wdata;

    wire        imem_prog_we;
    wire [5:0]  imem_prog_addr;
    wire [31:0] imem_prog_wdata;
    wire        regfile_prog_we;
    wire [4:0]  regfile_prog_addr;
    wire [31:0] regfile_prog_wdata;
    wire        dmem_prog_we;
    wire [5:0]  dmem_prog_addr;
    wire [31:0] dmem_prog_wdata;
    wire [7:0]  csr;
    wire [31:0] target_pc;
    wire [31:0] mmio_rdata;
    wire [31:0] imem_prog_rdata, regfile_prog_rdata, dmem_prog_rdata;
    reg done;
    reg [2:0] drain_count;

    wire IF_ID_comparator;
    wire run_req = csr[0];

    // TARGET_PC is the first instruction that must NOT execute.
    // Once PC_out reaches it, insert a NOP into IF/ID and allow the
    // already-fetched instructions to move through EX/MEM/WB.
    //
    // A beq resolves in EX, two cycles after it is fetched. While a beq is
    // still unresolved in ID, PC_out is only a PREDICTED successor, so a match
    // with TARGET_PC there may be a wrong-path fetch. In that case fetch is held
    // (bubble into IF/ID) but the drain is NOT started ("target_defer"); one cycle
    // later the branch is in EX and either PC_flush redirects the PC to the true
    // successor (run continues) or the true successor really is TARGET_PC and the
    // drain starts normally. Without this, a loop that ends on its back-branch
    // (TARGET_PC = branch + 4) halted early whenever the branch was mispredicted.
    wire target_match   = run_req && !done && (PC_out == target_pc) && !PC_flush;
    wire branch_in_id   = (IF_ID_comparator == 1'b0);
    wire target_reached = target_match && !branch_in_id;
    wire target_defer   = target_match &&  branch_in_id;
    wire drain_active = (drain_count != 3'd0);

    // LOAD/HALT: freeze every pipeline register.
    // DRAIN: stop fetching new instructions, but allow existing pipeline
    // contents to advance normally.
    wire pipeline_freeze = !run_req || done;
    wire stop_fetch = target_reached || target_defer || drain_active;
    wire pc_hold = pipeline_freeze || stop_fetch;

    // ================================================================
    // FETCH STAGE WIRES
    // ================================================================
    wire [31:0] PC_in;
    wire [31:0] instruction_code;
    wire comparator_w;
    wire PC_stall;
    wire [31:0] PC_calculated;

    wire [31:0] IF_ID_instruction_code;
    wire [31:0] IF_ID_PC_out;
    wire [3:0] IF_ID_BHT_rd_addr;
    wire IF_ID_stall, IF_ID_flush;

    // 16 registers: only the low 4 bits of each 5-bit register field are used,
    // so r16-r31 alias r0-r15 (r16 reads 0). Truncating HERE, at decode, keeps
    // the regfile, forwarding and stall comparisons consistent.
    wire [4:0] rs_w = {1'b0, IF_ID_instruction_code[24:21]};
    wire [4:0] rt_w = {1'b0, IF_ID_instruction_code[19:16]};
    wire [4:0] rd_maybe_w = {1'b0, IF_ID_instruction_code[14:11]};
    wire [15:0] imm_w = IF_ID_instruction_code[15:0];
    wire [5:0] opcode_w = IF_ID_instruction_code[31:26];

    i2c_slave_mmio #(.SLAVE_ADDR(7'h42)) I2C_inst(
        .clk(clk), .rst(rst), .scl(scl),
        .sda_in(sda_in), .sda_out(sda_out), .sda_oe(sda_oe),
        .mmio_wr(mmio_wr), .mmio_addr(mmio_addr), .mmio_wdata(mmio_wdata),
        .mmio_rdata(mmio_rdata)
    );

    mmio_decoder MMIO_inst(
        .clk(clk), .rst(rst), .mmio_wr(mmio_wr),
        .mmio_addr(mmio_addr), .mmio_wdata(mmio_wdata), .done(done),
        .imem_prog_we(imem_prog_we), .imem_prog_addr(imem_prog_addr),
        .imem_prog_wdata(imem_prog_wdata),
        .regfile_prog_we(regfile_prog_we), .regfile_prog_addr(regfile_prog_addr),
        .regfile_prog_wdata(regfile_prog_wdata),
        .dmem_prog_we(dmem_prog_we), .dmem_prog_addr(dmem_prog_addr),
        .dmem_prog_wdata(dmem_prog_wdata),
        .csr(csr), .target_pc(target_pc),
        .imem_rdata(imem_prog_rdata), .regfile_rdata(regfile_prog_rdata),
        .dmem_rdata(dmem_prog_rdata), .pc(PC_out), .mmio_rdata(mmio_rdata)
    );

    Pc PC_inst(
        .clk(clk), .PC_flush(PC_flush), .PC_stall(PC_stall),
        .PC_in(PC_in), .PC_calculated(PC_calculated),
        .hold(pc_hold), .rst(rst), .PC_out(PC_out)
    );

    instruction_memory IM_inst(
        .rst(rst), .clk(clk), .PC_out(PC_out),
        .instruction_code(instruction_code),
        .prog_we(imem_prog_we), .prog_addr(imem_prog_addr),
        .prog_wdata(imem_prog_wdata), .halted(pipeline_freeze), .prog_rdata(imem_prog_rdata)
    );

    comparator COMP_inst(.instruction_code(instruction_code), .comparator(comparator_w));

    // Static not-taken branch prediction (PHT/BHT/BTB removed for area).
    // A taken beq is caught in EX by Flushing_unit (PC_calculated !=
    // IF_ID_PC_out) and redirected, costing 2 bubbles; results are unchanged.
    assign PC_in = PC_out + 32'd4;

    IF_ID_register IF_ID_inst(
        .clk(clk), .instruction_code(instruction_code), .comparator(comparator_w),
        .PC_out(PC_out), .BHT_rd_addr(4'b0), .IF_ID_stall(IF_ID_stall),
        .IF_ID_flush(IF_ID_flush), .rst(rst), .freeze(pipeline_freeze), .stop_fetch(stop_fetch),
        .IF_ID_instruction_code(IF_ID_instruction_code), .IF_ID_comparator(IF_ID_comparator),
        .IF_ID_PC_out(IF_ID_PC_out), .IF_ID_BHT_rd_addr(IF_ID_BHT_rd_addr)
    );

    reg_file REGFILE_inst(
        .clk(clk), .rst(rst), .rs(rs_w), .rt(rt_w), .rd(MEM_WB_rd),
        .write_data(write_data_w), .Reg_write(MEM_WB_WB[1]), .D_1(D_1), .D_2(D_2),
        .prog_we(regfile_prog_we), .prog_addr(regfile_prog_addr), .prog_wdata(regfile_prog_wdata),
        .halted(pipeline_freeze), .prog_rdata(regfile_prog_rdata)
    );

    control_unit CU_inst(.rst(rst), .opcode(opcode_w), .EX(EX_w), .M(M_w), .WB(WB_w));
    Sign_extender SE_inst(.imm(imm_w), .immediate(immediate_w));

    ID_EX_register ID_EX_inst(
        .rst(rst), .clk(clk), .ID_EX_flush(ID_EX_flush), .ID_EX_stall(ID_EX_stall), .freeze(pipeline_freeze),
        .EX(EX_w), .M(M_w), .WB(WB_w), .rs(rs_w), .rt(rt_w), .D_1(D_1), .D_2(D_2),
        .comparator(IF_ID_comparator), .rd_maybe(rd_maybe_w), .immediate(immediate_w),
        .PC_out(IF_ID_PC_out), .BHT_rd_addr(IF_ID_BHT_rd_addr), .ID_EX_EX(ID_EX_EX),
        .ID_EX_M(ID_EX_M), .ID_EX_WB(ID_EX_WB), .ID_EX_rs(ID_EX_rs), .ID_EX_rt(ID_EX_rt),
        .ID_EX_D_1(ID_EX_D_1), .ID_EX_D_2(ID_EX_D_2), .ID_EX_comparator(ID_EX_comparator),
        .ID_EX_rd_maybe(ID_EX_rd_maybe), .ID_EX_immediate(ID_EX_immediate),
        .ID_EX_PC_out(ID_EX_PC_out), .ID_EX_BHT_rd_addr(ID_EX_BHT_rd_addr)
    );

    Forwarding_unit FWD_inst(
        .ID_EX_rs(ID_EX_rs), .ID_EX_rt(ID_EX_rt), .EX_MEM_rd(EX_MEM_rd), .MEM_WB_rd(MEM_WB_rd),
        .EX_MEM_Reg_write(EX_MEM_WB[1]), .MEM_WB_Reg_write(MEM_WB_WB[1]),
        .forward_rs(forward_rs_w), .forward_rt(forward_rt_w)
    );
    Forward_rs FWD_RS_inst(.forward_rs(forward_rs_w), .D_1(ID_EX_D_1), .Mem_WB_write_data(write_data_w), .EX_MEM_R(EX_MEM_R), .operand_1(operand_1_w));
    forward_rt FWD_RT_inst(.forward_rt(forward_rt_w), .D_2(ID_EX_D_2), .Mem_WB_write_data(write_data_w), .EX_MEM_R(EX_MEM_R), .rt_f(rt_f_w));
    mux_1_execution MUX1_EX_inst(.rt_f(rt_f_w), .ID_EX_immediate(ID_EX_immediate), .Alu_src(ID_EX_EX[0]), .operand_2(operand_2_w));
    ALU ALU_inst(.operand_1(operand_1_w), .operand_2(operand_2_w), .ALUOp(ID_EX_EX[2:1]), .R(R_w), .zero(zero_w));
    mux_2_execution MUX2_EX_inst(.zero(zero_w), .immediate(ID_EX_immediate), .PC_out(ID_EX_PC_out), .PC_calculated(PC_calculated));
    mux_3_execution MUX3_EX_inst(.ID_EX_rd_maybe(ID_EX_rd_maybe), .ID_EX_rt(ID_EX_rt), .Reg_dst(ID_EX_EX[3]), .rd(rd_w));
    Flushing_unit FLUSH_inst(.comparator(ID_EX_comparator), .IF_ID_PC_out(IF_ID_PC_out), .PC_calculated(PC_calculated), .IF_ID_flush(IF_ID_flush), .PC_flush(PC_flush), .ID_EX_flush(ID_EX_flush));
    stalling_unit STALL_inst(
        .IF_ID_IC(IF_ID_instruction_code), .rst(rst), .ID_EX_MEM_Rd(ID_EX_M[1]), .ID_EX_rt(ID_EX_rt),
        .IF_ID_rs(rs_w), .IF_ID_rt(rt_w), .IF_ID_stall(IF_ID_stall), .ID_EX_stall(ID_EX_stall), .PC_stall(PC_stall)
    );

    EX_MEM_Register EX_MEM_inst(
        .clk(clk), .rst(rst), .ID_EX_M(ID_EX_M), .ID_EX_WB(ID_EX_WB), .ID_EX_rs(ID_EX_rs),
        .ID_EX_rt(ID_EX_rt), .R(R_w), .rt_f(rt_f_w), .rd(rd_w), .freeze(pipeline_freeze), .EX_MEM_M(EX_MEM_M),
        .EX_MEM_WB(EX_MEM_WB), .EX_MEM_rs(EX_MEM_rs), .EX_MEM_rt(EX_MEM_rt),
        .EX_MEM_R(EX_MEM_R), .EX_MEM_rt_f(EX_MEM_rt_f), .EX_MEM_rd(EX_MEM_rd)
    );

    data_memory DM_inst(
        .clk(clk), .rst(rst), .Mem_rd(EX_MEM_M[1]), .Mem_write(EX_MEM_M[0]),
        .rd_addr(EX_MEM_R), .write_data(EX_MEM_rt_f), .rd_data(mem_rd_data_w),
        .prog_we(dmem_prog_we), .prog_addr(dmem_prog_addr), .prog_wdata(dmem_prog_wdata),
        .halted(pipeline_freeze), .prog_rdata(dmem_prog_rdata)
    );

    M_WB_Register MEM_WB_inst(
        .rst(rst), .clk(clk), .EX_MEM_WB(EX_MEM_WB), .Rd_data(mem_rd_data_w),
        .EX_MEM_R(EX_MEM_R), .EX_MEM_rd(EX_MEM_rd), .freeze(pipeline_freeze), .MEM_WB_WB(MEM_WB_WB),
        .MEM_WB_Rd_data(MEM_WB_Rd_data), .MEM_WB_R(MEM_WB_R), .MEM_WB_rd(MEM_WB_rd)
    );
    Write_back_mux WB_MUX_inst(.MEM_WB_Rd_data(MEM_WB_Rd_data), .MEM_WB_R(MEM_WB_R), .Mem_to_Reg(MEM_WB_WB[0]), .write_data(write_data_w));

    // Four clocks are enough for the 5-stage pipeline to drain after the
    // target fetch is replaced by a NOP. No new instruction enters IF/ID.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            drain_count <= 3'd0;
            done <= 1'b0;
            led <= 1'b0;
        end else begin
            if (!run_req) begin
                drain_count <= 3'd0;
                led <= 1'b0;
            end else if (!done) begin
                if (target_reached && drain_count == 3'd0)
                    drain_count <= 3'd4;
                else if (drain_count != 3'd0) begin
                    if (drain_count == 3'd1) begin
                        drain_count <= 3'd0;
                        done <= 1'b1;
                        led <= 1'b1;
                    end else begin
                        drain_count <= drain_count - 1'b1;
                    end
                end
            end
        end
    end
endmodule
