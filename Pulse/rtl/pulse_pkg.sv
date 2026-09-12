// Copyright (c) 2026 Henrry
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v2.1 (the "License"); you may not use
// this file except in compliance with the License, or, at your option, the Apache
// License version 2.0. You may obtain a copy of the License at
//
//     https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under
// the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
// ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

// pulse_pkg — shared RV32IMAC opcode constants and decode-side enums.
//
// Consumed by pulse_c_expander (opcode constants only, to build expanded 32-bit
// instructions) and pulse_idu (both). Kept in one place so the EX/MEM/WB stages
// that eventually consume pulse_idu's outputs share the same type definitions
// instead of re-declaring them.

package pulse_pkg;

  // --------------------------------------------------------------------------
  // RV32 base opcode map (instr[6:0]); C-extension instructions never reach
  // this list directly — pulse_c_expander turns them into one of these first.
  // --------------------------------------------------------------------------
  localparam logic [6:0] OPC_LOAD      = 7'b0000011;
  localparam logic [6:0] OPC_MISC_MEM  = 7'b0001111;  // FENCE / FENCE.I
  localparam logic [6:0] OPC_OP_IMM    = 7'b0010011;
  localparam logic [6:0] OPC_AUIPC     = 7'b0010111;
  localparam logic [6:0] OPC_STORE     = 7'b0100011;
  localparam logic [6:0] OPC_AMO       = 7'b0101111;  // A extension
  localparam logic [6:0] OPC_OP        = 7'b0110011;  // also carries M extension (funct7=7)
  localparam logic [6:0] OPC_LUI       = 7'b0110111;
  localparam logic [6:0] OPC_BRANCH    = 7'b1100011;
  localparam logic [6:0] OPC_JALR      = 7'b1100111;
  localparam logic [6:0] OPC_JAL       = 7'b1101111;
  localparam logic [6:0] OPC_SYSTEM    = 7'b1110011;  // ECALL/EBREAK/MRET/WFI + Zicsr

  // --------------------------------------------------------------------------
  // Decode -> EX control enums.
  // --------------------------------------------------------------------------

  // ALU operation.
  typedef enum logic [3:0] {
    ALU_ADD, ALU_SUB, ALU_SLL, ALU_SLT, ALU_SLTU,
    ALU_XOR, ALU_SRL, ALU_SRA, ALU_OR,  ALU_AND
  } alu_op_e;

  // ALU operand-A / operand-B source muxes.
  typedef enum logic [1:0] { OP_A_RS1, OP_A_PC,  OP_A_ZERO } alu_a_sel_e;
  typedef enum logic [1:0] { OP_B_RS2, OP_B_IMM, OP_B_FOUR } alu_b_sel_e;

  // Register write-back source mux.
  typedef enum logic [1:0] { WB_EX, WB_MEM, WB_PC4, WB_CSR } wb_sel_e;

  // Branch comparison op — deliberately numbered to match BRANCH-opcode funct3
  // directly, so branch_op_o can be cast straight from instr[14:12].
  typedef enum logic [2:0] {
    BR_EQ  = 3'b000, BR_NE  = 3'b001,
    BR_RSVD0 = 3'b010, BR_RSVD1 = 3'b011,
    BR_LT  = 3'b100, BR_GE  = 3'b101,
    BR_LTU = 3'b110, BR_GEU = 3'b111
  } branch_op_e;

  // Load/store transfer size (byte/half/word); sign-extension on loads is a
  // separate mem_sign_ext_o bit since stores never sign-extend.
  typedef enum logic [1:0] { MEM_B, MEM_H, MEM_W } mem_size_e;

  // M-extension op — numbered to match OP-opcode funct3 directly (valid only
  // when funct7 == 7'b0000001, signalled separately via mul_div_o).
  typedef enum logic [2:0] {
    MULDIV_MUL,  MULDIV_MULH, MULDIV_MULHSU, MULDIV_MULHU,
    MULDIV_DIV,  MULDIV_DIVU, MULDIV_REM,    MULDIV_REMU
  } muldiv_op_e;

  // A-extension AMO op, decoded from funct5 (instr[31:27]). RV32A is word-only
  // (funct3 must be 3'b010); there is no separate 64-bit variant to encode.
  typedef enum logic [3:0] {
    AMO_LR, AMO_SC, AMO_SWAP, AMO_ADD, AMO_XOR, AMO_AND, AMO_OR,
    AMO_MIN, AMO_MAX, AMO_MINU, AMO_MAXU
  } amo_op_e;

  // Zicsr access op — numbered to match SYSTEM-opcode funct3 directly.
  // Bundled in here because a core can't run real firmware without CSR access
  // even though Zicsr is technically its own extension letter.
  typedef enum logic [2:0] {
    CSR_RSVD0 = 3'b000,
    CSR_RW    = 3'b001, CSR_RS  = 3'b010, CSR_RC  = 3'b011,
    CSR_RSVD1 = 3'b100,
    CSR_RWI   = 3'b101, CSR_RSI = 3'b110, CSR_RCI = 3'b111
  } csr_op_e;

  // --------------------------------------------------------------------------
  // Pipeline registers (pulse_core). Only `rd`/`rf_we` are carried for hazard
  // purposes — the RAW check itself compares a live rs1/rs2 (from the
  // in-flight decode) against these, so rs1/rs2 addresses of the producer
  // instructions don't need to ride along here at all.
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] pc_plus;
    logic [4:0]  rd;
    logic [31:0] rs1_data;
    logic [31:0] rs2_data;
    logic [31:0] imm;
    alu_op_e     alu_op;
    alu_a_sel_e  alu_a_sel;
    alu_b_sel_e  alu_b_sel;
    wb_sel_e     wb_sel;
    logic        rf_we;
    logic        mem_req;
    logic        mem_we;
    mem_size_e   mem_size;
    logic        mem_sign_ext;
    logic        branch;
    logic        jal;
    logic        jalr;
    branch_op_e  branch_op;
    logic        mul_div;
    muldiv_op_e  muldiv_op;
    logic        atomic;
    amo_op_e     amo_op;
    logic        aq;
    logic        rl;
    logic        illegal_instr;
    logic        ecall;
    logic        ebreak;
    logic        mret;
    logic        wfi;
  } id_ex_t;

  localparam id_ex_t ID_EX_BUBBLE = '{
    valid: 1'b0, pc: 32'b0, pc_plus: 32'b0, rd: 5'b0,
    rs1_data: 32'b0, rs2_data: 32'b0, imm: 32'b0,
    alu_op: ALU_ADD, alu_a_sel: OP_A_RS1, alu_b_sel: OP_B_IMM,
    wb_sel: WB_EX, rf_we: 1'b0,
    mem_req: 1'b0, mem_we: 1'b0, mem_size: MEM_W, mem_sign_ext: 1'b0,
    branch: 1'b0, jal: 1'b0, jalr: 1'b0, branch_op: BR_EQ,
    mul_div: 1'b0, muldiv_op: MULDIV_MUL,
    atomic: 1'b0, amo_op: AMO_ADD, aq: 1'b0, rl: 1'b0,
    illegal_instr: 1'b0, ecall: 1'b0, ebreak: 1'b0, mret: 1'b0, wfi: 1'b0
  };

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [31:0] pc_plus;
    logic [4:0]  rd;
    logic [31:0] rs2_data;   // store data / AMO operand, passthrough from ID/EX
    logic [31:0] ex_result;  // ALU/muldiv result; also the LSU address for mem ops
    wb_sel_e     wb_sel;
    logic        rf_we;
    logic        mem_req;
    logic        mem_we;
    mem_size_e   mem_size;
    logic        mem_sign_ext;
    logic        atomic;
    amo_op_e     amo_op;
    logic        aq;
    logic        rl;
    logic        illegal_instr;
    logic        ecall;
    logic        ebreak;
    logic        mret;
    logic        wfi;
  } ex_mem_t;

  localparam ex_mem_t EX_MEM_BUBBLE = '{
    valid: 1'b0, pc: 32'b0, pc_plus: 32'b0, rd: 5'b0,
    rs2_data: 32'b0, ex_result: 32'b0,
    wb_sel: WB_EX, rf_we: 1'b0,
    mem_req: 1'b0, mem_we: 1'b0, mem_size: MEM_W, mem_sign_ext: 1'b0,
    atomic: 1'b0, amo_op: AMO_ADD, aq: 1'b0, rl: 1'b0,
    illegal_instr: 1'b0, ecall: 1'b0, ebreak: 1'b0, mret: 1'b0, wfi: 1'b0
  };

  typedef struct packed {
    logic        valid;
    logic [31:0] pc;
    logic [4:0]  rd;
    logic        rf_we;
    wb_sel_e     wb_sel;
    logic [31:0] ex_result;
    logic [31:0] mem_result;
    logic [31:0] pc_plus;
    logic        illegal_instr;
    logic        ecall;
    logic        ebreak;
    logic        mret;
    logic        wfi;
  } mem_wb_t;

  localparam mem_wb_t MEM_WB_BUBBLE = '{
    valid: 1'b0, pc: 32'b0, rd: 5'b0, rf_we: 1'b0, wb_sel: WB_EX,
    ex_result: 32'b0, mem_result: 32'b0, pc_plus: 32'b0,
    illegal_instr: 1'b0, ecall: 1'b0, ebreak: 1'b0, mret: 1'b0, wfi: 1'b0
  };

endpackage : pulse_pkg
