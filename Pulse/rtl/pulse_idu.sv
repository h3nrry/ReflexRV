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

// pulse_idu — Instruction Decode Unit (ReflexRV-Pulse), RV32IMAC.
//
// Purely combinational: one instruction in, one fully-decoded control bundle
// out, same cycle. No ID/EX pipeline register lives in here — this module is
// the decode logic only, meant to sit behind whatever register the pipeline
// places between ID and EX (mirrors pulse_ifu not owning an IF/ID register
// either, since imem's own output register serves that role there).
//
// Compressed (C-extension) instructions are expanded internally via
// pulse_c_expander before any of the RV32I/M/A decode logic below ever runs,
// so everything past that point only ever sees a standard 32-bit instruction.
//
// Input contract / what's still missing upstream:
//   instr_i[1:0] == 2'b11  -> instr_i is a native 32-bit instruction.
//   instr_i[1:0] != 2'b11  -> instr_i[15:0] holds one compressed instruction,
//                             right-justified; instr_i[31:16] is ignored.
// Producing that from a raw fetch stream — picking the right halfword out of
// a wider fetch word, and reassembling a 32-bit instruction that straddles
// two fetch words — is the job of an instruction-realignment buffer in front
// of this stage. ifu.sv in its current form is a fixed PC+4 fetcher and does
// not implement one; that buffer (plus the resulting "PC advances by 2 or 4"
// change to next-PC logic) is out of scope here and needed before this module
// can run on a live C-extension fetch stream. is_compressed_o / pc_plus_o
// below are exposed specifically so that future buffer and the branch/link
// logic downstream don't have to re-derive them.
//
// Also out of scope: the ID/EX pipeline register, register-file read (rs1_o/
// rs2_o are addresses, not data), the actual ALU/multiplier/divider/AMO
// execute units, and branch/jump redirect into pulse_ifu — this module only
// produces the control signals those stages consume.

module pulse_idu
  import pulse_pkg::*;
(
  // From IFU / the instruction-realignment buffer described above.
  input  logic        instr_valid_i,
  input  logic [31:0] instr_i,
  input  logic [31:0] instr_pc_i,

  // Fetch/link bookkeeping.
  output logic         is_compressed_o,  // instr_i was a 16-bit encoding
  output logic [31:0]  pc_plus_o,        // instr_pc_i + (2 or 4): JAL/JALR link value

  // Register file read/write addresses (data comes from the register file).
  output logic [4:0]  rs1_o,
  output logic [4:0]  rs2_o,
  output logic [4:0]  rd_o,
  output logic         rf_we_o,

  // Immediate, already sign/zero-extended to 32 bits per the instruction's format.
  output logic [31:0] imm_o,

  // ALU control.
  output alu_op_e      alu_op_o,
  output alu_a_sel_e   alu_a_sel_o,
  output alu_b_sel_e   alu_b_sel_o,

  // Write-back source mux.
  output wb_sel_e       wb_sel_o,

  // Data memory control.
  output logic          mem_req_o,
  output logic          mem_we_o,
  output mem_size_e      mem_size_o,
  output logic          mem_sign_ext_o,

  // Control flow.
  output logic          branch_o,
  output logic          jal_o,
  output logic          jalr_o,
  output branch_op_e     branch_op_o,

  // M extension.
  output logic          mul_div_o,
  output muldiv_op_e     muldiv_op_o,

  // A extension.
  output logic          atomic_o,
  output amo_op_e        amo_op_o,
  output logic          aq_o,
  output logic          rl_o,

  // Zicsr / SYSTEM.
  output logic          csr_access_o,
  output csr_op_e        csr_op_o,
  output logic [11:0]   csr_addr_o,
  output logic          ecall_o,
  output logic          ebreak_o,
  output logic          mret_o,
  output logic          wfi_o,

  // Set on any encoding this core can't execute (reserved, unimplemented
  // extension, malformed compressed instruction, ...).
  output logic          illegal_instr_o
);

  // ---------------------------------------------------------------------
  // Compressed-instruction expansion.
  // ---------------------------------------------------------------------
  assign is_compressed_o = (instr_i[1:0] != 2'b11);
  assign pc_plus_o        = instr_pc_i + (is_compressed_o ? 32'd2 : 32'd4);

  logic [31:0] instr_exp;
  logic         c_illegal;

  pulse_c_expander u_c_expander (
    .c_instr_i (instr_i[15:0]),
    .instr_o   (instr_exp),
    .illegal_o (c_illegal)
  );

  logic [31:0] instr;  // the standard 32-bit instruction actually decoded below
  assign instr = is_compressed_o ? instr_exp : instr_i;

  // ---------------------------------------------------------------------
  // Field extraction (post-expansion, always in standard RV32 positions).
  // ---------------------------------------------------------------------
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;
  logic [4:0] funct5;
  logic [4:0] rs1, rs2, rd;

  assign opcode = instr[6:0];
  assign rd     = instr[11:7];
  assign funct3 = instr[14:12];
  assign rs1    = instr[19:15];
  assign rs2    = instr[24:20];
  assign funct7 = instr[31:25];
  assign funct5 = instr[31:27];

  assign rs1_o = rs1;
  assign rs2_o = rs2;
  assign rd_o  = rd;

  // ---------------------------------------------------------------------
  // Immediate generation, one per format; muxed into imm_o per opcode below.
  // ---------------------------------------------------------------------
  logic [31:0] i_imm, s_imm, b_imm, u_imm, j_imm;

  assign i_imm = {{20{instr[31]}}, instr[31:20]};
  assign s_imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign b_imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
  assign u_imm = {instr[31:12], 12'b0};
  assign j_imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

  // ---------------------------------------------------------------------
  // Main decode.
  // ---------------------------------------------------------------------
  always_comb begin
    // Defaults: an inert, non-writing, non-memory, non-branching decode.
    // Every field is fully assigned here so nothing latches; opcode cases
    // below only need to override what actually differs.
    imm_o           = i_imm;
    alu_op_o        = ALU_ADD;
    alu_a_sel_o     = OP_A_RS1;
    alu_b_sel_o     = OP_B_IMM;
    rf_we_o         = 1'b0;
    wb_sel_o        = WB_EX;
    mem_req_o       = 1'b0;
    mem_we_o        = 1'b0;
    mem_size_o      = MEM_W;
    mem_sign_ext_o  = 1'b0;
    branch_o        = 1'b0;
    jal_o           = 1'b0;
    jalr_o          = 1'b0;
    branch_op_o     = branch_op_e'(funct3);
    mul_div_o       = 1'b0;
    muldiv_op_o     = muldiv_op_e'(funct3);
    atomic_o        = 1'b0;
    amo_op_o        = AMO_ADD;
    aq_o            = instr[26];
    rl_o            = instr[25];
    csr_access_o    = 1'b0;
    csr_op_o        = csr_op_e'(funct3);
    csr_addr_o      = instr[31:20];
    ecall_o         = 1'b0;
    ebreak_o        = 1'b0;
    mret_o          = 1'b0;
    wfi_o           = 1'b0;
    illegal_instr_o = 1'b0;

    unique case (opcode)

      OPC_LUI: begin
        imm_o       = u_imm;
        alu_a_sel_o = OP_A_ZERO;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_EX;
      end

      OPC_AUIPC: begin
        imm_o       = u_imm;
        alu_a_sel_o = OP_A_PC;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_EX;
      end

      OPC_JAL: begin
        imm_o       = j_imm;
        jal_o       = 1'b1;
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_PC4;
        alu_a_sel_o = OP_A_PC;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;  // target = pc + imm
      end

      OPC_JALR: begin
        imm_o       = i_imm;
        jalr_o      = 1'b1;
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_PC4;
        alu_a_sel_o = OP_A_RS1;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;  // target = (rs1 + imm) & ~1
        if (funct3 != 3'b000) illegal_instr_o = 1'b1;
      end

      OPC_BRANCH: begin
        imm_o       = b_imm;
        branch_o    = 1'b1;
        branch_op_o = branch_op_e'(funct3);
        alu_a_sel_o = OP_A_PC;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;  // target = pc + imm; rs1/rs2 compared separately
        if (funct3 == 3'b010 || funct3 == 3'b011) illegal_instr_o = 1'b1;  // reserved
      end

      OPC_LOAD: begin
        imm_o       = i_imm;
        mem_req_o   = 1'b1;
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_MEM;
        alu_a_sel_o = OP_A_RS1;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;  // address = rs1 + imm
        unique case (funct3)
          3'b000: begin mem_size_o = MEM_B; mem_sign_ext_o = 1'b1; end  // LB
          3'b001: begin mem_size_o = MEM_H; mem_sign_ext_o = 1'b1; end  // LH
          3'b010: begin mem_size_o = MEM_W; mem_sign_ext_o = 1'b0; end  // LW
          3'b100: begin mem_size_o = MEM_B; mem_sign_ext_o = 1'b0; end  // LBU
          3'b101: begin mem_size_o = MEM_H; mem_sign_ext_o = 1'b0; end  // LHU
          default: illegal_instr_o = 1'b1;
        endcase
      end

      OPC_STORE: begin
        imm_o       = s_imm;
        mem_req_o   = 1'b1;
        mem_we_o    = 1'b1;
        alu_a_sel_o = OP_A_RS1;
        alu_b_sel_o = OP_B_IMM;
        alu_op_o    = ALU_ADD;  // address = rs1 + imm
        unique case (funct3)
          3'b000: mem_size_o = MEM_B;  // SB
          3'b001: mem_size_o = MEM_H;  // SH
          3'b010: mem_size_o = MEM_W;  // SW
          default: illegal_instr_o = 1'b1;
        endcase
      end

      OPC_OP_IMM: begin
        imm_o       = i_imm;
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_EX;
        alu_a_sel_o = OP_A_RS1;
        alu_b_sel_o = OP_B_IMM;
        unique case (funct3)
          3'b000: alu_op_o = ALU_ADD;   // ADDI
          3'b010: alu_op_o = ALU_SLT;   // SLTI
          3'b011: alu_op_o = ALU_SLTU;  // SLTIU
          3'b100: alu_op_o = ALU_XOR;   // XORI
          3'b110: alu_op_o = ALU_OR;    // ORI
          3'b111: alu_op_o = ALU_AND;   // ANDI
          3'b001: begin                 // SLLI
            alu_op_o = ALU_SLL;
            if (funct7 != 7'b0000000) illegal_instr_o = 1'b1;  // shamt[5] must be 0 on RV32
          end
          3'b101: begin                 // SRLI / SRAI
            alu_op_o = instr[30] ? ALU_SRA : ALU_SRL;
            if (funct7 != 7'b0000000 && funct7 != 7'b0100000) illegal_instr_o = 1'b1;
          end
        endcase
      end

      OPC_OP: begin
        rf_we_o     = 1'b1;
        wb_sel_o    = WB_EX;
        alu_a_sel_o = OP_A_RS1;
        alu_b_sel_o = OP_B_RS2;
        unique case (funct7)
          7'b0000000, 7'b0100000: begin  // base RV32I R-type
            unique case (funct3)
              3'b000:  alu_op_o = instr[30] ? ALU_SUB : ALU_ADD;  // ADD/SUB
              3'b001:  alu_op_o = ALU_SLL;
              3'b010:  alu_op_o = ALU_SLT;
              3'b011:  alu_op_o = ALU_SLTU;
              3'b100:  alu_op_o = ALU_XOR;
              3'b101:  alu_op_o = instr[30] ? ALU_SRA : ALU_SRL;  // SRL/SRA
              3'b110:  alu_op_o = ALU_OR;
              3'b111:  alu_op_o = ALU_AND;
            endcase
            // funct7==0100000 only distinguishes ADD/SUB (000) and SRL/SRA (101).
            if (funct7 == 7'b0100000 && funct3 != 3'b000 && funct3 != 3'b101) begin
              illegal_instr_o = 1'b1;
            end
          end
          7'b0000001: begin  // M extension
            mul_div_o   = 1'b1;
            muldiv_op_o = muldiv_op_e'(funct3);
            wb_sel_o    = WB_EX;  // EX stage muxes ALU vs. mul/div result onto this bus
          end
          default: illegal_instr_o = 1'b1;
        endcase
      end

      OPC_AMO: begin
        if (funct3 == 3'b010) begin  // RV32A is word-only
          atomic_o    = 1'b1;
          mem_req_o   = 1'b1;
          rf_we_o     = 1'b1;
          wb_sel_o    = WB_MEM;     // pre-op memory value is the register result
          mem_size_o  = MEM_W;
          mem_we_o    = 1'b1;       // default: read-modify-write; LR overrides below
          alu_a_sel_o = OP_A_RS1;   // address = rs1 (no offset for AMO)
          imm_o       = 32'b0;      // ... so operand B must be 0, not the default i_imm — the
                                    // AMO encoding has no immediate field; those bit positions
                                    // are funct5/aq/rl/rs2, and misreading them as i_imm corrupts
                                    // the address (rs1 + garbage instead of just rs1)
          unique case (funct5)
            5'b00010: begin
              amo_op_o = AMO_LR; mem_we_o = 1'b0;
              if (rs2 != 5'd0) illegal_instr_o = 1'b1;  // rs2 must be 0 for LR.W
            end
            5'b00011: amo_op_o = AMO_SC;
            5'b00001: amo_op_o = AMO_SWAP;
            5'b00000: amo_op_o = AMO_ADD;
            5'b00100: amo_op_o = AMO_XOR;
            5'b01100: amo_op_o = AMO_AND;
            5'b01000: amo_op_o = AMO_OR;
            5'b10000: amo_op_o = AMO_MIN;
            5'b10100: amo_op_o = AMO_MAX;
            5'b11000: amo_op_o = AMO_MINU;
            5'b11100: amo_op_o = AMO_MAXU;
            default:  illegal_instr_o = 1'b1;
          endcase
        end else begin
          illegal_instr_o = 1'b1;
        end
      end

      OPC_MISC_MEM: begin
        // FENCE / FENCE.I: no register or immediate side effects on this
        // single-hart, in-order core; decoded as a no-op bubble. A future
        // multi-hart or memory-ordering-sensitive implementation would stall
        // the pipeline here instead of just falling through.
        if (funct3 != 3'b000 && funct3 != 3'b001) illegal_instr_o = 1'b1;
      end

      OPC_SYSTEM: begin
        if (funct3 == 3'b000) begin
          unique case (instr[31:20])
            12'h000: ecall_o  = 1'b1;
            12'h001: ebreak_o = 1'b1;
            12'h302: mret_o   = 1'b1;
            12'h105: wfi_o    = 1'b1;
            default: illegal_instr_o = 1'b1;
          endcase
          if (rd != 5'd0 || rs1 != 5'd0) illegal_instr_o = 1'b1;  // must be zero per spec
        end else if (funct3 != 3'b100) begin
          // Zicsr: bundled here since practical firmware needs it even though
          // it is technically a separate extension letter from IMAC.
          csr_access_o = 1'b1;
          csr_op_o     = csr_op_e'(funct3);
          csr_addr_o   = instr[31:20];
          rf_we_o      = (rd != 5'd0);
          wb_sel_o     = WB_CSR;
        end else begin
          illegal_instr_o = 1'b1;  // funct3 == 3'b100 is reserved
        end
      end

      default: illegal_instr_o = 1'b1;

    endcase

    // A malformed compressed encoding is illegal regardless of what garbage
    // it happened to expand to; an invalid instr_valid_i cycle decodes to
    // "nothing," not "illegal" (that's for the consumer to ignore outright).
    illegal_instr_o = instr_valid_i & (illegal_instr_o | (is_compressed_o & c_illegal));
  end

endmodule : pulse_idu
