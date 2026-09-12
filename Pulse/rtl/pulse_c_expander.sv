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

// pulse_c_expander — RV32C compressed-instruction expander.
//
// Purely combinational. Takes one 16-bit compressed instruction and produces
// the equivalent standard 32-bit RV32I/M instruction word, so downstream logic
// (pulse_idu) never needs to know an instruction was originally compressed.
//
// Scope: RV32C only. No F/D/Q-suffixed compressed loads/stores (C.FLW/C.FSW/
// C.FLD/C.FSD/...) since this core has no F/D extension yet, and no RV64/128
// C-extension-only encodings (C.ADDIW/C.SUBW/C.ADDW, wide C.SLLI/SRLI/SRAI
// shift amounts). Any such encoding, or a reserved bit pattern, sets illegal_o.
//
// Precondition: c_instr_i is a genuine 16-bit instruction, i.e. its own
// c_instr_i[1:0] != 2'b11. Distinguishing 16- vs 32-bit instructions and
// picking the right halfword out of a wider fetch word (including the case
// where a 32-bit instruction straddles two fetch words) is fetch/aligner work,
// not this module's — see pulse_idu's header for the current state of that
// boundary.

module pulse_c_expander (
  input  logic [15:0] c_instr_i,

  output logic [31:0] instr_o,   // expanded, standard 32-bit instruction
  output logic         illegal_o  // c_instr_i is not a legal RV32C encoding
);

  import pulse_pkg::*;

  // ---------------------------------------------------------------------
  // Raw field extraction.
  // ---------------------------------------------------------------------
  logic [1:0] c_op;
  logic [2:0] c_f3;
  assign c_op = c_instr_i[1:0];
  assign c_f3 = c_instr_i[15:13];

  logic [4:0] rd_rs1_5;   // full 5-bit rd/rs1 field (CI/CR/CSS/CIW-adjacent forms)
  logic [4:0] rs2_5;      // full 5-bit rs2 field (CR/CSS forms)
  assign rd_rs1_5 = c_instr_i[11:7];
  assign rs2_5    = c_instr_i[6:2];

  logic [4:0] rs1p, rs2p; // compressed 3-bit reg fields, expanded to x8-x15
  assign rs1p = {2'b01, c_instr_i[9:7]};
  assign rs2p = {2'b01, c_instr_i[4:2]};  // also serves as rd' for CIW/CL

  // ---------------------------------------------------------------------
  // Standard 32-bit instruction-format packers.
  // ---------------------------------------------------------------------
  function automatic logic [31:0] enc_r(
      input logic [6:0] f7, input logic [4:0] rs2, input logic [4:0] rs1,
      input logic [2:0] f3, input logic [4:0] rd,  input logic [6:0] op);
    return {f7, rs2, rs1, f3, rd, op};
  endfunction

  function automatic logic [31:0] enc_i(
      input logic [11:0] imm, input logic [4:0] rs1,
      input logic [2:0] f3, input logic [4:0] rd, input logic [6:0] op);
    return {imm, rs1, f3, rd, op};
  endfunction

  function automatic logic [31:0] enc_s(
      input logic [11:0] imm, input logic [4:0] rs2, input logic [4:0] rs1,
      input logic [2:0] f3, input logic [6:0] op);
    return {imm[11:5], rs2, rs1, f3, imm[4:0], op};
  endfunction

  // B/J immediates are always even (bit 0 is implicitly 0 and never encoded),
  // so these two take imm with bit 0 already dropped.
  function automatic logic [31:0] enc_b(
      input logic [12:1] imm, input logic [4:0] rs2, input logic [4:0] rs1,
      input logic [2:0] f3, input logic [6:0] op);
    return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], op};
  endfunction

  function automatic logic [31:0] enc_u(
      input logic [19:0] imm20, input logic [4:0] rd, input logic [6:0] op);
    return {imm20, rd, op};
  endfunction

  function automatic logic [31:0] enc_j(
      input logic [20:1] imm, input logic [4:0] rd, input logic [6:0] op);
    return {imm[20], imm[10:1], imm[11], imm[19:12], rd, op};
  endfunction

  // ---------------------------------------------------------------------
  // Expansion. Scratch immediate fields are declared once here (rather than
  // per case-item) and reused across the mutually-exclusive branches below —
  // plain module-scope combinational temporaries, portable across simulators
  // that are fussy about declarations nested inside case-item blocks.
  // ---------------------------------------------------------------------
  logic [9:0]  t_uimm10;  // ADDI4SPN t_uimm10 / ADDI16SP nzimm (shared 10-bit field)
  logic [6:0]  t_off7;    // C.LW / C.SW word offset (shared 7-bit field)
  logic [5:0]  t_imm6;    // ADDI / LI / LUI / ANDI (shared 6-bit signed field)
  logic [11:1] t_j;       // C.JAL / C.J jump-offset field
  logic [8:1]  t_b;       // C.BEQZ / C.BNEZ branch-offset field
  logic [7:0]  t_off8;    // C.LWSP / C.SWSP stack-relative offset field

  always_comb begin
    instr_o   = 32'h0000_0013;  // safe default: ADDI x0,x0,0 (NOP); ignored when illegal_o
    illegal_o = 1'b0;

    unique case (c_op)

      // ---------------- Quadrant 0: CIW / CL / CS -----------------------
      2'b00: begin
        unique case (c_f3)
          3'b000: begin  // C.ADDI4SPN -> ADDI rd', x2, t_uimm10[9:2]00
            t_uimm10 = {c_instr_i[10:7], c_instr_i[12:11], c_instr_i[5], c_instr_i[6], 2'b00};
            instr_o   = enc_i({2'b00, t_uimm10}, 5'd2, 3'b000, rs2p, OPC_OP_IMM);
            illegal_o = (t_uimm10 == '0);
          end
          3'b010: begin  // C.LW -> LW rd', offset(rs1')
            t_off7 = {c_instr_i[5], c_instr_i[12:10], c_instr_i[6], 2'b00};
            instr_o = enc_i({5'b00000, t_off7}, rs1p, 3'b010, rs2p, OPC_LOAD);
          end
          3'b110: begin  // C.SW -> SW rs2', offset(rs1')
            t_off7 = {c_instr_i[5], c_instr_i[12:10], c_instr_i[6], 2'b00};
            instr_o = enc_s({5'b00000, t_off7}, rs2p, rs1p, 3'b010, OPC_STORE);
          end
          default: illegal_o = 1'b1;  // C.FLD/C.FLW/C.FSD/C.FSW (no F/D) / reserved
        endcase
      end

      // ---------------- Quadrant 1: CI / CR / CA / CB / CJ ---------------
      2'b01: begin
        unique case (c_f3)
          3'b000: begin  // C.ADDI / C.NOP -> ADDI rd, rd, imm (rd==0 is HINT)
            t_imm6 = {c_instr_i[12], c_instr_i[6:2]};
            instr_o = enc_i({{6{t_imm6[5]}}, t_imm6}, rd_rs1_5, 3'b000, rd_rs1_5, OPC_OP_IMM);
          end

          3'b001: begin  // C.JAL (RV32-only) -> JAL x1, offset
            t_j = {c_instr_i[12], c_instr_i[8], c_instr_i[10], c_instr_i[9], c_instr_i[6],
                 c_instr_i[7], c_instr_i[2], c_instr_i[11], c_instr_i[5], c_instr_i[4],
                 c_instr_i[3]};
            instr_o = enc_j({{9{t_j[11]}}, t_j}, 5'd1, OPC_JAL);
          end

          3'b010: begin  // C.LI -> ADDI rd, x0, imm (rd==0 is HINT)
            t_imm6 = {c_instr_i[12], c_instr_i[6:2]};
            instr_o = enc_i({{6{t_imm6[5]}}, t_imm6}, 5'd0, 3'b000, rd_rs1_5, OPC_OP_IMM);
          end

          3'b011: begin
            if (rd_rs1_5 == 5'd2) begin  // C.ADDI16SP -> ADDI x2, x2, nzimm
              t_uimm10 = {c_instr_i[12], c_instr_i[4:3], c_instr_i[5], c_instr_i[2],
                         c_instr_i[6], 4'b0000};
              instr_o   = enc_i({{2{t_uimm10[9]}}, t_uimm10}, 5'd2, 3'b000, 5'd2, OPC_OP_IMM);
              illegal_o = (t_uimm10 == '0);
            end else begin              // C.LUI -> LUI rd, nzimm
                t_imm6 = {c_instr_i[12], c_instr_i[6:2]};
              instr_o   = enc_u({{14{t_imm6[5]}}, t_imm6}, rd_rs1_5, OPC_LUI);
              illegal_o = (t_imm6 == '0);
            end
          end

          3'b100: begin  // arithmetic group on rd'/rs1'
            unique case (c_instr_i[11:10])
              2'b00: begin  // C.SRLI
                instr_o   = enc_i({7'b0000000, c_instr_i[6:2]}, rs1p, 3'b101, rs1p, OPC_OP_IMM);
                illegal_o = c_instr_i[12];  // shamt[5]!=0: RV128-only, reserved here
              end
              2'b01: begin  // C.SRAI
                instr_o   = enc_i({7'b0100000, c_instr_i[6:2]}, rs1p, 3'b101, rs1p, OPC_OP_IMM);
                illegal_o = c_instr_i[12];
              end
              2'b10: begin  // C.ANDI
                    t_imm6 = {c_instr_i[12], c_instr_i[6:2]};
                instr_o = enc_i({{6{t_imm6[5]}}, t_imm6}, rs1p, 3'b111, rs1p, OPC_OP_IMM);
              end
              2'b11: begin
                if (c_instr_i[12]) begin
                  illegal_o = 1'b1;  // C.SUBW/C.ADDW: RV64/128-only
                end else begin
                  unique case (c_instr_i[6:5])
                    2'b00: instr_o = enc_r(7'b0100000, rs2p, rs1p, 3'b000, rs1p, OPC_OP); // SUB
                    2'b01: instr_o = enc_r(7'b0000000, rs2p, rs1p, 3'b100, rs1p, OPC_OP); // XOR
                    2'b10: instr_o = enc_r(7'b0000000, rs2p, rs1p, 3'b110, rs1p, OPC_OP); // OR
                    2'b11: instr_o = enc_r(7'b0000000, rs2p, rs1p, 3'b111, rs1p, OPC_OP); // AND
                  endcase
                end
              end
            endcase
          end

          3'b101: begin  // C.J -> JAL x0, offset
            t_j = {c_instr_i[12], c_instr_i[8], c_instr_i[10], c_instr_i[9], c_instr_i[6],
                 c_instr_i[7], c_instr_i[2], c_instr_i[11], c_instr_i[5], c_instr_i[4],
                 c_instr_i[3]};
            instr_o = enc_j({{9{t_j[11]}}, t_j}, 5'd0, OPC_JAL);
          end

          3'b110, 3'b111: begin  // C.BEQZ / C.BNEZ -> BEQ/BNE rs1', x0, offset
            t_b = {c_instr_i[12], c_instr_i[6], c_instr_i[5], c_instr_i[2], c_instr_i[11],
                 c_instr_i[10], c_instr_i[4], c_instr_i[3]};
            instr_o = enc_b({{4{t_b[8]}}, t_b}, 5'd0, rs1p, (c_f3 == 3'b110) ? 3'b000 : 3'b001,
                             OPC_BRANCH);
          end
        endcase
      end

      // ---------------- Quadrant 2: CI / CSS / CR ------------------------
      2'b10: begin
        unique case (c_f3)
          3'b000: begin  // C.SLLI -> SLLI rd, rd, shamt (rd==0 is HINT)
            instr_o   = enc_i({7'b0000000, c_instr_i[6:2]}, rd_rs1_5, 3'b001, rd_rs1_5, OPC_OP_IMM);
            illegal_o = c_instr_i[12];  // shamt[5]!=0: RV128-only, reserved here
          end

          3'b010: begin  // C.LWSP -> LW rd, offset(x2)
            t_off8 = {c_instr_i[3:2], c_instr_i[12], c_instr_i[6:4], 2'b00};
            instr_o   = enc_i({4'b0000, t_off8}, 5'd2, 3'b010, rd_rs1_5, OPC_LOAD);
            illegal_o = (rd_rs1_5 == 5'd0);  // rd==0 reserved
          end

          3'b100: begin
            if (!c_instr_i[12]) begin
              if (rs2_5 == 5'd0) begin  // C.JR -> JALR x0, 0(rs1)
                instr_o   = enc_i(12'd0, rd_rs1_5, 3'b000, 5'd0, OPC_JALR);
                illegal_o = (rd_rs1_5 == 5'd0);  // rs1==0: reserved
              end else begin            // C.MV -> ADD rd, x0, rs2 (rd==0 is HINT)
                instr_o = enc_r(7'b0000000, rs2_5, 5'd0, 3'b000, rd_rs1_5, OPC_OP);
              end
            end else begin
              if (rd_rs1_5 == 5'd0 && rs2_5 == 5'd0) begin      // C.EBREAK
                instr_o = enc_i(12'h001, 5'd0, 3'b000, 5'd0, OPC_SYSTEM);
              end else if (rs2_5 == 5'd0) begin                 // C.JALR -> JALR x1, 0(rs1)
                instr_o = enc_i(12'd0, rd_rs1_5, 3'b000, 5'd1, OPC_JALR);
              end else begin                                    // C.ADD (rd==0 is HINT)
                instr_o = enc_r(7'b0000000, rs2_5, rd_rs1_5, 3'b000, rd_rs1_5, OPC_OP);
              end
            end
          end

          3'b110: begin  // C.SWSP -> SW rs2, offset(x2)
            t_off8 = {c_instr_i[8:7], c_instr_i[12:9], 2'b00};
            instr_o = enc_s({4'b0000, t_off8}, rs2_5, 5'd2, 3'b010, OPC_STORE);
          end

          default: illegal_o = 1'b1;  // C.FLDSP/C.FLWSP/C.FSDSP/C.FSWSP (no F/D)
        endcase
      end

      // op==11 is a full 32-bit instruction, not a compressed one — the caller
      // should never route that halfword through this module (see precondition
      // in the header). Flag it rather than silently misinterpreting it.
      default: illegal_o = 1'b1;

    endcase
  end

endmodule : pulse_c_expander
