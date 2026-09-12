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

// pulse_exu — Execute stage (ReflexRV-Pulse).
//
// Selects ALU operands per pulse_idu's alu_a_sel/alu_b_sel, runs pulse_alu,
// and for M-extension instructions runs pulse_muldiv instead and presents
// its (later) result on the same result_o bus — pulse_core's EX/MEM register
// doesn't need to know which functional unit actually produced a value, only
// when it's ready (stall_o).
//
// Also resolves branches/jumps here (branch comparison against the raw
// register values, not the ALU-muxed operands — those are busy computing the
// pc+imm target address for branch/JAL, or the rs1+imm target for JALR, per
// how pulse_idu sets alu_a_sel/alu_b_sel for those opcodes) and folds the
// result straight into redirect_valid_o/redirect_pc_o, already shaped to
// drive pulse_ifu's redirect_valid_i/redirect_pc_i directly.
//
// stall_o must gate the pipeline for the *entire* duration a mul/div op is
// in flight, including the first setup cycle — see pulse_muldiv's header for
// why that means deriving it as (mul_div_i & ~muldiv_valid) rather than
// wiring pulse_muldiv's own busy_o straight through.

module pulse_exu
  import pulse_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic         valid_i,  // this EX-stage instruction is real, not a bubble

  input  logic [31:0]  rs1_data_i,
  input  logic [31:0]  rs2_data_i,
  input  logic [31:0]  pc_i,
  input  logic [31:0]  imm_i,

  input  alu_op_e      alu_op_i,
  input  alu_a_sel_e   alu_a_sel_i,
  input  alu_b_sel_e   alu_b_sel_i,

  input  logic         mul_div_i,
  input  muldiv_op_e   muldiv_op_i,

  input  logic         branch_i,
  input  logic         jal_i,
  input  logic         jalr_i,
  input  branch_op_e   branch_op_i,

  output logic [31:0]  result_o,          // ALU/muldiv result; also the LSU address (rs1+imm)
  output logic         stall_o,           // mul/div in flight — freeze the pipeline
  output logic         redirect_valid_o,  // branch/jal/jalr resolved taken this cycle
  output logic [31:0]  redirect_pc_o      // corrected fetch target
);

  // ---------------------------------------------------------------------
  // Operand muxes and ALU.
  // ---------------------------------------------------------------------
  logic [31:0] operand_a, operand_b;

  always_comb begin
    unique case (alu_a_sel_i)
      OP_A_RS1:  operand_a = rs1_data_i;
      OP_A_PC:   operand_a = pc_i;
      OP_A_ZERO: operand_a = 32'b0;
      default:   operand_a = rs1_data_i;
    endcase
    unique case (alu_b_sel_i)
      OP_B_RS2:  operand_b = rs2_data_i;
      OP_B_IMM:  operand_b = imm_i;
      OP_B_FOUR: operand_b = 32'd4;
      default:   operand_b = imm_i;
    endcase
  end

  logic [31:0] alu_result;
  pulse_alu u_alu (
    .operand_a_i (operand_a),
    .operand_b_i (operand_b),
    .alu_op_i    (alu_op_i),
    .result_o    (alu_result)
  );

  // ---------------------------------------------------------------------
  // M-extension multiply/divide.
  // ---------------------------------------------------------------------
  logic [31:0] muldiv_result;
  logic        muldiv_valid;

  pulse_muldiv u_muldiv (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .start_i      (valid_i & mul_div_i),
    .operand_a_i  (rs1_data_i),
    .operand_b_i  (rs2_data_i),
    .op_i         (muldiv_op_i),
    /* verilator lint_off PINCONNECTEMPTY */
    .busy_o       (),
    /* verilator lint_on PINCONNECTEMPTY */
    .valid_o      (muldiv_valid),
    .result_o     (muldiv_result)
  );

  assign stall_o   = valid_i & mul_div_i & ~muldiv_valid;
  assign result_o  = mul_div_i ? muldiv_result : alu_result;

  // ---------------------------------------------------------------------
  // Branch/jump resolution.
  // ---------------------------------------------------------------------
  logic branch_taken;
  always_comb begin
    unique case (branch_op_i)
      BR_EQ:  branch_taken = (rs1_data_i == rs2_data_i);
      BR_NE:  branch_taken = (rs1_data_i != rs2_data_i);
      BR_LT:  branch_taken = ($signed(rs1_data_i) <  $signed(rs2_data_i));
      BR_GE:  branch_taken = ($signed(rs1_data_i) >= $signed(rs2_data_i));
      BR_LTU: branch_taken = (rs1_data_i <  rs2_data_i);
      BR_GEU: branch_taken = (rs1_data_i >= rs2_data_i);
      default: branch_taken = 1'b0;  // reserved funct3s; pulse_idu already flags these illegal
    endcase
  end

  // alu_result already holds pc+imm (branch target / JAL target) or rs1+imm
  // (JALR target, per pulse_idu's alu_a_sel/alu_b_sel choice for JALR) — only
  // JALR needs the low-bit mask the spec requires.
  assign redirect_pc_o    = jalr_i ? {alu_result[31:1], 1'b0} : alu_result;
  assign redirect_valid_o = valid_i & ((branch_i & branch_taken) | jal_i | jalr_i);

endmodule : pulse_exu
