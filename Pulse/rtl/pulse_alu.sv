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

// pulse_alu — single-cycle ALU (ReflexRV-Pulse).
//
// Purely combinational, fixed one-cycle latency regardless of operands or
// operation — no data-dependent early-out, in keeping with this core's
// bounded-WCET goal. Operand selection (rs1/pc/zero, rs2/imm/4) is done by the
// caller (pulse_exu); this module only ever sees the two final operands.

module pulse_alu
  import pulse_pkg::*;
(
  input  logic [31:0] operand_a_i,
  input  logic [31:0] operand_b_i,
  input  alu_op_e      alu_op_i,

  output logic [31:0] result_o
);

  logic [4:0] shamt;
  assign shamt = operand_b_i[4:0];

  always_comb begin
    unique case (alu_op_i)
      ALU_ADD:  result_o = operand_a_i + operand_b_i;
      ALU_SUB:  result_o = operand_a_i - operand_b_i;
      ALU_SLL:  result_o = operand_a_i << shamt;
      ALU_SLT:  result_o = {31'b0, $signed(operand_a_i) < $signed(operand_b_i)};
      ALU_SLTU: result_o = {31'b0, operand_a_i < operand_b_i};
      ALU_XOR:  result_o = operand_a_i ^ operand_b_i;
      ALU_SRL:  result_o = operand_a_i >> shamt;
      ALU_SRA:  result_o = $signed(operand_a_i) >>> shamt;
      ALU_OR:   result_o = operand_a_i | operand_b_i;
      ALU_AND:  result_o = operand_a_i & operand_b_i;
      default:  result_o = 32'b0;
    endcase
  end

endmodule : pulse_alu
