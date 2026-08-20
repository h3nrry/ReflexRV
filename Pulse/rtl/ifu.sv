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

// pulse_ifu — Instruction Fetch Unit (ReflexRV-Pulse)
//
// Sequential (PC+4) instruction fetch against a synchronous, single-cycle-latency
// instruction memory (registered-output SRAM/BRAM style): the address driven on
// imem_addr_o is sampled by imem on this clock edge, and the corresponding
// instruction appears on imem_rdata_i one cycle later. No extra IF/ID instruction
// register is needed inside this module — imem's own output register serves that
// role. Only the PC is shadowed by one cycle (pc_out_q) so it stays paired with
// the instruction it fetched.
//
// stall_i freezes the PC, the shadow PC, and (via imem_req_o deasserting) the
// memory read, so the currently-held instruction/PC/valid stay stable across
// stall cycles until the next stage is ready to accept them.
//
// Out of scope for this revision: branch/jump redirect and pipeline flush. Once
// a branch-resolution stage exists, add redirect_valid_i / redirect_pc_i inputs
// that mux into pc_next and a flush_i that forces instr_valid_o low for one cycle.

module pulse_ifu #(
  parameter logic [31:0] RESET_PC = 32'h0000_0000
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Backpressure from the next stage (or from imem not being ready).
  input  logic        stall_i,

  // Instruction memory port (synchronous, 1-cycle read latency).
  output logic        imem_req_o,
  output logic [31:0] imem_addr_o,
  input  logic [31:0] imem_rdata_i,

  // To the next pipeline stage.
  output logic        instr_valid_o,
  output logic [31:0] instr_o,
  output logic [31:0] instr_pc_o
);

  logic [31:0] pc_q;      // PC of the fetch request driven this cycle
  logic [31:0] pc_out_q;  // PC of the instruction currently on imem_rdata_i
  logic        valid_q;   // instr_o / instr_pc_o are valid

  logic [31:0] pc_next;
  assign pc_next = pc_q + 32'd4;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pc_q     <= RESET_PC;
      pc_out_q <= RESET_PC;
      valid_q  <= 1'b0;
    end else if (!stall_i) begin
      pc_q     <= pc_next;
      pc_out_q <= pc_q;
      valid_q  <= 1'b1;
    end
  end

  assign imem_req_o  = rst_ni & ~stall_i;
  assign imem_addr_o = pc_q;

  assign instr_valid_o = valid_q;
  assign instr_o       = imem_rdata_i;
  assign instr_pc_o    = pc_out_q;

endmodule
