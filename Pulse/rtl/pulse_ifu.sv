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
// redirect_valid_i / redirect_pc_i implement branch/jump redirect and pipeline
// flush, driven by whatever stage resolves control flow (pulse_exu, via
// pulse_core). redirect_valid_i takes priority over stall_i — a resolved
// redirect must never be deferred by an unrelated stall — and does two things
// on the same edge: (1) loads pc_q with redirect_pc_i instead of pc_next, so
// the corrected address is requested starting next cycle, and (2) forces
// instr_valid_o low for the following cycle. Only one cycle of suppression is
// needed, not two: this module only ever has one fetch outstanding (that's
// the whole point of the one-cycle pc_out_q shadow instead of a real FIFO), so
// the moment redirect_pc_i is loaded, the *next* imem_addr_o is already the
// corrected one — the single cycle of squashed output covers exactly the one
// wrong-path request that was already in flight when the redirect arrived.

module pulse_ifu #(
  parameter logic [31:0] RESET_PC = 32'h0000_0000
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Backpressure from the next stage (or from imem not being ready).
  input  logic        stall_i,

  // Redirect from a later stage (branch/jump resolution, trap entry, ...).
  input  logic         redirect_valid_i,
  input  logic [31:0]  redirect_pc_i,

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
    end else if (redirect_valid_i) begin
      pc_q     <= redirect_pc_i;
      pc_out_q <= pc_q;   // don't-care: instr_valid_o is low this cycle
      valid_q  <= 1'b0;
    end else if (!stall_i) begin
      pc_q     <= pc_next;
      pc_out_q <= pc_q;
      valid_q  <= 1'b1;
    end
  end

  assign imem_req_o  = rst_ni & (redirect_valid_i | ~stall_i);
  assign imem_addr_o = pc_q;

  assign instr_valid_o = valid_q;
  assign instr_o       = imem_rdata_i;
  assign instr_pc_o    = pc_out_q;

endmodule : pulse_ifu
