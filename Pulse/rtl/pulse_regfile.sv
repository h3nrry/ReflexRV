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

// pulse_regfile — 32x32 integer register file (ReflexRV-Pulse).
//
// Two asynchronous read ports, one synchronous write port, x0 hardwired to
// zero. Reads include same-cycle write-forwarding: if the write port commits
// to the same register a read port is addressing this cycle, the read
// returns the value being written, not the stale stored one. This is what
// lets pulse_core read a register in ID the same cycle an earlier
// instruction writes it back in WB, with no separate bypass network needed
// for that specific case — RAW hazards against instructions still in EX/MEM
// are still pulse_core's job (see its header).

module pulse_regfile (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic [4:0]  rs1_addr_i,
  output logic [31:0] rs1_rdata_o,

  input  logic [4:0]  rs2_addr_i,
  output logic [31:0] rs2_rdata_o,

  input  logic         we_i,
  input  logic [4:0]   waddr_i,
  input  logic [31:0]  wdata_i
);

  logic [31:0] regs_q [1:31];  // regs_q[i] backs architectural register xi, i=1..31 (x0 isn't stored)

  logic we_x1_31;
  assign we_x1_31 = we_i & (waddr_i != 5'd0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 1; i <= 31; i++) regs_q[i] <= 32'b0;
    end else if (we_x1_31) begin
      regs_q[waddr_i] <= wdata_i;
    end
  end

  function automatic logic [31:0] read(input logic [4:0] addr);
    if (addr == 5'd0) return 32'b0;
    if (we_x1_31 && addr == waddr_i) return wdata_i;  // same-cycle write-forward
    return regs_q[addr];
  endfunction

  assign rs1_rdata_o = read(rs1_addr_i);
  assign rs2_rdata_o = read(rs2_addr_i);

endmodule : pulse_regfile
