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

// pulse_mpu — region-based Memory Protection Unit (ReflexRV-Pulse).
//
// NUM_REGIONS fixed [base, limit) regions, each with R/W/X permission bits, a
// lock bit, and a valid bit. Region 0 has the highest priority; the first
// valid, matching region decides the access (this is PMP's "first match
// wins" rule, deliberately kept the same since it's a well-understood
// convention, not because this models real PMP encoding — see below).
//
// Two independent, combinational check ports (`a` and `b`) look up the *same*
// region table every cycle — one instance, not two, since instruction fetch
// and data access both need a same-cycle answer against a single shared
// configuration; giving each its own copy of the config would mean keeping
// two tables in sync on every write. Each port has fixed one-cycle latency
// regardless of which region (or none) matches, in keeping with this core's
// bounded-WCET goal.
//
// Access policy once a match (or its absence) is known:
//   - Machine mode:  default-allow. A matching region's R/W/X bits only gate
//     the access if that region is locked (cfg_lock, PMP's "L" bit) — an
//     unlocked region is a restriction on U-mode only, not on M-mode itself.
//   - User mode: default-deny. No matching region means no access; a
//     matching region's R/W/X bits gate the access regardless of its lock
//     bit (lock only matters for whether M-mode is also bound by it).
// This is a deliberately simplified, PMP-*inspired* policy, not a compliant
// implementation of the RISC-V PMP CSR spec: real PMP also has NAPOT/TOR/OFF
// address-matching modes and is programmed through pmpcfg*/pmpaddr* CSRs.
// This module only implements the region-check datapath, with its own direct
// base/limit config port; mapping that port to the real PMP CSR encoding is
// pulse_csr's job once it exists.

module pulse_mpu #(
  parameter int unsigned NUM_REGIONS = 8
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Region configuration (one region written per cycle; typically driven by
  // a future pulse_csr on writes to the region-config CSRs, tied off for now).
  input  logic                            cfg_we_i,
  input  logic [$clog2(NUM_REGIONS)-1:0]  cfg_idx_i,
  input  logic [31:0]                     cfg_base_i,
  input  logic [31:0]                     cfg_limit_i,  // exclusive upper bound
  input  logic                            cfg_r_i,
  input  logic                            cfg_w_i,
  input  logic                            cfg_x_i,
  input  logic                            cfg_lock_i,
  input  logic                            cfg_valid_i,  // region enabled

  // Check port A — pulse_core wires this to instruction fetch.
  input  logic [31:0] a_addr_i,
  input  logic         a_is_fetch_i,
  input  logic         a_is_write_i,
  input  logic         a_priv_m_i,
  output logic         a_allow_o,

  // Check port B — pulse_core wires this to the LSU.
  input  logic [31:0] b_addr_i,
  input  logic         b_is_fetch_i,
  input  logic         b_is_write_i,
  input  logic         b_priv_m_i,
  output logic         b_allow_o
);

  logic [31:0] base_q   [NUM_REGIONS];
  logic [31:0] limit_q  [NUM_REGIONS];
  logic        r_q      [NUM_REGIONS];
  logic        w_q      [NUM_REGIONS];
  logic        x_q      [NUM_REGIONS];
  logic        lock_q   [NUM_REGIONS];
  logic        valid_q  [NUM_REGIONS];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int unsigned i = 0; i < NUM_REGIONS; i++) begin
        base_q[i]  <= 32'b0;
        limit_q[i] <= 32'b0;
        r_q[i]     <= 1'b0;
        w_q[i]     <= 1'b0;
        x_q[i]     <= 1'b0;
        lock_q[i]  <= 1'b0;
        valid_q[i] <= 1'b0;
      end
    end else if (cfg_we_i && !lock_q[cfg_idx_i]) begin
      // A locked region ignores further writes, same as PMP's L bit.
      base_q[cfg_idx_i]  <= cfg_base_i;
      limit_q[cfg_idx_i] <= cfg_limit_i;
      r_q[cfg_idx_i]     <= cfg_r_i;
      w_q[cfg_idx_i]     <= cfg_w_i;
      x_q[cfg_idx_i]     <= cfg_x_i;
      lock_q[cfg_idx_i]  <= cfg_lock_i;
      valid_q[cfg_idx_i] <= cfg_valid_i;
    end
  end

  function automatic logic check(
      input logic [31:0] addr, input logic is_fetch, input logic is_write, input logic priv_m);
    logic                           hit;
    logic [$clog2(NUM_REGIONS)-1:0] hit_idx;
    logic                           needs_r, needs_w, needs_x;
    logic                           region_permits;

    hit     = 1'b0;
    hit_idx = '0;
    for (int unsigned i = 0; i < NUM_REGIONS; i++) begin
      if (!hit && valid_q[i] && (addr >= base_q[i]) && (addr < limit_q[i])) begin
        hit     = 1'b1;
        hit_idx = i[$clog2(NUM_REGIONS)-1:0];
      end
    end

    needs_x = is_fetch;
    needs_w = !is_fetch & is_write;
    needs_r = !is_fetch & !is_write;
    region_permits = (needs_r & r_q[hit_idx]) | (needs_w & w_q[hit_idx]) |
                     (needs_x & x_q[hit_idx]);

    if (priv_m) begin
      // M-mode: unlocked regions don't restrict M-mode itself; a locked hit does.
      return !hit || !lock_q[hit_idx] || region_permits;
    end else begin
      // U-mode: no matching region is default-deny; a hit is gated by its permissions.
      return hit && region_permits;
    end
  endfunction

  assign a_allow_o = check(a_addr_i, a_is_fetch_i, a_is_write_i, a_priv_m_i);
  assign b_allow_o = check(b_addr_i, b_is_fetch_i, b_is_write_i, b_priv_m_i);

endmodule : pulse_mpu
