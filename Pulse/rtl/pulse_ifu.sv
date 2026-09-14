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
// Sequential (PC+4) instruction fetch against a decoupled, variable-latency
// instruction memory port: imem_req_o/imem_addr_o present a request that
// stays held (unchanged) until imem_gnt_i accepts it; independently,
// imem_valid_i pulses for exactly one cycle whenever imem_rdata_i carries
// that (one, since this module is single-outstanding) accepted request's
// data. Grant and valid are never asserted on the same cycle for the same
// request — there is always at least one cycle between "accepted" and
// "here's your data" — which is what a real memory or bus bridge (e.g.
// pulse_l1sys) actually looks like, unlike the old fixed-1-cycle-latency
// contract this module used before. Because valid_i's timing is no longer
// fixed, instr_o is now a real register (instr_data_q) rather than a
// passthrough of imem_rdata_i — the old design could get away with wiring
// straight through because the memory was assumed to hold its output
// indefinitely; this module can't assume that about imem_rdata_i, which is
// only meaningful for the one cycle imem_valid_i pulses.
//
// stall_i gates two separate things, not just one: it prevents issuing a new
// request (an already-outstanding one always runs to completion regardless —
// outstanding_q serializes that on its own), and — just as importantly —
// instr_valid_o drops the instant stall_i first reads low (consumed_this_cycle),
// not whenever the next fetch's grant happens to land. Those two events can
// be many cycles apart under variable memory latency, and conflating them
// would let instr_valid_o sit high with stall_i low across several cycles
// while genuinely still fetching — indistinguishable, from the consumer's
// side, from a second brand-new instruction, and enough on its own to get an
// instruction latched and executed twice.
//
// redirect_valid_i / redirect_pc_i implement branch/jump redirect. The
// subtlety here (found by simulation, not by inspection, in a tight-loop
// test that redirects to the same address over and over — see the RTL
// history for the failing trace) is that pc_q can't just be clobbered
// unconditionally: imem_addr_o must stay stable, per the port's own
// contract above, from the cycle a request first appears until imem_gnt_i
// accepts it. If a redirect arrives while such a request is still waiting
// on its grant (addr_phase_active, and that grant isn't landing on this
// very cycle), changing pc_q right then would change imem_addr_o out from
// under an address phase a real bus target already has in flight — a
// protocol violation, not just a timing quirk, and exactly how a stale
// grant ends up read back against the *new* address, silently corrupting
// which instruction's data lands where. Such a redirect is deferred
// instead (pending_redirect_q/pending_redirect_pc_q) and only actually
// applied to pc_q once that outstanding grant finally lands — at which
// point the address bus is free again and it's safe. A redirect that
// arrives while nothing is mid-request (idle, or already past the grant
// and merely waiting on data, or racing a grant landing on this same
// cycle) applies immediately, same as before; discard_q handles making
// sure whatever that stranded request eventually returns gets thrown away
// rather than presented as the next instruction, in both cases alike.

module pulse_ifu #(
  parameter logic [31:0] RESET_PC = 32'h0000_0000
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Backpressure from the next stage.
  input  logic        stall_i,

  // Redirect from a later stage (branch/jump resolution, trap entry, ...).
  input  logic         redirect_valid_i,
  input  logic [31:0]  redirect_pc_i,

  // Instruction memory port: decoupled request/grant, independent response valid.
  output logic        imem_req_o,
  output logic [31:0] imem_addr_o,
  input  logic        imem_gnt_i,
  input  logic        imem_valid_i,
  input  logic [31:0] imem_rdata_i,

  // To the next pipeline stage.
  output logic        instr_valid_o,
  output logic [31:0] instr_o,
  output logic [31:0] instr_pc_o
);

  logic [31:0] pc_q;          // PC of the outstanding/next fetch request
  logic [31:0] pc_out_q;      // PC of instr_data_q
  logic [31:0] instr_data_q;  // latched instruction data (see header: no longer a passthrough)
  logic        valid_q;       // instr_o / instr_pc_o are valid
  logic        outstanding_q; // a request for pc_q has been granted; waiting for imem_valid_i
  logic        discard_q;     // the outstanding (or about-to-be-outstanding) request is wrong-path

  logic        pending_redirect_q;     // a redirect arrived while the bus couldn't accept it yet
  logic [31:0] pending_redirect_pc_q;  // ... and this is where it's actually headed

  logic [31:0] pc_next;
  assign pc_next = pc_q + 32'd4;

  logic completes_this_cycle;           // the outstanding request's data is arriving now
  logic becomes_outstanding_this_cycle; // a fresh request is being accepted now
  assign completes_this_cycle           = outstanding_q & imem_valid_i;
  assign becomes_outstanding_this_cycle = ~outstanding_q & ~stall_i & imem_gnt_i;

  // The consumer is accepting the currently-held instruction this cycle.
  // Note this is deliberately *not* becomes_outstanding_this_cycle: a grant
  // for the next fetch can land the same cycle (fast memory) or many cycles
  // later, but instr_valid_o must drop the instant it's accepted regardless.
  logic consumed_this_cycle;
  assign consumed_this_cycle = valid_q & ~stall_i;

  // An un-granted request is currently sitting on the bus (imem_req_o's own
  // condition, named separately here for what it means to a redirect).
  logic addr_phase_active;
  assign addr_phase_active = ~outstanding_q & ~stall_i;

  // A redirect can't safely touch pc_q this cycle if there's a request
  // already on the bus that hasn't been granted *this* cycle either — see
  // header. (If imem_gnt_i is high this same cycle, the address is being
  // sampled right now at its old, still-stable value; the pc_q update below
  // only takes effect at the next edge, so applying the redirect
  // immediately is safe precisely in that case.)
  logic needs_defer;
  assign needs_defer = addr_phase_active & ~imem_gnt_i;

  // A redirect needs to mark something for discard only if it actually
  // strands a request in flight — one already outstanding, or one racing the
  // redirect into outstanding this very cycle — and that request isn't also
  // resolving on this same cycle (in which case there's nothing left to
  // discard later; the valid_q/pc_q update below handles that directly).
  // Never true while needs_defer is — deferring means outstanding_q is 0 and
  // no grant is landing this cycle, so neither disjunct here can hold yet;
  // the deferred case gets its own discard trigger below (apply_deferred).
  logic strands_in_flight;
  assign strands_in_flight = redirect_valid_i & ~completes_this_cycle &
                              (outstanding_q | becomes_outstanding_this_cycle);

  // A previously-deferred redirect is finally being applied: the grant it
  // was waiting on just landed, for a request that's now known wrong-path
  // (it was issued before the redirect was even known about) — so this is
  // the deferred case's own trigger for discard_q, parallel to
  // strands_in_flight above, not covered by it (strands_in_flight fires off
  // a *fresh* redirect_valid_i pulse; by the time apply_deferred fires, that
  // pulse is long gone).
  logic apply_deferred;
  assign apply_deferred = becomes_outstanding_this_cycle & pending_redirect_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      pc_q                  <= RESET_PC;
      pc_out_q              <= RESET_PC;
      instr_data_q          <= 32'b0;
      valid_q               <= 1'b0;
      outstanding_q         <= 1'b0;
      discard_q             <= 1'b0;
      pending_redirect_q    <= 1'b0;
      pending_redirect_pc_q <= 32'b0;
    end else begin
      if (strands_in_flight || apply_deferred) discard_q <= 1'b1;
      else if (completes_this_cycle)  discard_q <= 1'b0;

      if (completes_this_cycle)               outstanding_q <= 1'b0;
      else if (becomes_outstanding_this_cycle) outstanding_q <= 1'b1;

      if (redirect_valid_i && needs_defer) begin
        pending_redirect_q    <= 1'b1;
        pending_redirect_pc_q <= redirect_pc_i;
      end else if ((redirect_valid_i && !needs_defer) || apply_deferred) begin
        // Either an immediately-applicable redirect supersedes any stale
        // deferred one, or a deferred one is being applied right now —
        // either way, nothing left pending after this.
        pending_redirect_q <= 1'b0;
      end

      if (redirect_valid_i && !needs_defer) begin
        pc_q <= redirect_pc_i;
      end else if (apply_deferred) begin
        pc_q <= pending_redirect_pc_q;
      end else if (!redirect_valid_i && completes_this_cycle && !discard_q) begin
        pc_q <= pc_next;
      end
      // else: hold pc_q (including, deliberately, the needs_defer case).

      if (!redirect_valid_i && completes_this_cycle && !discard_q) begin
        pc_out_q     <= pc_q;
        instr_data_q <= imem_rdata_i;
      end

      if (redirect_valid_i) begin
        valid_q <= 1'b0;
      end else if (completes_this_cycle) begin
        valid_q <= discard_q ? 1'b0 : 1'b1;
      end else if (consumed_this_cycle) begin
        valid_q <= 1'b0;
      end
      // else: hold (mid-flight wait, or idle-and-stalled).
    end
  end

  assign imem_req_o  = rst_ni & ~outstanding_q & ~stall_i;
  assign imem_addr_o = pc_q;

  assign instr_valid_o = valid_q;
  assign instr_o       = instr_data_q;
  assign instr_pc_o    = pc_out_q;

endmodule : pulse_ifu
