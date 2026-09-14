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

// pulse_l1sys — L1-to-system-bus bridge (ReflexRV-Pulse).
//
// Arbitrates pulse_core's two CPU-side memory ports (instruction fetch and
// data access — port shapes match pulse_ifu's imem_* and pulse_lsu's dmem_*
// exactly, decoupled req/gnt + independent valid, see their headers) onto a
// single AXI4 master with a 256-bit (32-byte) data channel. This is a
// standalone module, not instantiated inside pulse_core: pulse_core stays a
// plain CPU pipeline that knows nothing about AXI, and a future SoC-level
// wrapper is what would place pulse_l1sys between it and the interconnect.
//
// Single-outstanding: at most one AXI transaction is in flight at a time,
// matching pulse_core's own memory ports (neither pulse_ifu nor pulse_lsu
// ever has more than one request outstanding). When both CPU-side ports
// want the bus in the same cycle, the data port wins — a stalled load/store
// blocks retirement more directly than a stalled fetch blocks it, so it
// gets to go first. The instruction port simply keeps re-presenting its
// request (per its own req/gnt contract) until its turn comes.
//
// Narrow transfers, not line fills: every CPU-side access — one 32-bit word
// (with byte lanes marked by d_be_i for a data write; a full word for every
// instruction fetch and every other data access) — becomes exactly one
// AXI beat sized to that access (ARSIZE/AWSIZE = 3'b010, 4 bytes), placed at
// its natural byte lane within the 256-bit bus (lane = addr[4:2], one of 8
// 32-bit lanes). This uses the bus's width without changing this module's
// job into a cache's: it does not fetch or buffer more than what was asked
// for, so it earns no bandwidth amortization from the wider bus by itself.
// Coalescing multiple narrow CPU-side accesses that land in the same
// 256-bit line into fewer, wider AXI bursts — the way an actual L1 cache
// would — is a natural, valuable follow-on, and a big enough design on its
// own (tags, line state, fills, eviction) that it belongs in a separate
// module (something like a future pulse_l1cache in front of this one), not
// folded into a bridge named for what it is here: a protocol/width
// converter.
//
// AXI-side simplifications made explicit: every transaction is a single
// beat (ARLEN/AWLEN = 0, ARBURST/AWBURST = INCR per convention though burst
// mode is moot at length 1); all IDs are tied to a constant (single-
// outstanding means there is never more than one transaction to
// distinguish); a write's completion is signaled only once BVALID/BREADY
// closes the transaction (not merely on AWREADY/WREADY), so d_valid_o for a
// store means the interconnect actually accepted it, not just that this
// module handed it off; RRESP/BRESP are not checked (no error reporting
// path exists yet upstream — see pulse_core's own list of what pulse_csr
// still needs to provide).

module pulse_l1sys #(
  parameter int unsigned AxiIdWidth = 4
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Instruction-fetch CPU-side port (matches pulse_core's imem_* port).
  input  logic         i_req_i,
  input  logic [31:0]  i_addr_i,
  output logic         i_gnt_o,
  output logic         i_valid_o,
  output logic [31:0]  i_rdata_o,

  // Data CPU-side port (matches pulse_core's dmem_* port).
  input  logic         d_req_i,
  input  logic         d_we_i,
  input  logic [31:0]  d_addr_i,
  input  logic [3:0]   d_be_i,
  input  logic [31:0]  d_wdata_i,
  output logic         d_gnt_o,
  output logic         d_valid_o,
  output logic [31:0]  d_rdata_o,

  // AXI4 master, 256-bit data.
  output logic [AxiIdWidth-1:0] m_axi_arid_o,
  output logic [31:0]           m_axi_araddr_o,
  output logic [7:0]            m_axi_arlen_o,
  output logic [2:0]            m_axi_arsize_o,
  output logic [1:0]            m_axi_arburst_o,
  output logic                  m_axi_arvalid_o,
  input  logic                  m_axi_arready_i,

  /* verilator lint_off UNUSED */
  input  logic [AxiIdWidth-1:0] m_axi_rid_i,    // single-outstanding: nothing to match it against
  /* verilator lint_on UNUSED */
  input  logic [255:0]          m_axi_rdata_i,
  /* verilator lint_off UNUSED */
  input  logic [1:0]            m_axi_rresp_i,  // no error-reporting path upstream yet — see header
  input  logic                  m_axi_rlast_i,  // always true at burst length 1; not checked
  /* verilator lint_on UNUSED */
  input  logic                  m_axi_rvalid_i,
  output logic                  m_axi_rready_o,

  output logic [AxiIdWidth-1:0] m_axi_awid_o,
  output logic [31:0]           m_axi_awaddr_o,
  output logic [7:0]            m_axi_awlen_o,
  output logic [2:0]            m_axi_awsize_o,
  output logic [1:0]            m_axi_awburst_o,
  output logic                  m_axi_awvalid_o,
  input  logic                  m_axi_awready_i,

  output logic [255:0]          m_axi_wdata_o,
  output logic [31:0]           m_axi_wstrb_o,
  output logic                  m_axi_wlast_o,
  output logic                  m_axi_wvalid_o,
  input  logic                  m_axi_wready_i,

  /* verilator lint_off UNUSED */
  input  logic [AxiIdWidth-1:0] m_axi_bid_i,    // single-outstanding: nothing to match it against
  input  logic [1:0]            m_axi_bresp_i,  // no error-reporting path upstream yet — see header
  /* verilator lint_on UNUSED */
  input  logic                  m_axi_bvalid_i,
  output logic                  m_axi_bready_o
);

  typedef enum logic [2:0] { S_IDLE, S_AR, S_R, S_AW_W, S_B } state_e;
  state_e state_q;

  logic        is_instr_q;  // the in-flight transaction came from the i_* port (always a read)
  logic [31:0] addr_q;      // word-aligned AXI address
  logic [2:0]  lane_q;      // addr[4:2]: which of the 8 32-bit lanes on the 256-bit bus
  logic [3:0]  be_q;
  logic [31:0] wdata_q;

  logic        aw_done_q, w_done_q;  // independent AW/W handshake completion, within S_AW_W

  // ---------------------------------------------------------------------
  // Arbitration (only meaningful in S_IDLE): data port wins ties.
  // ---------------------------------------------------------------------
  logic grant_d, grant_i;
  assign grant_d = (state_q == S_IDLE) && d_req_i;
  assign grant_i = (state_q == S_IDLE) && i_req_i && !d_req_i;

  logic [31:0] sel_addr;
  logic        sel_we;
  logic [3:0]  sel_be;
  logic [31:0] sel_wdata;
  assign sel_addr  = grant_d ? d_addr_i  : i_addr_i;
  assign sel_we    = grant_d ? d_we_i    : 1'b0;
  assign sel_be    = grant_d ? d_be_i    : 4'b1111;
  assign sel_wdata = grant_d ? d_wdata_i : 32'b0;

  assign i_gnt_o = grant_i;
  assign d_gnt_o = grant_d;

  // ---------------------------------------------------------------------
  // State machine.
  // ---------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q   <= S_IDLE;
      aw_done_q <= 1'b0;
      w_done_q  <= 1'b0;
    end else begin
      unique case (state_q)
        S_IDLE: begin
          if (grant_d || grant_i) begin
            is_instr_q <= grant_i;
            addr_q     <= sel_addr & ~32'd3;
            lane_q     <= sel_addr[4:2];
            be_q       <= sel_be;
            wdata_q    <= sel_wdata;
            aw_done_q  <= 1'b0;
            w_done_q   <= 1'b0;
            state_q    <= sel_we ? S_AW_W : S_AR;
          end
        end

        S_AR: if (m_axi_arready_i) state_q <= S_R;

        S_R: if (m_axi_rvalid_i) state_q <= S_IDLE;

        S_AW_W: begin
          if (m_axi_awready_i) aw_done_q <= 1'b1;
          if (m_axi_wready_i)  w_done_q  <= 1'b1;
          if ((aw_done_q || m_axi_awready_i) && (w_done_q || m_axi_wready_i)) begin
            state_q <= S_B;
          end
        end

        S_B: if (m_axi_bvalid_i) state_q <= S_IDLE;

        default: state_q <= S_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // AXI request-side outputs.
  // ---------------------------------------------------------------------
  assign m_axi_arid_o    = '0;
  assign m_axi_araddr_o  = addr_q;
  assign m_axi_arlen_o   = 8'd0;
  assign m_axi_arsize_o  = 3'b010;
  assign m_axi_arburst_o = 2'b01;  // INCR (moot at length 1, kept for convention)
  assign m_axi_arvalid_o = (state_q == S_AR);

  assign m_axi_awid_o    = '0;
  assign m_axi_awaddr_o  = addr_q;
  assign m_axi_awlen_o   = 8'd0;
  assign m_axi_awsize_o  = 3'b010;
  assign m_axi_awburst_o = 2'b01;
  assign m_axi_awvalid_o = (state_q == S_AW_W) && !aw_done_q;

  assign m_axi_wdata_o   = {8{wdata_q}};             // replicate; wstrb picks the real bytes
  assign m_axi_wstrb_o   = {28'b0, be_q} << (lane_q * 4);
  assign m_axi_wlast_o   = 1'b1;                      // single beat
  assign m_axi_wvalid_o  = (state_q == S_AW_W) && !w_done_q;

  assign m_axi_rready_o = (state_q == S_R);
  assign m_axi_bready_o = (state_q == S_B);

  // ---------------------------------------------------------------------
  // CPU-side responses.
  // ---------------------------------------------------------------------
  logic [31:0] read_lane;
  assign read_lane = m_axi_rdata_i[lane_q*32 +: 32];

  assign i_valid_o = (state_q == S_R) && m_axi_rvalid_i && is_instr_q;
  assign i_rdata_o = read_lane;

  assign d_valid_o = ((state_q == S_R) && m_axi_rvalid_i && !is_instr_q) ||
                      ((state_q == S_B) && m_axi_bvalid_i);
  assign d_rdata_o = read_lane;  // don't-care for a write completion

endmodule : pulse_l1sys
