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

// pulse_lsu — Load/Store Unit (ReflexRV-Pulse), the MEM pipeline stage.
//
// Talks to a decoupled, variable-latency data memory port — the same
// contract pulse_ifu's imem port uses: dmem_req_o/dmem_we_o/dmem_addr_o/
// dmem_be_o/dmem_wdata_o present a request that stays held until dmem_gnt_i
// accepts it; dmem_valid_i then pulses (on some later, independent cycle)
// when dmem_rdata_i carries that request's result (or, for a write, just
// signals completion — dmem_rdata_i is meaningless then). Byte/halfword
// stores use dmem_be_o to mark the live lanes; dmem_wdata_o replicates the
// store data into the matching lane position and is don't-care elsewhere.
// Grant and valid are never asserted on the same cycle for the same
// request — see pulse_ifu's header for why that's the contract, not an
// accident.
//
// Every operation but a failed SC.W now needs a full request/grant/valid
// round trip, including a plain store — the old fixed-1-cycle contract let
// a store "complete" the instant it was issued (a synchronous-write SRAM
// doesn't need to report back), but a real bus target generally does need
// its own accept+complete handshake even for a write.
//
// Interface contract this relies on: req_i and every field alongside it
// (we_i/addr_i/wdata_i/size_i/sign_ext_i/atomic_i/amo_op_i) must stay
// stable from the cycle req_i first asserts until valid_o finally pulses.
// This module relies on that instead of latching its own copies — which is
// safe specifically because pulse_core's mem_stall already freezes ex_mem_q
// (the source of all of these) for exactly that whole window. The one
// exception is old_val_q: the AMO read-modify-write's intermediate read
// result, which is genuinely new information with nothing upstream to hold
// it, so it's the only thing this module actually latches.
//
// A read-modify-write AMO therefore runs two full request/grant/valid
// rounds back to back (read, then write-back with the combined value);
// everything else runs exactly one, and a failing SC.W runs zero — resolved
// combinationally against the reservation below, never touching memory.
//
// LR.W/SC.W reservation: a single address + valid bit, adequate for a
// single-hart core (this covers the RISC-V-mandated case of an intervening
// store from this hart itself breaking the reservation). It is cleared by
// any store this unit performs — a plain store, a successful SC.W, or an
// AMO's write-back — regardless of whether the address matches; that is a
// safe superset of the spec's minimum invalidation requirement, just
// possibly more conservative than necessary. A real multi-hart or bus-snoop
// invalidation scheme (for stores arriving from *other* agents) is out of
// scope here — there being only one hart in scope for this revision.
//
// Also out of scope: misaligned-access faulting. A misaligned word/halfword
// address is not detected or trapped; it silently reads/writes across the
// intended lane pattern instead. Add the alignment check once this core has
// exception plumbing to report it through.

module pulse_lsu
  import pulse_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  // From EX/MEM (see header: must stay stable until valid_o pulses).
  input  logic         req_i,
  input  logic         we_i,        // store vs. load; ignored when atomic_i
  input  logic [31:0]  addr_i,      // rs1 + imm
  input  logic [31:0]  wdata_i,     // rs2 (store data / AMO operand)
  input  mem_size_e    size_i,
  input  logic         sign_ext_i,
  input  logic         atomic_i,
  input  amo_op_e      amo_op_i,

  // Data memory port: decoupled request/grant, independent response valid.
  output logic        dmem_req_o,
  output logic        dmem_we_o,
  output logic [31:0] dmem_addr_o,
  output logic [3:0]  dmem_be_o,
  output logic [31:0] dmem_wdata_o,
  input  logic        dmem_gnt_i,
  input  logic        dmem_valid_i,
  input  logic [31:0] dmem_rdata_i,

  // Back to the pipeline.
  output logic         valid_o,   // rdata_o is valid this cycle
  output logic [31:0]  rdata_o    // load result, AMO pre-op value, or SC 0/1 outcome
);

  typedef enum logic [1:0] { S_IDLE, S_WAIT1, S_REQ2, S_WAIT2 } state_e;
  state_e state_q;

  logic [31:0] old_val_q;  // AMO's phase-1 (read) result, latched for phase 2

  logic        rsrv_valid_q;
  logic [31:0] rsrv_addr_q;

  // -----------------------------------------------------------------------
  // Store data/byte-enable packing.
  // -----------------------------------------------------------------------
  logic [3:0]  store_be;
  logic [31:0] store_wdata;
  always_comb begin
    unique case (size_i)
      MEM_B: begin
        unique case (addr_i[1:0])
          2'b00: begin store_be = 4'b0001; store_wdata = {24'b0, wdata_i[7:0]}; end
          2'b01: begin store_be = 4'b0010; store_wdata = {16'b0, wdata_i[7:0], 8'b0}; end
          2'b10: begin store_be = 4'b0100; store_wdata = {8'b0, wdata_i[7:0], 16'b0}; end
          2'b11: begin store_be = 4'b1000; store_wdata = {wdata_i[7:0], 24'b0}; end
        endcase
      end
      MEM_H: begin
        if (addr_i[1]) begin store_be = 4'b1100; store_wdata = {wdata_i[15:0], 16'b0}; end
        else            begin store_be = 4'b0011; store_wdata = {16'b0, wdata_i[15:0]}; end
      end
      default: begin store_be = 4'b1111; store_wdata = wdata_i; end  // MEM_W
    endcase
  end

  // -----------------------------------------------------------------------
  // Load-data extraction (shared by plain loads and LR.W's word-only case).
  // -----------------------------------------------------------------------
  function automatic logic [31:0] load_extract(
      input logic [31:0] rdata, input logic [1:0] addr_lsb,
      input mem_size_e sz, input logic sign_ext);
    logic [7:0]  byte_v;
    logic [15:0] half_v;
    unique case (sz)
      MEM_B: begin
        unique case (addr_lsb)
          2'b00: byte_v = rdata[7:0];
          2'b01: byte_v = rdata[15:8];
          2'b10: byte_v = rdata[23:16];
          2'b11: byte_v = rdata[31:24];
        endcase
        return sign_ext ? {{24{byte_v[7]}}, byte_v} : {24'b0, byte_v};
      end
      MEM_H: begin
        half_v = addr_lsb[1] ? rdata[31:16] : rdata[15:0];
        return sign_ext ? {{16{half_v[15]}}, half_v} : {16'b0, half_v};
      end
      default: return rdata;  // MEM_W
    endcase
  endfunction

  // -----------------------------------------------------------------------
  // AMO read-modify-write combine function.
  // -----------------------------------------------------------------------
  function automatic logic [31:0] amo_alu(
      input logic [31:0] old_val, input logic [31:0] rs2_val, input amo_op_e op);
    unique case (op)
      AMO_SWAP: return rs2_val;
      AMO_ADD:  return old_val + rs2_val;
      AMO_XOR:  return old_val ^ rs2_val;
      AMO_AND:  return old_val & rs2_val;
      AMO_OR:   return old_val | rs2_val;
      AMO_MIN:  return ($signed(old_val) < $signed(rs2_val)) ? old_val : rs2_val;
      AMO_MAX:  return ($signed(old_val) > $signed(rs2_val)) ? old_val : rs2_val;
      AMO_MINU: return (old_val < rs2_val) ? old_val : rs2_val;
      AMO_MAXU: return (old_val > rs2_val) ? old_val : rs2_val;
      default:  return old_val;  // LR/SC never reach here
    endcase
  endfunction

  logic sc_success;
  assign sc_success = rsrv_valid_q && (rsrv_addr_q == addr_i);

  // An atomic op other than LR/SC is one of the read-modify-write ops.
  logic is_rmw;
  assign is_rmw = atomic_i && (amo_op_i != AMO_LR) && (amo_op_i != AMO_SC);

  // -----------------------------------------------------------------------
  // Combinational request/response generation.
  // -----------------------------------------------------------------------
  always_comb begin
    dmem_req_o   = 1'b0;
    dmem_we_o    = 1'b0;
    dmem_addr_o  = addr_i;
    dmem_be_o    = 4'b1111;
    dmem_wdata_o = 32'b0;
    valid_o      = 1'b0;
    rdata_o      = 32'b0;

    unique case (state_q)
      S_IDLE: begin
        if (req_i) begin
          if (atomic_i) begin
            if (amo_op_i == AMO_SC) begin
              if (sc_success) begin
                dmem_req_o   = 1'b1;
                dmem_we_o    = 1'b1;
                dmem_wdata_o = wdata_i;
              end else begin
                valid_o = 1'b1;
                rdata_o = 32'd1;  // immediate failure, no memory access
              end
            end else begin
              dmem_req_o = 1'b1;  // LR.W or an RMW's read phase
            end
          end else if (we_i) begin
            dmem_req_o   = 1'b1;
            dmem_we_o    = 1'b1;
            dmem_be_o    = store_be;
            dmem_wdata_o = store_wdata;
          end else begin
            dmem_req_o = 1'b1;  // plain load
          end
        end
      end

      S_WAIT1: begin
        if (dmem_valid_i) begin
          if (atomic_i) begin
            if (amo_op_i == AMO_LR) begin
              valid_o = 1'b1;
              rdata_o = dmem_rdata_i;  // RV32A is word-only
            end else if (amo_op_i == AMO_SC) begin
              valid_o = 1'b1;
              rdata_o = 32'd0;  // success
            end
            // else: RMW read phase just landed — not done yet, no valid_o
            // this cycle; the state machine below moves on to phase 2.
          end else if (we_i) begin
            valid_o = 1'b1;  // plain store completion
          end else begin
            valid_o = 1'b1;
            rdata_o = load_extract(dmem_rdata_i, addr_i[1:0], size_i, sign_ext_i);
          end
        end
      end

      S_REQ2: begin
        dmem_req_o   = 1'b1;
        dmem_we_o    = 1'b1;
        dmem_wdata_o = amo_alu(old_val_q, wdata_i, amo_op_i);
      end

      S_WAIT2: begin
        if (dmem_valid_i) begin
          valid_o = 1'b1;
          rdata_o = old_val_q;  // AMO returns the pre-op value
        end
      end
    endcase
  end

  // -----------------------------------------------------------------------
  // State: request/grant/valid sequencing, the RMW intermediate value, and
  // the LR/SC reservation.
  // -----------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q      <= S_IDLE;
      old_val_q    <= 32'b0;
      rsrv_valid_q <= 1'b0;
      rsrv_addr_q  <= 32'b0;
    end else begin
      unique case (state_q)
        S_IDLE: begin
          // A failing SC.W (atomic_i && AMO_SC && !sc_success) never asserts
          // dmem_req_o (see the combinational block above) and must not be
          // gated into S_WAIT1 even if dmem_gnt_i happens to be high anyway
          // (e.g. a bridge that ties it to an idle-high ARREADY) — it
          // completes via valid_o combinationally, same cycle, full stop.
          if (req_i && !(atomic_i && amo_op_i == AMO_SC && !sc_success) && dmem_gnt_i) begin
            state_q <= S_WAIT1;
          end
        end

        S_WAIT1: begin
          if (dmem_valid_i) begin
            if (is_rmw) begin
              old_val_q <= dmem_rdata_i;
              state_q   <= S_REQ2;
            end else begin
              state_q <= S_IDLE;
              if (atomic_i && amo_op_i == AMO_LR) begin
                rsrv_valid_q <= 1'b1;
                rsrv_addr_q  <= addr_i;
              end else if (atomic_i && amo_op_i == AMO_SC) begin
                rsrv_valid_q <= 1'b0;  // successful SC consumes it
              end else if (!atomic_i && we_i) begin
                rsrv_valid_q <= 1'b0;  // any plain store invalidates it
              end
            end
          end
        end

        S_REQ2: begin
          if (dmem_gnt_i) state_q <= S_WAIT2;
        end

        S_WAIT2: begin
          if (dmem_valid_i) begin
            state_q      <= S_IDLE;
            rsrv_valid_q <= 1'b0;  // the write-back this cycle invalidates it
          end
        end
      endcase
    end
  end

endmodule : pulse_lsu
