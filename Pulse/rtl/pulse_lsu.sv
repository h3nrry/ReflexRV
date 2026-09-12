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
// Talks to a synchronous, single-cycle-read-latency data memory (same
// contract as pulse_ifu's imem port): an address driven on dmem_addr_o is
// sampled this edge, and the corresponding word appears on dmem_rdata_i one
// cycle later. Byte/halfword stores use dmem_be_o to mark the live lanes;
// dmem_wdata_o replicates the store data into the matching lane position and
// is don't-care elsewhere.
//
// Timing:
//   - A plain store completes the same cycle it's issued (stall_o low,
//     valid_o high) — a synchronous-write memory doesn't need a round trip.
//   - A plain load, LR.W, and the read phase of an AMO read-modify-write all
//     need the memory's one-cycle turnaround: stall_o goes high the cycle
//     the read is issued, and the result appears (with, for an RMW op, the
//     write phase issued in the very same cycle) the cycle after.
//   - SC.W is resolved combinationally against the reservation below and
//     never touches memory on failure, so it always completes same-cycle
//     regardless of outcome.
//
// LR.W/SC.W reservation: a single address + valid bit, adequate for a
// single-hart core (this covers the RISC-V-mandated case of an intervening
// store from this hart itself breaking the reservation). It is cleared by
// any store this unit performs — a plain store, a successful SC.W, or an
// AMO's write phase — regardless of whether the address matches; that is a
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

  // From EX/MEM.
  input  logic         req_i,
  input  logic         we_i,        // store vs. load; ignored when atomic_i
  input  logic [31:0]  addr_i,      // rs1 + imm
  input  logic [31:0]  wdata_i,     // rs2 (store data / AMO operand)
  input  mem_size_e    size_i,
  input  logic         sign_ext_i,
  input  logic         atomic_i,
  input  amo_op_e      amo_op_i,

  // Data memory port (synchronous, 1-cycle read latency).
  output logic        dmem_req_o,
  output logic        dmem_we_o,
  output logic [31:0] dmem_addr_o,
  output logic [3:0]  dmem_be_o,
  output logic [31:0] dmem_wdata_o,
  input  logic [31:0] dmem_rdata_i,

  // Back to the pipeline.
  output logic         valid_o,   // rdata_o is valid this cycle
  output logic         stall_o,   // freeze the pipeline: a multi-cycle op is in flight
  output logic [31:0]  rdata_o    // load result, AMO pre-op value, or SC 0/1 outcome
);

  typedef enum logic [1:0] { PENDING_LOAD, PENDING_LR, PENDING_RMW } pending_kind_e;

  logic          waiting_q;
  pending_kind_e kind_q;
  logic [31:0]   addr_q;
  logic [31:0]   rs2_q;
  mem_size_e     size_q;
  logic          sign_ext_q;
  amo_op_e       amo_op_q;

  logic          rsrv_valid_q;
  logic [31:0]   rsrv_addr_q;

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

  // -----------------------------------------------------------------------
  // Combinational request/response generation.
  // -----------------------------------------------------------------------
  logic sc_success;
  assign sc_success = rsrv_valid_q && (rsrv_addr_q == addr_i);

  always_comb begin
    dmem_req_o   = 1'b0;
    dmem_we_o    = 1'b0;
    dmem_addr_o  = addr_i;
    dmem_be_o    = 4'b1111;
    dmem_wdata_o = 32'b0;
    valid_o      = 1'b0;
    stall_o      = 1'b0;
    rdata_o      = 32'b0;

    if (!waiting_q) begin
      if (req_i) begin
        if (atomic_i) begin
          unique case (amo_op_i)
            AMO_LR: begin
              dmem_req_o  = 1'b1;
              dmem_we_o   = 1'b0;
              dmem_addr_o = addr_i;
              stall_o     = 1'b1;
            end
            AMO_SC: begin
              valid_o = 1'b1;
              if (sc_success) begin
                dmem_req_o   = 1'b1;
                dmem_we_o    = 1'b1;
                dmem_addr_o  = addr_i;
                dmem_be_o    = 4'b1111;
                dmem_wdata_o = wdata_i;
                rdata_o      = 32'd0;  // success
              end else begin
                rdata_o = 32'd1;  // failure
              end
            end
            default: begin  // RMW ops: issue the read phase
              dmem_req_o  = 1'b1;
              dmem_we_o   = 1'b0;
              dmem_addr_o = addr_i;
              stall_o     = 1'b1;
            end
          endcase
        end else if (we_i) begin
          dmem_req_o   = 1'b1;
          dmem_we_o    = 1'b1;
          dmem_addr_o  = addr_i;
          dmem_be_o    = store_be;
          dmem_wdata_o = store_wdata;
          valid_o      = 1'b1;
        end else begin
          dmem_req_o  = 1'b1;
          dmem_we_o   = 1'b0;
          dmem_addr_o = addr_i;
          stall_o     = 1'b1;
        end
      end
    end else begin
      // dmem_rdata_i now holds the data for addr_q, requested last cycle.
      unique case (kind_q)
        PENDING_LOAD: begin
          valid_o = 1'b1;
          rdata_o = load_extract(dmem_rdata_i, addr_q[1:0], size_q, sign_ext_q);
        end
        PENDING_LR: begin
          valid_o = 1'b1;
          rdata_o = dmem_rdata_i;  // RV32A is word-only
        end
        PENDING_RMW: begin
          valid_o      = 1'b1;
          rdata_o      = dmem_rdata_i;  // AMO returns the pre-op value
          dmem_req_o   = 1'b1;
          dmem_we_o    = 1'b1;
          dmem_addr_o  = addr_q;
          dmem_be_o    = 4'b1111;
          dmem_wdata_o = amo_alu(dmem_rdata_i, rs2_q, amo_op_q);
        end
        default: ;  // kind_q is 2 bits for a 3-value enum; unreachable, defaults above hold
      endcase
    end
  end

  // -----------------------------------------------------------------------
  // State: the wait-for-read-data latch, and the LR/SC reservation.
  // -----------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      waiting_q    <= 1'b0;
      rsrv_valid_q <= 1'b0;
      rsrv_addr_q  <= 32'b0;
    end else begin
      if (!waiting_q) begin
        if (req_i && !(atomic_i && (amo_op_i == AMO_SC))) begin
          // Every path above except SC.W either finishes same-cycle (plain
          // store) or needs one more cycle for read data; only the latter
          // needs to latch anything. Within this branch SC.W is already
          // excluded, so "atomic, or a non-atomic load" covers exactly
          // LR.W/RMW/plain-load and excludes only the plain-store case.
          if (atomic_i || !we_i) begin
            waiting_q  <= 1'b1;
            addr_q     <= addr_i;
            rs2_q      <= wdata_i;
            size_q     <= size_i;
            sign_ext_q <= sign_ext_i;
            amo_op_q   <= amo_op_i;
            kind_q     <= atomic_i ? (amo_op_i == AMO_LR ? PENDING_LR : PENDING_RMW)
                                    : PENDING_LOAD;
          end
        end
        // Reservation updates for the same-cycle-completing ops.
        if (req_i && !atomic_i && we_i) begin
          rsrv_valid_q <= 1'b0;  // any plain store invalidates it
        end else if (req_i && atomic_i && amo_op_i == AMO_SC && sc_success) begin
          rsrv_valid_q <= 1'b0;  // a successful SC consumes it
        end
      end else begin
        waiting_q <= 1'b0;
        if (kind_q == PENDING_LR) begin
          rsrv_valid_q <= 1'b1;
          rsrv_addr_q  <= addr_q;
        end else if (kind_q == PENDING_RMW) begin
          rsrv_valid_q <= 1'b0;  // the write phase this cycle invalidates it
        end
      end
    end
  end

endmodule : pulse_lsu
