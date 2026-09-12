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

// pulse_muldiv — M-extension multiply/divide unit (ReflexRV-Pulse).
//
// Sequential shift-add multiplier / shift-subtract restoring divider, one bit
// per cycle. Every operation — MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU, any
// operand values, including divide-by-zero and the signed-overflow case —
// takes exactly the same fixed number of cycles (33: one setup/latch cycle
// implicit in the start_i->busy_o transition, 32 iteration cycles, with the
// final iteration also registering the sign-corrected result). No early-out
// for e.g. a small divisor or a zero operand: fixed latency, not best-case
// latency, is the point on a core built around bounded WCET — data-dependent
// timing is exactly what this module is written to avoid.
//
// Interface: level-sensitive start_i. Assert it for as long as the calling
// instruction occupies EX (which pulse_exu does naturally, by just wiring it
// to the decoded mul_div control bit for that instruction); this module only
// acts on it once, on the IDLE->busy transition, and ignores it thereafter
// until it returns to IDLE. valid_o pulses for exactly one cycle when
// result_o is ready. Callers should derive their own EX-stall condition as
// (mul_div_instruction & ~valid_o) rather than from busy_o directly — busy_o
// only goes high the cycle *after* start_i, so gating a stall on busy_o alone
// would fail to hold the pipeline during that first setup cycle.

module pulse_muldiv
  import pulse_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic         start_i,
  input  logic [31:0]  operand_a_i,  // rs1
  input  logic [31:0]  operand_b_i,  // rs2
  input  muldiv_op_e   op_i,

  output logic         busy_o,
  output logic         valid_o,
  output logic [31:0]  result_o
);

  // ---------------------------------------------------------------------
  // Per-op setup: signedness of each operand, and which half of the result
  // (multiply) or which of quotient/remainder (divide) this op wants.
  // ---------------------------------------------------------------------
  logic is_div, signed_a, signed_b, want_high, want_rem;

  always_comb begin
    is_div    = 1'b0;
    signed_a  = 1'b0;
    signed_b  = 1'b0;
    want_high = 1'b0;
    want_rem  = 1'b0;
    unique case (op_i)
      MULDIV_MUL:    begin signed_a = 1'b1; signed_b = 1'b1; end
      MULDIV_MULH:   begin signed_a = 1'b1; signed_b = 1'b1; want_high = 1'b1; end
      MULDIV_MULHSU: begin signed_a = 1'b1;                  want_high = 1'b1; end
      MULDIV_MULHU:  begin                                   want_high = 1'b1; end
      MULDIV_DIV:    begin is_div = 1'b1; signed_a = 1'b1; signed_b = 1'b1; end
      MULDIV_DIVU:   begin is_div = 1'b1; end
      MULDIV_REM:    begin is_div = 1'b1; signed_a = 1'b1; signed_b = 1'b1; want_rem = 1'b1; end
      MULDIV_REMU:   begin is_div = 1'b1; want_rem = 1'b1; end
    endcase
  end

  logic sign_a, sign_b;
  assign sign_a = signed_a & operand_a_i[31];
  assign sign_b = signed_b & operand_b_i[31];

  logic [31:0] abs_a, abs_b;
  assign abs_a = sign_a ? -operand_a_i : operand_a_i;
  assign abs_b = sign_b ? -operand_b_i : operand_b_i;

  logic divisor_is_zero;
  assign divisor_is_zero = is_div & (operand_b_i == 32'b0);
  logic div_overflow;  // most-negative dividend / -1
  assign div_overflow = is_div & signed_a & (operand_a_i == 32'h8000_0000) &&
                        (operand_b_i == 32'hFFFF_FFFF);

  // ---------------------------------------------------------------------
  // Datapath: one combined 64-bit register pair reused for both algorithms
  // (multiply and divide never run concurrently), plus the one fixed 32-bit
  // operand each iteration needs (multiplicand, or divisor).
  // ---------------------------------------------------------------------
  logic [31:0] dp_hi_q, dp_lo_q;
  logic [31:0] fixed_operand_q;

  logic        is_div_q, want_high_q, want_rem_q;
  logic        result_sign_q;     // negate the extracted magnitude (product/quotient)
  logic        rem_sign_q;        // sign to apply to the remainder (REM only: dividend's sign)
  logic        divisor_is_zero_q, div_overflow_q;
  logic [31:0] op_a_q;            // original dividend, for the div-by-zero/overflow special cases

  logic [5:0]  cnt_q;
  logic        busy_q;

  // Multiply step: classic shift-add — add the fixed operand into the high
  // half when the LSB of the low half is set, then shift the combined
  // 64-bit register right by one.
  logic [32:0] mul_sum;
  assign mul_sum = {1'b0, dp_hi_q} + (dp_lo_q[0] ? {1'b0, fixed_operand_q} : 33'b0);

  // Divide step: classic restoring division — shift the combined register
  // left by one, then subtract the divisor from the high half if it fits,
  // recording that outcome as the new quotient LSB.
  logic [63:0] div_shifted;
  logic        div_fits;
  assign div_shifted = {dp_hi_q, dp_lo_q} << 1;
  assign div_fits    = div_shifted[63:32] >= fixed_operand_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q  <= 1'b0;
      valid_o <= 1'b0;
      cnt_q   <= '0;
    end else begin
      valid_o <= 1'b0;  // default: pulses only on the cycle a result completes

      if (!busy_q) begin
        if (start_i) begin
          busy_q          <= 1'b1;
          cnt_q           <= 6'd31;
          is_div_q        <= is_div;
          want_high_q     <= want_high;
          want_rem_q      <= want_rem;
          result_sign_q   <= sign_a ^ sign_b;
          rem_sign_q      <= sign_a;
          divisor_is_zero_q <= divisor_is_zero;
          div_overflow_q  <= div_overflow;
          op_a_q          <= operand_a_i;
          fixed_operand_q <= is_div ? abs_b : abs_a;
          if (is_div) begin
            dp_hi_q <= 32'b0;
            dp_lo_q <= abs_a;  // dividend magnitude
          end else begin
            dp_hi_q <= 32'b0;
            dp_lo_q <= abs_b;  // multiplier magnitude
          end
        end
      end else begin
        // One iteration.
        if (is_div_q) begin
          if (div_fits) begin
            dp_hi_q <= div_shifted[63:32] - fixed_operand_q;
            dp_lo_q <= {div_shifted[31:1], 1'b1};
          end else begin
            dp_hi_q <= div_shifted[63:32];
            dp_lo_q <= div_shifted[31:0];
          end
        end else begin
          {dp_hi_q, dp_lo_q} <= {mul_sum, dp_lo_q[31:1]};
        end

        if (cnt_q == 6'd0) begin
          busy_q  <= 1'b0;
          valid_o <= 1'b1;
        end else begin
          cnt_q <= cnt_q - 6'd1;
        end
      end
    end
  end

  assign busy_o = busy_q;

  // ---------------------------------------------------------------------
  // Final result extraction. By the cycle valid_o is high, the transition
  // that just landed (the 32nd and last iteration) has already updated
  // dp_hi_q/dp_lo_q in place — they directly hold the finished
  // product/{remainder,quotient} pair, no separate "final" shadow needed.
  // (An earlier draft recomputed one more step from these already-final
  // registers here, which silently produced the 33rd, wrong, iteration —
  // caught by simulation, not by inspection; left as a cautionary comment.)
  // ---------------------------------------------------------------------
  // Multiply: quotient/remainder aren't in play, but the sign correction is —
  // and it must be applied to the full 64-bit product before splitting into
  // halves, not to each 32-bit half independently. A borrow out of the low
  // word can propagate into the high word (e.g. -1 = 64'hFFFF_FFFF_FFFF_FFFF,
  // whose high word is all-ones even though the *unsigned* high word before
  // negation was zero) — negating dp_hi_q on its own silently drops that
  // borrow. Caught by simulation (MULH -1*1), not by inspection.
  logic [63:0] mul_mag, mul_signed;
  assign mul_mag    = {dp_hi_q, dp_lo_q};
  assign mul_signed = result_sign_q ? -mul_mag : mul_mag;

  // Divide: quotient and remainder are independent 32-bit values (each within
  // the 32-bit magnitude range since dividend/divisor already were), so each
  // gets its own ordinary 32-bit negation — no cross-word borrow to worry
  // about here.
  logic [31:0] quotient_mag, remainder_mag;
  assign quotient_mag  = dp_lo_q;
  assign remainder_mag = dp_hi_q;

  logic [31:0] quotient_signed, div_result, rem_result;
  assign quotient_signed = result_sign_q ? -quotient_mag : quotient_mag;
  assign div_result      = divisor_is_zero_q ? 32'hFFFF_FFFF
                          : div_overflow_q   ? op_a_q
                                             : quotient_signed;
  assign rem_result      = divisor_is_zero_q ? op_a_q
                          : div_overflow_q   ? 32'b0
                                             : (rem_sign_q ? -remainder_mag : remainder_mag);

  always_comb begin
    if (is_div_q) result_o = want_rem_q ? rem_result : div_result;
    else          result_o = want_high_q ? mul_signed[63:32] : mul_signed[31:0];
  end

endmodule : pulse_muldiv
