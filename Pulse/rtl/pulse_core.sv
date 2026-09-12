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

// pulse_core — top-level in-order pipeline (ReflexRV-Pulse, RV32IMAC).
//
// IF (pulse_ifu) -> ID (pulse_idu + pulse_regfile read, combinational) ->
// EX (pulse_exu, wrapping pulse_alu/pulse_muldiv) -> MEM (pulse_lsu) -> WB
// (a mux into pulse_regfile's write port). pulse_core owns the three
// pipeline registers between these stages (id_ex_q/ex_mem_q/mem_wb_q,
// typed in pulse_pkg) — neither pulse_ifu nor pulse_idu carries one
// internally (see their own headers).
//
// Hazard handling: full stall, no forwarding network. Every RAW hazard
// against an in-flight, not-yet-written-back producer (an instruction
// currently sitting in ID/EX or EX/MEM with rf_we set) freezes fetch and
// decode until that producer retires; a producer in MEM/WB never needs a
// stall because pulse_regfile's read ports already forward a same-cycle
// write. This is deliberately the simplest correct interlock rather than
// the fastest one: on a core built around bounded, easily-analyzed WCET
// rather than peak throughput, "every hazard costs a fixed, fully
// predictable number of stall cycles" is a feature, not just a shortcut.
// A forwarding network is the natural follow-up once throughput matters
// more than how easy the pipeline is to reason about.
//
// Multi-cycle stages (pulse_exu for mul/div, pulse_lsu for anything but a
// plain store) each report their own stall, which freezes every stage at or
// before them and inserts a bubble into the stage right after them — see
// the pipeline-register block below for exactly which register holds and
// which one bubbles on each combination.
//
// Branch/jump redirect: pulse_exu folds comparison + target computation into
// redirect_valid_o/redirect_pc_o, wired straight into pulse_ifu's own
// redirect_valid_i/redirect_pc_i (which squashes exactly the one wrong-path
// fetch already in flight — see pulse_ifu's header) and used here to also
// flush the instruction currently in ID (id_ex_q) — the only other
// instruction guaranteed to be on the wrong path when a branch resolves.
// EX/MEM and MEM/WB are never flushed on a redirect: the branch/jump
// instruction that produced it is not itself wrong-path and must retire
// normally (e.g. JAL/JALR still need to write their link register).
//
// Out of scope for this revision:
//   - No CSR unit (pulse_csr doesn't exist yet). CSR instructions decode and
//     occupy a pipeline slot but WB_CSR reads back as 0, and MRET does not
//     redirect execution (mret_o just exposes a retirement pulse for a
//     future privilege/CSR unit to consume). ECALL/EBREAK/illegal-instruction
//     detection is real and precisely pipelined (trap_valid_o/trap_pc_o/
//     trap_cause_o), but nothing here vectors control flow to a trap handler.
//   - pulse_mpu is instantiated and checked live for both fetch and data
//     accesses, but with priv_m_i hardwired to machine mode and no region
//     ever configured (cfg_we_i tied low), it always allows every access —
//     mpu_instr_fault_o/mpu_data_fault_o are exposed as live diagnostic taps,
//     not yet folded into a unified, precisely-pipelined exception the way
//     illegal_instr/ecall/ebreak are. Wiring real privilege-mode tracking and
//     region programming through, and deciding whether/how an MPU fault
//     should suppress the underlying access, is pulse_csr's job.
//   - No interrupts.

module pulse_core
  import pulse_pkg::*;
#(
  parameter logic [31:0]  RESET_PC       = 32'h0000_0000,
  parameter int unsigned  NUM_MPU_REGIONS = 8
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Instruction memory port (synchronous, 1-cycle read latency).
  output logic        imem_req_o,
  output logic [31:0] imem_addr_o,
  input  logic [31:0] imem_rdata_i,

  // Data memory port (synchronous, 1-cycle read latency).
  output logic        dmem_req_o,
  output logic        dmem_we_o,
  output logic [31:0] dmem_addr_o,
  output logic [3:0]  dmem_be_o,
  output logic [31:0] dmem_wdata_o,
  input  logic [31:0] dmem_rdata_i,

  // Precisely-pipelined trap detection (see header: not yet vectored anywhere).
  output logic         trap_valid_o,
  output logic [31:0]  trap_pc_o,
  output logic [1:0]   trap_cause_o,  // 0=illegal instr, 1=ECALL, 2=EBREAK
  output logic         mret_o,
  output logic         wfi_o,

  // Live (non-pipelined) MPU diagnostic taps — see header.
  output logic         mpu_instr_fault_o,
  output logic         mpu_data_fault_o
);

  // =========================================================================
  // IF
  // =========================================================================
  logic        if_valid;
  logic [31:0] if_instr, if_pc;

  logic        global_stall;
  logic        redirect_valid;
  logic [31:0] redirect_pc;

  pulse_ifu #(
    .RESET_PC (RESET_PC)
  ) u_ifu (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .stall_i          (global_stall),
    .redirect_valid_i (redirect_valid),
    .redirect_pc_i    (redirect_pc),
    .imem_req_o       (imem_req_o),
    .imem_addr_o      (imem_addr_o),
    .imem_rdata_i     (imem_rdata_i),
    .instr_valid_o    (if_valid),
    .instr_o          (if_instr),
    .instr_pc_o       (if_pc)
  );

  // =========================================================================
  // ID (combinational: decode + register-file read)
  // =========================================================================
  /* verilator lint_off UNUSED */
  logic         id_is_compressed;  // pc_plus_o already folds this in; kept for a future
                                    // fetch-realignment consumer (see pulse_idu's header)
  /* verilator lint_on UNUSED */
  logic [31:0]  id_pc_plus;
  logic [4:0]   id_rs1, id_rs2, id_rd;
  logic         id_rf_we;
  logic [31:0]  id_imm;
  alu_op_e      id_alu_op;
  alu_a_sel_e   id_alu_a_sel;
  alu_b_sel_e   id_alu_b_sel;
  wb_sel_e      id_wb_sel;
  logic         id_mem_req, id_mem_we;
  mem_size_e    id_mem_size;
  logic         id_mem_sign_ext;
  logic         id_branch, id_jal, id_jalr;
  branch_op_e   id_branch_op;
  logic         id_mul_div;
  muldiv_op_e   id_muldiv_op;
  logic         id_atomic;
  amo_op_e      id_amo_op;
  logic         id_aq, id_rl;
  logic         id_ecall, id_ebreak, id_mret, id_wfi;
  logic         id_illegal_instr;

  /* verilator lint_off PINCONNECTEMPTY */
  logic         id_csr_access_unused;
  csr_op_e      id_csr_op_unused;
  logic [11:0]  id_csr_addr_unused;
  /* verilator lint_on PINCONNECTEMPTY */

  pulse_idu u_idu (
    .instr_valid_i    (if_valid),
    .instr_i          (if_instr),
    .instr_pc_i       (if_pc),
    .is_compressed_o  (id_is_compressed),
    .pc_plus_o        (id_pc_plus),
    .rs1_o            (id_rs1),
    .rs2_o            (id_rs2),
    .rd_o             (id_rd),
    .rf_we_o          (id_rf_we),
    .imm_o            (id_imm),
    .alu_op_o         (id_alu_op),
    .alu_a_sel_o      (id_alu_a_sel),
    .alu_b_sel_o      (id_alu_b_sel),
    .wb_sel_o         (id_wb_sel),
    .mem_req_o        (id_mem_req),
    .mem_we_o         (id_mem_we),
    .mem_size_o       (id_mem_size),
    .mem_sign_ext_o   (id_mem_sign_ext),
    .branch_o         (id_branch),
    .jal_o            (id_jal),
    .jalr_o           (id_jalr),
    .branch_op_o      (id_branch_op),
    .mul_div_o        (id_mul_div),
    .muldiv_op_o      (id_muldiv_op),
    .atomic_o         (id_atomic),
    .amo_op_o         (id_amo_op),
    .aq_o             (id_aq),
    .rl_o             (id_rl),
    .csr_access_o     (id_csr_access_unused),
    .csr_op_o         (id_csr_op_unused),
    .csr_addr_o       (id_csr_addr_unused),
    .ecall_o          (id_ecall),
    .ebreak_o         (id_ebreak),
    .mret_o           (id_mret),
    .wfi_o            (id_wfi),
    .illegal_instr_o  (id_illegal_instr)
  );

  logic [31:0] id_rs1_data, id_rs2_data;

  // =========================================================================
  // Pipeline registers
  // =========================================================================
  id_ex_t  id_ex_q;
  // aq/rl (memory-ordering hints) ride along in ex_mem_q for a future
  // multi-hart pulse_lsu but aren't consumed by anything yet: this
  // single-hart, in-order core has no reordering for them to constrain.
  /* verilator lint_off UNUSED */
  ex_mem_t ex_mem_q;
  /* verilator lint_on UNUSED */
  mem_wb_t mem_wb_q;

  // ---- Hazard detection (ID stage, against ID/EX and EX/MEM producers) ----
  logic hazard_rs1, hazard_rs2, hazard;
  assign hazard_rs1 = (id_rs1 != 5'd0) &&
                      ((id_ex_q.valid  && id_ex_q.rf_we  && id_ex_q.rd  == id_rs1) ||
                       (ex_mem_q.valid && ex_mem_q.rf_we && ex_mem_q.rd == id_rs1));
  assign hazard_rs2 = (id_rs2 != 5'd0) &&
                      ((id_ex_q.valid  && id_ex_q.rf_we  && id_ex_q.rd  == id_rs2) ||
                       (ex_mem_q.valid && ex_mem_q.rf_we && ex_mem_q.rd == id_rs2));
  assign hazard = if_valid && (hazard_rs1 || hazard_rs2);

  // ---- Multi-cycle stage stalls ----
  logic ex_stall, mem_stall;
  assign global_stall = hazard | ex_stall | mem_stall;

  // ---- ID/EX ----
  id_ex_t id_ex_d;
  assign id_ex_d = if_valid ? '{
    valid:         1'b1,
    pc:            if_pc,
    pc_plus:       id_pc_plus,
    rd:            id_rd,
    rs1_data:      id_rs1_data,
    rs2_data:      id_rs2_data,
    imm:           id_imm,
    alu_op:        id_alu_op,
    alu_a_sel:     id_alu_a_sel,
    alu_b_sel:     id_alu_b_sel,
    wb_sel:        id_wb_sel,
    rf_we:         id_rf_we,
    mem_req:       id_mem_req,
    mem_we:        id_mem_we,
    mem_size:      id_mem_size,
    mem_sign_ext:  id_mem_sign_ext,
    branch:        id_branch,
    jal:           id_jal,
    jalr:          id_jalr,
    branch_op:     id_branch_op,
    mul_div:       id_mul_div,
    muldiv_op:     id_muldiv_op,
    atomic:        id_atomic,
    amo_op:        id_amo_op,
    aq:            id_aq,
    rl:            id_rl,
    illegal_instr: id_illegal_instr,
    ecall:         id_ecall,
    ebreak:        id_ebreak,
    mret:          id_mret,
    wfi:           id_wfi
  } : ID_EX_BUBBLE;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      id_ex_q <= ID_EX_BUBBLE;
    end else if (redirect_valid) begin
      id_ex_q <= ID_EX_BUBBLE;  // whatever's being decoded is on the wrong path
    end else if (ex_stall || mem_stall) begin
      // EX (or the stage after it) is still busy with the instruction
      // already in id_ex_q — hold it, don't accept a new decode yet.
    end else if (hazard) begin
      id_ex_q <= ID_EX_BUBBLE;  // ID must wait; EX gets a bubble this cycle
    end else begin
      id_ex_q <= id_ex_d;
    end
  end

  // =========================================================================
  // EX
  // =========================================================================
  logic [31:0] ex_result;

  pulse_exu u_exu (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),
    .valid_i           (id_ex_q.valid),
    .rs1_data_i        (id_ex_q.rs1_data),
    .rs2_data_i        (id_ex_q.rs2_data),
    .pc_i              (id_ex_q.pc),
    .imm_i             (id_ex_q.imm),
    .alu_op_i          (id_ex_q.alu_op),
    .alu_a_sel_i       (id_ex_q.alu_a_sel),
    .alu_b_sel_i       (id_ex_q.alu_b_sel),
    .mul_div_i         (id_ex_q.mul_div),
    .muldiv_op_i       (id_ex_q.muldiv_op),
    .branch_i          (id_ex_q.branch),
    .jal_i             (id_ex_q.jal),
    .jalr_i            (id_ex_q.jalr),
    .branch_op_i       (id_ex_q.branch_op),
    .result_o          (ex_result),
    .stall_o           (ex_stall),
    .redirect_valid_o  (redirect_valid),
    .redirect_pc_o     (redirect_pc)
  );

  // ---- EX/MEM ----
  ex_mem_t ex_mem_d;
  assign ex_mem_d = '{
    valid:         id_ex_q.valid,
    pc:            id_ex_q.pc,
    pc_plus:       id_ex_q.pc_plus,
    rd:            id_ex_q.rd,
    rs2_data:      id_ex_q.rs2_data,
    ex_result:     ex_result,
    wb_sel:        id_ex_q.wb_sel,
    rf_we:         id_ex_q.rf_we,
    mem_req:       id_ex_q.mem_req,
    mem_we:        id_ex_q.mem_we,
    mem_size:      id_ex_q.mem_size,
    mem_sign_ext:  id_ex_q.mem_sign_ext,
    atomic:        id_ex_q.atomic,
    amo_op:        id_ex_q.amo_op,
    aq:            id_ex_q.aq,
    rl:            id_ex_q.rl,
    illegal_instr: id_ex_q.illegal_instr,
    ecall:         id_ex_q.ecall,
    ebreak:        id_ex_q.ebreak,
    mret:          id_ex_q.mret,
    wfi:           id_ex_q.wfi
  };

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ex_mem_q <= EX_MEM_BUBBLE;
    end else if (mem_stall) begin
      // LSU still processing the instruction already in ex_mem_q.
    end else if (ex_stall) begin
      ex_mem_q <= EX_MEM_BUBBLE;  // EX has no valid result yet this cycle
    end else begin
      ex_mem_q <= ex_mem_d;
    end
  end

  // =========================================================================
  // MEM
  // =========================================================================
  logic        lsu_req;
  logic        lsu_valid;
  logic [31:0] lsu_rdata;

  assign lsu_req    = ex_mem_q.valid & ex_mem_q.mem_req;
  assign mem_stall  = lsu_req & ~lsu_valid;

  pulse_lsu u_lsu (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .req_i         (lsu_req),
    .we_i          (ex_mem_q.mem_we),
    .addr_i        (ex_mem_q.ex_result),
    .wdata_i       (ex_mem_q.rs2_data),
    .size_i        (ex_mem_q.mem_size),
    .sign_ext_i    (ex_mem_q.mem_sign_ext),
    .atomic_i      (ex_mem_q.atomic),
    .amo_op_i      (ex_mem_q.amo_op),
    .dmem_req_o    (dmem_req_o),
    .dmem_we_o     (dmem_we_o),
    .dmem_addr_o   (dmem_addr_o),
    .dmem_be_o     (dmem_be_o),
    .dmem_wdata_o  (dmem_wdata_o),
    .dmem_rdata_i  (dmem_rdata_i),
    .valid_o       (lsu_valid),
    /* verilator lint_off PINCONNECTEMPTY */
    .stall_o       (),  // pulse_core derives its own mem_stall from lsu_req & ~lsu_valid
    /* verilator lint_on PINCONNECTEMPTY */
    .rdata_o       (lsu_rdata)
  );

  // ---- MEM/WB ----
  mem_wb_t mem_wb_d;
  assign mem_wb_d = (lsu_req & ~lsu_valid) ? MEM_WB_BUBBLE : '{
    valid:         ex_mem_q.valid,
    pc:            ex_mem_q.pc,
    rd:            ex_mem_q.rd,
    rf_we:         ex_mem_q.rf_we,
    wb_sel:        ex_mem_q.wb_sel,
    ex_result:     ex_mem_q.ex_result,
    mem_result:    lsu_rdata,
    pc_plus:       ex_mem_q.pc_plus,
    illegal_instr: ex_mem_q.illegal_instr,
    ecall:         ex_mem_q.ecall,
    ebreak:        ex_mem_q.ebreak,
    mret:          ex_mem_q.mret,
    wfi:           ex_mem_q.wfi
  };

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) mem_wb_q <= MEM_WB_BUBBLE;
    else         mem_wb_q <= mem_wb_d;
  end

  // =========================================================================
  // WB
  // =========================================================================
  logic [31:0] wb_data;
  always_comb begin
    unique case (mem_wb_q.wb_sel)
      WB_EX:   wb_data = mem_wb_q.ex_result;
      WB_MEM:  wb_data = mem_wb_q.mem_result;
      WB_PC4:  wb_data = mem_wb_q.pc_plus;
      WB_CSR:  wb_data = 32'b0;  // no CSR unit yet — see header
      default: wb_data = mem_wb_q.ex_result;
    endcase
  end

  pulse_regfile u_regfile (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .rs1_addr_i   (id_rs1),
    .rs1_rdata_o  (id_rs1_data),
    .rs2_addr_i   (id_rs2),
    .rs2_rdata_o  (id_rs2_data),
    .we_i         (mem_wb_q.valid & mem_wb_q.rf_we),
    .waddr_i      (mem_wb_q.rd),
    .wdata_i      (wb_data)
  );

  assign trap_valid_o = mem_wb_q.valid & (mem_wb_q.illegal_instr | mem_wb_q.ecall | mem_wb_q.ebreak);
  assign trap_pc_o    = mem_wb_q.pc;
  assign trap_cause_o = mem_wb_q.illegal_instr ? 2'd0 : mem_wb_q.ecall ? 2'd1 : 2'd2;
  assign mret_o        = mem_wb_q.valid & mem_wb_q.mret;
  assign wfi_o          = mem_wb_q.valid & mem_wb_q.wfi;

  // =========================================================================
  // MPU (see header: live diagnostic taps, not yet enforced/unified)
  // =========================================================================
  logic mpu_a_allow, mpu_b_allow;

  pulse_mpu #(
    .NUM_REGIONS (NUM_MPU_REGIONS)
  ) u_mpu (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .cfg_we_i      (1'b0),
    .cfg_idx_i     ('0),
    .cfg_base_i    ('0),
    .cfg_limit_i   ('0),
    .cfg_r_i       (1'b0),
    .cfg_w_i       (1'b0),
    .cfg_x_i       (1'b0),
    .cfg_lock_i    (1'b0),
    .cfg_valid_i   (1'b0),
    .a_addr_i      (if_pc),
    .a_is_fetch_i  (1'b1),
    .a_is_write_i  (1'b0),
    .a_priv_m_i    (1'b1),
    .a_allow_o     (mpu_a_allow),
    .b_addr_i      (ex_mem_q.ex_result),
    .b_is_fetch_i  (1'b0),
    .b_is_write_i  (ex_mem_q.mem_we),
    .b_priv_m_i    (1'b1),
    .b_allow_o     (mpu_b_allow)
  );

  assign mpu_instr_fault_o = if_valid & ~mpu_a_allow;
  assign mpu_data_fault_o  = ex_mem_q.valid & ex_mem_q.mem_req & ~mpu_b_allow;

endmodule : pulse_core
