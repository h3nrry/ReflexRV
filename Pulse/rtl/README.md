# Pulse RTL

RTL source for [ReflexRV-Pulse](../README.md).

## Conventions

- **Reset** — asynchronous, active-low (`rst_ni`).
- **Signal suffixes** — `_i` input, `_o` output, `_q` registered/flopped state, `_d` combinational next-state feeding a `_q`.
- **File/module naming** — one module per file, filename matches the module name exactly (`pulse_idu.sv` holds `pulse_idu`), and every module is prefixed `pulse_` so it can't collide with a sibling tier's (Nano/Apex) RTL of the same shape in a shared build.

## Pipeline overview

`pulse_core` is the top-level, in-order 5-stage pipeline: IF (`pulse_ifu`) -> ID
(`pulse_idu` + `pulse_regfile` read, combinational) -> EX (`pulse_exu`, wrapping
`pulse_alu`/`pulse_muldiv`) -> MEM (`pulse_lsu`) -> WB (a mux into
`pulse_regfile`'s write port). `pulse_core` owns the three pipeline registers
between these stages (typed in `pulse_pkg` as `id_ex_t`/`ex_mem_t`/`mem_wb_t`);
neither `pulse_ifu` nor `pulse_idu` carries one internally. `pulse_mpu` sits
alongside, checked against both the fetch and data-access streams.

Hazard handling is full-stall, no forwarding: any RAW hazard against an
in-flight, not-yet-retired producer freezes fetch and decode until that
producer writes back (a producer already in MEM/WB never needs a stall,
since `pulse_regfile`'s reads forward a same-cycle write). This is the
simplest correct interlock, not the fastest one — deliberately, since a
fixed, easily-analyzed stall cost per hazard fits this core's bounded-WCET
goal better than chasing throughput does. See `pulse_core`'s header for the
full reasoning and for exactly what's still out of scope (no CSR unit yet,
so no trap vectoring or real privilege/MPU-region programming; no
interrupts).

### `pulse_ifu.sv` — `pulse_ifu`

Instruction Fetch Unit. Sequential (PC+4) fetch against a synchronous, single-cycle-latency
instruction memory (registered-output SRAM/BRAM style), with branch/jump redirect support.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i` | input | 1 | Clock |
| `rst_ni` | input | 1 | Asynchronous active-low reset |
| `stall_i` | input | 1 | Backpressure from the next stage (or imem not ready); freezes PC and held outputs |
| `redirect_valid_i` | input | 1 | A later stage resolved a taken branch/jump this cycle |
| `redirect_pc_i` | input | 32 | Corrected fetch target, valid when `redirect_valid_i` |
| `imem_req_o` | output | 1 | Read request/enable to imem for this cycle |
| `imem_addr_o` | output | 32 | Fetch address (PC) to imem |
| `imem_rdata_i` | input | 32 | Instruction data from imem, 1 cycle after the matching `imem_addr_o` |
| `instr_valid_o` | output | 1 | `instr_o` / `instr_pc_o` are valid |
| `instr_o` | output | 32 | Fetched instruction, for the next stage |
| `instr_pc_o` | output | 32 | PC of `instr_o` |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `RESET_PC` | `32'h0000_0000` | PC value after reset |

**Timing**

Because imem has 1-cycle read latency, the address on `imem_addr_o` this cycle returns as
`imem_rdata_i` next cycle. `pulse_ifu` does not add its own IF/ID instruction register for
this — imem's own output register already serves that role. Instead, it shadows the PC by
one cycle (`pc_out_q`) so `instr_pc_o` stays paired with whichever instruction is currently
on `imem_rdata_i`:

```
cycle N:   pc_q = A          -> imem_addr_o = A                 (request for A)
cycle N+1: pc_q = A+4        -> imem_addr_o = A+4  (next req)
           pc_out_q = A      -> instr_pc_o = A
           imem_rdata_i = instr(A) -> instr_o = instr(A)
```

`instr_valid_o` is held low for the one cycle after reset while the pipeline is still
priming (no instruction has returned from imem yet), then goes high and stays high —
including through stall cycles, since a stall means "hold what you have," not "invalidate
it." `imem_req_o` drops during stall/reset (but not during a redirect, which always gets
its request out); this assumes imem holds its output data when its enable is deasserted
(typical registered-SRAM behavior).

`redirect_valid_i` takes priority over `stall_i` and does two things on the same edge:
loads `pc_q` with `redirect_pc_i` so the corrected address is requested starting next
cycle, and forces `instr_valid_o` low for the following cycle — exactly one cycle, since
this module only ever has one fetch outstanding at a time (see the module header for why
that's enough here and wouldn't be for a deeper fetch buffer).

**Out of scope for this revision:** pipeline flush beyond the one squashed cycle described
above — a redirect only invalidates what `pulse_ifu` itself is holding; `pulse_core` is
responsible for also flushing whatever's sitting in ID (see its header).

### `pulse_pkg.sv` — `pulse_pkg`

Shared RV32IMAC opcode constants (`OPC_*`), decode-side enums (`alu_op_e`, `wb_sel_e`,
`branch_op_e`, `muldiv_op_e`, `amo_op_e`, `csr_op_e`, ...), and the three pipeline-register
struct types `pulse_core` uses between its stages (`id_ex_t`, `ex_mem_t`, `mem_wb_t`, each
with a `*_BUBBLE` localparam constant).

### `pulse_c_expander.sv` — `pulse_c_expander`

Purely combinational RV32C compressed-instruction expander. Takes one 16-bit compressed
instruction and produces the equivalent standard 32-bit RV32I/M instruction word, so
`pulse_idu`'s main decode logic never needs separate handling for compressed encodings.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `c_instr_i` | input | 16 | Compressed instruction (precondition: `c_instr_i[1:0] != 2'b11`) |
| `instr_o` | output | 32 | Expanded, standard 32-bit instruction |
| `illegal_o` | output | 1 | `c_instr_i` is not a legal RV32C encoding |

**Scope:** RV32C only — no F/D/Q-suffixed compressed loads/stores (`C.FLW`/`C.FSW`/`C.FLD`/
`C.FSD`/...), since this core has no F/D extension, and no RV64/128 C-only encodings
(`C.ADDIW`/`C.SUBW`/`C.ADDW`, wide `C.SLLI`/`C.SRLI`/`C.SRAI` shift amounts). Any such
encoding, or a reserved bit pattern, sets `illegal_o`.

### `pulse_idu.sv` — `pulse_idu`

Instruction Decode Unit, RV32IMAC. Purely combinational: one instruction in, one fully
decoded control bundle out, same cycle — no ID/EX pipeline register lives here, mirroring
`pulse_ifu` not owning an IF/ID register either. Internally instantiates `pulse_c_expander`
so compressed instructions are transparently expanded before the RV32I/M/A decode logic
runs.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | — | — | None — purely combinational |
| `instr_valid_i` | input | 1 | `instr_i` is valid this cycle |
| `instr_i` | input | 32 | Native 32-bit instruction, or a 16-bit compressed instruction right-justified in `instr_i[15:0]` (see below) |
| `instr_pc_i` | input | 32 | PC of `instr_i` |
| `is_compressed_o` | output | 1 | `instr_i` was a 16-bit encoding |
| `pc_plus_o` | output | 32 | `instr_pc_i + (2 or 4)` — the JAL/JALR link value |
| `rs1_o`, `rs2_o`, `rd_o` | output | 5 each | Register file addresses (not data) |
| `rf_we_o` | output | 1 | Register file write enable |
| `imm_o` | output | 32 | Immediate, sign/zero-extended per the instruction's format |
| `alu_op_o`, `alu_a_sel_o`, `alu_b_sel_o` | output | — | ALU control (`pulse_pkg::alu_op_e`/`alu_a_sel_e`/`alu_b_sel_e`) |
| `wb_sel_o` | output | — | Write-back source mux (`pulse_pkg::wb_sel_e`) |
| `mem_req_o`, `mem_we_o`, `mem_size_o`, `mem_sign_ext_o` | output | — | Data memory control |
| `branch_o`, `jal_o`, `jalr_o`, `branch_op_o` | output | — | Control flow |
| `mul_div_o`, `muldiv_op_o` | output | — | M-extension |
| `atomic_o`, `amo_op_o`, `aq_o`, `rl_o` | output | — | A-extension |
| `csr_access_o`, `csr_op_o`, `csr_addr_o` | output | — | Zicsr |
| `ecall_o`, `ebreak_o`, `mret_o`, `wfi_o` | output | 1 each | SYSTEM instructions |
| `illegal_instr_o` | output | 1 | Set on any encoding this core can't execute |

**Input contract / what's still missing upstream:** when `instr_i[1:0] == 2'b11`, `instr_i`
is a native 32-bit instruction; otherwise `instr_i[15:0]` holds one compressed instruction,
right-justified, and `instr_i[31:16]` is ignored. Producing that from a raw fetch
stream — picking the right halfword out of a wider fetch word, and reassembling a 32-bit
instruction that straddles two fetch words — is the job of an instruction-realignment
buffer in front of this stage. `pulse_ifu` in its current form is a fixed PC+4 fetcher and
does not implement one; that buffer (plus the resulting "PC advances by 2 or 4" change to
next-PC logic) is needed before this module can run on a live C-extension fetch stream.
`is_compressed_o`/`pc_plus_o` are exposed specifically so that buffer and the downstream
branch/link logic don't have to re-derive them.

**Also out of scope:** the ID/EX pipeline register and register-file read (`rs1_o`/`rs2_o`
are addresses, not data) — both `pulse_core`'s job — and CSR *execution* (a real
`pulse_csr` unit to back `csr_access_o`/`csr_op_o`/`csr_addr_o`); decode is complete, only
the functional units are still stubs (see `pulse_core`).

### `pulse_alu.sv` — `pulse_alu`

Single-cycle, purely combinational ALU. Fixed one-cycle latency regardless of operands or
operation, in keeping with this core's bounded-WCET goal — no data-dependent early-out.
Operand selection (rs1/pc/zero, rs2/imm/4) is the caller's job (`pulse_exu`); this module
only ever sees the two final operands.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `operand_a_i`, `operand_b_i` | input | 32 each | Final ALU operands |
| `alu_op_i` | input | — | Operation (`pulse_pkg::alu_op_e`) |
| `result_o` | output | 32 | Result |

### `pulse_muldiv.sv` — `pulse_muldiv`

M-extension multiply/divide unit: a sequential shift-add multiplier / shift-subtract
restoring divider, one bit per cycle. Every operation — any of MUL/MULH/MULHSU/MULHU/
DIV/DIVU/REM/REMU, any operand values, including divide-by-zero and the signed-overflow
case — takes exactly the same fixed number of cycles. No early-out for a small divisor or
a zero operand: fixed latency, not best-case latency, is the point here.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | input | — | |
| `start_i` | input | 1 | Level-sensitive; assert for as long as the op occupies EX (see Interface below) |
| `operand_a_i`, `operand_b_i` | input | 32 each | rs1, rs2 |
| `op_i` | input | — | `pulse_pkg::muldiv_op_e` |
| `busy_o` | output | 1 | Computation in progress |
| `valid_o` | output | 1 | `result_o` is valid this cycle (one-cycle pulse) |
| `result_o` | output | 32 | Result |

**Interface:** `start_i` is level-sensitive — this module only acts on it once, on the
IDLE->busy transition, and ignores it thereafter until it returns to IDLE, so a caller can
just wire it to the decoded mul/div control bit for the instruction currently occupying EX
rather than generating a one-shot pulse. Callers should derive their own EX-stall condition
as `(mul_div_instruction & ~valid_o)` rather than from `busy_o` directly — `busy_o` only
goes high the cycle *after* `start_i`, so gating a stall on `busy_o` alone fails to hold the
pipeline during that first setup cycle. `pulse_exu` does exactly this.

### `pulse_regfile.sv` — `pulse_regfile`

32x32 integer register file. Two asynchronous read ports, one synchronous write port, x0
hardwired to zero. Reads include same-cycle write-forwarding, which is what lets
`pulse_core` read a register in ID the same cycle an earlier instruction writes it back in
WB with no separate bypass network for that specific case.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | input | — | |
| `rs1_addr_i`/`rs1_rdata_o` | input/output | 5/32 | Read port 1 |
| `rs2_addr_i`/`rs2_rdata_o` | input/output | 5/32 | Read port 2 |
| `we_i`, `waddr_i`, `wdata_i` | input | 1, 5, 32 | Write port |

### `pulse_mpu.sv` — `pulse_mpu`

Region-based Memory Protection Unit. `NUM_REGIONS` fixed `[base, limit)` regions, each with
R/W/X permission bits, a lock bit, and a valid bit; lowest-index valid, matching region
wins. Two independent, combinational check ports (`a`/`b`) look up the *same* region table
every cycle, so instruction fetch and data access can both get a same-cycle answer against
one shared configuration instead of each needing its own copy kept in sync.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | input | — | |
| `cfg_we_i`, `cfg_idx_i`, `cfg_base_i`, `cfg_limit_i`, `cfg_r_i`, `cfg_w_i`, `cfg_x_i`, `cfg_lock_i`, `cfg_valid_i` | input | — | Region configuration, one region per write |
| `a_addr_i`, `a_is_fetch_i`, `a_is_write_i`, `a_priv_m_i` / `a_allow_o` | — | — | Check port A (`pulse_core` wires this to fetch) |
| `b_addr_i`, `b_is_fetch_i`, `b_is_write_i`, `b_priv_m_i` / `b_allow_o` | — | — | Check port B (`pulse_core` wires this to the LSU) |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `NUM_REGIONS` | `8` | Number of protection regions |

**Policy:** machine mode is default-allow (a matching region's permissions only bind M-mode
if that region is locked); user mode is default-deny (no match means no access). This is a
deliberately simplified, PMP-*inspired* policy, not a compliant implementation of the
RISC-V PMP CSR spec — see the module header for what's missing (NAPOT/TOR/OFF matching
modes, real pmpcfg/pmpaddr CSR encoding) and what `pulse_csr` will eventually need to wire
through.

### `pulse_lsu.sv` — `pulse_lsu`

Load/Store Unit — the MEM pipeline stage. Talks to a synchronous, 1-cycle-read-latency
data memory (same contract as `pulse_ifu`'s imem port). Handles byte/halfword pack and
sign-extend, full LR.W/SC.W (single-address reservation) and AMO read-modify-write
sequencing.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | input | — | |
| `req_i`, `we_i`, `addr_i`, `wdata_i` | input | 1, 1, 32, 32 | Request (store vs. load ignored when `atomic_i`), address (`rs1+imm`), store/AMO data |
| `size_i`, `sign_ext_i` | input | — | `pulse_pkg::mem_size_e`, sign-extend loads |
| `atomic_i`, `amo_op_i` | input | 1, — | A-extension request, `pulse_pkg::amo_op_e` |
| `dmem_req_o`, `dmem_we_o`, `dmem_addr_o`, `dmem_be_o`, `dmem_wdata_o`, `dmem_rdata_i` | — | — | Data memory port |
| `valid_o`, `stall_o`, `rdata_o` | output | 1, 1, 32 | Result timing + load/AMO-old-value/SC-0-or-1 result |

**Timing:** a plain store completes the same cycle it's issued; a plain load, LR.W, and an
AMO's read phase all need one more cycle for the memory's turnaround (`stall_o` high that
cycle); SC.W is resolved combinationally against the reservation and never touches memory
on failure, so it always completes same-cycle either way.

**Scope:** the LR/SC reservation is a single address + valid bit — correct for the
single-hart case the RISC-V spec requires (an intervening store from this hart breaks it),
not a multi-hart/bus-snoop scheme. Misaligned-access faulting isn't implemented (no
exception plumbing to report it through yet).

### `pulse_exu.sv` — `pulse_exu`

Execute stage. Selects ALU operands per `pulse_idu`'s `alu_a_sel`/`alu_b_sel`, runs
`pulse_alu`, and for M-extension instructions runs `pulse_muldiv` instead, presenting its
(later) result on the same `result_o` bus. Also resolves branches/jumps: the comparison
uses the raw register values (not the ALU-muxed operands, which are busy computing the
branch/JAL/JALR target address), and folds the outcome straight into
`redirect_valid_o`/`redirect_pc_o`, already shaped to drive `pulse_ifu`'s
`redirect_valid_i`/`redirect_pc_i` directly.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | input | — | (needed by the internal `pulse_muldiv`) |
| `valid_i` | input | 1 | This EX-stage instruction is real, not a bubble |
| `rs1_data_i`, `rs2_data_i`, `pc_i`, `imm_i` | input | 32 each | |
| `alu_op_i`, `alu_a_sel_i`, `alu_b_sel_i` | input | — | |
| `mul_div_i`, `muldiv_op_i` | input | — | |
| `branch_i`, `jal_i`, `jalr_i`, `branch_op_i` | input | — | |
| `result_o` | output | 32 | ALU/muldiv result; also the LSU address (`rs1+imm`) |
| `stall_o` | output | 1 | Mul/div in flight |
| `redirect_valid_o`, `redirect_pc_o` | output | 1, 32 | Branch/jal/jalr resolved taken this cycle |

### `pulse_core.sv` — `pulse_core`

Top-level in-order pipeline (see "Pipeline overview" above). Ties every module on this page
together, owns the pipeline registers and the stall/hazard/flush logic, and exposes trap
detection and MPU diagnostics at the top level.

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i`/`rst_ni` | input | — | |
| `imem_req_o`/`imem_addr_o`/`imem_rdata_i` | — | — | Instruction memory port |
| `dmem_req_o`/`dmem_we_o`/`dmem_addr_o`/`dmem_be_o`/`dmem_wdata_o`/`dmem_rdata_i` | — | — | Data memory port |
| `trap_valid_o`, `trap_pc_o`, `trap_cause_o` | output | 1, 32, 2 | Precisely-pipelined illegal-instruction/ECALL/EBREAK detection (cause: 0/1/2) |
| `mret_o`, `wfi_o` | output | 1 each | Retirement pulses for a future privilege/CSR unit |
| `mpu_instr_fault_o`, `mpu_data_fault_o` | output | 1 each | Live (non-pipelined) MPU diagnostic taps |

**Parameters**

| Parameter | Default | Description |
|---|---|---|
| `RESET_PC` | `32'h0000_0000` | Passed through to `pulse_ifu` |
| `NUM_MPU_REGIONS` | `8` | Passed through to `pulse_mpu` |

**Out of scope for this revision** (see the module header for full reasoning):
- **No CSR unit.** CSR instructions decode and occupy a pipeline slot, but the write-back
  value reads as 0 and MRET does not redirect execution. ECALL/EBREAK/illegal-instruction
  detection is real, but nothing vectors control flow to a trap handler yet.
- **MPU checks are live but not enforced or unified.** `priv_m_i` is hardwired to machine
  mode and no region is ever configured, so every access is always allowed regardless of
  the check logic; `mpu_instr_fault_o`/`mpu_data_fault_o` are diagnostic taps, not yet
  folded into `trap_valid_o`. Wiring real privilege tracking and region programming through
  is `pulse_csr`'s job.
- **No interrupts.**

**Verification:** exercised with a hand-assembled RV32IMA smoke-test program (register
hazards back-to-back and across a branch loop, MUL, and an AMOADD.W read-modify-write) run
to completion against a behavioral Verilator/C++ testbench, not checked into this
directory. `pulse_idu`, `pulse_c_expander`, `pulse_muldiv`, and `pulse_lsu` each also have
their own standalone self-checks built the same way.
