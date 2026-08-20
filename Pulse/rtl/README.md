# Pulse RTL

RTL source for [ReflexRV-Pulse](../README.md).

## Conventions

- **Reset** — asynchronous, active-low (`rst_ni`).
- **Signal suffixes** — `_i` input, `_o` output, `_q` registered/flopped state, `_d` combinational next-state feeding a `_q`.

## Modules

### `ifu.sv` — `pulse_ifu`

Instruction Fetch Unit. Sequential (PC+4) fetch against a synchronous, single-cycle-latency
instruction memory (registered-output SRAM/BRAM style).

**Ports**

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_i` | input | 1 | Clock |
| `rst_ni` | input | 1 | Asynchronous active-low reset |
| `stall_i` | input | 1 | Backpressure from the next stage (or imem not ready); freezes PC and held outputs |
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
it." `imem_req_o` drops during stall/reset; this assumes imem holds its output data when
its enable is deasserted (typical registered-SRAM behavior).

**Out of scope for this revision:** branch/jump redirect and pipeline flush. Once a
branch-resolution stage exists, add `redirect_valid_i` / `redirect_pc_i` inputs that mux
into the next-PC computation, and a `flush_i` that forces `instr_valid_o` low for one
cycle.
