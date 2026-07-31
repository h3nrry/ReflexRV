# ReflexRV

**R**apid **E**fficient **F**ixed-latency **L**ow-power **E**xtensible e**X**ecution

ReflexRV is a RISC-V CPU core IP project organized as a family of products, each targeting a different performance/determinism tradeoff while sharing a common architectural lineage.

## Name Breakdown

| Letter | Word | Rationale |
|---|---|---|
| R | Rapid | High-speed execution across the core family |
| E | Efficient | Standard, clear efficiency claim |
| F | Fixed-latency | Deterministic timing — the basis of the Pulse tier's real-time value proposition |
| L | Low-power | Explicit power-efficiency claim, relevant across all tiers and especially Nano |
| E | Extensible | Reflects the actual product strategy — one architecture scaling across Prime/Apex/Pulse/Nano |
| X | eXecution | Core function of a CPU |

## Product Family Overview

| Tier | Product Name | Pipeline | Memory Management | Target Market | Status |
|---|---|---|---|---|---|
| Flagship | `ReflexRV-Prime` | TBD (likely wide-issue OoO, multi-core) | TBD (likely MMU/TLB) | Server / infrastructure / edge-compute class | Reserved — pending concrete differentiation |
| High Performance | `ReflexRV-Apex-E` | In-order | MMU (TLB) | Application class | Defined |
| High Performance | `ReflexRV-Apex-X` | Out-of-order | MMU (TLB) | Application class | Defined |
| Real-Time | `ReflexRV-Pulse` | In-order | MPU | Mid-range real-time / deterministic control | Defined |
| Microcontroller | `ReflexRV-Nano` | In-order | MPU | Low-end, low-power embedded | Defined |

## Tier Details

### ReflexRV-Prime — flagship (reserved)

Positioned above Apex as a distinct product class, not a faster variant of it. Reserved for a future design that targets a genuinely different microarchitecture segment:

- **Multi-core, cache-coherent clusters** for server/infrastructure workloads
- **Hypervisor/virtualization extensions**
- **Wider superscalar issue width** (6–8 wide vs. Apex's narrower pipeline)
- **Larger cache hierarchy** for data-center/edge-compute use cases

If the eventual design turns out to be "a bigger/faster Apex" rather than a distinct class, it folds back into an Apex grade (e.g., `Apex-X8`) instead of standing as a separate tier.

### ReflexRV-Apex — high performance, application class

Top-of-line application-class core, offered in two pipeline variants:

- **Apex-E** (in-order) — efficient variant for high performance without OoO complexity/power cost.
- **Apex-X** (out-of-order) — max-throughput variant using dynamic instruction scheduling.
- **TLB-based virtual memory (MMU)** — full memory management unit with a translation lookaside buffer, enabling virtual memory and process isolation required to run general-purpose operating systems (Linux-class software).

Targets general application workloads, capable of running full OS/Linux-class software.

### ReflexRV-Pulse — real-time, mid-range

Built around one priority: deterministic, analyzable timing — not peak throughput.

- **In-order pipeline** — instructions execute in program order, giving each instruction a predictable, statically analyzable execution time. This determinism is exactly what real-time workloads and certification processes require.
- **Bounded WCET (Worst-Case Execution Time)** — the upper bound on how long any instruction sequence can take is knowable in advance. Timing is predictable by design, the core requirement for real-time certification and control-loop guarantees.
- **Fast, deterministic interrupt/exception response** — low and bounded interrupt latency ensures time-critical events are serviced predictably, essential for real-time control loops and safety-critical event handling.
- **MPU-based memory protection** — a memory protection unit enforces region-based access control with fixed, predictable timing, avoiding the variable-latency page-table walks of a TLB and preserving real-time determinism.
- **Mid-range target** — sits between Apex's raw performance and Nano's minimal footprint, suited for industrial control, automotive, and robotics-class deterministic workloads.

Throughput-hungry, loosely-timed workloads belong on Apex; Pulse exists specifically for workloads that can't tolerate timing variance.

### ReflexRV-Nano — microcontroller, low-end

- **Simple in-order pipeline** — minimizes area and power overhead, keeping the core lean for cost- and power-sensitive designs.
- **MPU-based memory protection** — lightweight region-based protection without the area, power, or complexity cost of a full MMU/TLB, fitting the minimal-footprint goal of this tier.
- **Minimal footprint** — optimized for low-power, low-cost microcontroller applications.
- **Target use cases** — sensors, simple embedded control, cost-sensitive high-volume designs.

## Product Variants

Within each tier, numbered suffixes denote specific performance/feature grades of the same core family. Each grade adds a real architectural capability (ISA extension, cache size, pipeline width, safety feature) rather than just a different clock bin. Only grades with a defined differentiator are published here; higher numbers are reserved as future headroom.

### ReflexRV-Nano (`N0`–`N7`)

| Variant | Frequency Range | Pipeline | Differentiator | Target Competitor (ARM) | Target Competitor (SiFive) | Status |
|---|---|---|---|---|---|---|
| `ReflexRV-N0` | up to ~50 MHz | 2-stage | RV32I base only, smallest area | Cortex-M0 / M0+ | Essential E2 Series | Defined |
| `ReflexRV-N1` | ~50–150 MHz | 2–3 stage | + M extension (multiply/divide) | Cortex-M3 | Essential E2 Series | Defined |
| `ReflexRV-N2` | ~150–300 MHz | 3-stage | + C extension (compressed) / optional FPU | Cortex-M4 | Essential E24 | Defined |
| `N3`–`N7` | — | — | Reserved for future frequency/feature grades | — | — | Reserved |

### ReflexRV-Pulse (`P0`–`P7`)

| Variant | Frequency Range | Pipeline | Differentiator | Target Competitor (ARM) | Target Competitor (SiFive) | Status |
|---|---|---|---|---|---|---|
| `ReflexRV-P0` | ~300–600 MHz | 5-stage | Base real-time core, M extension | Cortex-R5 | Essential S7 Series | Defined |
| `ReflexRV-P1` | ~600 MHz–1 GHz | 6–7 stage | + FPU option, expanded MPU region count | Cortex-R7 | Essential S7 Series | Defined |
| `ReflexRV-P2` | ~1–1.5 GHz | 7–8 stage | Dual-core lockstep option for functional safety | Cortex-R52 (lockstep) | Essential S7 Series | Defined |
| `P3`–`P7` | — | — | Reserved for future frequency/feature grades | — | — | Reserved |

### ReflexRV-Apex (`E0`–`E7` / `X0`–`X7`)

| Variant | Frequency Range | Pipeline | Differentiator | Target Competitor (ARM) | Target Competitor (SiFive) | Status |
|---|---|---|---|---|---|---|
| `ReflexRV-Apex-E0` | >1.5 GHz | 8-stage, in-order | Base application core, single-issue | Cortex-A55 (single-issue config) | Performance U54 | Defined |
| `ReflexRV-Apex-E1` | >1.5 GHz | 9–10 stage, in-order | Dual-issue, larger cache | Cortex-A55 (dual-issue) | Performance U74 | Defined |
| `ReflexRV-Apex-X0` | >1.5 GHz | 10-stage, OoO | Entry OoO, narrow reorder window | Cortex-A76 | Performance P550 | Defined |
| `ReflexRV-Apex-X1` | >1.5 GHz | 12–15 stage, OoO | Wide OoO, deeper reorder buffer, multi-core cluster option | Cortex-X2 | Performance P550 (multi-core cluster) | Defined |
| `E2`–`E7` / `X2`–`X7` | — | — | Reserved for future performance grades | — | — | Reserved |

> Competitor mappings are directional (based on pipeline depth, issue width, and target frequency), not exact spec-for-spec equivalents — useful for positioning conversations, not for marketing claims without independent verification.

## License

ReflexRV is released under the [Solderpad Hardware License v2.1](https://solderpad.org/licenses/SHL-2.1/) — a permissive open hardware license that wraps around Apache License 2.0. It covers copyright, design rights, and semiconductor topography (mask work) rights, and carries over Apache 2.0's patent grant, patent retaliation clause, and warranty/liability disclaimers. No fees, royalties, or registration are required to use it.

Recommended header for source files:

```
Copyright (c) 2026 Henrry
SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

Licensed under the Solderpad Hardware License v2.1 (the "License"); you may not use
this file except in compliance with the License, or, at your option, the Apache
License version 2.0. You may obtain a copy of the License at

    https://solderpad.org/licenses/SHL-2.1/

Unless required by applicable law or agreed to in writing, any work distributed under
the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF
ANY KIND, either express or implied. See the License for the specific language
governing permissions and limitations under the License.
```

A full copy of the license text should also be placed in a `LICENSE` file at the repository root.

## Naming Convention

- **Apex** — "the peak/highest point." Used for the high-performance application tier to signal top-tier throughput.
- **Pulse** — evokes rhythm and timing precision, fitting the real-time/deterministic positioning.
- **Nano** — signals small size and low power; chosen over "Pico" to avoid direct naming collision with the Raspberry Pi Pico, which occupies the same microcontroller product category.
- **Prime** — reserved for a potential flagship tier above Apex. Because "Apex" literally means the highest point, nothing should sit above it without breaking the word's meaning — hence Prime, not a modifier on Apex, is the candidate name for anything positioned higher.