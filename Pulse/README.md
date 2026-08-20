# ReflexRV-Pulse

Real-time, mid-range RISC-V core in the [ReflexRV](../README.md) family. Built for deterministic, analyzable timing rather than peak throughput.

## Key Features

- **In-order pipeline** — instructions execute in program order for predictable, statically analyzable timing.
- **Bounded WCET** — worst-case execution time is knowable in advance, required for real-time certification and control-loop guarantees.
- **Fast, deterministic interrupt/exception response** — low, bounded interrupt latency for time-critical event handling.
- **MPU-based memory protection** — region-based access control with fixed, predictable timing (no TLB page-table walk variance).
- **Mid-range target** — between Apex's raw performance and Nano's minimal footprint; suited for industrial control, automotive, and robotics-class deterministic workloads.

## Variants

| Variant | Frequency Range | Pipeline | Differentiator | Status |
|---|---|---|---|---|
| `ReflexRV-P0` | ~300–600 MHz | 5-stage | Base real-time core, M extension | Defined |
| `ReflexRV-P1` | ~600 MHz–1 GHz | 6–7 stage | + FPU option, expanded MPU region count | Defined |
| `ReflexRV-P2` | ~1–1.5 GHz | 7–8 stage | Dual-core lockstep option for functional safety | Defined |
| `P3`–`P7` | — | — | Reserved for future frequency/feature grades | Reserved |

## Repository Layout

```
Pulse/
└── rtl/    # RTL source (see rtl/README.md for module details)
```

## License

Released under the [Solderpad Hardware License v2.1](https://solderpad.org/licenses/SHL-2.1/), per the [top-level ReflexRV license](../README.md#license).
