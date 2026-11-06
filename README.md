# Bit-Synchronizer

## Interface

`bit_sync` is a VHDL-2008 DTTL/NCO bit synchronizer. It uses one `clk` reference
clock, an active-high asynchronous `reset`, a dynamic `target_baud`, and a
synchronized single-ended `async_data` input. `sync_data` is NRZ-L output,
`recovered_clk` is gated low until lock, and `lock` indicates qualified timing
alignment. The default reference clock is 200 MHz; the supported nominal baud
range is 100 bps through 20 Mbps, subject to `SYS_CLK_FREQ_HZ >= 2*target_baud`.

| Port | Meaning |
| --- | --- |
| `target_baud` | Unsigned nominal baud rate in bits per second. It may be changed while running. |
| `line_code` | `00`: NRZ-L, `01`: Bi-Phase-L/Manchester, `10`: RNRZ-L degree 11, `11`: RNRZ-L degree 15. |
| `clock_phase` | `00`: 0 degrees, `01`: 90 degrees, `10`: 180 degrees, `11`: 270 degrees. |
| `async_data` | Single-ended logic input. Place the appropriate I/O buffer/receiver outside this core for RS-422. |

The 48-bit phase accumulator gives much finer than 1 ppm NCO resolution. The
loop limits initial frequency correction to +/-10% and expands to +/-20% after
first lock. Lock acquisition requires `LOCK_THRESHOLD` consecutive valid edge
observations; lock release requires the same number of consecutive bad timing
observations. Manchester timing uses the mandatory center transition and
ignores optional boundary transitions for lock qualification. The recovered
clock phase taps are derived from the same NCO and have quarter-UI spacing.
The input synchronizer carries an `ASYNC_REG` implementation attribute for FPGA
tools that recognize it; GHDL may warn that this vendor-specific hint is
unhandled.

The RNRZ-L descrambler starts with an all-ones state. Degree 11 uses feedback
`state(10) xor state(8)` and output `state(10)`; degree 15 uses feedback
`state(14) xor state(0)` and output `state(14)`. The encoder/test source must
use the matching bit order, polynomial, and seed. These are additive (synchronous)
descramblers: this interface has no frame marker or seed-acquisition input, so
the transmitter and receiver LFSR states must be aligned externally. Arbitrary
startup-phase sweeps validate timing/lock for NRZ-L and Manchester; an RNRZ-L
data comparison additionally requires a matching first-symbol seed phase.

## Simulation

The parameterized `tb_bit_sync` runs all four phase taps concurrently and
checks their quarter-UI spacing, lock retention, decoded data, runtime baud
changes, and selectable line codes. The serial stream begins 37% of a UI after
reset release by default, intentionally avoiding phase alignment with the
reference clock and NCO. `START_PHASE_PERCENT` can be varied from 0 to 99 for
additional NRZ-L/Manchester startup-phase tests. Example GHDL commands:

```sh
ghdl -a --std=08 bit_sync.vhd tb_bit_sync.vhd
ghdl -e --std=08 tb_bit_sync
ghdl -r --std=08 tb_bit_sync --assert-level=error
ghdl -r --std=08 tb_bit_sync -gBAUD_HZ=20000000 -gBAUD_OFFSET_PERCENT=10 --assert-level=error
ghdl -r --std=08 tb_bit_sync -gLINE_CODE_MODE=1 --assert-level=error
ghdl -r --std=08 tb_bit_sync -gLINE_CODE_MODE=2 --assert-level=error
ghdl -r --std=08 tb_bit_sync -gLINE_CODE_MODE=3 --assert-level=error
```

The 100 bps test may use a proportionally scaled `SYS_CLK_FREQ_HZ` to keep
simulation runtime practical. GHDL simulation does not replace synthesis,
static timing analysis, CDC review, or validation on the target FPGA and its
specific I/O and clocking resources.