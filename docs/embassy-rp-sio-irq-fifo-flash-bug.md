# RP2350: `SIO_IRQ_FIFO` handler on Core0 deadlocks `pause_core1()` during flash operations

## Summary

> **Status (2026-09):** worked around in Elle by `mask_sio_fifo()` / `unmask_sio_fifo()`
> in `crates/elle-hardware/src/flash/manager.rs`. Elle does not use an
> `InterruptExecutor`, so dropping the `executor-interrupt` feature from the workspace
> `embassy-rp` dependency should remove the trigger altogether — not yet tried.

When both `executor-thread` and `executor-interrupt` features are enabled on RP2350, flash write/erase operations cause a **deadlock followed by HardFault**. The `SIO_IRQ_FIFO` interrupt handler on Core0 consumes the `PAUSE_TOKEN` acknowledgment from the inter-core FIFO before `pause_core1()`'s polling loop can read it, causing Core0's handler to enter the pause/wait path meant for Core1.

## Environment

- Chip: RP2350 (rp235xa)
- embassy-rp rev: `5565326e` (found); still present at `340bf1de` (current, line numbers below)
- Features: `executor-thread`, `executor-interrupt`, `time-driver`, `critical-section-impl`
- Both cores active: Core0 runs the main application (thread executor), Core1 runs sensor tasks (thread executor via `spawn_core1` + `Executor::new()`)
- Flash operations via `sequential-storage` crate, which calls embassy's `NorFlash` impl internally

## Symptom

Any flash write or erase triggers `Firmware exited unexpectedly: Exception` (HardFault). The crash is immediate and deterministic — every flash operation fails.

Minimal reproduction: a single `flash.erase(0x210000, 0x211000).await` is enough to trigger the fault.

## Root cause

### The shared handler problem

On RP2350, there is a single `SIO_IRQ_FIFO` interrupt (unlike RP2040 which has per-core `SIO_IRQ_PROC0`/`SIO_IRQ_PROC1`). The handler in `multicore.rs` is compiled once and runs on whichever core's NVIC has the interrupt enabled:

```rust
// multicore.rs:160 (SIO_IRQ_FIFO handler)
#[cfg(all(feature = "rt", feature = "_rp235x"))]
#[interrupt]
unsafe fn SIO_IRQ_FIFO() {
    // ...
    while sio.fifo().st().read().vld() {
        let fifo_read = fifo_read_wfe();
        if fifo_read == PAUSE_TOKEN {
            cortex_m::interrupt::disable();
            fifo_write(PAUSE_TOKEN);          // ACK the pause
            while fifo_read_wfe() != RESUME_TOKEN { nop(); }
            cortex_m::interrupt::enable();
            fifo_write(RESUME_TOKEN);
        } else if fifo_read & 0xFFFF0000 == PEND_IRQ_TOKEN {
            // ...
        }
    }
}
```

This handler is designed for **Core1**: it receives `PAUSE_TOKEN` from Core0, acknowledges it, waits for `RESUME_TOKEN`, then acknowledges that. This is the protocol that `in_ram()` depends on.

### The enablement asymmetry

- **Core1** always enables `SIO_IRQ_FIFO` in `core1_startup()` (line 221) — **unconditional**, needed for `pause_core1()` to work.
- **Core0** enables `SIO_IRQ_FIFO` in `spawn_core1()` (line 323-325) — **only when `executor-interrupt` is enabled**, intended for the interrupt executor's `PEND_IRQ_TOKEN` cross-core waking.

### The deadlock sequence

When `executor-interrupt` is enabled and Core0 performs a flash operation:

1. `in_ram()` calls `pause_core1()` (flash.rs:943)
2. `pause_core1()` writes `PAUSE_TOKEN` to the FIFO and polls: `while fifo_read() != PAUSE_TOKEN {}` (multicore.rs:330-334)
3. Core1's `SIO_IRQ_FIFO` handler fires, receives `PAUSE_TOKEN`, sends back `PAUSE_TOKEN` as acknowledgment
4. **Core0's `SIO_IRQ_FIFO` handler fires** (because it's enabled via `executor-interrupt`), reads the acknowledgment `PAUSE_TOKEN` from the FIFO **before** the polling loop in step 2 can read it
5. Core0's handler sees `PAUSE_TOKEN` and interprets it as a pause request — it disables interrupts on Core0, sends `PAUSE_TOKEN` (a second ack), and waits for `RESUME_TOKEN`
6. **Deadlock**: Core0 is now stuck in the handler waiting for `RESUME_TOKEN`. Core1 is paused waiting for `RESUME_TOKEN`. Nobody can send `RESUME_TOKEN`. Both cores are stuck.
7. This manifests as a HardFault / exception.

```
Core0                           FIFO                          Core1
  |                               |                              |
  |-- fifo_write(PAUSE_TOKEN) --> |                              |
  |   polling fifo_read()...      | --- SIO_IRQ_FIFO fires ----> |
  |                               |                              |-- handler: sees PAUSE_TOKEN
  |                               | <-- fifo_write(PAUSE_TOKEN)  |   (ACK, then waits for RESUME)
  |                               |                              |   ... blocked ...
  |<-- SIO_IRQ_FIFO fires -----  |                              |
  |    handler: sees PAUSE_TOKEN  |                              |
  |    (thinks it's a pause req)  |                              |
  |    disables interrupts        |                              |
  |    fifo_write(PAUSE_TOKEN)    |                              |
  |    waits for RESUME_TOKEN ... |                              |
  |                               |                              |
  |    DEADLOCK: both cores waiting for RESUME_TOKEN             |
```

## Workaround

Mask `SIO_IRQ_FIFO` on Core0's NVIC before any flash operation, unmask after:

```rust
use cortex_m::peripheral::NVIC;
use embassy_rp::interrupt;

// Before flash operation
NVIC::mask(interrupt::SIO_IRQ_FIFO);

// ... flash write/erase ...

// After flash operation
unsafe { NVIC::unmask(interrupt::SIO_IRQ_FIFO) };
```

NVIC is per-core on Cortex-M33, so masking on Core0 does not affect Core1's handler. The `PEND_IRQ_TOKEN` functionality is temporarily unavailable on Core0 during flash ops, but thread-mode executors use `SEV`/`WFE` for waking and don't depend on it.

Note: simply removing `executor-interrupt` also fixes the flash deadlock (Core0 never enables the handler), but breaks other multicore functionality that depends on the FIFO infrastructure being available.

## Suggested fix

The handler should not process `PAUSE_TOKEN` on Core0. Two possible approaches:

### Option A: Check core ID in the handler

```rust
#[interrupt]
unsafe fn SIO_IRQ_FIFO() {
    let sio = pac::SIO;
    sio.fifo().st().write(|w| w.set_wof(false));

    let core_id = pac::SIO.cpuid().read();

    while sio.fifo().st().read().vld() {
        let fifo_read = fifo_read_wfe();
        if fifo_read == PAUSE_TOKEN && core_id == 1 {
            // Only Core1 should handle pause requests
            cortex_m::interrupt::disable();
            fifo_write(PAUSE_TOKEN);
            while fifo_read_wfe() != RESUME_TOKEN { nop(); }
            cortex_m::interrupt::enable();
            fifo_write(RESUME_TOKEN);
        } else if fifo_read & 0xFFFF0000 == PEND_IRQ_TOKEN {
            let irq = Irq((fifo_read & 0xFFFF) as u16);
            let mut nvic: NVIC = core::mem::transmute(());
            nvic.request(irq);
        }
    }
}
```

### Option B: Mask the interrupt in `in_ram()` itself

```rust
pub(crate) unsafe fn in_ram(operation: impl FnOnce()) -> Result<(), Error> {
    let core_id: u32 = pac::SIO.cpuid().read();
    if core_id != 0 {
        return Err(Error::InvalidCore);
    }

    // Prevent SIO_IRQ_FIFO handler from consuming the PAUSE_TOKEN ack
    NVIC::mask(interrupt::SIO_IRQ_FIFO);

    crate::multicore::pause_core1();

    critical_section::with(|_| {
        // ... DMA wait, XIP wait ...
        operation();
    });

    crate::multicore::resume_core1();

    unsafe { NVIC::unmask(interrupt::SIO_IRQ_FIFO) };
    Ok(())
}
```

Option A is more surgical (the handler simply ignores pause tokens on the wrong core). Option B is safer as defense-in-depth but temporarily disables `PEND_IRQ_TOKEN` handling on Core0 during flash ops.

## Affected configurations

- RP2350 with both `executor-thread` and `executor-interrupt` enabled
- Any application using `spawn_core1()` + flash write/erase
- RP2040 is likely unaffected because it has separate per-core interrupts (`SIO_IRQ_PROC0` / `SIO_IRQ_PROC1`)
