# State Diagrams

## Pilot State Machine

### ASCII

```
                         POWER ON
                            │
                            ▼
                   ┌────────────────┐
                   │    DISARMED    │
                   │    Manual      │
                   │  (LED: solid   │
                   │    green)      │
                   └───────┬────────┘
                           │ throttle low (auto-arm)
                           │ beep: single
                           ▼
              ┌────────────────────────────┐
    ┌────────►│       ARMED MANUAL         │◄───────────────┐
    │         │  (LED: double-blink green)  │                │
    │         └─────┬──────────────┬───────┘                │
    │               │              │                         │
    │         CH6 mid│        CH6 high                       │
    │               │              │                         │
    │               ▼              ▼                         │
    │    ┌──────────────┐  ┌───────────────┐                │
    │    │    ARMED     │  │    ARMED      │                │
    │    │ STABILIZED   │  │ ALTITUDE HOLD │                │
    │    │ (LED: pulse  │  │ (LED: pulse   │                │
    │    │   cyan)      │  │   cyan)       │                │
    │    └──┬───┬───────┘  └───────┬───────┘                │
    │       │   │                  │                         │
    │ CH6 low   │ CH7 mid/high    │ CH6 low                 │
    │       │   │                  │                         │
    ├───────┘   ▼                  └─────────────────────────┘
    │    ┌──────────────┐
    │    │   AUTOTUNE   │
    │    │   (relay)    │
    │    │              │
    │    │ setpoint:    │
    │    │  ±5° square  │
    │    │  wave        │
    │    └──┬───┬───┬───┘
    │       │   │   │
    │  CH7 low  │   │ timeout / amplitude > 20°
    │  (abort)  │   │ attitude data lost
    │       │   │   │
    │       │   │   ▼
    │       │   │  STABILIZED (gains restored)
    │       │   │
    │       │   │ completed (6 cycles)
    │       │   ▼
    │       │  STABILIZED (new gains auto-saved)
    │       │
    │       ▼
    │      STABILIZED (gains restored)
    │
    │
    │   ──── CH5 HIGH (from ANY armed state) ────
    │                    │
    │                    ▼
    │         ┌──────────────────┐
    │         │      KILLED      │
    │         │                  │
    │         │ motors: OFF      │
    │         │ elevons: CENTER  │
    │         │ beep: double     │
    │         │ ULog: continues  │
    │         └────────┬─────────┘
    │                  │ CH5 low + throttle low
    │                  │ beep: single (re-arm)
    └──────────────────┘


  RC LINK LOST (from any armed state):
  ┌──────────┐    200ms    ┌──────────┐    300ms    ┌──────────┐
  │  RC OK   │───────────►│ WARNING  │───────────►│ FAILSAFE │
  │          │             │ (LED:    │             │ (LED:    │
  │          │◄────────────│ fast-blink│            │ rapid-   │
  │          │  RC returns │ orange)  │             │ flash    │
  └──────────┘             └──────────┘             │ orange)  │
                                                    │          │
                                                    │ motors   │
                                                    │ cut      │
                                                    └──────────┘

  ULog LIFECYCLE (independent of flight state):
  ┌──────────┐  SD mounted  ┌───────────┐          ┌──────────┐
  │  IDLE    │─────────────►│ RECORDING │─────────►│  DONE    │
  │          │  auto-start  │ LOG_N.ULG │  power   │          │
  └──────────┘              │ flush 1Hz │  off     └──────────┘
                            └───────────┘
```

### Mermaid

```mermaid
stateDiagram-v2
    [*] --> Disarmed_Manual: Power On

    state "Disarmed Manual" as Disarmed_Manual
    state "Armed Manual" as Armed_Manual
    state "Armed Stabilized" as Armed_Stab
    state "Armed AltitudeHold" as Armed_AltHold
    state "Autotune" as Autotune
    state "Killed" as Killed

    Disarmed_Manual --> Armed_Manual: Throttle low\n(auto-arm, beep)

    Armed_Manual --> Armed_Stab: CH6 mid
    Armed_Manual --> Armed_AltHold: CH6 high

    Armed_Stab --> Armed_Manual: CH6 low
    Armed_Stab --> Autotune: CH7 mid/high\n(armed + attitude OK)
    Armed_Stab --> Armed_AltHold: CH6 high

    Armed_AltHold --> Armed_Manual: CH6 low
    Armed_AltHold --> Armed_Stab: CH6 mid

    Autotune --> Armed_Stab: Completed\n(gains auto-saved)
    Autotune --> Armed_Stab: CH7 low\n(abort, gains restored)
    Autotune --> Armed_Stab: Timeout / amplitude\n(safety abort)
    Autotune --> Armed_Stab: Attitude lost\n(safety abort)
    Autotune --> Armed_Manual: CH6 low\n(PID off, autotune paused)

    Armed_Manual --> Killed: CH5 high
    Armed_Stab --> Killed: CH5 high
    Armed_AltHold --> Killed: CH5 high
    Autotune --> Killed: CH5 high

    Killed --> Armed_Manual: CH5 low +\nthrottle low\n(re-arm, beep)

    note right of Killed
        Motors: OFF
        Elevons: CENTER
        Beep: double
        ULog: continues
    end note

    note right of Armed_Stab
        Stick = attitude setpoint
        PID: 100% output
        Throttle/yaw: manual
    end note

    note right of Armed_AltHold
        Setpoint: 0/0 (level)
        PID: 100% output
        Throttle: manual
    end note

    note right of Autotune
        Relay: +/-5 deg
        6 cycles to complete
        Tyreus-Luyben rule
    end note
```

```mermaid
stateDiagram-v2
    direction LR

    state "RC OK" as RC_OK
    state "RC Warning" as RC_Warn
    state "Failsafe" as Failsafe

    [*] --> RC_OK
    RC_OK --> RC_Warn: No packets > 200ms
    RC_Warn --> Failsafe: No packets > 300ms
    RC_Warn --> RC_OK: Packet received
    Failsafe --> RC_OK: Packet received

    note right of RC_Warn
        LED: fast-blink orange
    end note

    note right of Failsafe
        LED: rapid-flash orange
        Motors: CUT
    end note
```

```mermaid
stateDiagram-v2
    direction LR

    state "Idle" as Idle
    state "Recording" as Rec
    state "Done" as Done

    [*] --> Idle: Power on
    Idle --> Rec: SD card mounted\n(auto-start)
    Rec --> Done: Power off
    Rec --> Rec: Flush every 1s

    note right of Rec
        LOG_NNNN.ULG
        ~11 KB/s
        Runs until power off
    end note
```
