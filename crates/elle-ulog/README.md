# elle-ulog

ULog flash logging implementation for the ELLE-RS flight controller.

## Overview

This crate implements the PX4 ULog file format for embedded flash storage on the RP2350. ULog is a self-describing binary format with little-endian byte ordering, widely used in the drone/flight controller community.

## Features

- **Self-describing format**: Contains message format definitions
- **Little-endian**: Compatible with PX4 tooling
- **Flash-optimized**: Buffered writes to minimize flash wear
- **Type-safe**: Rust message definitions with compile-time checks
- **Embedded-friendly**: No_std, no heap allocations

## ULog Format Structure

```
[Header (16 bytes)]
  - Magic: "ULog" + version marker [0x01, 0x12, 0x35]
  - Version: 1
  - Timestamp: uint64_t microseconds

[Definitions Section]
  - Flag Bits ('B'): Feature flags
  - Format Definitions ('F'): Message schemas
  - Info Messages ('I'): System metadata
  - Subscriptions ('A'): Message instances

[Data Section]
  - Data Messages ('D'): Logged telemetry
  - String Messages ('L'): Debug logs
```

## Message Types

### AttitudeMessage
Logs IMU orientation and angular rates:
- `timestamp`: uint64_t (microseconds)
- `pitch`, `roll`, `yaw`: float (radians)
- `pitch_rate`, `roll_rate`, `yaw_rate`: float (rad/s)
- Size: 32 bytes

### CommandsMessage
Logs pilot commands and setpoints:
- `timestamp`: uint64_t (microseconds)
- `throttle`, `pitch`, `roll`, `yaw`: float [-1.0, 1.0]
- `attitude_mode`: uint8_t (0=Rate, 1=Angle)
- `pitch_setpoint_deg`, `roll_setpoint_deg`: float (degrees)
- Size: 37 bytes

### StatusMessage
Logs system health and performance:
- `timestamp`: uint64_t (microseconds)
- `loop_time_us`: uint32_t (control loop time)
- `imu_errors`: uint32_t (error count)
- `calibrated`, `armed`: uint8_t (boolean flags)
- `cpu_load`: float (percentage 0-100)
- Size: 22 bytes

## Usage

### Enabling ULog Logging

Add to your `Cargo.toml`:

```toml
[dependencies]
elle-hardware = { version = "0.1", features = ["ulog-logging"] }
```

### Basic Usage

```rust
use elle_hardware::ULogLogger;

// Create logger
static mut LOGGER: ULogLogger = ULogLogger::new();

// Initialize (writes header and definitions to flash)
logger.initialize().await?;

// Log attitude data (from IMU)
logger.log_attitude(
    pitch, roll, yaw,
    pitch_rate, roll_rate, yaw_rate
).await?;

// Log commands (from RC input)
logger.log_commands(
    throttle, pitch, roll, yaw,
    attitude_mode,
    pitch_setpoint_deg, roll_setpoint_deg
).await?;

// Log system status
logger.log_status(
    loop_time_us,
    imu_errors,
    calibrated,
    armed,
    cpu_load
).await?;

// Manually flush buffer to flash
logger.flush().await?;
```

### Integration Example

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize logger
    let mut logger = ULogLogger::new();
    logger.initialize().await.unwrap();

    // Main control loop (77Hz)
    loop {
        let attitude = imu.read_attitude().await;
        let commands = rc.read_commands().await;

        // Log data (buffered, auto-flushes when >75% full)
        logger.log_attitude(
            attitude.pitch, attitude.roll, attitude.yaw,
            attitude.pitch_rate, attitude.roll_rate, attitude.yaw_rate
        ).await.ok();

        logger.log_commands(
            commands.throttle, commands.pitch, commands.roll, commands.yaw,
            commands.attitude_mode as u8,
            commands.pitch_setpoint_deg, commands.roll_setpoint_deg
        ).await.ok();

        // Every 10th iteration, log status
        if loop_count % 10 == 0 {
            logger.log_status(
                loop_time_us, imu_errors,
                calibrated, armed, cpu_load
            ).await.ok();
        }

        Timer::after(Duration::from_millis(13)).await;
    }
}
```

## Flash Layout

```
Total: 16MB (0x000000 - 0xFFFFFF)
├─ 0x000000 - 0xEFFFFF: Program code (~15MB)
├─ 0xF00000 - 0xF0FFFF: Calibration storage (64KB)
└─ 0xF10000 - 0xFFFFFF: ULog storage (~960KB)
```

## Performance Characteristics

- **Write size**: 4096 bytes per chunk
- **Buffering**: Auto-flush at 75% (3KB)
- **Flash wear**: Managed by sequential-storage wear-leveling
- **Overhead**: ~3 bytes per message (header)
- **Control loop impact**: Minimal - writes are async and buffered

## Data Recovery

ULog files can be extracted from flash and analyzed with PX4 tools:

```bash
# Extract flash region (requires probe-rs or similar)
probe-rs dump --chip RP2350 --address 0xF10000 --size 983040 ulog.bin

# Analyze with pyulog (Python)
pip install pyulog
ulog_info ulog.bin
ulog_messages ulog.bin

# Convert to CSV
ulog2csv ulog.bin
```

## Specifications

Implements the official PX4 ULog file format specification:
https://docs.px4.io/main/en/dev_log/ulog_file_format

## License

Same as parent project (ELLE-RS).
