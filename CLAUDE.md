# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Fevicol** is a smart plant watering system for the Xiao ESP32-C6. The project monitors soil moisture levels and automatically waters plants when moisture drops below a configured threshold, while sending telemetry data to Home Assistant via MQTT.

The name is a nod to the classic Fevicol adhesive commercial where a woman hanging from a cliff says "pakde rehna, chhoddna nahin" (hold on, don't let go) - similar to how houseplants desperately cling to life when neglected.

**Current Status**: Moisture sensing implemented and calibrated. The system can read soil moisture levels via ADC and convert them to meaningful percentages (0-100%). Wi-Fi connectivity is functional. Architecture refactored to separate Embassy tasks for fault isolation between sensor and network operations. Still to implement: MQTT client integration, pump control, and Home Assistant integration.

## Features

### Implemented
1. **Moisture Sensing**: ✅ ADC-based soil moisture reading with calibration
   - Resistive moisture sensor on GPIO0 (pin A0)
   - Calibrated range: dry (2188) to wet (4095)
   - Real-time percentage conversion (0-100%)
   - Configurable threshold monitoring (currently 30%)
2. **Wi-Fi Connectivity**: ✅ STA mode connection with auto-reconnect

### Planned
3. **MQTT Integration**: Publish moisture readings and pump status to Home Assistant
4. **Automatic Watering**: Trigger water pump when moisture falls below threshold
5. **Pump Control**: GPIO-based relay/MOSFET control with safety limits
6. **Home Assistant Discovery**: Automatic entity creation via MQTT discovery

## Technical Foundation

This project was created following the [Espressif Rust Getting Started Guide](https://docs.espressif.com/projects/rust/book/getting-started/index.html) using the `esp-generate` interactive configuration tool.

The project targets bare-metal RISC-V (riscv32imac-unknown-none-elf) with no standard library (`#![no_std]`), running an Embassy async executor for concurrent task scheduling.

The project has Wi-Fi and Bluetooth Low Energy (BLE) capabilities with COEX (Wi-Fi/BLE coexistence) support available via `esp-radio`, though currently only Wi-Fi will be needed for MQTT connectivity.

## Target Hardware

- **Board**: Seeed Studio Xiao ESP32-C6
- **Chip**: ESP32-C6 (RISC-V based, single-core 160MHz)
- **Features**: Wi-Fi 6, Bluetooth 5.3, Ultra-low power consumption
- **Peripherals**:
  - ADC1 (GPIO0/A0): Resistive soil moisture sensor (6dB attenuation, 0-2450mV range)
  - GPIO (TBD): Water pump control via relay or MOSFET
  - UART/I2C: Available for additional sensors
- **Simulator**: Wokwi (configured in `wokwi.toml` for debugging)

## Development Prerequisites

The project requires:
- **Rust nightly toolchain** (specified in `rust-toolchain.toml`)
- **Target**: `riscv32imac-unknown-none-elf`
- **Components**: `rust-src` for building core/alloc from source
- **probe-rs**: For flashing and debugging (configured as the cargo runner)

Install probe-rs if not already available:
```bash
cargo install probe-rs-tools --locked
```

## Build and Flash Commands

The project uses **probe-rs** (not espflash) as the cargo runner, configured in `.cargo/config.toml`. This enables Real-Time Transfer (RTT) for logging and on-chip debugging.

### Build and Flash to Device
```bash
cargo run  # Use debug mode (release has linker issues with esp-radio NVS symbols)
```

This command builds the project, flashes it to the ESP32-C6, and starts monitoring RTT output. The runner is configured with:
- Chip: ESP32-C6
- Stack trace printing on panics
- Hardfault catching

**Note**: Currently use debug builds due to linker errors with esp-radio NVS symbols in release mode. Debug builds are optimized (`opt-level = "s"`) and work well on device.

### Build Only
```bash
cargo build  # Debug mode (recommended)
cargo build --release  # Currently has linker issues
```

### Format Check
```bash
cargo fmt --all -- --check
```

**Note**: A temporary `src/bin/secrets.rs` file must exist for `cargo fmt` to work (even if empty), as the formatter needs to resolve the conditional `mod secrets`. This file is git-ignored and used for local Wi-Fi credentials.

**Important**: Always run `cargo fmt --all` and fix any formatting issues before committing code changes.

### Lint
```bash
cargo clippy --all-features --workspace -- -D warnings
```

### Run Tests
```bash
cargo test
```

Tests use the `embedded-test` framework with Embassy executor and run on-device via probe-rs.

## Important Build Configuration

The project uses `build-std` (unstable feature) to build `core` and `alloc` from source. This is required for `no_std` embedded targets and configured in `.cargo/config.toml`.

**Rust flags** (automatically applied):
- `-C force-frame-pointers`: Required for backtraces (e.g., with `esp-backtrace`)
- `-Z stack-protector=all`: Stack overflow protection

**Environment variables**:
- `DEFMT_LOG`: Controls defmt logging level (default: "info"). Can be set to "trace", "debug", "info", "warn", or "error"

## Architecture

### Entry Point
- **Binary**: `src/bin/main.rs` - Main application entry with `#[esp_rtos::main]` macro
- **Library**: `src/lib.rs` - Empty `#![no_std]` library root

### Async Runtime
The project uses Embassy's cooperative scheduler via `esp-rtos`:
- `esp_rtos::start()` initializes the scheduler with a hardware timer (TIMG0) and software interrupt
- All application code runs in async tasks spawned via `Spawner`
- Main function signature: `async fn main(spawner: Spawner) -> !`

### Memory Management
- Heap allocators are configured at runtime via `esp_alloc::heap_allocator!` macros
- Two heap regions: 64KB reclaimed RAM + 64KB standard RAM (extra for COEX)
- All dynamic allocations use the `alloc` crate (not `std`)

### Radio Initialization Sequence
1. Initialize `esp_radio` to get `RadioInitialization` handle
2. Create Wi-Fi controller with `esp_radio::wifi::new()`
3. *(Optional)* Create BLE transport with `BleConnector::new()` and initialize `trouble-host` stack

The radio initialization handle is shared between Wi-Fi and BLE (required for COEX). For this project, only Wi-Fi is needed for MQTT connectivity to Home Assistant.

### Application Architecture

The application uses Embassy's async executor to run concurrent tasks with clear separation of concerns:

**Implemented Tasks**:
1. **Network Task** (`network_task`): Manages Wi-Fi connection in STA mode with automatic reconnection
2. **Sensor Task** (`moisture_sensor_task`): Reads ADC every 5 seconds, converts raw values to percentages, sends readings to channel, and monitors threshold
3. **MQTT Connection Task** (`mqtt_connection_task`): Waits for network availability and will manage MQTT broker connection lifecycle (currently placeholder)
4. **MQTT Publish Task** (`mqtt_publish_task`): Receives sensor readings from channel and will publish to broker (currently logs readings as placeholder)

**To Be Implemented**:
5. **Pump Control Task**: Trigger pump when moisture drops below threshold, enforce safety limits (max run time, minimum interval between waterings)

### Inter-Task Communication

The architecture uses `embassy-sync::channel::Channel` for passing sensor data between tasks:

- **Channel Type**: `Channel<NoopRawMutex, SensorReading, 20>`
- **Capacity**: 20 readings (buffers ~100 seconds of data during network outages)
- **Data Structure**: `SensorReading` contains:
  - `moisture: u8` - moisture percentage (0-100%)
  - `raw: u16` - raw ADC value (0-4095)
  - `timestamp: u64` - milliseconds since boot (from `embassy_time::Instant`)
- **Flow**: Sensor task → Channel → MQTT publish task
- **Benefits**:
  - Sensor readings continue uninterrupted during network issues
  - Fault isolation between sensor hardware and network operations
  - Buffering prevents data loss during temporary MQTT disconnections

### Linker Configuration
The `build.rs` script configures custom linker scripts and implements helpful error messages:
- `embedded-test.x` for test harness
- `defmt.x` for logging infrastructure
- `linkall.x` must be last (critical ordering)
- Custom error handler provides diagnostic messages for missing symbols

### Logging
Uses `defmt` + `rtt-target` for logging via RTT (Real-Time Transfer):
- Initialize with `rtt_target::rtt_init_defmt!()` before any logging
- Use `defmt::info!()`, `defmt::debug!()`, etc. for output
- Logs are viewable via RTT tools or debugger

### Important Code Rules

**Memory Safety**:
- `clippy::mem_forget` is explicitly denied - never use `mem::forget` with esp_hal types that hold DMA buffers

**COEX Configuration**:
- When using both Wi-Fi and BLE, both must be initialized even if only one is actively used
- The `RadioInitialization` handle enables resource coordination between protocols

## Test Framework

Tests use `embedded-test` with Embassy executor:
- Test files go in `tests/` directory (e.g., `tests/hello_test.rs`)
- Tests are marked with `#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]`
- Use `#[init]` function to initialize hardware before tests run
- Tests can be async and use `embassy_time::Timer` for delays
- RTT must be initialized in test `init()` to see `defmt` output

## Creating New Projects

This project was generated with `esp-generate`, the recommended tool for creating ESP32 Rust projects. To create a similar project:

```bash
esp-generate
```

The interactive TUI will prompt for:
- Target chip (this project uses ESP32-C6)
- Project name
- Flashing method (probe-rs or espflash)
- Additional configuration options

After generation, run `cargo run --release` to build, flash, and monitor.

## Wokwi Simulation

The project includes Wokwi simulator configuration in `wokwi.toml`:
- GDB server runs on port 3333
- Points to the debug build ELF file

This allows testing firmware in simulation before deploying to hardware.

## Implementation Notes

### Moisture Sensing (Implemented)

**Architecture**:
- Sensor reading runs in dedicated `moisture_sensor_task`, independent of network status
- Readings are sent via `embassy-sync::channel` to MQTT publish task
- Task-based design provides fault isolation and continuous operation

**Hardware Configuration** (src/bin/main.rs):
- ADC1 configured on GPIO0 (Xiao ESP32-C6 pin A0)
- 6dB attenuation (measuring range 0-2450mV)
- Resistive moisture sensor (shows higher voltage when wet, lower when dry)

**Calibration** (constants in src/bin/main.rs):
- `SENSOR_DRY = 2188`: ADC reading in air (0% moisture)
- `SENSOR_WET = 4095`: ADC reading fully submerged (100% moisture)
- Run calibration routine (commented at end of main.rs) to recalibrate for different sensors

**Conversion Function** (src/bin/main.rs):
```rust
fn raw_to_moisture_percent(raw: u16) -> u8
```
- Linear interpolation between calibration points
- Returns 0-100% moisture level
- Handles out-of-range values (clamps to 0% or 100%)

**Sensor Task Behavior** (`moisture_sensor_task`):
- Reads sensor every 5 seconds
- Converts raw ADC values to moisture percentage
- Creates `SensorReading` struct with moisture, raw value, and timestamp
- Sends readings to channel (non-blocking, drops if channel full)
- Logs readings and warns when moisture < threshold (30% default)
- Continues operation independently of network/MQTT status

### Calibration Procedure

A calibration routine is preserved as commented code at the end of `src/bin/main.rs` (lines 276-375). To recalibrate:

1. Replace the main loop with the calibration code
2. Flash to device: `cargo run`
3. Follow RTT prompts:
   - Keep sensor in air for 10 dry readings
   - Submerge sensor in water for 10 wet readings
4. Update `SENSOR_DRY` and `SENSOR_WET` constants with the averages
5. Restore the monitoring loop

### Next Implementation Steps

**MQTT Integration** (Not Yet Implemented):
- Use `embassy-net` TCP stack with `rust-mqtt` or similar `no_std` MQTT client
- Implement reconnection logic for network interruptions
- Use Home Assistant MQTT discovery for automatic entity creation
- Publish topics:
  - `fevicol/sensor/moisture` - moisture percentage
  - `fevicol/sensor/raw` - raw ADC value
  - `fevicol/pump/status` - pump on/off state
- Subscribe topics:
  - `fevicol/pump/command` - manual pump control
  - `fevicol/config/threshold` - adjust moisture threshold

**Pump Control** (Not Yet Implemented):
- Use GPIO output to control relay or MOSFET for pump power
- Safety features needed:
  - Maximum run time (e.g., 30 seconds per activation)
  - Minimum interval between waterings (e.g., 1 hour)
  - Emergency shutoff if sensor reads errors
- Add manual override capability via MQTT commands

**Wi-Fi** (Implemented):
- Credentials configured via compile-time env vars or `src/secrets.rs`
- Connection state management with automatic reconnection in `network_task`
- Network ready signal for coordinating dependent tasks

## Useful Resources

- **Espressif Rust Getting Started Guide**: https://docs.espressif.com/projects/rust/book/getting-started/index.html
- **esp-hal examples**: https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
- **esp-hal ADC examples**: Look for `adc_continuous.rs` and `adc_oneshot.rs`
- **embassy-net examples**: https://github.com/embassy-rs/embassy/tree/main/examples
- **edge-net networking protocols**: https://crates.io/crates/edge-net
- **probe-rs documentation**: https://probe.rs/
- **Xiao ESP32-C6 documentation**: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/
