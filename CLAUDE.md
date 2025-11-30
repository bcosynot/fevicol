# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Fevicol** is a smart plant watering system for the Xiao ESP32-C6. The project monitors soil moisture levels and automatically waters plants when moisture drops below a configured threshold, while sending telemetry data to Home Assistant via MQTT.

The name is a nod to the classic Fevicol adhesive commercial where a woman hanging from a cliff says "pakde rehna, chhoddna nahin" (hold on, don't let go) - similar to how houseplants desperately cling to life when neglected.

**Current Status**: Moisture sensing implemented and calibrated. The system can read soil moisture levels via ADC and convert them to meaningful percentages (0-100%). Wi-Fi connectivity is functional. Architecture refactored to separate Embassy tasks for fault isolation between sensor and network operations. MQTT client configuration infrastructure added with connection management task. Home Assistant MQTT Discovery protocol implemented with automatic sensor entity creation (JSON payloads ready for publishing). Still to implement: Full MQTT client TCP integration, actual publishing of discovery and sensor data, pump control.

## Features

### Implemented
1. **Moisture Sensing**: ✅ ADC-based soil moisture reading with calibration
   - Resistive moisture sensor on GPIO0 (pin A0)
   - Calibrated range: dry (2188) to wet (4095)
   - Real-time percentage conversion (0-100%)
   - Configurable threshold monitoring (currently 30%)
2. **Wi-Fi Connectivity**: ✅ STA mode connection with auto-reconnect
3. **Home Assistant Discovery**: ✅ MQTT Discovery protocol implementation
   - Auto-discovery messages for moisture percentage sensor
   - Auto-discovery for raw ADC value sensor (debugging)
   - Device information with manufacturer, model, version
   - Availability topic support (online/offline status)
   - Discovery messages with retain flag for HA restart resilience
   - Ready to publish when MQTT client is connected

### In Progress
4. **MQTT Infrastructure**: 🔧 Configuration, client abstraction, and discovery publishing ready
   - MQTT 5.0 broker configuration (host, port, credentials)
   - Client ID format: `fevicol-{DEVICE_ID}`
   - New crate‑agnostic client interface: `MqQos` + `MqttPublish` trait
   - `publish_discovery(&mut impl MqttPublish, device_id, sensor_id) -> Result` publishes retained availability + HA discovery with QoS 1
   - Default build uses a `LoggerPublisher` (log‑only) until a concrete client feature is enabled
   - Publish task waits for MQTT readiness before processing sensor readings
   - Next: Implement TCP socket adapter for esp-radio's smoltcp stack and wire a concrete client

### Planned
5. **MQTT Publishing**: Publish sensor readings to broker (TCP integration required)
6. **Automatic Watering**: Trigger water pump when moisture falls below threshold
7. **Pump Control**: GPIO-based relay/MOSFET control with safety limits

## MQTT Client Abstraction and Feature Flags (2025‑11‑30)

- Abstraction
  - `src/bin/main.rs` defines a minimal MQTT publish interface to decouple the app from a specific client crate:
    - `enum MqQos { AtMostOnce, AtLeastOnce }`
    - `trait MqttPublish { async fn publish(&mut self, topic: &str, payload: &[u8], qos: MqQos, retain: bool) -> Result<(), Self::Err>; }`
  - `publish_discovery` now accepts `&mut impl MqttPublish` and returns `Result`, publishing retained availability and Home Assistant discovery payloads with pacing.

- Default mode (log‑only)
  - Without a concrete client feature, `LoggerPublisher` logs topics/payload lengths so discovery flow can be validated on RTT.

- Feature flags (Cargo.toml)
  - `mqtt_impl_rust_mqtt` — integrate `rust-mqtt` 0.3 over a smoltcp TCP adapter (preferred short‑term path)
  - `mqtt_impl_embedded_mqttc` — alternative client option
  - `mqtt_impl_mountain_mqtt` — alternative client option
  - With none enabled (default), the build remains in log‑only mode and signals `MQTT_CONNECTED` after discovery logging.

- LWT
  - The connection task will configure Last Will Testament to publish `offline` retained to `fevicol/{device_id}/status` on unexpected disconnect when a concrete client is active. After connect, it publishes `online` retained and discovery configs.

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
cargo run --release
```

This command builds the project, flashes it to the ESP32-C6, and starts monitoring RTT output. The runner is configured with:
- Chip: ESP32-C6
- Stack trace printing on panics
- Hardfault catching

**Note**: Currently use debug builds due to linker errors with esp-radio NVS symbols in release mode. Debug builds are optimized (`opt-level = "s"`) and work well on device.

### Build Only
```bash
cargo build  # Debug mode (recommended)
cargo build --release 
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

### Home Assistant MQTT Discovery (Implemented)

**Overview**: The project implements the Home Assistant MQTT Discovery protocol for automatic sensor entity creation. When the device connects to the MQTT broker, it publishes discovery messages that tell Home Assistant about available sensors and how to display them.

**Discovery Protocol**:
- Discovery messages are published to special topics: `homeassistant/{component}/{device_id}/{entity_id}/config`
- Discovery topics for this project:
  - `homeassistant/sensor/fevicol-01/moisture/config` - Moisture percentage sensor
  - `homeassistant/sensor/fevicol-01/raw/config` - Raw ADC value sensor (debugging)
- All discovery messages are published with `retain=true` so they persist across Home Assistant restarts
- Discovery messages are re-published on reconnection to handle Home Assistant restarts

**Sensor Entities Created**:
1. **Moisture Sensor** (`moisture`):
   - Name: "Moisture Sensor"
   - Device class: `moisture`
   - Unit: `%` (percentage)
   - Icon: `mdi:water-percent`
   - State topic: `fevicol/{device_id}/{sensor_id}/moisture`

2. **Raw ADC Sensor** (`raw`):
   - Name: "Moisture Raw ADC"
   - Unit: `ADC`
   - Icon: `mdi:chip`
   - State topic: `fevicol/{device_id}/{sensor_id}/raw`
   - Purpose: Debugging and calibration verification

**Device Information** (shared by all entities):
- Device ID: `fevicol-01` (or as configured in `DEVICE_ID`)
- Device Name: "Fevicol Plant Monitor - fevicol-01"
- Manufacturer: "Fevicol Project"
- Model: "ESP32-C6 Moisture Sensor"
- Software Version: From `Cargo.toml` version (currently 0.1.0)

**Availability Topic**:
- Topic: `fevicol/{device_id}/status`
- Payloads: `online` / `offline`
- Published as `online` after connection
- Last Will Testament (LWT) will publish `offline` on unexpected disconnect (when MQTT client is implemented)
- Shows device connectivity status in Home Assistant

**JSON Payload Generation**:
- Uses `heapless::String<1024>` for manual JSON formatting (no `serde_json` in `no_std`)
- Helper functions in `src/bin/main.rs`:
  - `create_moisture_discovery_payload()` - Moisture sensor discovery JSON
  - `create_raw_discovery_payload()` - Raw ADC sensor discovery JSON
  - `build_discovery_topic()` - Format discovery topic strings
  - `build_state_topic()` - Format state topic strings
  - `build_availability_topic()` - Format availability topic

**Publishing Sequence** (when MQTT client is connected):
1. Publish availability as `online` (retain=true)
2. Publish moisture sensor discovery (retain=true)
3. Publish raw ADC sensor discovery (retain=true)
4. Small delays (100ms) between publishes to avoid overwhelming broker

**Current Status**: Discovery message generation is fully implemented. Actual publishing awaits MQTT client TCP integration (next step).

### Topic Structure

The MQTT topic hierarchy is organized by device ID and sensor ID for scalability:

**Discovery Topics** (Home Assistant):
```
homeassistant/sensor/{device_id}/moisture/config    # Moisture % discovery
homeassistant/sensor/{device_id}/raw/config         # Raw ADC discovery
```

**State Topics** (sensor readings):
```
fevicol/{device_id}/{sensor_id}/moisture            # Moisture percentage (0-100)
fevicol/{device_id}/{sensor_id}/raw                 # Raw ADC value (0-4095)
```

**Status Topics** (availability):
```
fevicol/{device_id}/status                          # Device online/offline
```

**Future Topics** (planned):
```
fevicol/{device_id}/pump/status                     # Pump on/off state
fevicol/{device_id}/pump/command                    # Manual pump control
fevicol/{device_id}/config/threshold                # Adjust moisture threshold
```

**Example with default configuration**:
- Device ID: `fevicol-01`
- Sensor ID: `moisture-1`

Discovery topics:
- `homeassistant/sensor/fevicol-01/moisture/config`
- `homeassistant/sensor/fevicol-01/raw/config`

State topics:
- `fevicol/fevicol-01/moisture-1/moisture`
- `fevicol/fevicol-01/moisture-1/raw`

Status topic:
- `fevicol/fevicol-01/status`

### MQTT Configuration (Infrastructure Added)

**Configuration Constants** (src/bin/main.rs):
- `MQTT_BROKER_HOST`: Broker IP or hostname (default: "192.168.1.100") - configure for your network
- `MQTT_BROKER_PORT`: Broker port (default: 1883)
- `MQTT_KEEP_ALIVE_SECS`: Keep-alive interval (60 seconds)
- `MQTT_SESSION_EXPIRY_SECS`: Session expiry for battery-powered mode (3600 seconds / 1 hour)
- `MQTT_USERNAME` / `MQTT_PASSWORD`: Authentication credentials (empty strings for no auth)

**Task Architecture**:
- `mqtt_connection_task`: Waits for network, logs configuration, publishes Home Assistant discovery messages (placeholder), signals readiness
- `mqtt_publish_task`: Waits for MQTT readiness, receives sensor readings from channel, ready to publish
- `publish_discovery()`: Async function that generates and publishes discovery messages for all sensors

**Discovery Integration**:
- Discovery messages are published after successful MQTT connection (before signaling readiness)
- Discovery is re-published on reconnection to handle Home Assistant restarts
- Discovery function logs what would be published (actual publishing awaits TCP integration)

**Status**: Configuration infrastructure and Home Assistant discovery protocol complete. Next step is implementing TCP socket adapter for esp-radio's smoltcp stack to connect rust-mqtt client.

### Next Implementation Steps

**MQTT TCP Integration** (Next Priority):
- Implement embedded-io adapter for esp-radio's smoltcp TCP socket
- Integrate rust-mqtt client with adapted socket
- Implement actual broker connection with MQTT 5.0 protocol
- Set Last Will Testament (LWT) to publish availability as `offline` on disconnect
- Add exponential backoff reconnection (2s → 30s max)
- Implement PINGREQ for connection health monitoring

**MQTT Publishing** (After TCP Integration):
- Publish Home Assistant discovery messages on connection:
  - `homeassistant/sensor/{device_id}/moisture/config` (retain=true)
  - `homeassistant/sensor/{device_id}/raw/config` (retain=true)
  - `fevicol/{device_id}/status` = `online` (retain=true)
- Publish sensor readings:
  - `fevicol/{device_id}/{sensor_id}/moisture` - moisture percentage
  - `fevicol/{device_id}/{sensor_id}/raw` - raw ADC value
- Re-publish discovery on reconnection (HA might have restarted)
- Subscribe topics (future):
  - `fevicol/{device_id}/pump/command` - manual pump control
  - `fevicol/{device_id}/config/threshold` - adjust moisture threshold

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
