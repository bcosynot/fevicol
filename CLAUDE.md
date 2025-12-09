# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Fevicol** is a smart plant watering system for the Xiao ESP32-C6. The project monitors soil moisture levels and automatically waters plants when moisture drops below a configured threshold, while sending telemetry data to Home Assistant via MQTT.

The name is a nod to the classic Fevicol adhesive commercial where a woman hanging from a cliff says "pakde rehna, chhoddna nahin" (hold on, don't let go) - similar to how houseplants desperately cling to life when neglected.

**Current Status**: 

- Moisture sensing implemented and calibrated. The system can read soil moisture levels via ADC and convert them to meaningful percentages (0-100%). 
- Wi-Fi connectivity is functional with automatic reconnection.
- Architecture refactored to separate Embassy tasks for fault isolation between sensor and network operations. 
- **MQTT v5 integration fully implemented** with rust-mqtt v0.3 and embassy-net: TCP connection with DNS resolution, authentication, Last Will Testament (LWT), Home Assistant MQTT Discovery protocol with automatic sensor entity creation, telemetry publishing (moisture % and raw ADC), exponential backoff reconnection (2s → 30s max), and connection health monitoring. 
- Sensor readings are published every 5 seconds to Home Assistant. 
- Still to implement: pump control with safety limits and automatic watering logic.


## Features

### Implemented
1. **Moisture Sensing**: ✅ ADC-based soil moisture reading with calibration
   - Resistive moisture sensor on GPIO0 (pin A0)
   - Calibrated range: dry (2188) to wet (4095)
   - Real-time percentage conversion (0-100%)
   - Configurable threshold monitoring (currently 30%)
2. **Wi-Fi Connectivity**: ✅ STA mode connection with auto-reconnect
   - embassy-net stack with DHCP for automatic IP assignment
   - DNS resolution for broker hostname with fallback to IP address
   - Exponential backoff reconnection on Wi-Fi failures
3. **MQTT v5 Integration**: ✅ Full MQTT v5 implementation with rust-mqtt v0.3
   - MQTT v5 protocol with session expiry (3600s) and authentication
   - Client ID format: `fevicol-{DEVICE_ID}`
   - Crate-agnostic client interface: `MqQos` + `MqttPublish` trait
   - TCP connection via `EmbassyNetTransport` adapter for `embassy_net::tcp::TcpSocket`
   - Last Will Testament (LWT): `fevicol/{device_id}/status` = `offline` (retained)
   - Availability publishing: `online` after connection (retained)
   - Exponential backoff reconnection (2s → 30s max)
   - Connection health monitoring with automatic reconnection
   - Feature-gated with `mqtt` flag
   - Default build uses `LoggerPublisher` (log-only mode) for testing without broker
4. **Home Assistant MQTT Discovery**: ✅ Automatic sensor entity creation
   - Discovery messages for moisture percentage sensor
   - Discovery messages for raw ADC value sensor (debugging)
   - Device information with manufacturer, model, version
   - Availability topic support (online/offline status)
   - Discovery messages with retain flag for HA restart resilience
   - Published on connect/reconnect with QoS 1
   - 100ms pacing between discovery publishes
5. **MQTT Telemetry Publishing**: ✅ Sensor readings published every 5 seconds
   - Moisture percentage to `fevicol/{device_id}/{sensor_id}/moisture` (QoS 0)
   - Raw ADC value to `fevicol/{device_id}/{sensor_id}/raw` (QoS 0)
   - Inter-task communication via `embassy-sync::channel` (20-reading buffer)
   - Sensor task continues operating during network outages
   - Readings buffered during MQTT disconnection

### Planned
6. **Automatic Watering**: Trigger water pump when moisture falls below threshold
7. **Pump Control**: GPIO-based relay/MOSFET control with safety limits

## MQTT Client Implementation (rust-mqtt with MQTT v5)

### Recommended Implementation: rust-mqtt

The project uses **rust-mqtt v0.3** as the primary MQTT client, providing full MQTT v5 support with all required features including:
- MQTT v5 protocol with session expiry, authentication, and enhanced error reporting
- Last Will Testament (LWT) for availability tracking
- Retain flag support for discovery messages
- QoS 0 and QoS 1 support
- Integration with `embassy-net` TCP stack

### MQTT Client Abstraction

`src/mqtt/client.rs` defines a minimal MQTT publish interface to decouple the app from a specific client crate:
- `enum MqQos { AtMostOnce, AtLeastOnce }`
- `trait MqttPublish { async fn publish(&mut self, topic: &str, payload: &[u8], qos: MqQos, retain: bool) -> Result<(), Self::Err>; }`
- `publish_discovery` accepts `&mut impl MqttPublish` and returns `Result`, publishing retained availability and Home Assistant discovery payloads with pacing.

### Feature Flags

**MQTT Implementation** (Cargo.toml `[features]`):
- `mqtt` — rust-mqtt v0.3 with MQTT v5 support via embassy-net

**Default Mode** (no feature enabled):
- Uses `LoggerPublisher` for log-only mode
- Validates discovery flow on RTT without requiring a broker
- Signals `MQTT_CONNECTED` after discovery logging

### Building with MQTT Support

**Enable rust-mqtt client** (recommended):
```bash
cargo build --features mqtt
cargo run --release --features mqtt
```

**Default build** (log-only mode):
```bash
cargo build
cargo run --release
```

### MQTT v5 Configuration

When `mqtt` is enabled:
- **Protocol**: MQTT v5
- **Client ID**: `fevicol-{DEVICE_ID}`
- **Keep-alive**: 60 seconds
- **Session expiry**: 3600 seconds (1 hour)
- **Authentication**: Username/password support
- **LWT Topic**: `fevicol/{device_id}/status`
- **LWT Payload**: `offline` (retained, published on unexpected disconnect)
- **Availability**: `online` published to status topic after successful connection (retained)
- **Discovery**: Home Assistant MQTT Discovery messages published on connect/reconnect

### Network Stack

When `mqtt` is enabled, the project uses `embassy-net` for high-level network operations:
- DHCP for automatic IP assignment
- DNS resolution for broker hostname (with fallback to IP address)
- TCP socket management via `embassy_net::tcp::TcpSocket`
- Integration with esp-radio's Wi-Fi device driver

### rust-mqtt v0.3 Implementation Details

**Why rust-mqtt?**

The project uses rust-mqtt v0.3 as the MQTT client because it provides complete MQTT v5 support with all required features. An earlier attempt with mqtt-async-embedded v1.0.0 was abandoned because that library was found to be incomplete (publish() method was a stub, no LWT support, no authentication, no retain flag support).

**MQTT v5 Features Implemented**:
- **Protocol Version**: MQTT v5 with full property support
- **Session Management**: Session expiry interval (3600 seconds) for persistent sessions
- **Authentication**: Username/password authentication support
- **Last Will Testament (LWT)**: Configured to publish `offline` to availability topic on unexpected disconnect
- **Retain Flag**: Discovery messages and availability status published with retain=true
- **Quality of Service**: QoS 0 for telemetry (at most once), QoS 1 for discovery (at least once)
- **Keep-Alive**: 60-second keep-alive interval with automatic PINGREQ/PINGRESP handling
- **Clean Start**: Hardcoded to true in rust-mqtt v0.3.0 (no API to configure)

**Transport Adapter**:

The `EmbassyNetTransport` struct (src/mqtt/client.rs) adapts `embassy_net::tcp::TcpSocket` for use with rust-mqtt:
```rust
struct EmbassyNetTransport<'a> {
    socket: embassy_net::tcp::TcpSocket<'a>,
}
```

Implements:
- `embedded_io_async::ErrorType` - Error type definition
- `embedded_io_async::Read` - Async read from TCP socket
- `embedded_io_async::Write` - Async write to TCP socket with flush support

**Client Initialization** (init_rust_mqtt_client function):
1. Create `EmbassyNetTransport` wrapping a connected TCP socket
2. Allocate receive buffer (2048 bytes) and write buffer (2048 bytes)
3. Configure MQTT client with:
   - Client ID: `fevicol-{DEVICE_ID}`
   - Keep-alive: 60 seconds
   - Session expiry: 3600 seconds (MQTT v5 property)
   - Username/password authentication
   - LWT topic, payload, and retain flag
4. Call `MqttClient::connect()` to perform MQTT CONNECT handshake
5. Parse CONNACK response and check reason code
6. Return `RustMqttPublisher` wrapper implementing `MqttPublish` trait

**Connection Lifecycle** (mqtt_connection_task):
1. Wait for network stack to be ready (DHCP assigned IP)
2. Perform DNS resolution for broker hostname (with exponential backoff and IP fallback)
3. Establish TCP connection via `embassy_net::tcp::TcpSocket`
4. Initialize rust-mqtt client with `init_rust_mqtt_client()`
5. Publish availability as `online` (retained)
6. Publish Home Assistant discovery messages with 100ms pacing
7. Send sensor readings from channel to broker
8. On connection loss: exponential backoff (2s → 30s max) and reconnect
9. Re-publish discovery on reconnection

**Error Handling**:
- DNS resolution failures: exponential backoff, fallback to IP address after repeated failures
- TCP connection failures: exponential backoff (2s, 4s, 8s, 16s, 30s max)
- MQTT CONNACK errors: logged with reason code interpretation (bad credentials, server unavailable, etc.)
- Publish failures: trigger reconnection (rust-mqtt v0.3 doesn't expose PUBACK reason codes)
- Keep-alive timeout: rust-mqtt handles internally, failures trigger reconnection
- All errors logged via defmt without panicking

**Limitations of rust-mqtt v0.3.0**:
- Clean start is hardcoded to true (no API to set to false for session persistence)
- LWT QoS is not configurable (uses default)
- PUBACK reason codes not exposed to application
- Server DISCONNECT packets not exposed (detected via operation failures)

**Buffer Sizing**:
- TCP RX buffer: 4096 bytes (embassy_net::tcp::TcpSocket)
- TCP TX buffer: 4096 bytes (embassy_net::tcp::TcpSocket)
- MQTT receive buffer: 2048 bytes (for incoming packets)
- MQTT write buffer: 2048 bytes (for outgoing packets)
- Channel capacity: 20 sensor readings (buffers ~100 seconds during outages)

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

**Hardware Configuration** (ADC setup in src/bin/main.rs):
- ADC1 configured on GPIO0 (Xiao ESP32-C6 pin A0)
- 6dB attenuation (measuring range 0-2450mV)
- Resistive moisture sensor (shows higher voltage when wet, lower when dry)

**Calibration** (constants in src/sensor.rs):
- `SENSOR_DRY = 2188`: ADC reading in air (0% moisture)
- `SENSOR_WET = 4095`: ADC reading fully submerged (100% moisture)
- `MOISTURE_THRESHOLD = 30`: Watering threshold (30% moisture)
- Run calibration routine (commented at end of main.rs) to recalibrate for different sensors

**Conversion Function** (src/sensor.rs):
```rust
pub fn raw_to_moisture_percent(raw: u16) -> u8
```
- Linear interpolation between calibration points
- Returns 0-100% moisture level
- Handles out-of-range values (clamps to 0% or 100%)

**Sensor Task Behavior** (`moisture_sensor_task` in src/sensor.rs):
- Reads sensor every 5 seconds
- Converts raw ADC values to moisture percentage
- Creates `SensorReading` struct with moisture, raw value, and timestamp
- Sends readings to channel (non-blocking, drops if channel full)
- Logs readings and warns when moisture < threshold (30% default)
- Continues operation independently of network/MQTT status

### Calibration Procedure

A calibration routine is preserved as commented code at the end of `src/bin/main.rs`. To recalibrate:

1. Replace the main loop with the calibration code
2. Flash to device: `cargo run`
3. Follow RTT prompts:
   - Keep sensor in air for 10 dry readings
   - Submerge sensor in water for 10 wet readings
4. Update `SENSOR_DRY` and `SENSOR_WET` constants in `src/sensor.rs` with the averages
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
- Helper functions in `src/mqtt/discovery.rs`:
  - `create_moisture_discovery_payload()` - Moisture sensor discovery JSON
  - `create_raw_discovery_payload()` - Raw ADC sensor discovery JSON
  - `build_discovery_topic()` - Format discovery topic strings
  - `build_state_topic()` - Format state topic strings (also used for telemetry publishing)
  - `build_availability_topic()` - Format availability topic
  - `publish_discovery()` - Publishes all discovery messages with proper pacing

**Publishing Sequence** (when MQTT client is connected):
1. Publish availability as `online` (retain=true)
2. Publish moisture sensor discovery (retain=true)
3. Publish raw ADC sensor discovery (retain=true)
4. Small delays (100ms) between publishes to avoid overwhelming broker

**Current Status**: Discovery message generation and publishing are fully implemented with rust-mqtt v0.3 and embassy-net.

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

### MQTT Configuration and Architecture (Fully Implemented)

**Configuration Constants** (src/bin/main.rs):
- `MQTT_BROKER_HOST`: Broker IP or hostname (default: "192.168.1.100") - configure for your network
- `MQTT_BROKER_PORT`: Broker port (default: 1883)
- `MQTT_KEEP_ALIVE_SECS`: Keep-alive interval (60 seconds)
- `MQTT_SESSION_EXPIRY_SECS`: Session expiry for battery-powered mode (3600 seconds / 1 hour)
- `MQTT_USERNAME` / `MQTT_PASSWORD`: Authentication credentials (empty strings for no auth)

**Task Architecture**:
- `embassy_net_task`: Runs embassy-net stack runner for packet processing (TCP, DHCP, DNS)
- `network_task`: Manages Wi-Fi connection in STA mode with automatic reconnection, coordinates with embassy-net stack
- `mqtt_connection_task`: Manages MQTT broker connection lifecycle with rust-mqtt v5 client, DNS resolution, TCP connection via `EmbassyNetTransport`, exponential backoff reconnection, publishes discovery and availability messages
- `mqtt_publish_task`: Receives sensor readings from channel and publishes telemetry to MQTT broker
- `moisture_sensor_task`: Reads ADC every 5 seconds, converts to percentage, sends readings to channel (independent of network status)
- `publish_discovery()`: Async function that generates and publishes discovery messages for all sensors with QoS 1 and retain flag

**Transport Layer**:
- `EmbassyNetTransport`: Adapter wrapping `embassy_net::tcp::TcpSocket` for rust-mqtt client
- Implements `embedded_io_async::Read` and `embedded_io_async::Write` traits
- Handles TCP connection establishment and error mapping

**Discovery Integration**:
- Discovery messages are published after successful MQTT connection with 100ms pacing
- Discovery is re-published on reconnection to handle Home Assistant restarts
- Availability status (`online`/`offline`) published with retain flag

**Error Handling and Resilience**:
- DNS resolution with exponential backoff and fallback to IP address
- TCP connection with exponential backoff (2s → 30s max)
- Automatic reconnection on connection loss
- Sensor readings buffered in channel (20-reading capacity) during network outages
- All errors logged without panicking

### Next Implementation Steps

**Pump Control** (Next Priority):
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
