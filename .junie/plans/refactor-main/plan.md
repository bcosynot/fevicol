# Refactoring Plan: src/bin/main.rs

## Overview

The `src/bin/main.rs` file has grown to 1441 lines and contains multiple distinct concerns mixed together. This plan outlines a pragmatic refactoring approach to improve readability, maintainability, and extensibility without being overly granular.

## Current State Analysis

### File Structure (1441 lines)

| Section | Lines | Description |
|---------|-------|-------------|
| Imports & attributes | 1-41 | Standard imports, feature flags, lint attributes |
| MQTT abstraction | 43-96 | `MqQos`, `MqttPublish` trait, `LoggerPublisher` |
| Transport adapter | 98-146 | `EmbassyNetTransport` for rust-mqtt |
| Client wrapper | 148-234 | `RustMqttPublisher`, `interpret_connack_reason` |
| Client init | 236-317 | `MqttClientConfig`, `init_rust_mqtt_client` |
| Constants & types | 319-386 | Type aliases, device IDs, calibration constants |
| Sensor conversion | 388-402 | `raw_to_moisture_percent` |
| HA discovery helpers | 404-583 | Topic builders, payload creators, `publish_discovery` |
| Embassy tasks | 585-1171 | 4 async tasks (sensor, mqtt, net runner, wifi) |
| Main function | 1173-1339 | Initialization and task spawning |
| Calibration routine | 1341-1440 | Commented-out calibration code |

### Key Pain Points

1. **Monolithic mqtt_connection_task (445 lines)**: This single function handles DNS resolution, TCP connection, MQTT handshake, discovery publishing, telemetry loop, error handling, and reconnection logic. It has deeply nested control flow that's hard to follow.

2. **Feature flag complexity**: Heavy use of `#[cfg(feature = "mqtt")]` throughout the file creates visual noise and makes it hard to understand the code flow for either configuration.

3. **Mixed concerns**: MQTT client implementation details, Home Assistant discovery protocol, sensor logic, and network management are all interleaved in one file.

4. **Dead code**: ~100 lines of commented calibration routine adds noise.

5. **Implicit dependencies**: Static signals (`NETWORK_READY`, `MQTT_CONNECTED`, `MQTT_CONNECTION_HEALTHY`) and channels create hidden coupling between components.

---

## Refactoring Plan

### Phase 1: Extract MQTT Client Module

**What**: Move all MQTT client infrastructure to `src/mqtt/client.rs`

**Includes**:
- `MqQos` enum
- `MqttPublish` trait
- `LoggerPublisher` struct and impl
- `EmbassyNetTransport` struct and impls
- `RustMqttPublisher` struct and impl
- `MqttClientConfig` struct
- `init_rust_mqtt_client` function
- `interpret_connack_reason` function

**Why**: 
- These components form a cohesive MQTT client abstraction layer
- They're reusable and independent of the application logic
- Isolating feature-gated code in one place reduces `#[cfg]` noise in main.rs
- Future MQTT client changes (e.g., switching libraries) are contained

**Result**: ~275 lines moved out, clean `pub use` re-exports

---

### Phase 2: Extract Home Assistant Discovery Module

**What**: Move Home Assistant MQTT Discovery logic to `src/mqtt/discovery.rs` or `src/homeassistant.rs`

**Includes**:
- `build_discovery_topic` function
- `build_state_topic` function  
- `build_availability_topic` function
- `create_moisture_discovery_payload` function
- `create_raw_discovery_payload` function
- `format_unique_id` function
- `publish_discovery` function
- Device metadata constants (`DEVICE_NAME`, `DEVICE_MANUFACTURER`, `DEVICE_MODEL`, `VERSION`)

**Why**:
- Home Assistant discovery is a distinct protocol with its own conventions
- Payload generation is verbose (manual JSON building) and benefits from isolation
- Adding new sensor types (e.g., pump status, temperature) will require similar discovery payloads
- Keeps main.rs focused on application orchestration

**Result**: ~180 lines moved out, discovery becomes a reusable component

---

### Phase 3: Extract Sensor Module

**What**: Move moisture sensor logic to `src/sensor.rs`

**Includes**:
- `SensorReading` struct
- Calibration constants (`SENSOR_DRY`, `SENSOR_WET`, `MOISTURE_THRESHOLD`)
- `raw_to_moisture_percent` function
- Type aliases (`MoistureAdc`, `MoistureAdcPin`)
- `moisture_sensor_task` function

**Why**:
- Sensor logic is self-contained and hardware-specific
- Future sensors (temperature, light) will follow similar patterns
- Calibration constants and conversion logic belong together
- Task can be tested independently (with mock ADC in future)

**Consideration**: The calibration routine (currently commented) could become a feature-gated calibration mode in this module.

**Result**: ~60 lines moved out, sensor becomes extensible

---

### Phase 4: Refactor mqtt_connection_task

**What**: Break down the 445-line mqtt_connection_task into smaller, focused functions

**Current structure** (deeply nested):
```
mqtt_connection_task
└── outer loop (reconnection)
    ├── DNS resolution with fallback
    ├── TCP socket setup
    ├── MQTT client init
    └── inner loop (telemetry)
        ├── select on sensor/timeout
        ├── publish moisture
        └── publish raw
```

**Proposed structure**:
```
mqtt_connection_task
└── outer loop
    └── run_mqtt_session() -> Result<(), MqttError>
        ├── resolve_broker_address() -> Result<Ipv4Address, DnsError>
        ├── establish_tcp_connection() -> Result<TcpSocket, TcpError>
        ├── connect_mqtt_client() -> Result<RustMqttPublisher, ReasonCode>
        └── run_telemetry_loop() -> Result<(), PublishError>
```

**Why**:
- Each function has a single responsibility
- Error handling becomes clearer (each function returns Result)
- Easier to understand the connection lifecycle
- Individual stages can be tested or modified independently
- Reduces nesting depth from 5+ levels to 2-3

**Note**: These helper functions can stay in main.rs initially, or move to `src/mqtt/connection.rs` if they grow.

**Result**: Same line count but dramatically improved readability

---

### Phase 5: Simplify Main Function

**What**: After phases 1-4, main.rs should be streamlined

**Remaining in main.rs**:
- `#![no_std]` attributes and essential imports
- Static resources (`NETWORK_READY`, `MQTT_CONNECTED`, `SENSOR_CHANNEL`, etc.)
- Device/sensor ID constants (`DEVICE_ID`, `SENSOR_ID`)
- `network_task` and `embassy_net_task` (or extract to `src/network.rs` if desired)
- `mqtt_connection_task` (refactored)
- `main` function (initialization and task spawning)

**Why**:
- main.rs becomes the application entry point and orchestrator
- Easy to see what tasks are spawned and how they communicate
- New developers can understand the system architecture at a glance

**Result**: main.rs reduced to ~400-500 lines focused on orchestration

---

### Phase 6: Remove Dead Code

**What**: Delete the commented calibration routine (lines 1341-1440). Convert to a `#[cfg(feature = "calibration")]` mode in the sensor module

**Why**:
- 100 lines of commented code adds noise
- Implement it properly as a feature flag

**Result**: ~100 lines removed

---

## Proposed Module Structure

```
src/
├── bin/
│   ├── main.rs          # Entry point, task orchestration (~400-500 lines)
│   └── secrets.rs       # Local credentials (git-ignored)
├── lib.rs               # Crate root, module declarations
├── mqtt/
│   ├── mod.rs           # Re-exports
│   ├── client.rs        # MQTT client abstraction (~275 lines)
│   └── discovery.rs     # Home Assistant discovery (~180 lines)
├── sensor.rs            # Moisture sensor logic (~60 lines)
└── network.rs           # (Optional) Wi-Fi/network tasks (~90 lines)
```

---

## Benefits

1. **Readability**: Each file has a clear, single purpose
2. **Maintainability**: Changes to MQTT client don't affect sensor logic
3. **Extensibility**: Adding new sensors or MQTT features follows established patterns
4. **Testability**: Modules can be unit tested independently (future improvement)
5. **Onboarding**: New developers can understand the system by reading main.rs first

---

## Implementation Notes

- **Feature flags**: Keep `#[cfg(feature = "mqtt")]` in module files, not scattered throughout
- **Visibility**: Use `pub(crate)` for internal APIs, `pub` only for truly public interfaces
- **Re-exports**: Use `pub use` in mod.rs files for clean imports in main.rs
- **Static resources**: Consider grouping related statics into a `Resources` struct
- **Error types**: Consider a unified error enum for mqtt_connection_task stages

---

## Execution Order

The phases are ordered by dependency and impact:

1. **Phase 1 (MQTT Client)** - Largest extraction, enables Phase 2
2. **Phase 2 (HA Discovery)** - Depends on MqttPublish trait from Phase 1
3. **Phase 3 (Sensor)** - Independent, can be done in parallel with 1-2
4. **Phase 4 (Refactor mqtt_connection_task)** - Best done after 1-2 reduce the file size
5. **Phase 5 (Simplify Main)** - Natural result of 1-4
6. **Phase 6 (Remove Dead Code)** - Quick win, can be done anytime

---

## Success Criteria

- [ ] main.rs is under 500 lines
- [ ] No function exceeds 100 lines
- [ ] Each module has a clear, documented purpose
- [ ] Feature flag logic is contained within relevant modules
- [ ] `cargo build` and `cargo build --features mqtt` both succeed
- [ ] `cargo test --no-run` succeeds
- [ ] Existing functionality is preserved (manual testing on device)
