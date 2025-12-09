# Task Breakdown: Refactoring src/bin/main.rs

This document provides a detailed task breakdown for implementing the refactoring plan outlined in `plan.md`. Tasks are organized by phase and follow the execution order specified in the plan.

---

## Task List

### Phase 1: Extract MQTT Client Module

**Goal**: Move all MQTT client infrastructure to `src/mqtt/client.rs` (~275 lines)

**Reference**: Plan sections "Phase 1: Extract MQTT Client Module" and "Proposed Module Structure"

#### Sub-phase 1.1: Create Module Structure
- [x] (Phase 1.1) Create the `src/mqtt/` directory structure
- [x] (Phase 1.1) Create `src/mqtt/mod.rs` with placeholder module declarations
- [x] (Phase 1.1) Create empty `src/mqtt/client.rs` file
- [x] (Phase 1.1) Update `src/lib.rs` to declare the `mqtt` module

#### Sub-phase 1.2: Extract Core MQTT Types
- [x] (Phase 1.2) Move `MqQos` enum from `main.rs` to `src/mqtt/client.rs`
- [x] (Phase 1.2) Move `MqttPublish` trait from `main.rs` to `src/mqtt/client.rs`
- [x] (Phase 1.2) Move `LoggerPublisher` struct and its `MqttPublish` impl to `src/mqtt/client.rs`
- [x] (Phase 1.2) Add appropriate visibility modifiers (`pub`, `pub(crate)`) to exported items
- [x] (Phase 1.2) Add re-exports in `src/mqtt/mod.rs` for public API

#### Sub-phase 1.3: Extract Transport Adapter (feature-gated)
- [x] (Phase 1.3) Move `EmbassyNetTransport` struct to `src/mqtt/client.rs`
- [x] (Phase 1.3) Move `EmbassyNetTransport::new()` and `socket_mut()` methods
- [x] (Phase 1.3) Move `ErrorType`, `Read`, and `IoWrite` trait implementations
- [x] (Phase 1.3) Wrap with `#[cfg(feature = "mqtt")]` attribute
- [x] (Phase 1.3) Add necessary imports for `embassy_net`, `embedded_io_async` types

#### Sub-phase 1.4: Extract Client Wrapper (feature-gated)
- [x] (Phase 1.4) Move `RustMqttPublisher` struct to `src/mqtt/client.rs`
- [x] (Phase 1.4) Move `MqttPublish` implementation for `RustMqttPublisher`
- [x] (Phase 1.4) Move `interpret_connack_reason` function
- [x] (Phase 1.4) Move `MqttClientConfig` struct
- [x] (Phase 1.4) Move `init_rust_mqtt_client` function
- [x] (Phase 1.4) Wrap all with `#[cfg(feature = "mqtt")]` attribute
- [x] (Phase 1.4) Add necessary imports for `rust_mqtt` types

#### Sub-phase 1.5: Update main.rs Imports
- [x] (Phase 1.5) Remove extracted code from `main.rs`
- [x] (Phase 1.5) Add import statements for the new `mqtt::client` module items
- [x] (Phase 1.5) Update any type references to use the new module path
- [x] (Phase 1.5) Verify feature-gated imports work correctly for both `mqtt` and non-`mqtt` builds

#### Sub-phase 1.6: Verification and Quality Checks
- [x] (Phase 1.6) Run `cargo fmt --all -- --check` and fix any formatting issues
- [x] (Phase 1.6) Run `cargo clippy --all-features --workspace -- -D warnings` and address warnings
- [x] (Phase 1.6) Run `cargo build` (without mqtt feature) to verify non-mqtt build
- [x] (Phase 1.6) Run `cargo build --features mqtt` to verify mqtt build
- [x] (Phase 1.6) Run `cargo test --no-run` to verify test compilation

#### Sub-phase 1.7: Documentation Updates
- [x] (Phase 1.7) Update `CLAUDE.md` section "MQTT Client Implementation" to reference new module location
- [x] (Phase 1.7) Update `guidelines.md` section "MQTT Client Implementation" to reference new module location
- [x] (Phase 1.7) Update `README.md` "Project Structure" section to reflect new `src/mqtt/` directory

---

### Phase 2: Extract Home Assistant Discovery Module

**Goal**: Move Home Assistant MQTT Discovery logic to `src/mqtt/discovery.rs` (~180 lines)

**Reference**: Plan section "Phase 2: Extract Home Assistant Discovery Module"

**Dependency**: Phase 1 must be completed (depends on `MqttPublish` trait)

#### Sub-phase 2.1: Create Discovery Module
- [x] (Phase 2.1) Create `src/mqtt/discovery.rs` file
- [x] (Phase 2.1) Add `discovery` module declaration to `src/mqtt/mod.rs`

#### Sub-phase 2.2: Extract Device Metadata Constants
- [x] (Phase 2.2) Move `DEVICE_NAME` constant to `src/mqtt/discovery.rs`
- [x] (Phase 2.2) Move `DEVICE_MANUFACTURER` constant
- [x] (Phase 2.2) Move `DEVICE_MODEL` constant
- [x] (Phase 2.2) Move `VERSION` constant
- [x] (Phase 2.2) Add appropriate visibility modifiers

#### Sub-phase 2.3: Extract Topic Builder Functions
- [x] (Phase 2.3) Move `build_discovery_topic` function to `src/mqtt/discovery.rs`
- [x] (Phase 2.3) Move `build_state_topic` function
- [x] (Phase 2.3) Move `build_availability_topic` function
- [x] (Phase 2.3) Move `format_unique_id` function
- [x] (Phase 2.3) Add necessary imports for `heapless::String`

#### Sub-phase 2.4: Extract Payload Creator Functions
- [x] (Phase 2.4) Move `create_moisture_discovery_payload` function to `src/mqtt/discovery.rs`
- [x] (Phase 2.4) Move `create_raw_discovery_payload` function
- [x] (Phase 2.4) Ensure functions reference device metadata constants correctly

#### Sub-phase 2.5: Extract Discovery Publisher
- [x] (Phase 2.5) Move `publish_discovery` function to `src/mqtt/discovery.rs`
- [x] (Phase 2.5) Update function to import `MqttPublish` trait from `super::client`
- [x] (Phase 2.5) Add re-exports in `src/mqtt/mod.rs` for public discovery API

#### Sub-phase 2.6: Update main.rs Imports
- [x] (Phase 2.6) Remove extracted code from `main.rs`
- [x] (Phase 2.6) Add import statements for the new `mqtt::discovery` module items
- [x] (Phase 2.6) Update any function calls to use the new module path

#### Sub-phase 2.7: Verification and Quality Checks
- [x] (Phase 2.7) Run `cargo fmt --all -- --check` and fix any formatting issues
- [x] (Phase 2.7) Run `cargo clippy --all-features --workspace -- -D warnings` and address warnings
- [x] (Phase 2.7) Run `cargo build` (without mqtt feature) to verify non-mqtt build
- [x] (Phase 2.7) Run `cargo build --features mqtt` to verify mqtt build
- [x] (Phase 2.7) Run `cargo test --no-run` to verify test compilation

#### Sub-phase 2.8: Documentation Updates
- [x] (Phase 2.8) Update `CLAUDE.md` section "Home Assistant MQTT Discovery" to reference new module location
- [x] (Phase 2.8) Update `guidelines.md` if it references discovery functions
- [x] (Phase 2.8) Update `README.md` "Project Structure" section if not already updated in Phase 1

---

### Phase 3: Extract Sensor Module

**Goal**: Move moisture sensor logic to `src/sensor.rs` (~60 lines)

**Reference**: Plan section "Phase 3: Extract Sensor Module"

**Note**: This phase is independent and can be done in parallel with Phases 1-2

#### Sub-phase 3.1: Create Sensor Module
- [x] (Phase 3.1) Create `src/sensor.rs` file
- [x] (Phase 3.1) Add `sensor` module declaration to `src/lib.rs`

#### Sub-phase 3.2: Extract Sensor Types and Constants
- [x] (Phase 3.2) Move `SensorReading` struct to `src/sensor.rs`
- [x] (Phase 3.2) Move `SENSOR_DRY` calibration constant
- [x] (Phase 3.2) Move `SENSOR_WET` calibration constant
- [x] (Phase 3.2) Move `MOISTURE_THRESHOLD` constant
- [x] (Phase 3.2) Move `MoistureAdc` type alias
- [x] (Phase 3.2) Move `MoistureAdcPin` type alias
- [x] (Phase 3.2) Add appropriate visibility modifiers and derive macros

#### Sub-phase 3.3: Extract Conversion Function
- [x] (Phase 3.3) Move `raw_to_moisture_percent` function to `src/sensor.rs`
- [x] (Phase 3.3) Ensure function references calibration constants correctly
- [x] (Phase 3.3) Add documentation comments if not present

#### Sub-phase 3.4: Extract Sensor Task
- [x] (Phase 3.4) Move `moisture_sensor_task` function to `src/sensor.rs`
- [x] (Phase 3.4) Add necessary imports for `embassy_executor`, `embassy_time`, `embassy_sync`, `defmt`
- [x] (Phase 3.4) Add necessary imports for ADC types from `esp_hal`
- [x] (Phase 3.4) Update function signature visibility as needed

#### Sub-phase 3.5: Update main.rs Imports
- [x] (Phase 3.5) Remove extracted code from `main.rs`
- [x] (Phase 3.5) Add import statements for the new `sensor` module items
- [x] (Phase 3.5) Update task spawning to use the new module path
- [x] (Phase 3.5) Ensure `SensorReading` is accessible where needed (e.g., in mqtt_connection_task)

#### Sub-phase 3.6: Verification and Quality Checks
- [x] (Phase 3.6) Run `cargo fmt --all -- --check` and fix any formatting issues
- [x] (Phase 3.6) Run `cargo clippy --all-features --workspace -- -D warnings` and address warnings
- [x] (Phase 3.6) Run `cargo build` to verify build
- [x] (Phase 3.6) Run `cargo build --features mqtt` to verify mqtt build
- [x] (Phase 3.6) Run `cargo test --no-run` to verify test compilation

#### Sub-phase 3.7: Documentation Updates
- [x] (Phase 3.7) Update `CLAUDE.md` section "Moisture Sensing" to reference new module location
- [x] (Phase 3.7) Update `CLAUDE.md` section "Calibration Procedure" to reference new module
- [x] (Phase 3.7) Update `guidelines.md` section "Sensor calibration" to reference new module
- [x] (Phase 3.7) Update `README.md` "Project Structure" section to include `src/sensor.rs`

---

### Phase 4: Refactor mqtt_connection_task

**Goal**: Break down the 445-line `mqtt_connection_task` into smaller, focused functions

**Reference**: Plan section "Phase 4: Refactor mqtt_connection_task"

**Dependency**: Best done after Phases 1-2 reduce the file size

#### Sub-phase 4.1: Define Error Types
- [ ] (Phase 4.1) Create a unified error enum for mqtt_connection_task stages (e.g., `MqttSessionError`)
- [ ] (Phase 4.1) Add variants for DNS resolution, TCP connection, MQTT connection, and publish errors
- [ ] (Phase 4.1) Implement `defmt::Format` for the error type for logging

#### Sub-phase 4.2: Extract DNS Resolution Helper
- [ ] (Phase 4.2) Create `resolve_broker_address` async function
- [ ] (Phase 4.2) Move DNS resolution logic from mqtt_connection_task
- [ ] (Phase 4.2) Include exponential backoff and IP fallback logic
- [ ] (Phase 4.2) Return `Result<Ipv4Address, DnsError>` or similar

#### Sub-phase 4.3: Extract TCP Connection Helper
- [ ] (Phase 4.3) Create `establish_tcp_connection` async function
- [ ] (Phase 4.3) Move TCP socket creation and connection logic
- [ ] (Phase 4.3) Return `Result<TcpSocket, TcpError>` or similar

#### Sub-phase 4.4: Extract MQTT Client Connection Helper
- [ ] (Phase 4.4) Create `connect_mqtt_client` async function
- [ ] (Phase 4.4) Move MQTT client initialization and CONNECT handshake logic
- [ ] (Phase 4.4) Include availability and discovery publishing
- [ ] (Phase 4.4) Return `Result<RustMqttPublisher, ReasonCode>` or similar

#### Sub-phase 4.5: Extract Telemetry Loop Helper
- [ ] (Phase 4.5) Create `run_telemetry_loop` async function
- [ ] (Phase 4.5) Move the inner telemetry publishing loop logic
- [ ] (Phase 4.5) Handle sensor reading reception and MQTT publishing
- [ ] (Phase 4.5) Return `Result<(), PublishError>` or similar

#### Sub-phase 4.6: Refactor mqtt_connection_task
- [ ] (Phase 4.6) Refactor mqtt_connection_task to use the new helper functions
- [ ] (Phase 4.6) Create `run_mqtt_session` function that orchestrates the helpers
- [ ] (Phase 4.6) Simplify the outer reconnection loop
- [ ] (Phase 4.6) Reduce nesting depth from 5+ levels to 2-3

#### Sub-phase 4.7: Decide on Helper Function Location
- [ ] (Phase 4.7) Evaluate whether helpers should stay in `main.rs` or move to `src/mqtt/connection.rs`
- [ ] (Phase 4.7) If moving, create `src/mqtt/connection.rs` and update `src/mqtt/mod.rs`
- [ ] (Phase 4.7) Update imports accordingly

#### Sub-phase 4.8: Verification and Quality Checks
- [ ] (Phase 4.8) Run `cargo fmt --all -- --check` and fix any formatting issues
- [ ] (Phase 4.8) Run `cargo clippy --all-features --workspace -- -D warnings` and address warnings
- [ ] (Phase 4.8) Run `cargo build --features mqtt` to verify mqtt build
- [ ] (Phase 4.8) Run `cargo test --no-run` to verify test compilation
- [ ] (Phase 4.8) Verify no function exceeds 100 lines (success criteria from plan)

#### Sub-phase 4.9: Documentation Updates
- [ ] (Phase 4.9) Update `CLAUDE.md` section "Connection Lifecycle" to reflect new function structure
- [ ] (Phase 4.9) Update `CLAUDE.md` section "Error Handling" if error types changed
- [ ] (Phase 4.9) Update `guidelines.md` if mqtt_connection_task documentation needs updating

---

### Phase 5: Simplify Main Function

**Goal**: Ensure main.rs is streamlined and focused on orchestration (~400-500 lines)

**Reference**: Plan section "Phase 5: Simplify Main Function"

**Dependency**: Natural result of Phases 1-4

#### Sub-phase 5.1: Review Remaining main.rs Content
- [ ] (Phase 5.1) Verify main.rs contains only: `#![no_std]` attributes and essential imports, static resources, device/sensor ID constants, network tasks, refactored `mqtt_connection_task`, and `main` function
- [ ] (Phase 5.1) Identify any remaining code that should be extracted

#### Sub-phase 5.2: Optional Network Module Extraction
- [ ] (Phase 5.2) Evaluate if `network_task` and `embassy_net_task` should move to `src/network.rs`
- [ ] (Phase 5.2) If yes, create `src/network.rs` and move network tasks
- [ ] (Phase 5.2) Update `src/lib.rs` to declare the `network` module
- [ ] (Phase 5.2) Update imports in `main.rs`

#### Sub-phase 5.3: Organize Static Resources
- [ ] (Phase 5.3) Review static resources grouping
- [ ] (Phase 5.3) Consider grouping related statics (as noted in plan's Implementation Notes)
- [ ] (Phase 5.3) Ensure clear documentation comments for static resources

#### Sub-phase 5.4: Clean Up Imports
- [ ] (Phase 5.4) Organize imports following existing code style
- [ ] (Phase 5.4) Remove any unused imports
- [ ] (Phase 5.4) Group imports logically (std/core, external crates, internal modules)

#### Sub-phase 5.5: Verification and Quality Checks
- [ ] (Phase 5.5) Run `cargo fmt --all -- --check` and fix any formatting issues
- [ ] (Phase 5.5) Run `cargo clippy --all-features --workspace -- -D warnings` and address warnings
- [ ] (Phase 5.5) Run `cargo build` to verify non-mqtt build
- [ ] (Phase 5.5) Run `cargo build --features mqtt` to verify mqtt build
- [ ] (Phase 5.5) Run `cargo test --no-run` to verify test compilation
- [ ] (Phase 5.5) Verify main.rs is under 500 lines (success criteria from plan)

#### Sub-phase 5.6: Documentation Updates
- [ ] (Phase 5.6) Update `CLAUDE.md` "Architecture" section to reflect final module structure
- [ ] (Phase 5.6) Update `CLAUDE.md` "Application Architecture" section with task locations
- [ ] (Phase 5.6) Update `guidelines.md` "Main binary structure" section
- [ ] (Phase 5.6) Update `README.md` "Project Structure" section with final structure

---

### Phase 6: Remove Dead Code

**Goal**: Delete commented calibration routine and implement as feature flag (~100 lines removed)

**Reference**: Plan section "Phase 6: Remove Dead Code"

**Note**: This can be done anytime but is placed last for logical flow

#### Sub-phase 6.1: Preserve Calibration Logic
- [ ] (Phase 6.1) Review the commented calibration routine (lines 1341-1440 in original main.rs)
- [ ] (Phase 6.1) Document the calibration procedure if not already documented
- [ ] (Phase 6.1) Decide on implementation approach: Option A (feature-gated calibration mode in sensor module), Option B (separate calibration binary), or Option C (runtime calibration command via MQTT)

#### Sub-phase 6.2: Implement Calibration Feature (if Option A)
- [ ] (Phase 6.2) Add `calibration` feature to `Cargo.toml`
- [ ] (Phase 6.2) Create calibration function in `src/sensor.rs`
- [ ] (Phase 6.2) Add `#[cfg(feature = "calibration")]` gating
- [ ] (Phase 6.2) Implement calibration loop that can replace normal sensor task

#### Sub-phase 6.3: Remove Dead Code
- [ ] (Phase 6.3) Delete the commented calibration routine from `main.rs`
- [ ] (Phase 6.3) Remove any other commented-out code blocks
- [ ] (Phase 6.3) Remove any unused imports or dead code identified by clippy

#### Sub-phase 6.4: Verification and Quality Checks
- [ ] (Phase 6.4) Run `cargo fmt --all -- --check` and fix any formatting issues
- [ ] (Phase 6.4) Run `cargo clippy --all-features --workspace -- -D warnings` and address warnings
- [ ] (Phase 6.4) Run `cargo build` to verify build
- [ ] (Phase 6.4) Run `cargo build --features mqtt` to verify mqtt build
- [ ] (Phase 6.4) Run `cargo build --features calibration` (if implemented) to verify calibration build
- [ ] (Phase 6.4) Run `cargo test --no-run` to verify test compilation

#### Sub-phase 6.5: Documentation Updates
- [ ] (Phase 6.5) Update `CLAUDE.md` "Calibration Procedure" section to reference new calibration feature/approach
- [ ] (Phase 6.5) Update `guidelines.md` "Sensor calibration" section
- [ ] (Phase 6.5) Update `README.md` if calibration feature is added to features list

---

### Final Verification

**Goal**: Verify all success criteria from the plan are met

- [ ] (Final) Verify main.rs is under 500 lines
- [ ] (Final) Verify no function exceeds 100 lines
- [ ] (Final) Verify each module has a clear, documented purpose
- [ ] (Final) Verify feature flag logic is contained within relevant modules
- [ ] (Final) Run `cargo build` and verify success
- [ ] (Final) Run `cargo build --features mqtt` and verify success
- [ ] (Final) Run `cargo test --no-run` and verify success
- [ ] (Final) Run `cargo fmt --all -- --check` and verify no formatting issues
- [ ] (Final) Run `cargo clippy --all-features --workspace -- -D warnings` and verify no warnings
- [ ] (Final) Review all documentation updates for consistency
- [ ] (Final) Manual testing on device (if hardware available) to verify existing functionality is preserved

---

## Summary

| Phase | Estimated Lines Moved/Removed | Key Deliverables |
|-------|------------------------------|------------------|
| Phase 1 | ~275 lines moved | `src/mqtt/client.rs`, `src/mqtt/mod.rs` |
| Phase 2 | ~180 lines moved | `src/mqtt/discovery.rs` |
| Phase 3 | ~60 lines moved | `src/sensor.rs` |
| Phase 4 | 0 (refactor in place) | Smaller, focused functions |
| Phase 5 | Variable | Clean orchestration in main.rs |
| Phase 6 | ~100 lines removed | Calibration feature (optional) |

**Total Impact**: main.rs reduced from ~1441 lines to ~400-500 lines
