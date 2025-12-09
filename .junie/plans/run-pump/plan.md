# Pump Control Implementation Plan

## Overview

This plan outlines the implementation of pump control for the Fevicol smart plant watering system. The pump is controlled via a relay module connected to the ESP32-C6, enabling both automatic watering based on moisture thresholds and manual control via MQTT commands.

## Hardware Specifications

| Component | Specification |
|-----------|---------------|
| **Relay Module** | DC 12V working voltage, 2-5mA trigger current, 0-4V trigger voltage (low) |
| **Pump** | 12V peristaltic pump |
| **Control GPIO** | GPIO1 (D0 on Xiao ESP32-C6) |
| **Relay Power** | Separate power supply (not from ESP32-C6 rail) |
| **Trigger Logic** | Configurable (Active LOW default, with constant to switch to Active HIGH) |

## Requirements Summary

### Safety Limits
- **Maximum pump run time**: 30 seconds per activation
- **Minimum interval between waterings**: 2 hours
- **Error handling**: Disable automatic pump activation on sensor errors, trigger MQTT alert

### Control Modes
- **Automatic**: Pump activates when moisture drops below configurable threshold
- **Manual**: MQTT commands to start/stop pump (overrides automatic)

### Architecture
- **Initial scope**: Single pump implementation
- **Design for**: Up to 4 pumps per device (extensible)
- **Sensor-pump mapping**: Configurable via MQTT (which sensor triggers which pump)

### MQTT Integration
- **Home Assistant Discovery**: Expose pump as `switch` entity
- **Telemetry**: Pump state, last run timestamp, total run count, cumulative run time, last watering duration
- **State persistence**: Session-only (Home Assistant tracks history)
- **Remote configuration**: Moisture threshold via MQTT `number` entity (HA stores value persistently)

---

## Implementation Tasks

### Phase 1: Core Pump Control Module

#### Task 1.1: Create Pump Hardware Abstraction
**File**: `src/pump.rs`

Create a pump control module with hardware abstraction:

```
- Define `PumpConfig` struct:
  - gpio_pin: u8
  - active_low: bool (default: true)
  - max_run_time_secs: u16 (default: 30)
  - min_interval_secs: u32 (default: 7200 = 2 hours)

- Define `PumpState` struct:
  - is_running: bool
  - last_start_time: Option<Instant>
  - last_stop_time: Option<Instant>
  - current_run_duration_ms: u64
  - total_run_count: u32 (since boot)
  - cumulative_run_time_ms: u64 (since boot)

- Define `PumpController` struct:
  - config: PumpConfig
  - state: PumpState
  - gpio_output: Output<'static>

- Implement methods:
  - new(pin: GpioPin, config: PumpConfig) -> Self
  - start(&mut self) -> Result<(), PumpError>
  - stop(&mut self) -> Result<(), PumpError>
  - is_running(&self) -> bool
  - get_state(&self) -> &PumpState
  - can_water(&self) -> bool (checks min_interval)
  - time_until_can_water(&self) -> Option<Duration>
```

**Acceptance Criteria**:
- [ ] GPIO output correctly controls relay (respects active_low setting)
- [ ] State tracking is accurate
- [ ] Safety checks prevent starting if min_interval not elapsed

---

#### Task 1.2: Create Pump Control Task
**File**: `src/pump.rs` (add to module)

Create an Embassy task for pump control with safety enforcement:

```
- Define `PumpCommand` enum:
  - Start { manual: bool }
  - Stop
  - EmergencyStop

- Define `PumpEvent` enum (for telemetry):
  - Started { manual: bool, timestamp: u64 }
  - Stopped { duration_ms: u64, reason: StopReason }
  - Error { error: PumpError }

- Define `StopReason` enum:
  - Manual
  - AutoComplete
  - MaxRunTimeExceeded
  - EmergencyStop
  - SensorError

- Create pump_control_task:
  - Receives commands via Channel<PumpCommand>
  - Sends events via Channel<PumpEvent>
  - Enforces max_run_time (auto-stop after 30s)
  - Tracks all state transitions
  - Logs all operations via defmt
```

**Acceptance Criteria**:
- [ ] Task responds to start/stop commands
- [ ] Auto-stops after max_run_time (30 seconds)
- [ ] Emits events for all state changes
- [ ] Emergency stop works immediately

---

#### Task 1.3: Integrate with Main Binary
**File**: `src/bin/main.rs`

Add pump initialization and task spawning:

```
- Add pump GPIO configuration (GPIO1)
- Create PumpController with default config
- Create command and event channels
- Spawn pump_control_task
- Add pump state to shared application state
```

**Acceptance Criteria**:
- [ ] Pump task starts successfully
- [ ] GPIO1 is configured as output
- [ ] No conflicts with existing peripherals

---

### Phase 2: Automatic Watering Logic

#### Task 2.1: Create Watering Decision Logic
**File**: `src/watering.rs`

Create module for automatic watering decisions:

```
- Define `WateringConfig` struct:
  - moisture_threshold: u8 (default: 30%)
  - enabled: bool (default: true)
  - sensor_id: heapless::String<32> (which sensor to monitor)
  - pump_id: heapless::String<32> (which pump to control)

- Define `WateringState` struct:
  - last_watering_time: Option<Instant>
  - sensor_error_active: bool
  - auto_watering_disabled_reason: Option<DisabledReason>

- Define `DisabledReason` enum:
  - SensorError
  - ManualOverride
  - ConfigDisabled

- Implement decision function:
  - should_water(moisture: u8, config: &WateringConfig, state: &WateringState, pump_state: &PumpState) -> WateringDecision

- Define `WateringDecision` enum:
  - Water
  - Wait { reason: WaitReason }
  - Disabled { reason: DisabledReason }
```

**Acceptance Criteria**:
- [ ] Correctly identifies when watering is needed
- [ ] Respects minimum interval between waterings
- [ ] Disables on sensor errors
- [ ] Pure function, easily testable

---

#### Task 2.2: Create Watering Coordinator Task
**File**: `src/watering.rs` (add to module)

Create task that coordinates sensor readings with pump control:

```
- watering_coordinator_task:
  - Receives sensor readings from existing channel
  - Monitors moisture levels against threshold
  - Sends PumpCommand::Start when moisture < threshold
  - Handles sensor error alerts
  - Respects manual override state
  - Logs decisions via defmt
```

**Acceptance Criteria**:
- [ ] Triggers pump when moisture drops below threshold
- [ ] Respects 2-hour minimum interval
- [ ] Stops automatic watering on sensor errors
- [ ] Manual commands take priority

---

### Phase 3: MQTT Integration

#### Task 3.1: Add Pump MQTT Discovery
**File**: `src/mqtt/discovery.rs`

Add Home Assistant discovery for pump switch:

```
- Add pump discovery payload generation:
  - Component type: switch
  - Command topic: fevicol/{device_id}/pump/{pump_id}/command
  - State topic: fevicol/{device_id}/pump/{pump_id}/status
  - Availability topic: fevicol/{device_id}/status (shared)
  - Device class: switch
  - Icon: mdi:water-pump

- Add pump telemetry topics:
  - fevicol/{device_id}/pump/{pump_id}/last_run (JSON with timestamp, duration)
  - fevicol/{device_id}/pump/{pump_id}/stats (JSON with run_count, cumulative_time)
```

**Acceptance Criteria**:
- [ ] Pump appears as switch in Home Assistant
- [ ] On/Off control works from HA dashboard
- [ ] Telemetry visible in HA

---

#### Task 3.1b: Add Threshold Configuration MQTT Number Entity
**File**: `src/mqtt/discovery.rs`

Add Home Assistant discovery for moisture threshold as a `number` entity:

```
- Add threshold number discovery payload:
  - Component type: number
  - Discovery topic: homeassistant/number/{device_id}/threshold/config
  - Command topic: fevicol/{device_id}/config/threshold/set
  - State topic: fevicol/{device_id}/config/threshold/state
  - Min: 0, Max: 100, Step: 1
  - Unit: %
  - Icon: mdi:water-percent
  - Name: "Moisture Threshold"
  - Device class: moisture (if supported) or none

- Publish current threshold to state topic after applying changes
```

**How HA Persistence Works**:
1. Device publishes discovery → HA creates number entity with slider UI
2. User adjusts slider in HA → HA publishes new value to command topic
3. Device receives value, applies it, publishes to state topic
4. HA stores the value in its database
5. On device reboot: device subscribes → HA sends stored value → device applies it

**Acceptance Criteria**:
- [ ] Threshold appears as number slider in Home Assistant (0-100%)
- [ ] Adjusting slider updates device threshold
- [ ] Device state topic reflects current threshold
- [ ] Threshold persists across device reboots (via HA)

---

#### Task 3.2: Add MQTT Command Subscription
**File**: `src/mqtt/client.rs`

Add MQTT subscription handling for pump commands and configuration:

```
- Subscribe to command topics:
  - fevicol/{device_id}/pump/{pump_id}/command (payload: "ON" / "OFF")
  - fevicol/{device_id}/config/threshold/set (payload: number 0-100)

- Parse incoming commands and route to appropriate channels
- Handle subscription in mqtt_connection_task
- On connect: subscribe to threshold topic to receive HA's stored value
```

**Configuration Persistence via Home Assistant**:
- Device uses compile-time default threshold (30%) on boot
- Device subscribes to threshold command topic on MQTT connect
- Home Assistant sends its stored value when device subscribes
- Device applies received threshold, overriding default
- HA stores threshold persistently in its database (survives HA and device restarts)

**Acceptance Criteria**:
- [ ] MQTT "ON" command starts pump
- [ ] MQTT "OFF" command stops pump
- [ ] Threshold updates are applied immediately
- [ ] Device receives HA's stored threshold on reconnect
- [ ] Invalid commands are logged and ignored

---

#### Task 3.3: Add Pump Telemetry Publishing
**File**: `src/mqtt/client.rs`

Add pump state and telemetry publishing:

```
- Publish pump state changes (on/off) to status topic
- Publish telemetry on pump stop:
  - Last run timestamp
  - Last run duration
  - Total run count (since boot)
  - Cumulative run time (since boot)
- Publish sensor error alerts to alert topic
```

**Acceptance Criteria**:
- [ ] Pump state published on every change
- [ ] Telemetry published after each pump cycle
- [ ] Sensor error alerts published immediately

---

#### Task 3.4: Add Sensor Error Alert Topic
**File**: `src/mqtt/discovery.rs`

Add alert/diagnostic sensor for Home Assistant:

```
- Discovery for binary_sensor:
  - Topic: homeassistant/binary_sensor/{device_id}/sensor_error/config
  - State topic: fevicol/{device_id}/alerts/sensor_error
  - Device class: problem
  - Payload on: "error"
  - Payload off: "ok"
```

**Acceptance Criteria**:
- [ ] Sensor error shows as problem indicator in HA
- [ ] Clears when sensor readings resume normal

---

### Phase 4: Testing

#### Task 4.1: Unit Tests for Watering Logic
**File**: `tests/watering_test.rs`

Create tests for watering decision logic:

```
- Test: should_water returns Water when moisture < threshold
- Test: should_water returns Wait when min_interval not elapsed
- Test: should_water returns Disabled when sensor_error_active
- Test: should_water respects manual override
- Test: threshold boundary conditions (exactly at threshold)
```

**Acceptance Criteria**:
- [ ] All decision logic paths tested
- [ ] Edge cases covered
- [ ] Tests pass with `cargo test --no-run`

---

#### Task 4.2: Integration Test for Pump Task
**File**: `tests/pump_test.rs`

Create on-device test for pump control task:

```
- Test: pump starts and stops on command
- Test: pump auto-stops after max_run_time
- Test: pump respects min_interval
- Test: emergency stop works immediately
```

**Acceptance Criteria**:
- [ ] Tests run on device via `cargo test`
- [ ] All safety features verified

---

### Phase 5: Documentation

#### Task 5.1: Update CLAUDE.md
**File**: `CLAUDE.md`

Update documentation with pump control details:

```
- Add pump control to implemented features
- Document GPIO pin assignment
- Document MQTT topics for pump
- Document safety limits and configuration
- Update implementation status
```

**Acceptance Criteria**:
- [ ] All new features documented
- [ ] Configuration options explained
- [ ] MQTT topic structure updated

---

#### Task 5.2: Update README
**File**: `README.md`

Add user-facing documentation:

```
- Hardware setup instructions for relay
- Wiring diagram reference
- Safety warnings
- Home Assistant configuration examples
```

**Acceptance Criteria**:
- [ ] Clear setup instructions
- [ ] Safety information prominent

---

## File Structure

After implementation, the following files will be added/modified:

```
src/
├── bin/
│   └── main.rs          # Modified: add pump initialization
├── mqtt/
│   ├── client.rs        # Modified: add subscription handling, pump telemetry
│   └── discovery.rs     # Modified: add pump discovery payloads
├── pump.rs              # NEW: pump hardware abstraction and control task
├── watering.rs          # NEW: automatic watering logic and coordinator
├── sensor.rs            # Modified: add error detection/reporting
└── lib.rs               # Modified: add new modules

tests/
├── pump_test.rs         # NEW: pump control tests
└── watering_test.rs     # NEW: watering logic tests
```

---

## Configuration Constants

Add to `src/bin/main.rs` or dedicated config module:

```rust
// Pump Hardware
const PUMP_GPIO: u8 = 1;                    // GPIO1 (D0)
const PUMP_ACTIVE_LOW: bool = true;         // Relay trigger logic

// Safety Limits
const PUMP_MAX_RUN_TIME_SECS: u16 = 30;     // Maximum pump run time
const PUMP_MIN_INTERVAL_SECS: u32 = 7200;   // 2 hours between waterings

// Watering Logic
const MOISTURE_THRESHOLD_DEFAULT: u8 = 30;  // Default threshold (configurable via MQTT)
const CONFIG_SYNC_TIMEOUT_SECS: u8 = 5;     // Wait for HA config on boot before enabling auto-watering
const MANUAL_OVERRIDE_DURATION_SECS: u32 = 3600; // 1 hour override after manual stop

// Sensor Error Detection
const SENSOR_STUCK_THRESHOLD: u8 = 5;       // Consecutive identical readings to trigger stuck error
const SENSOR_STUCK_TOLERANCE: u16 = 10;     // ADC value tolerance for "same" reading

// Pump Identification
const PUMP_ID: &str = "pump-1";             // For multi-pump extensibility
```

---

## MQTT Topic Reference

### Command Topics (Subscribe)
| Topic | Payload | Description |
|-------|---------|-------------|
| `fevicol/{device_id}/pump/{pump_id}/command` | `ON` / `OFF` | Manual pump control |
| `fevicol/{device_id}/config/threshold/set` | `0-100` | Update moisture threshold |

### State Topics (Publish)
| Topic | Payload | Description |
|-------|---------|-------------|
| `fevicol/{device_id}/pump/{pump_id}/status` | `ON` / `OFF` | Current pump state |
| `fevicol/{device_id}/pump/{pump_id}/last_run` | JSON | Last run details |
| `fevicol/{device_id}/pump/{pump_id}/stats` | JSON | Session statistics |
| `fevicol/{device_id}/config/threshold/state` | `0-100` | Current threshold value |
| `fevicol/{device_id}/alerts/sensor_error` | `error` / `ok` | Sensor health |

### Discovery Topics (Publish, Retained)
| Topic | Description |
|-------|-------------|
| `homeassistant/switch/{device_id}/{pump_id}/config` | Pump switch discovery |
| `homeassistant/number/{device_id}/threshold/config` | Threshold number entity discovery |
| `homeassistant/binary_sensor/{device_id}/sensor_error/config` | Sensor error discovery |

---

## Implementation Order

Recommended implementation sequence:

1. **Task 1.1** - Pump hardware abstraction (foundation)
2. **Task 1.2** - Pump control task (safety enforcement)
3. **Task 1.3** - Main binary integration (get pump working)
4. **Task 4.2** - Integration test (verify hardware control)
5. **Task 2.1** - Watering decision logic (pure functions)
6. **Task 4.1** - Unit tests for watering logic
7. **Task 2.2** - Watering coordinator task
8. **Task 3.1** - MQTT discovery for pump switch
9. **Task 3.1b** - MQTT Number entity for threshold (HA-persisted config)
10. **Task 3.3** - Pump telemetry publishing
11. **Task 3.2** - MQTT command subscription (pump + threshold)
12. **Task 3.4** - Sensor error alerts
13. **Task 5.1** - Update CLAUDE.md
14. **Task 5.2** - Update README

---

## Future Extensibility

The design supports future enhancements:

- **Multi-pump**: Add additional `PumpController` instances with unique IDs (up to 4)
- **Per-pump thresholds**: Each pump gets its own threshold via `homeassistant/number/{device_id}/{pump_id}/threshold/config`
- **Sensor-pump mapping**: Store mapping in `WateringConfig`, configurable via MQTT
- **Scheduling**: Add time-based watering schedules (e.g., "only water between 6am-8pm")
- **Flow sensing**: Add flow meter to detect "pump ON but no flow" failures
- **Current sensing**: Monitor relay/pump current draw to verify operation
- **NTP time sync**: Real timestamps instead of uptime-based for better HA integration
- **NVS persistence**: Store configuration and statistics in non-volatile storage (alternative to HA persistence)

---

## Edge Cases & Safety Considerations

### Critical: Must Address in Implementation

#### 1. Config Sync Timeout on Boot
**Problem**: Device boots and subscribes to threshold topic, but HA might not send the stored value immediately. Device could start automatic watering with default 30% before HA's stored value (e.g., 50%) arrives.

**Solution**: 
- Add `CONFIG_SYNC_TIMEOUT_SECS: u8 = 5` constant
- On boot, wait up to 5 seconds for HA config before enabling automatic watering
- If timeout expires, use compile-time default and log warning
- Add to Task 2.2 (Watering Coordinator Task)

#### 2. Manual Override Behavior
**Problem**: If user manually stops pump via MQTT, should automatic watering be disabled? For how long?

**Solution**:
- Manual STOP command disables automatic watering for `MANUAL_OVERRIDE_DURATION_SECS: u32 = 3600` (1 hour)
- Manual START command runs pump for max_run_time, then resumes normal automatic behavior
- Add `manual_override_until: Option<Instant>` to `WateringState`
- Add to Task 2.1 (Watering Decision Logic)

#### 3. Safe GPIO State on Boot
**Problem**: If device crashes while pump was running, relay might still be energized depending on relay type.

**Solution**:
- In `PumpController::new()`, immediately set GPIO to OFF state before any other initialization
- This must happen before scheduler starts, in the synchronous init phase
- Add explicit requirement to Task 1.1 and Task 1.3

#### 4. Sensor Error Definition
**Problem**: Plan says disable auto-watering on sensor errors, but doesn't define what constitutes an error.

**Solution**: Define sensor error conditions in `src/sensor.rs`:
- `SensorError::ReadFailure` - ADC read returns error
- `SensorError::OutOfRange` - Value outside calibration range (< SENSOR_DRY - 200 or > SENSOR_WET + 100)
- `SensorError::StuckValue` - Same value (±10) for 5 consecutive readings (possible disconnection)

Add `SensorReading` variant or separate error channel:
```rust
enum SensorResult {
    Reading(SensorReading),
    Error(SensorError),
}
```
- Add to Task 2.1 and modify sensor task in `src/sensor.rs`

### Important: Should Address

#### 5. Pump Feedback / Verification
**Problem**: No way to verify pump actually ran (relay clicked but pump failed, tube disconnected).

**Solution for Initial Scope**:
- Log "pump commanded ON" and "pump cycle completed" separately
- Publish both events to MQTT for HA tracking
- Add to Future Extensibility: flow sensor or current sensing

**Future Enhancement** (document only):
- Add flow sensor on pump output
- Detect "pump ON but no flow" condition
- Alert via MQTT if pump runs but no water detected

#### 6. Multi-Pump Threshold Architecture
**Problem**: Plan mentions up to 4 pumps, but only one threshold entity. Each sensor-pump pair might need different thresholds.

**Solution**:
- Initial scope: Single threshold applies to all (simple)
- Document in Future Extensibility: Per-pump thresholds via `homeassistant/number/{device_id}/{pump_id}/threshold/config`
- Data structure should use `HashMap<PumpId, ThresholdConfig>` or similar for future expansion

#### 7. MQTT QoS for Commands
**Problem**: Pump commands and threshold updates should be reliably delivered.

**Solution**:
- Subscribe to pump command topic with QoS 1
- Subscribe to threshold topic with QoS 1
- Publish pump state changes with QoS 1
- Telemetry (stats, last_run) can use QoS 0
- Add QoS specifications to Task 3.2 and Task 3.3

#### 8. Timestamp Handling
**Problem**: ESP32-C6 has no RTC - timestamps are "ms since boot". HA might expect ISO timestamps.

**Solution**:
- Use milliseconds-since-boot for all device timestamps
- Document that HA should use message receive time for history/graphs
- Telemetry JSON includes `"uptime_ms": 12345` field
- Future enhancement: NTP time sync (add to Future Extensibility)

### Minor: Good to Address

#### 9. Discovery Message Buffer Size
**Problem**: Adding pump switch + number entity + binary sensor increases discovery payload size.

**Solution**:
- Verify 1024-byte buffer is sufficient for each discovery message
- If needed, increase to 1536 or 2048 bytes
- Add verification step to Task 3.1 acceptance criteria

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Pump runs indefinitely | Hard 30-second timeout in pump task |
| Overwatering | 2-hour minimum interval enforced |
| Sensor failure causes flooding | Auto-disable + MQTT alert on sensor errors (with defined error conditions) |
| GPIO misconfiguration | Configurable active_low with safe default |
| MQTT command injection | Validate all incoming payloads |
| Power loss during watering | Pump stops (relay de-energizes), state resets on boot |
| Device restart during pump cycle | GPIO forced OFF immediately on boot |
| Config race condition on boot | 5-second sync timeout before enabling auto-watering |
| Manual override confusion | Clear 1-hour override duration after manual stop |
| Unreliable command delivery | QoS 1 for pump commands and config updates |
