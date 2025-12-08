 # Fevicol — Smart Plant Watering (ESP32‑C6, Rust, no_std)
 
 Fevicol is embedded firmware for the Seeed Studio Xiao ESP32‑C6 that:
 - Reads a resistive soil‑moisture sensor via ADC1 (GPIO0/A0)
 - Converts raw readings to a calibrated 0–100% moisture value
 - Connects to Wi‑Fi (STA) and publishes telemetry to Home Assistant via MQTT v5
 - Will control a small water pump via a GPIO output with safety limits (planned)
 
 Status as of 2025‑12‑07:
 - **Implemented**: moisture sensing + calibration, threshold monitoring, Wi‑Fi STA with auto‑reconnect, **MQTT v5 integration with rust-mqtt v0.3** (TCP connection via embassy-net, DNS resolution, authentication, LWT, Home Assistant MQTT Discovery, telemetry publishing every 5 seconds, exponential backoff reconnection), RTT logging
 - **Pending**: pump control GPIO, safety limits, automatic watering logic
 
 
 ## Stack and Tooling
 - Language: Rust (no_std)
 - Target: riscv32imac‑unknown‑none‑elf (ESP32‑C6)
 - HAL/Runtime: esp‑hal v1, Embassy async executor hosted by esp‑rtos
 - Radio: esp‑radio (Wi‑Fi; BLE coexistence capable but not used yet)
 - Logging: defmt over RTT (rtt‑target), panic‑rtt‑target
 - Build: Cargo with build‑std for `core` and `alloc`
 - Flash/Run: probe‑rs configured as Cargo runner
 - Tests: embedded‑test with Embassy external executor (on‑device via probe‑rs)
 
 Key files:
 - src/bin/main.rs — application entrypoint (`#[esp_rtos::main] async fn main(...) -> !`)
 - src/lib.rs — library crate root (`#![no_std]`)
 - build.rs — links `defmt.x`, forces `linkall.x`, and provides helpful linker diagnostics
 - .cargo/config.toml — runner, target, rustflags, build‑std settings
 - rust-toolchain.toml — pins stable + installs `rust-src` and the target
 
 
 ## Requirements
 - Rust toolchain (stable) with components/target:
   - `rustup component add rust-src`
   - `rustup target add riscv32imac-unknown-none-elf`
 - probe‑rs tools for flashing/RTT (runner is preconfigured):
   - `cargo install probe-rs-tools --locked`
 - Hardware: Seeed Studio Xiao ESP32‑C6, resistive soil moisture sensor on A0 (GPIO0), programmer/debug probe compatible with probe‑rs
 
 Optional:
 - Wokwi simulator (project may include `wokwi.toml`) — manual GDB attach on :3333 to the debug ELF
 
 
 ## Setup
 1) Clone and enter the repo.
 2) Ensure toolchain/target are installed (see Requirements).
 3) Connect your ESP32‑C6 via a probe supported by probe‑rs.
 4) Optionally set Wi‑Fi credentials at build time (see Env Vars).
 
 
 ## Build, Flash, and Run
 The repo’s Cargo runner is probe‑rs, so standard Cargo commands will flash and stream RTT automatically.
 
 - Build only (debug):
   - `cargo build`
 - Build only (release):
   - `cargo build --release`  ← currently not recommended (see Known Issues)
 - Flash + monitor (debug build):
   - `cargo run`
 
 Runner details (from .cargo/config.toml):
 - `probe-rs run --chip=esp32c6 --preverify --always-print-stacktrace --no-location --catch-hardfault`
 - RTT is initialized early in `main` via `rtt_target::rtt_init_defmt!()`
 
 Selecting a specific probe:
 - Export `PROBE_RS_PROBE` env var, or temporarily override the runner with `--probe …` (see probe‑rs docs).
 
 
 ## Scripts and Useful Commands
 These are standard Cargo workflows; no custom shell scripts are defined.
 - Format check: `cargo fmt --all -- --check`
 - Lint: `cargo clippy --all-features -- -D warnings`
 - Size (example): `cargo bloat -n 20 --target riscv32imac-unknown-none-elf`  (if you install cargo‑bloat)
 
 
 ## Environment Variables and Features
 - DEFMT_LOG — defmt log level (default: "info"). Accepts: `trace|debug|info|warn|error`.
 - WIFI_SSID / WIFI_PASS — compile‑time credentials used when the `local_secrets` feature is NOT enabled.
 - Feature `local_secrets` — allows storing credentials in a git‑ignored `src/bin/secrets.rs`:
   ```bash
   # Copy the template and edit with your credentials
   cp src/bin/secrets.rs.example src/bin/secrets.rs
   # Then edit src/bin/secrets.rs with your actual SSID/password
   ```
   ```rust
   // src/bin/secrets.rs (git‑ignored)
   pub const WIFI_SSID: &str = "YourSSID";
   pub const WIFI_PASS: &str = "YourPassword";
   ```
   Build with: `cargo run --features local_secrets`
 - PROBE_RS_PROBE — optional, to select a specific debug probe

 ### MQTT v5 Integration (Fully Implemented)

 **Feature Flag**:
 - `mqtt` — Enables rust-mqtt v0.3 MQTT v5 client with embassy-net
 - Default build (no feature): Uses log-only mode for testing without a broker

 **Building with MQTT**:
 ```bash
 # With MQTT v5 client (recommended for production)
 cargo run --release --features mqtt

 # With MQTT + local secrets
 cargo run --release --features mqtt,local_secrets

 # Without MQTT (log-only mode for testing)
 cargo run --release
 ```

 **MQTT Broker Configuration**:

 Edit the constants in `src/bin/main.rs` to match your network:
 ```rust
 const MQTT_BROKER_HOST: &str = "192.168.1.100";  // Your broker IP or hostname
 const MQTT_BROKER_PORT: u16 = 1883;              // Standard MQTT port
 const MQTT_USERNAME: &str = "";                   // Leave empty for no auth
 const MQTT_PASSWORD: &str = "";                   // Leave empty for no auth
 const DEVICE_ID: &str = "fevicol-01";            // Unique device identifier
 const SENSOR_ID: &str = "moisture-1";            // Sensor identifier
 ```

 **MQTT v5 Features**:
 - Protocol: MQTT v5 with session expiry (3600s)
 - Authentication: Username/password support
 - Last Will Testament (LWT): Publishes `offline` to `fevicol/{device_id}/status` on disconnect
 - Availability: Publishes `online` to status topic after connection (retained)
 - Home Assistant MQTT Discovery: Automatic sensor entity creation
 - Telemetry: Publishes moisture % and raw ADC value every 5 seconds
 - Resilience: Exponential backoff reconnection (2s → 30s max), DNS resolution with fallback to IP
 - Network Stack: embassy-net with DHCP and DNS via esp-radio Wi-Fi device

 **Home Assistant Integration**:

 The firmware uses MQTT Discovery protocol for automatic sensor entity creation in Home Assistant.

 1. **Prerequisites**:
    - Home Assistant with MQTT integration enabled
    - MQTT broker (Mosquitto, Home Assistant built-in broker, etc.)
    - Broker accessible from ESP32-C6 network

 2. **Setup Steps**:
    - Configure MQTT broker in Home Assistant (Settings → Devices & Services → MQTT)
    - Update `MQTT_BROKER_HOST` in `src/bin/main.rs` to your broker IP/hostname
    - Set `MQTT_USERNAME` and `MQTT_PASSWORD` if your broker requires authentication
    - Build and flash firmware with `mqtt` feature
    - Device will automatically appear in Home Assistant after connection

 3. **Discovered Entities**:
    - **Moisture Sensor** (`sensor.fevicol_01_moisture`):
      - Device class: `moisture`
      - Unit: `%` (percentage)
      - Icon: `mdi:water-percent`
      - State topic: `fevicol/{device_id}/{sensor_id}/moisture`
    - **Raw ADC Sensor** (`sensor.fevicol_01_raw`):
      - Unit: `ADC`
      - Icon: `mdi:chip`
      - State topic: `fevicol/{device_id}/{sensor_id}/raw`
      - Purpose: Debugging and calibration verification

 4. **Device Information**:
    - Device ID: `fevicol-01` (configurable via `DEVICE_ID` constant)
    - Device Name: "Fevicol Plant Monitor - fevicol-01"
    - Manufacturer: "Fevicol Project"
    - Model: "ESP32-C6 Moisture Sensor"
    - Software Version: From `Cargo.toml` version

 5. **Availability Tracking**:
    - Availability topic: `fevicol/{device_id}/status`
    - Online: Published as `online` (retained) after connection
    - Offline: Published as `offline` (retained) via LWT on unexpected disconnect
    - Shows device connectivity status in Home Assistant

 6. **MQTT Topics**:
    ```
    # Discovery (published on connect/reconnect with retain=true)
    homeassistant/sensor/fevicol-01/moisture/config
    homeassistant/sensor/fevicol-01/raw/config
   
    # State (published every 5 seconds)
    fevicol/fevicol-01/moisture-1/moisture    # Moisture percentage (0-100)
    fevicol/fevicol-01/moisture-1/raw         # Raw ADC value (0-4095)
   
    # Availability (retained)
    fevicol/fevicol-01/status                 # online/offline
    ```

 7. **Troubleshooting**:
    - Check RTT logs for connection status: `DEFMT_LOG=debug cargo run --features mqtt`
    - Verify broker is reachable: ping broker IP from ESP32-C6 network
    - Check MQTT broker logs for connection attempts
    - Verify Home Assistant MQTT integration is enabled
    - Ensure discovery prefix is `homeassistant` (default in HA)


 ## Tests
 This project uses `embedded-test` with the Embassy executor and an external scheduler hosted by `esp-rtos`.
 
 - On device (recommended): `cargo test`
   - Each test binary is flashed via probe‑rs and prints defmt over RTT
 - Build only (no hardware): `cargo test --no-run`
   - Verified on 2025‑11‑29: the suite compiles and produces `target/riscv32imac-unknown-none-elf/debug/deps/hello_test-*.elf`
 
 Test layout:
 - Tests live under `tests/` (integration‑style). Example file name: `tests/hello_test.rs`.
 - If missing, create one using the template from `.junie/guidelines.md` (embedded here for convenience):
   ```rust
   #![no_std]
   #![no_main]
   esp_bootloader_esp_idf::esp_app_desc!();
 
   #[cfg(test)]
   #[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
   mod tests {
       use defmt::assert_eq;
 
       #[init]
       fn init() {
           let p = esp_hal::init(esp_hal::Config::default());
           let timg1 = esp_hal::timer::timg::TimerGroup::new(p.TIMG1);
           let sw = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
           esp_rtos::start(timg1.timer0, sw.software_interrupt0);
           rtt_target::rtt_init_defmt!();
       }
 
       #[test]
       async fn my_test() {
           defmt::info!("test running");
           embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
           assert_eq!(2 + 2, 4);
       }
   }
   ```
 
 
 ## Project Structure
 Abridged:
 - Cargo.toml — crate metadata and dependencies
 - build.rs — linker script setup and diagnostic messages
 - .cargo/config.toml — target, runner (probe‑rs), DEFMT_LOG, rustflags, build‑std
 - rust-toolchain.toml — stable channel + components/targets
 - src/
   - bin/
     - main.rs — entrypoint; configures clocks, allocators, starts scheduler, Wi‑Fi bring‑up, ADC loop
     - secrets.rs — optional (git‑ignored) when using the `local_secrets` feature
   - lib.rs — `#![no_std]`
 - tests/ — embedded‑test suites (e.g., `hello_test.rs`) [may need to be created]
 
 Hardware notes:
 - ADC1 on GPIO0 (A0) with 6dB attenuation for the resistive moisture sensor
 - Calibration constants (in main.rs):
   - SENSOR_DRY = 2188 (≈0%)
   - SENSOR_WET = 4095 (≈100%)
 - Threshold: `MOISTURE_THRESHOLD = 30` (%)
 
 
 ## Troubleshooting
 - Missing defmt symbols or `_stack_start`: ensure `build.rs` is linking `defmt.x` and `linkall.x` last (this repo does)
 - `embedded_test_linker_file_not_added_to_rustflags`: you’re likely not building this repo (our `build.rs` wires it up for tests)
 - No RTT output: confirm `rtt_target::rtt_init_defmt!()` is called before any logging (it is), and that DEFMT_LOG isn’t too restrictive
 - Multiple probes attached: set `PROBE_RS_PROBE` to pick the right one
 - Release build fails with esp‑radio NVS linker errors: use `cargo run` / `cargo build` (debug). Track upstream esp‑radio updates.
 
 
 ## Roadmap / TODOs
 - ✅ ~~MQTT v5 integration to Home Assistant (embassy‑net TCP/IP + rust-mqtt client)~~ — **Completed**
 - ✅ ~~Home Assistant MQTT discovery for auto‑entity provisioning~~ — **Completed**
 - Pump control GPIO + safety limits (max run time, minimum interval between activations)
 - Automatic watering logic based on moisture threshold
 - Resolve release‑mode linker errors with esp‑radio NVS
 - Add CI (fmt, clippy, `cargo test --no-run`)
 - Add board/pin diagram and wiring guide
 - MQTT command topics for manual pump control and threshold adjustment
 
 
 ## License
 - TODO: Add a LICENSE file and set the `license` or `license-file` field in Cargo.toml.
 
 
 ## Acknowledgements
 - Built with esp‑hal, esp‑rtos, esp‑radio, Embassy, defmt, rtt‑target
 - Probe‑rs is used for flashing and RTT logging
