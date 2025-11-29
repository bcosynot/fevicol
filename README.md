 # Fevicol — Smart Plant Watering (ESP32‑C6, Rust, no_std)
 
 Fevicol is embedded firmware for the Seeed Studio Xiao ESP32‑C6 that:
 - Reads a resistive soil‑moisture sensor via ADC1 (GPIO0/A0)
 - Converts raw readings to a calibrated 0–100% moisture value
 - Connects to Wi‑Fi (STA) and will publish telemetry via MQTT (planned)
 - Will control a small water pump via a GPIO output with safety limits (planned)
 
 Status as of 2025‑11‑29:
 - Implemented: moisture sensing + calibration, threshold monitoring, Wi‑Fi STA with auto‑reconnect, RTT logging
 - Pending: TCP/IP stack integration (embassy‑net), MQTT to Home Assistant, pump control GPIO, safety limits
 - Known issue: release builds currently fail with esp‑radio NVS linker errors — use debug builds
 
 
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
 - MQTT integration to Home Assistant (embassy‑net TCP/IP + MQTT client)
 - Pump control GPIO + safety limits (max run time, minimum interval between activations)
 - Home Assistant MQTT discovery for auto‑entity provisioning
 - Resolve release‑mode linker errors with esp‑radio NVS
 - Add CI (fmt, clippy, `cargo test --no-run`)
 - Add board/pin diagram and wiring guide
 
 
 ## License
 - TODO: Add a LICENSE file and set the `license` or `license-file` field in Cargo.toml.
 
 
 ## Acknowledgements
 - Built with esp‑hal, esp‑rtos, esp‑radio, Embassy, defmt, rtt‑target
 - Probe‑rs is used for flashing and RTT logging
