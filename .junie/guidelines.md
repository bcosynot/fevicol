Fevicol — Advanced Dev Guidelines

This document distills project‑specific build, test, and development practices discovered from the repo (CLAUDE.md, Cargo.toml, .cargo/config.toml, build.rs, sources). It is written for experienced embedded Rust developers targeting Xiao ESP32‑C6.

Build and Configuration

- Toolchain and target
  - rust-toolchain.toml pins channel=stable and installs `rust-src` with target `riscv32imac-unknown-none-elf`. Install via `rustup target add riscv32imac-unknown-none-elf` if the toolchain file is not honored by your environment.
  - The project relies on `build-std` for `core` and `alloc` (see `.cargo/config.toml [unstable].build-std = ["alloc","core"]`).

- Runner and flashing (probe-rs)
  - Cargo runner is set to `probe-rs run --chip=esp32c6 --preverify --always-print-stacktrace --no-location --catch-hardfault` under `[target.riscv32imac-unknown-none-elf]`.
  - This means `cargo run --release` will: build, flash, and attach RTT automatically. Logs use defmt over RTT.
  - To select a specific probe, export `PROBE_RS_PROBE` (see probe-rs docs) or add `--probe …` to the runner if you temporarily override it.

- Rust flags and env
  - `.cargo/config.toml` injects:
    - `-C force-frame-pointers` (for backtraces)
    - `-Z stack-protector=all` (stack canaries)
  - Set logging with `DEFMT_LOG` env (default "info"). Accepts `trace|debug|info|warn|error`.

- Linker scripts (via build.rs)
  - `defmt.x` is always linked for defmt symbol resolution.
  - `linkall.x` is forced last to avoid flip-link/ordering issues.
  - For tests, `embedded-test.x` is injected via `cargo:rustc-link-arg-tests`.
  - The custom error-handling script prints helpful hints for missing symbols such as `_defmt_timestamp`, `_stack_start`, embedded-test, or missing scheduler symbols for `esp-radio`.

- Profiles
  - `dev`: `opt-level = "s"` (debug builds are optimized enough to be usable on target).
  - `release`: LTO fat, codegen-units=1, `debug=2` (DWARF) for better debug on HW.

Build and Run

- Build only
  - `cargo build` (debug) or `cargo build --release`.

- Flash + monitor (RTT)
  - `cargo run --release`
  - Ensure `rtt_target::rtt_init_defmt!()` is called early (it is in `src/bin/main.rs`).

- Wokwi simulation (optional)
  - `wokwi.toml` configures a GDB server on port 3333 and points to the debug ELF: `target/riscv32imac-unknown-none-elf/debug/fevicol`.
  - Typical flow: build with `cargo build`, then connect your debugger to localhost:3333 per Wokwi docs.

Code Quality and Formatting

- **Formatting** (mandatory before commits)
  - Run `cargo fmt --all` to format all code according to project style
  - Verify with `cargo fmt --all -- --check` before committing
  - **Important**: A temporary `src/bin/secrets.rs` file must exist (even if empty) for `cargo fmt` to work, as it needs to resolve the conditional `mod secrets` declaration

- **Linting** (recommended)
  - Run `cargo clippy --all-features --workspace -- -D warnings` to catch common issues
  - Address all clippy warnings before committing

Architecture and Runtime Notes

- no_std + Embassy
  - Entry point is `#[esp_rtos::main] async fn main(spawner: Spawner) -> !` using `esp-rtos` to host Embassy tasks.
  - Scheduler is started explicitly with `esp_rtos::start(timg0.timer0, swint0)` using TIMG0 and a SW interrupt.

- Memory/alloc
  - Two heaps are configured with `esp_alloc::heap_allocator!`: 64 KiB reclaimed RAM and an extra 64 KiB standard RAM to ease Wi‑Fi/BLE COEX pressure.
  - Dynamic allocation must use `alloc` types; there is no `std`.

- Logging and panics
  - defmt + RTT via `rtt-target` and `panic-rtt-target`. Initialize RTT before any logs in both app and tests.

- Radio/COEX
  - Wi‑Fi enabled via `esp_radio::init()` and spawned `network_task` for STA mode connectivity
  - When enabling both Wi‑Fi and BLE, initialize both from the shared radio handle for coexistence
  - Currently only Wi‑Fi is active; BLE stack not needed for MQTT-based plant monitoring

- Lints
  - `#![deny(clippy::mem_forget)]` in `main.rs` — do not use `mem::forget` with `esp-hal` types that may own DMA/transfer buffers.

Testing

- Framework
  - Uses `embedded-test` with Embassy executor and an external scheduler (`esp-rtos`).
  - `Cargo.toml` contains `[[test]] harness=false` for a suite named `hello_test` and `dev-dependencies.embedded-test` with `defmt`, `embassy`, `external-executor` features.
  - `build.rs` injects `embedded-test.x` so you do not need to pass extra linker args.

- Layout and boilerplate
  - Place tests under `tests/` (integration‑style). See `tests/hello_test.rs` for a working example.
  - Required attributes/macros in each test crate:
    - `#![no_std]`, `#![no_main]`.
    - `esp_bootloader_esp_idf::esp_app_desc!();` to embed an ESP‑IDF app descriptor for the ESP bootloader.
    - `#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())] mod tests { … }` to register tests.
    - An `#[init] fn init()` that:
      - Calls `esp_hal::init(esp_hal::Config::default())` to set up clocks/peripherals.
      - Starts the Embassy scheduler with TIMG1 (so it doesn't clash with app TIMG0):
        `esp_rtos::start(timg1.timer0, swint0)`.
      - Initializes RTT via `rtt_target::rtt_init_defmt!()`.

- Running tests
  - On device (recommended): `cargo test`
    - Because the cargo runner is probe‑rs, each test binary is flashed to the ESP32‑C6 and executed, with defmt output streamed over RTT.
    - If you have multiple probes connected, specify one via environment or by temporarily overriding the runner to include `--probe`.
  - Build‑only sanity check (no hardware required): `cargo test --no-run`
    - Verified on 2025‑11‑29: the suite compiles successfully and produces `target/riscv32imac-unknown-none-elf/debug/deps/hello_test-*.elf`.
  - Common pitfalls
    - If you see errors about `embedded_test_linker_file_not_added_to_rustflags`, ensure you are building this project (which wires it in via `build.rs`).
    - Missing defmt symbols or stack start usually indicate linker script ordering; this repo’s `build.rs` already forces correct order.

- MQTT Client Implementation (rust-mqtt with MQTT v5)

- Recommended implementation: rust-mqtt
  - The project uses `rust-mqtt` v0.3 as the primary MQTT client with full MQTT v5 support
  - Features: session expiry, authentication, enhanced error reporting, LWT, retain flags, QoS 0/1
  - Integration with `embassy-net` TCP stack for high-level network operations

- Abstraction and publish semantics
  - Crate‑agnostic MQTT publish interface in `src/mqtt/client.rs`: 
    - `enum MqQos { AtMostOnce, AtLeastOnce }`
    - `trait MqttPublish { async fn publish(&mut self, topic: &str, payload: &[u8], qos: MqQos, retain: bool) -> Result<(), Self::Err>; }`
  - `publish_discovery(client, device_id, sensor_id) -> Result<(), _>` publishes retained availability and Home Assistant discovery configs with QoS 1 and ~100 ms pacing between publishes.

- Feature flags (Cargo.toml)
  - `mqtt` — MQTT v5 support via rust-mqtt v0.3 and embassy-net
  - With no `mqtt` feature enabled (default), the build uses log‑only mode with `LoggerPublisher`

- Building with MQTT support
  - Enable rust-mqtt client: `cargo build --features mqtt` or `cargo run --release --features mqtt`
  - Default build (log-only mode): `cargo build` or `cargo run --release`
  - Log-only mode validates discovery flow on RTT without requiring a broker

- MQTT v5 configuration (when mqtt enabled)
  - Protocol: MQTT v5
  - Client ID: `fevicol-{DEVICE_ID}`
  - Keep-alive: 60 seconds
  - Session expiry: 3600 seconds (1 hour)
  - Authentication: Username/password support
  - LWT Topic: `fevicol/{device_id}/status` with payload `offline` (retained)
  - Availability: `online` published to status topic after successful connection (retained)
  - Discovery: Home Assistant MQTT Discovery messages published on connect/reconnect

- Network stack (when mqtt enabled)
  - `embassy-net` provides high-level network operations wrapping smoltcp
  - DHCP for automatic IP assignment
  - DNS resolution for broker hostname with fallback to IP address
  - TCP socket management via `embassy_net::tcp::TcpSocket`
  - Integration with esp-radio's Wi-Fi device driver

- Transport adapter
  - `EmbassyNetTransport` wraps `embassy_net::tcp::TcpSocket` for rust-mqtt client
  - Implements `embedded_io_async::Read` and `embedded_io_async::Write` traits
  - Handles TCP connection establishment and error mapping

- Adding a new async test (template)
  - Minimal skeleton (mirrors `tests/hello_test.rs`):

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

    - Place this under `tests/` and run `cargo test` (device) or `cargo test --no-run` (compile only).

Additional Development Notes

- Main binary structure
  - `src/bin/main.rs` sets CPU clock to max, initializes HAL, configures both heap regions, starts the scheduler
  - ADC1 configured on GPIO0 (A0) for resistive moisture sensor with 6dB attenuation
  - Application runs in separate Embassy tasks:
    - `network_task`: Wi‑Fi STA mode with auto‑reconnect; coordinates with embassy-net stack when mqtt enabled
    - `embassy_net_task`: Runs embassy-net stack runner for packet processing (when mqtt enabled)
    - `moisture_sensor_task`: Reads ADC every 5s, converts to percentage (0-100%), monitors threshold (30%), sends readings to channel
    - `mqtt_connection_task`: Manages MQTT broker connection lifecycle with rust-mqtt v5 client, DNS resolution, TCP connection, exponential backoff reconnection, publishes discovery and availability messages, receives sensor readings from channel and publishes telemetry (when mqtt enabled; log-only mode otherwise)
  - Inter-task communication via `embassy-sync::channel::Channel<NoopRawMutex, SensorReading, 20>` with 20-reading buffer
  - `SensorReading` struct carries moisture percentage, raw ADC value, and timestamp between tasks

- Concurrency model
  - Use `embassy_executor::Spawner` to start tasks. Ensure all hardware shared across tasks uses appropriate synchronization (e.g., `critical-section`, `static_cell`).
  - **Task Architecture**: Use separate Embassy tasks for independent concerns (sensors, network, MQTT) to achieve fault isolation
  - **Inter-task Communication**: Use `embassy-sync::channel::Channel` for passing data between tasks (e.g., sensor readings from sensor task to MQTT publish task)
  - **Design Principle**: Keep sensor reading independent from network operations for resilience - sensor continues operating even during network outages

- ADC and networking crates
  - ADC1 configured for resistive soil moisture sensor on GPIO0
  - `smoltcp` for TCP/IP stack via esp-radio's integrated Wi-Fi device
  - `embassy-net` v0.7.1 for high-level network stack (TCP, DHCP, DNS) when mqtt feature is enabled
  - `rust-mqtt` v0.3 for MQTT v5 client (fully implemented with embassy-net integration)
  - Wi‑Fi operational in STA mode via esp‑radio with automatic reconnection

- Sensor calibration
  - Calibration constants in `main.rs`: `SENSOR_DRY = 2188`, `SENSOR_WET = 4095`
  - Calibration routine preserved as commented code at end of `main.rs` (lines 276-375)
  - To recalibrate: swap in calibration loop, flash, collect 10 dry + 10 wet readings, update constants
  - Conversion function `raw_to_moisture_percent()` performs linear interpolation to 0-100%

- Code style
  - Follow existing import ordering and module layout as shown in `main.rs`/`tests/hello_test.rs`.
  - Keep comments minimal and practical, mirroring the repository’s style. Prefer `defmt::info!` for high‑level progress and `debug!` for verbose internals.

Implementation Status (as of 2025‑12‑07)

**Completed**:
- ✅ Moisture sensing: ADC1 reading on GPIO0 with resistive sensor
- ✅ Sensor calibration: Automated dry/wet calibration routine with 10-sample averaging
- ✅ Percentage conversion: Linear interpolation from raw ADC (2188-4095) to 0-100%
- ✅ Wi‑Fi connectivity: STA mode with automatic reconnection
- ✅ Threshold monitoring: Configurable moisture threshold (30%) with warning logs
- ✅ Test framework: `cargo test --no-run` validates embedded-test harness
- ✅ **Task Architecture**: Refactored to separate Embassy tasks for fault isolation
  - Sensor reading in dedicated `moisture_sensor_task` (independent of network status)
  - MQTT connection management in `mqtt_connection_task` with full rust-mqtt integration
  - MQTT publishing in `mqtt_publish_task` (waits for MQTT readiness, receives sensor readings from channel)
  - Inter-task communication via `embassy-sync::channel` with 20-reading buffer
- ✅ **MQTT v5 Integration**: Full implementation with rust-mqtt v0.3
  - embassy-net stack with DHCP and DNS resolution
  - EmbassyNetTransport adapter implementing embedded_io_async traits
  - MQTT v5 protocol with session expiry (3600s) and authentication
  - Last Will Testament (LWT) for availability tracking
  - Exponential backoff reconnection (2s → 30s max)
  - Connection health monitoring and automatic reconnection
  - Home Assistant MQTT Discovery protocol implementation
  - Telemetry publishing (moisture %, raw ADC) with QoS 0
  - Feature-gated with `mqtt` flag

**Pending**:
- ⏳ Pump control GPIO implementation
- ⏳ Safety limits (max run time, minimum interval between waterings)
- ⏳ Automatic watering logic based on moisture threshold
