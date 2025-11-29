#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{error, info, warn};
use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use smoltcp::wire::Ipv4Address;

// Optional local secrets support
#[cfg(feature = "local_secrets")]
mod secrets;
#[cfg(feature = "local_secrets")]
use secrets::{WIFI_PASS as LOCAL_PASS, WIFI_SSID as LOCAL_SSID};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Signal to notify when network is ready
static NETWORK_READY: Signal<CriticalSectionRawMutex, Ipv4Address> = Signal::new();

// Calibration constants from sensor calibration routine
const SENSOR_DRY: u16 = 2188;  // ADC value when sensor is in air (0% moisture)
const SENSOR_WET: u16 = 4095;  // ADC value when sensor is in water (100% moisture)

// Watering threshold: trigger pump when moisture falls below this percentage
const MOISTURE_THRESHOLD: u8 = 30;  // 30% moisture

/// Convert raw ADC reading to moisture percentage (0-100%)
fn raw_to_moisture_percent(raw: u16) -> u8 {
    if raw <= SENSOR_DRY {
        return 0; // Drier than calibrated dry point
    }
    if raw >= SENSOR_WET {
        return 100; // Wetter than calibrated wet point
    }

    // Linear interpolation between dry (0%) and wet (100%)
    let range = SENSOR_WET - SENSOR_DRY;
    let offset = raw - SENSOR_DRY;
    let percent = (offset as u32 * 100) / range as u32;
    percent.min(100) as u8
}

/// Network task: manages Wi-Fi connection
#[embassy_executor::task]
async fn network_task(
    mut wifi: esp_radio::wifi::WifiController<'static>,
    _ifaces: esp_radio::wifi::Interfaces<'static>,
    client_config: esp_radio::wifi::ClientConfig,
) {
    // Set Wi-Fi configuration and start
    if let Err(e) = wifi.set_config(&esp_radio::wifi::ModeConfig::Client(client_config)) {
        error!("wifi set_config failed: {:?}", e);
        return;
    }

    if let Err(e) = wifi.start() {
        error!("wifi start failed: {:?}", e);
        return;
    }

    info!("wifi: started STA mode");

    // Connect to Wi-Fi
    if let Err(e) = wifi.connect() {
        error!("wifi connect failed: {:?}", e);
        return;
    }

    info!("wifi: connecting...");

    // Wait for connection
    loop {
        if wifi.is_connected().unwrap_or(false) {
            info!("wifi: connected!");
            break;
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    // TODO: Set up TCP/IP stack with DHCP when implementing MQTT
    // For now, just signal that Wi-Fi is connected
    // Using a dummy IP address since we don't have DHCP yet
    NETWORK_READY.signal(Ipv4Address::new(192, 168, 1, 100));
    info!("network: ready (TCP/IP stack not yet implemented)");

    // Monitor connection and reconnect if needed
    loop {
        Timer::after(Duration::from_secs(5)).await;

        if !wifi.is_connected().unwrap_or(true) {
            warn!("wifi: disconnected, attempting reconnect...");

            if let Err(e) = wifi.connect() {
                error!("wifi reconnect failed: {:?}", e);
            } else {
                // Wait for reconnection
                loop {
                    if wifi.is_connected().unwrap_or(false) {
                        info!("wifi: reconnected!");
                        break;
                    }
                    Timer::after(Duration::from_millis(100)).await;
                }
            }
        }
    }
}

// TODO: Re-enable sensor task once we resolve the generic task function limitation
// embassy_executor::task doesn't support generic parameters
// For now, sensor reading is done in the main loop
/*
/// Sensor task: reads soil moisture sensor periodically
#[embassy_executor::task]
async fn sensor_task<PIN>(
    mut adc: Adc<'static, esp_hal::peripherals::ADC1<'static>, Blocking>,
    mut pin: AdcPin<PIN, esp_hal::peripherals::ADC1<'static>>,
) {
    info!("sensor: task started");

    // Wait for network to be ready before starting sensor readings
    let ip = NETWORK_READY.wait().await;
    info!("sensor: network ready at {}, starting readings", ip);

    loop {
        // Read soil moisture sensor
        match adc.read_oneshot(&mut pin) {
            Ok(raw) => {
                info!("sensor: moisture raw={} (0..4095)", raw);
                // TODO: Publish to MQTT when MQTT client is implemented
            }
            Err(_) => {
                error!("sensor: ADC read failed");
            }
        }

        // Read every 5 seconds
        Timer::after(Duration::from_secs(5)).await;
    }
}
*/

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // --- Wi‑Fi bring‑up (STA) ---------------------------------------------------------------
    // Credentials source options:
    // - Preferred: create a local, Git-ignored `src/secrets.rs` with:
    //     pub const WIFI_SSID: &str = "YourSSID";
    //     pub const WIFI_PASS: &str = "YourPassword";
    //   and build with `--features local_secrets`.
    // - Fallback: compile-time env vars `WIFI_SSID` / `WIFI_PASS`.

    let (ssid, pass) = {
        #[cfg(feature = "local_secrets")]
        {
            (LOCAL_SSID, LOCAL_PASS)
        }
        #[cfg(not(feature = "local_secrets"))]
        {
            (
                option_env!("WIFI_SSID").unwrap_or(""),
                option_env!("WIFI_PASS").unwrap_or(""),
            )
        }
    };

    if !ssid.is_empty() {
        match esp_radio::init() {
            Ok(radio_init) => {
                // Store radio initialization in static storage to make it 'static
                use alloc::boxed::Box;
                let radio_init: &'static _ = Box::leak(Box::new(radio_init));

                // Create Wi-Fi controller and interfaces
                let wifi_cfg = esp_radio::wifi::Config::default();
                let (wifi, ifaces) = match esp_radio::wifi::new(radio_init, peripherals.WIFI, wifi_cfg) {
                    Ok(v) => v,
                    Err(e) => {
                        error!("wifi new() failed: {:?}", e);
                        loop {}
                    }
                };

                // Configure as Wi‑Fi station (client)
                // BuilderLite pattern: use with_ methods
                let client = esp_radio::wifi::ClientConfig::default()
                    .with_ssid(ssid.into())
                    .with_password(pass.into());

                // Spawn network task to manage Wi-Fi connection and TCP/IP stack
                spawner.spawn(network_task(wifi, ifaces, client)).ok();

                info!("wifi: network task spawned, waiting for connection...");
            }
            Err(e) => {
                error!("esp_radio init failed: {:?}", e);
            }
        }
    } else {
        warn!("wifi: set WIFI_SSID/WIFI_PASS env vars at build time to enable STA connection");
    }

    // Set up ADC1 on pin A0 (XIAO ESP32-C6: A0 = GPIO0). Power the sensor from 3V3, not 5V.
    // esp-hal v1.0 API uses AdcConfig + enable_pin on the GPIO peripheral directly.
    let mut adc1_cfg = AdcConfig::new();
    let mut a0 = adc1_cfg.enable_pin(peripherals.GPIO0, Attenuation::_6dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_cfg);

    // TODO: Spawn sensor reading task once we resolve generic task issue
    // For now, read sensor in main loop
    // spawner.spawn(sensor_task(adc1, a0)).ok();

    info!("application: all tasks spawned");
    info!("sensor: calibration - dry={}, wet={}", SENSOR_DRY, SENSOR_WET);
    info!("sensor: watering threshold = {}%", MOISTURE_THRESHOLD);

    // Main sensor monitoring loop
    // Reads moisture sensor and displays percentage
    // TODO: Add MQTT publishing when network stack is implemented
    // TODO: Add pump control when moisture drops below threshold

    loop {
        match adc1.read_oneshot(&mut a0) {
            Ok(raw) => {
                let moisture_percent = raw_to_moisture_percent(raw);

                if moisture_percent < MOISTURE_THRESHOLD {
                    warn!("sensor: moisture={}% (raw={}), BELOW threshold {}% - pump should activate",
                          moisture_percent, raw, MOISTURE_THRESHOLD);
                } else {
                    info!("sensor: moisture={}% (raw={}), above threshold",
                          moisture_percent, raw);
                }

                // TODO: Publish to MQTT when implemented
                // TODO: Trigger pump when below threshold
            }
            Err(_) => {
                error!("sensor: ADC read failed");
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

/*
// ============================================================================
// CALIBRATION ROUTINE - Replace main loop above with this code to recalibrate
// ============================================================================

    info!("application: all tasks spawned");

    // ===== CALIBRATION ROUTINE =====
    info!("=== MOISTURE SENSOR CALIBRATION ===");
    info!("This will collect dry and wet readings for calibration");

    // Wait a bit for the message to be seen
    Timer::after(Duration::from_secs(2)).await;

    // ===== DRY READINGS =====
    info!("");
    info!("STEP 1: DRY SENSOR READINGS");
    info!("Keep the sensor in AIR (completely dry)");
    info!("Starting in 10 seconds...");
    Timer::after(Duration::from_secs(10)).await;

    info!("Collecting 10 dry readings...");
    let mut dry_readings = [0u16; 10];

    for i in 0..10 {
        match adc1.read_oneshot(&mut a0) {
            Ok(raw) => {
                dry_readings[i] = raw;
                info!("  Dry reading {}: {}", i + 1, raw);
            }
            Err(_) => {
                error!("  ADC read {} failed, using 0", i + 1);
                dry_readings[i] = 0;
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Calculate dry average
    let dry_sum: u32 = dry_readings.iter().map(|&x| x as u32).sum();
    let dry_avg = dry_sum / 10;

    info!("");
    info!("DRY READINGS COMPLETE:");
    info!("  Raw values: {:?}", dry_readings);
    info!("  Average: {}", dry_avg);

    // ===== WET READINGS =====
    Timer::after(Duration::from_secs(3)).await;

    info!("");
    info!("STEP 2: WET SENSOR READINGS");
    info!("Place the sensor in WATER (fully submerged sensing area)");
    info!("Starting in 15 seconds...");
    Timer::after(Duration::from_secs(15)).await;

    info!("Collecting 10 wet readings...");
    let mut wet_readings = [0u16; 10];

    for i in 0..10 {
        match adc1.read_oneshot(&mut a0) {
            Ok(raw) => {
                wet_readings[i] = raw;
                info!("  Wet reading {}: {}", i + 1, raw);
            }
            Err(_) => {
                error!("  ADC read {} failed, using 0", i + 1);
                wet_readings[i] = 0;
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Calculate wet average
    let wet_sum: u32 = wet_readings.iter().map(|&x| x as u32).sum();
    let wet_avg = wet_sum / 10;

    info!("");
    info!("WET READINGS COMPLETE:");
    info!("  Raw values: {:?}", wet_readings);
    info!("  Average: {}", wet_avg);

    // ===== CALIBRATION SUMMARY =====
    info!("");
    info!("=== CALIBRATION SUMMARY ===");
    info!("DRY (air):   {} (individual: {:?})", dry_avg, dry_readings);
    info!("WET (water): {} (individual: {:?})", wet_avg, wet_readings);
    info!("");
    info!("Copy these values for calibration configuration:");
    info!("  const SENSOR_DRY: u16 = {};", dry_avg);
    info!("  const SENSOR_WET: u16 = {};", wet_avg);
    info!("");
    info!("Calibration complete! The device will now halt.");

    // Halt - calibration complete
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
*/
