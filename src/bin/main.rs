#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{error, info, warn};
use esp_hal::analog::adc::{Adc, AdcConfig, AdcPin, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use static_cell::StaticCell;

use core::fmt::Write;
use heapless::String;

// MQTT imports - configuration setup for future implementation
// Note: Full rust-mqtt integration with esp-radio's smoltcp stack requires
// additional adapter code that will be implemented in a future step

#[cfg(feature = "mqtt_impl_rust_mqtt")]
use embassy_sync::mutex::Mutex;

// Optional local secrets support
#[cfg(feature = "local_secrets")]
mod secrets;
#[cfg(feature = "local_secrets")]
use secrets::{WIFI_PASS as LOCAL_PASS, WIFI_SSID as LOCAL_SSID};

extern crate alloc;

// ----------------------------------------------------------------------------
// MQTT publishing abstraction (crate-agnostic) per plan in .junie/plans/mqtt-tcp/plan.md
// ----------------------------------------------------------------------------

/// MQTT QoS mapping for a minimal, crate-agnostic publish interface.
pub enum MqQos {
    /// QoS 0 — At most once
    AtMostOnce,
    /// QoS 1 — At least once
    AtLeastOnce,
}

/// Minimal MQTT publish trait to decouple app code from a specific client crate.
pub trait MqttPublish {
    type Err;
    /// Publish a binary payload to `topic` with the given QoS and retain flag.
    async fn publish(
        &mut self,
        topic: &str,
        payload: &[u8],
        qos: MqQos,
        retain: bool,
    ) -> Result<(), Self::Err>;
}

/// Log-only publisher used while TCP/MQTT client integration is feature-gated.
/// This allows exercising the discovery and topic-building code without a broker.
pub struct LoggerPublisher;

impl MqttPublish for LoggerPublisher {
    type Err = core::convert::Infallible;

    async fn publish(
        &mut self,
        topic: &str,
        payload: &[u8],
        qos: MqQos,
        retain: bool,
    ) -> Result<(), Self::Err> {
        let qos_str = match qos {
            MqQos::AtMostOnce => "QoS0",
            MqQos::AtLeastOnce => "QoS1",
        };
        info!(
            "mqtt(LOG): topic='{}' len={} {} retain={}",
            topic,
            payload.len(),
            qos_str,
            retain
        );
        Ok(())
    }
}

// ----------------------------------------------------------------------------
// MQTT concrete publisher wiring (feature-gated)
// ----------------------------------------------------------------------------

#[cfg(feature = "mqtt_impl_rust_mqtt")]
mod mqtt_impl {
    use super::*;
    use embassy_sync::mutex::Mutex;

    // Placeholder TCP stream adapter over esp-radio/smoltcp. The real
    // implementation will provide non-blocking read/write using smoltcp sockets.
    pub struct SmolTcpStream;
    impl SmolTcpStream {
        pub async fn connect(_host: &str, _port: u16) -> Result<Self, ()> {
            // TODO: implement real TCP connect using esp_radio::wifi::Interfaces
            Err(())
        }
        pub async fn close(&mut self) {}
    }

    // Publisher storage accessible to publish task
    pub type PubMutex = Mutex<NoopRawMutex, Option<LoggerPublisher>>;
    static PUB_CELL: StaticCell<PubMutex> = StaticCell::new();

    pub fn publisher_mutex() -> &'static PubMutex {
        PUB_CELL.init(Mutex::new(None))
    }

    pub async fn with_publisher<F, R>(f: F) -> Option<R>
    where
        F: for<'a> FnOnce(
            &'a mut LoggerPublisher,
        ) -> core::pin::Pin<Box<dyn core::future::Future<Output = R> + 'a>>,
    {
        let m = publisher_mutex();
        let mut g = m.lock().await;
        if let Some(p) = g.as_mut() {
            Some(f(p).await)
        } else {
            None
        }
    }
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Type aliases for moisture sensor task parameters
type MoistureAdc = Adc<'static, esp_hal::peripherals::ADC1<'static>, esp_hal::Blocking>;
type MoistureAdcPin =
    AdcPin<esp_hal::peripherals::GPIO0<'static>, esp_hal::peripherals::ADC1<'static>>;

// Device and sensor identifiers for MQTT topic namespacing
// TODO: Consider generating DEVICE_ID from MAC address suffix for uniqueness
const DEVICE_ID: &str = "fevicol-01";
const SENSOR_ID: &str = "moisture-1";

// MQTT Configuration
// Configure these constants for your Home Assistant MQTT broker
const MQTT_BROKER_HOST: &str = "192.168.1.100"; // Replace with your broker IP or hostname
const MQTT_BROKER_PORT: u16 = 1883;
const MQTT_KEEP_ALIVE_SECS: u16 = 60;
const MQTT_SESSION_EXPIRY_SECS: u32 = 3600; // 1 hour - supports battery-powered use
#[allow(dead_code)] // Will be used when MQTT client is implemented
const MQTT_USERNAME: &str = ""; // Empty for no authentication
#[allow(dead_code)] // Will be used when MQTT client is implemented
const MQTT_PASSWORD: &str = ""; // Empty for no authentication

// Home Assistant MQTT Discovery Configuration
// Discovery topics follow the pattern: homeassistant/{component}/{device_id}/{sensor_id}/config
// State topics follow the pattern: fevicol/{device_id}/{sensor_id}/{metric}

/// Project version from Cargo.toml for device metadata
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Home Assistant device information (shared across all entities)
const DEVICE_NAME: &str = "Fevicol Plant Monitor";
const DEVICE_MANUFACTURER: &str = "Fevicol Project";
const DEVICE_MODEL: &str = "ESP32-C6 Moisture Sensor";

/// Sensor reading data structure for inter-task communication
#[derive(Clone, Copy, defmt::Format)]
struct SensorReading {
    /// Moisture percentage (0-100%)
    moisture: u8,
    /// Raw ADC value (0-4095)
    raw: u16,
    /// Timestamp in milliseconds since boot
    timestamp: u64,
}

// Signal to notify when network is ready
static NETWORK_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Channel for passing sensor readings from sensor task to MQTT publisher
// Capacity of 20 readings allows buffering ~100 seconds of data during network outages
static SENSOR_CHANNEL: StaticCell<Channel<NoopRawMutex, SensorReading, 20>> = StaticCell::new();

// MQTT state management: signal to notify when MQTT client is connected
static MQTT_CONNECTED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// Expose esp-radio Interfaces to networking adapters when Wi‑Fi is up.
// Filled by network_task once Wi‑Fi and the smoltcp stack are ready.
#[cfg(feature = "mqtt_impl_rust_mqtt")]
static IFACES_CELL: StaticCell<Mutex<NoopRawMutex, Option<esp_radio::wifi::Interfaces<'static>>>> =
    StaticCell::new();

#[cfg(feature = "mqtt_impl_rust_mqtt")]
fn ifaces_mutex() -> &'static Mutex<NoopRawMutex, Option<esp_radio::wifi::Interfaces<'static>>> {
    IFACES_CELL.init(Mutex::new(None))
}

// Calibration constants from sensor calibration routine
const SENSOR_DRY: u16 = 2188; // ADC value when sensor is in air (0% moisture)
const SENSOR_WET: u16 = 4095; // ADC value when sensor is in water (100% moisture)

// Watering threshold: trigger pump when moisture falls below this percentage
const MOISTURE_THRESHOLD: u8 = 30; // 30% moisture

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

// ============================================================================
// Home Assistant MQTT Discovery Helpers
// ============================================================================

/// Build a Home Assistant discovery topic
/// Format: homeassistant/{component}/{device_id}/{entity_id}/config
fn build_discovery_topic(component: &str, device_id: &str, entity_id: &str) -> String<128> {
    let mut topic = String::new();
    write!(
        topic,
        "homeassistant/{}/{}/{}/config",
        component, device_id, entity_id
    )
    .ok();
    topic
}

/// Build a state topic for sensor readings
/// Format: fevicol/{device_id}/{sensor_id}/{metric}
fn build_state_topic(device_id: &str, sensor_id: &str, metric: &str) -> String<128> {
    let mut topic = String::new();
    write!(topic, "fevicol/{}/{}/{}", device_id, sensor_id, metric).ok();
    topic
}

/// Build availability topic for device online/offline status
/// Format: fevicol/{device_id}/status
fn build_availability_topic(device_id: &str) -> String<64> {
    let mut topic = String::new();
    write!(topic, "fevicol/{}/status", device_id).ok();
    topic
}

/// Create Home Assistant discovery payload for moisture percentage sensor
fn create_moisture_discovery_payload(device_id: &str, sensor_id: &str) -> String<1024> {
    let mut payload = String::new();
    let state_topic = build_state_topic(device_id, sensor_id, "moisture");
    let availability_topic = build_availability_topic(device_id);
    let unique_id = format_unique_id(device_id, sensor_id, "moisture");

    // Build JSON manually (no serde_json in no_std)
    write!(payload, "{{").ok();
    write!(payload, "\"unique_id\":\"{}\",", unique_id.as_str()).ok();
    write!(payload, "\"name\":\"Moisture Sensor\",").ok();
    write!(payload, "\"state_topic\":\"{}\",", state_topic.as_str()).ok();
    write!(payload, "\"device_class\":\"moisture\",").ok();
    write!(payload, "\"unit_of_measurement\":\"%\",").ok();
    write!(payload, "\"icon\":\"mdi:water-percent\",").ok();
    write!(
        payload,
        "\"availability_topic\":\"{}\",",
        availability_topic.as_str()
    )
    .ok();
    write!(payload, "\"payload_available\":\"online\",").ok();
    write!(payload, "\"payload_not_available\":\"offline\",").ok();

    // Add device information
    write!(payload, "\"device\":{{").ok();
    write!(
        payload,
        "\"identifiers\":[\"{}\"],",
        device_id.replace('-', "_")
    )
    .ok();
    write!(payload, "\"name\":\"{} - {}\",", DEVICE_NAME, device_id).ok();
    write!(payload, "\"model\":\"{}\",", DEVICE_MODEL).ok();
    write!(payload, "\"manufacturer\":\"{}\",", DEVICE_MANUFACTURER).ok();
    write!(payload, "\"sw_version\":\"{}\"", VERSION).ok();
    write!(payload, "}}").ok(); // close device

    write!(payload, "}}").ok(); // close root
    payload
}

/// Create Home Assistant discovery payload for raw ADC sensor (debugging)
fn create_raw_discovery_payload(device_id: &str, sensor_id: &str) -> String<1024> {
    let mut payload = String::new();
    let state_topic = build_state_topic(device_id, sensor_id, "raw");
    let availability_topic = build_availability_topic(device_id);
    let unique_id = format_unique_id(device_id, sensor_id, "raw");

    // Build JSON manually
    write!(payload, "{{").ok();
    write!(payload, "\"unique_id\":\"{}\",", unique_id.as_str()).ok();
    write!(payload, "\"name\":\"Moisture Raw ADC\",").ok();
    write!(payload, "\"state_topic\":\"{}\",", state_topic.as_str()).ok();
    write!(payload, "\"unit_of_measurement\":\"ADC\",").ok();
    write!(payload, "\"icon\":\"mdi:chip\",").ok();
    write!(
        payload,
        "\"availability_topic\":\"{}\",",
        availability_topic.as_str()
    )
    .ok();
    write!(payload, "\"payload_available\":\"online\",").ok();
    write!(payload, "\"payload_not_available\":\"offline\",").ok();

    // Add device information (same device as moisture sensor)
    write!(payload, "\"device\":{{").ok();
    write!(
        payload,
        "\"identifiers\":[\"{}\"],",
        device_id.replace('-', "_")
    )
    .ok();
    write!(payload, "\"name\":\"{} - {}\",", DEVICE_NAME, device_id).ok();
    write!(payload, "\"model\":\"{}\",", DEVICE_MODEL).ok();
    write!(payload, "\"manufacturer\":\"{}\",", DEVICE_MANUFACTURER).ok();
    write!(payload, "\"sw_version\":\"{}\"", VERSION).ok();
    write!(payload, "}}").ok(); // close device

    write!(payload, "}}").ok(); // close root
    payload
}

/// Format a unique ID for Home Assistant entities
/// Format: {device_id}_{sensor_id}_{metric}
fn format_unique_id(device_id: &str, sensor_id: &str, metric: &str) -> String<64> {
    let mut id = String::new();
    // Replace hyphens with underscores for valid entity IDs
    write!(
        id,
        "{}_{}_{}",
        device_id.replace('-', "_"),
        sensor_id.replace('-', "_"),
        metric
    )
    .ok();
    id
}

/// Publish Home Assistant discovery messages for all sensors over an MQTT client.
/// The client must already be connected. Messages are published retained with QoS 1.
async fn publish_discovery<C: MqttPublish + ?Sized>(
    client: &mut C,
    device_id: &str,
    sensor_id: &str,
) -> Result<(), C::Err> {
    info!("mqtt: publishing Home Assistant discovery messages...");

    // 1) Availability: publish online retained
    let availability_topic = build_availability_topic(device_id);
    client
        .publish(
            availability_topic.as_str(),
            b"online",
            MqQos::AtLeastOnce,
            true,
        )
        .await?;

    // Small delay between publishes to avoid overwhelming the broker
    Timer::after(Duration::from_millis(100)).await;

    // 2) Moisture discovery retained
    let moisture_topic = build_discovery_topic("sensor", device_id, "moisture");
    let moisture_payload = create_moisture_discovery_payload(device_id, sensor_id);
    client
        .publish(
            moisture_topic.as_str(),
            moisture_payload.as_bytes(),
            MqQos::AtLeastOnce,
            true,
        )
        .await?;

    Timer::after(Duration::from_millis(100)).await;

    // 3) Raw ADC discovery retained
    let raw_topic = build_discovery_topic("sensor", device_id, "raw");
    let raw_payload = create_raw_discovery_payload(device_id, sensor_id);
    client
        .publish(
            raw_topic.as_str(),
            raw_payload.as_bytes(),
            MqQos::AtLeastOnce,
            true,
        )
        .await?;

    info!("mqtt: Home Assistant discovery published");
    Ok(())
}

/// Moisture sensor task: reads ADC periodically and sends readings to MQTT publisher
#[embassy_executor::task]
async fn moisture_sensor_task(
    mut adc: MoistureAdc,
    mut pin: MoistureAdcPin,
    sender: embassy_sync::channel::Sender<'static, NoopRawMutex, SensorReading, 20>,
) {
    info!("sensor: task started");

    loop {
        // Read soil moisture sensor
        match adc.read_oneshot(&mut pin) {
            Ok(raw) => {
                let moisture_percent = raw_to_moisture_percent(raw);
                let timestamp = Instant::now().as_millis();

                let reading = SensorReading {
                    moisture: moisture_percent,
                    raw,
                    timestamp,
                };

                // Try to send reading to MQTT publisher (non-blocking)
                if sender.try_send(reading).is_err() {
                    warn!("sensor: channel full, dropping reading");
                }

                // Log moisture level and check threshold
                if moisture_percent < MOISTURE_THRESHOLD {
                    warn!(
                        "sensor: moisture={}% (raw={}), BELOW threshold {}% - pump should activate",
                        moisture_percent, raw, MOISTURE_THRESHOLD
                    );
                } else {
                    info!(
                        "sensor: moisture={}% (raw={}), above threshold",
                        moisture_percent, raw
                    );
                }
            }
            Err(_) => {
                error!("sensor: ADC read failed");
            }
        }

        // Read every 5 seconds
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// MQTT connection management task: maintains connection to MQTT broker
/// Note: This is currently a placeholder implementation that sets up the configuration
/// and demonstrates the reconnection logic structure. Full TCP socket integration with
/// esp-radio's smoltcp stack will be implemented in a follow-up.
#[embassy_executor::task]
async fn mqtt_connection_task() {
    info!("mqtt: connection task started, waiting for network...");

    // Wait for network to be ready
    NETWORK_READY.wait().await;
    info!("mqtt: network ready");

    // Log MQTT configuration
    info!(
        "mqtt: broker configured - {}:{}",
        MQTT_BROKER_HOST, MQTT_BROKER_PORT
    );
    info!("mqtt: client ID - {}", DEVICE_ID);
    info!("mqtt: keep-alive - {}s", MQTT_KEEP_ALIVE_SECS);
    info!("mqtt: session expiry - {}s", MQTT_SESSION_EXPIRY_SECS);

    // In default build (no client feature), use a log-only publisher to exercise the flow.
    #[cfg(not(any(
        feature = "mqtt_impl_rust_mqtt",
        feature = "mqtt_impl_embedded_mqttc",
        feature = "mqtt_impl_mountain_mqtt"
    )))]
    {
        let mut publog = LoggerPublisher;
        let _ = publish_discovery(&mut publog, DEVICE_ID, SENSOR_ID).await;
        MQTT_CONNECTED.signal(true);
        info!("mqtt: log-only mode active (enable an mqtt_impl_* feature for real client)");
    }

    // rust-mqtt client path (feature-gated). Note: the TCP adapter over smoltcp
    // will be provided as `SmolTcpStream` implementing embedded-io-async.
    #[cfg(feature = "mqtt_impl_rust_mqtt")]
    {
        // Temporary: store LoggerPublisher as the concrete publisher so the
        // rest of the pipeline exercises the same code paths. Once the TCP
        // adapter and real client are ready, replace this with an actual
        // client connection and publisher wrapper.
        use crate::mqtt_impl::publisher_mutex;
        let m = publisher_mutex();
        {
            let mut g = m.lock().await;
            *g = Some(LoggerPublisher);
        }

        // Publish discovery using the shared publisher
        {
            let mut publog = LoggerPublisher;
            let _ = publish_discovery(&mut publog, DEVICE_ID, SENSOR_ID).await;
        }
        MQTT_CONNECTED.signal(true);
        info!(
            "mqtt: rust-mqtt feature path active (temporary log-only publisher). TCP adapter + client pending."
        );
    }

    // Placeholder: in the actual implementation, this would:
    // 1. Handle reconnection with exponential backoff (2s → 30s max)
    // 2. Monitor connection health with PINGREQ
    // 3. Re-publish discovery messages on reconnection (HA might have restarted)

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// MQTT publishing task: receives sensor readings and publishes to broker
#[embassy_executor::task]
async fn mqtt_publish_task(
    receiver: embassy_sync::channel::Receiver<'static, NoopRawMutex, SensorReading, 20>,
    device_id: &'static str,
    sensor_id: &'static str,
) {
    info!(
        "mqtt: publish task started for device={}, sensor={}",
        device_id, sensor_id
    );

    // Wait for MQTT client to be connected
    info!("mqtt: publish task waiting for MQTT connection...");
    loop {
        let connected = MQTT_CONNECTED.wait().await;
        if connected {
            info!("mqtt: MQTT client ready, starting to receive sensor readings");
            break;
        } else {
            warn!("mqtt: MQTT disconnected, waiting for reconnection...");
        }
    }

    loop {
        // Receive sensor reading from channel (blocks until available)
        let reading = receiver.receive().await;

        // Default/log-only path: just log values
        #[cfg(not(feature = "mqtt_impl_rust_mqtt"))]
        {
            info!(
                "mqtt: ready to publish - moisture={}%, raw={}, timestamp={}ms (log-only mode)",
                reading.moisture, reading.raw, reading.timestamp
            );
        }

        // Feature path: use shared publisher (currently LoggerPublisher till TCP client lands)
        #[cfg(feature = "mqtt_impl_rust_mqtt")]
        {
            use crate::mqtt_impl::with_publisher;

            let state_topic = build_state_topic(device_id, sensor_id, "moisture");
            let raw_topic = build_state_topic(device_id, sensor_id, "raw");
            let ts_topic = build_state_topic(device_id, sensor_id, "timestamp");

            // Encode payloads as small ASCII
            let mut moist_buf: heapless::String<8> = heapless::String::new();
            let _ = write!(moist_buf, "{}", reading.moisture);
            let mut raw_buf: heapless::String<8> = heapless::String::new();
            let _ = write!(raw_buf, "{}", reading.raw);
            let mut ts_buf: heapless::String<16> = heapless::String::new();
            let _ = write!(ts_buf, "{}", reading.timestamp);

            let _ = with_publisher(|p| {
                Box::pin(async move {
                    let _ = p
                        .publish(
                            state_topic.as_str(),
                            moist_buf.as_bytes(),
                            MqQos::AtLeastOnce,
                            false,
                        )
                        .await;
                    let _ = p
                        .publish(
                            raw_topic.as_str(),
                            raw_buf.as_bytes(),
                            MqQos::AtLeastOnce,
                            false,
                        )
                        .await;
                    let _ = p
                        .publish(
                            ts_topic.as_str(),
                            ts_buf.as_bytes(),
                            MqQos::AtMostOnce,
                            false,
                        )
                        .await;
                })
            })
            .await;
        }
    }
}

/// Network task: manages Wi-Fi connection
/// Note: esp-radio 0.17.0 provides smoltcp stack integration via Interfaces
/// Future implementation will use Interfaces directly for TCP socket creation
#[embassy_executor::task]
async fn network_task(
    mut wifi: esp_radio::wifi::WifiController<'static>,
    ifaces: esp_radio::wifi::Interfaces<'static>,
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

    // Provide Interfaces to TCP adapter users (feature-gated)
    #[cfg(feature = "mqtt_impl_rust_mqtt")]
    {
        let m = ifaces_mutex();
        let mut g = m.lock().await;
        *g = Some(ifaces);
    }

    // Signal that network is ready for consumers
    // esp-radio's Interfaces provides the smoltcp stack
    // TCP/IP and DHCP are handled by the smoltcp stack within Interfaces
    NETWORK_READY.signal(());
    info!("network: ready, TCP/IP stack available via esp-radio Interfaces");

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
                let (wifi, ifaces) =
                    match esp_radio::wifi::new(radio_init, peripherals.WIFI, wifi_cfg) {
                        Ok(v) => v,
                        Err(e) => {
                            error!("wifi new() failed: {:?}", e);
                            panic!("wifi initialization failed");
                        }
                    };

                // Configure as Wi‑Fi station (client)
                // BuilderLite pattern: use with_ methods
                let client = esp_radio::wifi::ClientConfig::default()
                    .with_ssid(ssid.into())
                    .with_password(pass.into());

                // Spawn network task to manage Wi-Fi connection
                // Note: esp-radio's Interfaces provides the smoltcp TCP/IP stack
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

    // Initialize sensor reading channel for inter-task communication
    let sensor_channel = SENSOR_CHANNEL.init(Channel::new());
    let sensor_sender = sensor_channel.sender();
    let sensor_receiver = sensor_channel.receiver();

    // Spawn MQTT connection management task
    spawner.spawn(mqtt_connection_task()).ok();

    // Spawn MQTT publishing task
    spawner
        .spawn(mqtt_publish_task(sensor_receiver, DEVICE_ID, SENSOR_ID))
        .ok();

    // Set up ADC1 on pin A0 (XIAO ESP32-C6: A0 = GPIO0). Power the sensor from 3V3, not 5V.
    // esp-hal v1.0 API uses AdcConfig + enable_pin on the GPIO peripheral directly.
    let mut adc1_cfg = AdcConfig::new();
    let a0 = adc1_cfg.enable_pin(peripherals.GPIO0, Attenuation::_6dB);
    let adc1 = Adc::new(peripherals.ADC1, adc1_cfg);

    // Spawn moisture sensor task
    spawner
        .spawn(moisture_sensor_task(adc1, a0, sensor_sender))
        .ok();

    info!("application: all tasks spawned");
    info!(
        "sensor: calibration - dry={}, wet={}",
        SENSOR_DRY, SENSOR_WET
    );
    info!("sensor: watering threshold = {}%", MOISTURE_THRESHOLD);
    info!("sensor: device_id={}, sensor_id={}", DEVICE_ID, SENSOR_ID);
    info!(
        "mqtt: broker={}:{}, client_id={}",
        MQTT_BROKER_HOST, MQTT_BROKER_PORT, DEVICE_ID
    );

    // Main loop: all work is now done in spawned tasks
    // This loop just keeps the executor alive
    loop {
        Timer::after(Duration::from_secs(3600)).await;
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
