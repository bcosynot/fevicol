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

use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(feature = "mqtt")]
use embassy_net::{Config as NetConfig, Stack, StackResources};

use static_cell::StaticCell;

use core::fmt::Write;
use heapless::String;

// Optional local secrets support
#[cfg(feature = "local_secrets")]
mod secrets;
#[cfg(feature = "local_secrets")]
use secrets::{
    MQTT_BROKER_HOST as LOCAL_MQTT_BROKER_HOST, MQTT_BROKER_PORT as LOCAL_MQTT_BROKER_PORT,
    MQTT_PASSWORD as LOCAL_MQTT_PASSWORD, MQTT_USERNAME as LOCAL_MQTT_USERNAME,
    WIFI_PASS as LOCAL_PASS, WIFI_SSID as LOCAL_SSID,
};

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
#[allow(async_fn_in_trait)] // We intentionally use async fn here; the trait is crate-internal and futures need not be `Send`.
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
            "mqtt(LOG): publishing to channel='{}' len={} {} retain={}",
            topic,
            payload.len(),
            qos_str,
            retain
        );
        Ok(())
    }
}

// ----------------------------------------------------------------------------
// rust-mqtt Transport Adapter (feature-gated)
// ----------------------------------------------------------------------------

#[cfg(feature = "mqtt")]
use embedded_io_async::{ErrorType, Read, Write as IoWrite};

/// Transport adapter wrapping embassy_net::tcp::TcpSocket for rust-mqtt client.
/// Implements embedded_io_async traits required by rust-mqtt v0.3.
#[cfg(feature = "mqtt")]
pub struct EmbassyNetTransport<'a> {
    socket: embassy_net::tcp::TcpSocket<'a>,
}

#[cfg(feature = "mqtt")]
impl<'a> EmbassyNetTransport<'a> {
    /// Create a new transport from an embassy-net TCP socket.
    pub fn new(socket: embassy_net::tcp::TcpSocket<'a>) -> Self {
        Self { socket }
    }

    /// Get a mutable reference to the underlying TCP socket for connection establishment.
    pub fn socket_mut(&mut self) -> &mut embassy_net::tcp::TcpSocket<'a> {
        &mut self.socket
    }
}

#[cfg(feature = "mqtt")]
impl<'a> ErrorType for EmbassyNetTransport<'a> {
    type Error = embassy_net::tcp::Error;
}

#[cfg(feature = "mqtt")]
impl<'a> Read for EmbassyNetTransport<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.socket.read(buf).await
    }
}

#[cfg(feature = "mqtt")]
impl<'a> IoWrite for EmbassyNetTransport<'a> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.socket.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.socket.flush().await
    }
}

// ----------------------------------------------------------------------------
// rust-mqtt Client Wrapper (feature-gated)
// ----------------------------------------------------------------------------

#[cfg(feature = "mqtt")]
use rust_mqtt::client::client::MqttClient;
#[cfg(feature = "mqtt")]
use rust_mqtt::packet::v5::publish_packet::QualityOfService;
#[cfg(feature = "mqtt")]
use rust_mqtt::packet::v5::reason_codes::ReasonCode;
#[cfg(feature = "mqtt")]
use rust_mqtt::utils::rng_generator::CountingRng;

/// Wrapper for rust-mqtt client implementing the MqttPublish trait.
/// Provides MQTT v5 support with full feature set including LWT, authentication, and retain flags.
#[cfg(feature = "mqtt")]
pub struct RustMqttPublisher<'a, T: Read + IoWrite> {
    client: MqttClient<'a, T, 5, CountingRng>,
}

#[cfg(feature = "mqtt")]
impl<'a, T: Read + IoWrite> MqttPublish for RustMqttPublisher<'a, T> {
    type Err = ReasonCode;

    async fn publish(
        &mut self,
        topic: &str,
        payload: &[u8],
        qos: MqQos,
        retain: bool,
    ) -> Result<(), Self::Err> {
        let mqtt_qos = match qos {
            MqQos::AtMostOnce => QualityOfService::QoS0,
            MqQos::AtLeastOnce => QualityOfService::QoS1,
        };

        let qos_str = match qos {
            MqQos::AtMostOnce => "QoS0",
            MqQos::AtLeastOnce => "QoS1",
        };
        info!(
            "mqtt: publishing to channel='{}' len={} {} retain={}",
            topic,
            payload.len(),
            qos_str,
            retain
        );

        self.client
            .send_message(topic, payload, mqtt_qos, retain)
            .await
    }
}

/// Interpret MQTT v5 CONNACK reason codes for better error diagnostics.
/// Returns a human-readable description of the reason code.
#[cfg(feature = "mqtt")]
fn interpret_connack_reason(
    reason: &rust_mqtt::packet::v5::reason_codes::ReasonCode,
) -> &'static str {
    use rust_mqtt::packet::v5::reason_codes::ReasonCode;

    match *reason {
        ReasonCode::Success => "Success",
        ReasonCode::UnspecifiedError => "Unspecified error",
        ReasonCode::MalformedPacket => "Malformed packet",
        ReasonCode::ProtocolError => "Protocol error",
        ReasonCode::ImplementationSpecificError => "Implementation specific error",
        ReasonCode::UnsupportedProtocolVersion => "Unsupported protocol version",
        ReasonCode::ClientIdNotValid => "Client identifier not valid",
        ReasonCode::BadUserNameOrPassword => "Bad username or password",
        ReasonCode::NotAuthorized => "Not authorized",
        ReasonCode::ServerUnavailable => "Server unavailable",
        ReasonCode::ServerBusy => "Server busy",
        ReasonCode::Banned => "Client banned",
        ReasonCode::TopicNameInvalid => "Topic name invalid",
        ReasonCode::PacketTooLarge => "Packet too large",
        ReasonCode::QuotaExceeded => "Quota exceeded",
        ReasonCode::PayloadFormatInvalid => "Payload format invalid",
        ReasonCode::RetainNotSupported => "Retain not supported",
        ReasonCode::QoSNotSupported => "QoS not supported",
        ReasonCode::UseAnotherServer => "Use another server",
        ReasonCode::ServerMoved => "Server moved",
        ReasonCode::ConnectionRateExceeded => "Connection rate exceeded",
        _ => "Unknown reason code",
    }
}

/// Configuration for initializing rust-mqtt client
#[cfg(feature = "mqtt")]
pub struct MqttClientConfig<'a> {
    pub client_id: &'a str,
    pub keep_alive_secs: u16,
    pub session_expiry_secs: u32,
    pub username: &'a str,
    pub password: &'a str,
    pub lwt_topic: &'a str,
    pub lwt_payload: &'a [u8],
    pub lwt_retain: bool,
}

/// Initialize rust-mqtt client with full MQTT v5 configuration.
///
/// # Parameters
/// - `transport`: EmbassyNetTransport wrapping a connected TCP socket
/// - `config`: MQTT client configuration (credentials, keep-alive, LWT, etc.)
/// - `recv_buffer`: Buffer for receiving MQTT packets
/// - `write_buffer`: Buffer for writing MQTT packets
///
/// # Returns
/// A configured RustMqttPublisher ready for publishing messages.
#[cfg(feature = "mqtt")]
pub async fn init_rust_mqtt_client<'a>(
    transport: EmbassyNetTransport<'a>,
    config: MqttClientConfig<'a>,
    recv_buffer: &'a mut [u8],
    write_buffer: &'a mut [u8],
) -> Result<RustMqttPublisher<'a, EmbassyNetTransport<'a>>, ReasonCode> {
    use rust_mqtt::client::client_config::ClientConfig;
    use rust_mqtt::packet::v5::property::Property;

    let rng = CountingRng(0);

    let mut client_config = ClientConfig::new(rust_mqtt::client::client_config::MqttVersion::MQTTv5, rng);

    client_config.add_client_id(config.client_id);
    client_config.keep_alive = config.keep_alive_secs;
    client_config.add_property(Property::SessionExpiryInterval(config.session_expiry_secs));

    // Note: Clean start flag is hardcoded to true (0x02) in rust-mqtt v0.3.0
    // The library does not provide an API to set clean start to false.
    // This means the client always starts with a clean session, and session
    // persistence relies solely on the SessionExpiryInterval property.
    // For true session persistence (clean start = false), a library update or
    // alternative MQTT client would be required.

    if !config.username.is_empty() {
        client_config.add_username(config.username);
        if !config.password.is_empty() {
            client_config.add_password(config.password);
        }
    }

    // LWT QoS is not configurable in rust-mqtt v0.3
    client_config.add_will(config.lwt_topic, config.lwt_payload, config.lwt_retain);

    let mut client = MqttClient::<_, 5, _>::new(
        transport,
        write_buffer,
        write_buffer.len(),
        recv_buffer,
        recv_buffer.len(),
        client_config,
    );

    match client.connect_to_broker().await {
        Ok(()) => {
            info!("rust-mqtt: Connected to broker");
            Ok(RustMqttPublisher { client })
        }
        Err(e) => {
            error!(
                "rust-mqtt: Connection failed: {:?}",
                defmt::Debug2Format(&e)
            );
            Err(e)
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
const MQTT_KEEP_ALIVE_SECS: u16 = 60;
const MQTT_SESSION_EXPIRY_SECS: u32 = 3600; // 1 hour - supports battery-powered use
// Broker host, port, and authentication credentials are loaded from secrets.rs when local_secrets feature is enabled

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

// Connection health flag: set to false when disconnection is detected
// Used by publish task to signal errors and by connection task to detect disconnection
static MQTT_CONNECTION_HEALTHY: AtomicBool = AtomicBool::new(false);

// embassy-net stack resources (for rust-mqtt integration)
#[cfg(feature = "mqtt")]
static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
#[cfg(feature = "mqtt")]
static NET_STACK: StaticCell<Stack<'static>> = StaticCell::new();

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

    let availability_topic = build_availability_topic(device_id);
    client
        .publish(
            availability_topic.as_str(),
            b"online",
            MqQos::AtLeastOnce,
            true,
        )
        .await?;

    Timer::after(Duration::from_millis(100)).await;

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
        match adc.read_oneshot(&mut pin) {
            Ok(raw) => {
                let moisture_percent = raw_to_moisture_percent(raw);
                let timestamp = Instant::now().as_millis();

                let reading = SensorReading {
                    moisture: moisture_percent,
                    raw,
                    timestamp,
                };

                if sender.try_send(reading).is_err() {
                    warn!("sensor: channel full, dropping reading");
                }

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
///
/// This task implements the complete MQTT v5 client lifecycle with rust-mqtt v0.3:
///
/// **Lifecycle Stages**:
/// 1. Wait for network readiness (Wi-Fi connected, DHCP assigned IP)
/// 2. DNS resolution for broker hostname (with fallback to IP address)
/// 3. TCP connection establishment via embassy_net::tcp::TcpSocket
/// 4. MQTT CONNECT handshake with v5 properties (session expiry, authentication, LWT)
/// 5. Parse CONNACK response and check reason code
/// 6. Publish availability as "online" (retained)
/// 7. Publish Home Assistant discovery messages (retained, QoS 1, 100ms pacing)
/// 8. Receive sensor readings from channel and publish telemetry (QoS 0)
/// 9. On connection loss: exponential backoff (2s → 30s max) and reconnect
/// 10. Re-publish discovery on reconnection (handles Home Assistant restarts)
///
/// **Error Handling**:
/// - DNS failures: exponential backoff, fallback to IP address after 5 failures
/// - TCP failures: exponential backoff, logged with details
/// - MQTT CONNACK errors: logged with reason code interpretation (bad credentials, etc.)
/// - Publish failures: trigger reconnection (rust-mqtt v0.3 doesn't expose PUBACK codes)
/// - Keep-alive timeout: rust-mqtt handles internally, failures trigger reconnection
/// - All errors logged via defmt without panicking
///
/// **Resilience Features**:
/// - Sensor readings buffered in channel (20-reading capacity) during outages
/// - Automatic reconnection without manual intervention
/// - Discovery re-published on reconnection for HA restart resilience
/// - Exponential backoff prevents overwhelming broker during outages
///
/// When mqtt is disabled, uses log-only mode for testing.
#[embassy_executor::task]
async fn mqtt_connection_task(
    #[cfg(feature = "mqtt")] stack: &'static Stack<'static>,
    #[cfg(feature = "mqtt")] sensor_receiver: embassy_sync::channel::Receiver<
        'static,
        NoopRawMutex,
        SensorReading,
        20,
    >,
) {
    info!("mqtt: connection task started, waiting for network...");

    // Stage 1: Wait for network to be ready (Wi-Fi connected, DHCP assigned IP)
    // The NETWORK_READY signal is set by network_task after successful Wi-Fi connection
    NETWORK_READY.wait().await;
    info!("mqtt: network ready");

    let (mqtt_username, mqtt_password) = {
        #[cfg(feature = "local_secrets")]
        {
            (LOCAL_MQTT_USERNAME, LOCAL_MQTT_PASSWORD)
        }
        #[cfg(not(feature = "local_secrets"))]
        {
            ("", "")
        }
    };

    let (mqtt_broker_host, mqtt_broker_port) = {
        #[cfg(feature = "local_secrets")]
        {
            (LOCAL_MQTT_BROKER_HOST, LOCAL_MQTT_BROKER_PORT)
        }
        #[cfg(not(feature = "local_secrets"))]
        {
            ("192.168.0.245", 1883u16) // Default fallback values
        }
    };

    info!(
        "mqtt: broker configured - {}:{}",
        mqtt_broker_host, mqtt_broker_port
    );
    info!("mqtt: client ID - {}", DEVICE_ID);
    info!("mqtt: keep-alive - {}s", MQTT_KEEP_ALIVE_SECS);
    info!("mqtt: session expiry - {}s", MQTT_SESSION_EXPIRY_SECS);

    if !mqtt_username.is_empty() || !mqtt_password.is_empty() {
        let pw_len = mqtt_password.len();
        info!(
            "mqtt: authentication enabled (username='{}', password=*** len={})",
            mqtt_username, pw_len
        );
    } else {
        info!("mqtt: no authentication configured");
    }

    // rust-mqtt implementation with full MQTT v5 support
    #[cfg(feature = "mqtt")]
    {
        // --- Error Handling Strategy: Exponential Backoff ---
        // Start at 2s, double on each failure, max 30s
        // This prevents overwhelming the broker/network during outages while
        // still reconnecting quickly after transient failures.
        // Backoff is reset to 2s on successful connection.
        let mut backoff_secs = 2u64;
        const MAX_BACKOFF_SECS: u64 = 30;

        // --- Error Handling Strategy: DNS Fallback ---
        // Track DNS failures to detect persistent DNS issues.
        // After 5 consecutive failures, attempt to parse MQTT_BROKER_HOST as an IP address.
        // This allows the system to work even if DNS is broken, as long as an IP is configured.
        let mut dns_failure_count = 0u8;
        const DNS_FAILURE_THRESHOLD: u8 = 5;

        loop {
            info!("mqtt: attempting connection...");

            info!("mqtt: resolving broker hostname '{}'...", mqtt_broker_host);
            let broker_addr = match stack
                .dns_query(mqtt_broker_host, embassy_net::dns::DnsQueryType::A)
                .await
            {
                Ok(addrs) => {
                    if addrs.is_empty() {
                        error!("mqtt: DNS resolution returned no addresses");
                        dns_failure_count = dns_failure_count.saturating_add(1);

                        // After repeated DNS failures, attempt to parse as IP address
                        if dns_failure_count >= DNS_FAILURE_THRESHOLD {
                            warn!(
                                "mqtt: DNS failed {} times, attempting to parse '{}' as IP address",
                                dns_failure_count, mqtt_broker_host
                            );
                            match mqtt_broker_host.parse::<smoltcp::wire::Ipv4Address>() {
                                Ok(ip) => {
                                    info!("mqtt: using IP address directly: {}", ip);
                                    dns_failure_count = 0;
                                    ip
                                }
                                Err(_) => {
                                    error!(
                                        "mqtt: '{}' is not a valid IP address, cannot fallback",
                                        mqtt_broker_host
                                    );
                                    Timer::after(Duration::from_secs(backoff_secs)).await;
                                    backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                                    continue;
                                }
                            }
                        } else {
                            Timer::after(Duration::from_secs(backoff_secs)).await;
                            backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                            continue;
                        }
                    } else {
                        let addr = addrs[0];
                        info!("mqtt: resolved '{}' to {}", mqtt_broker_host, addr);
                        dns_failure_count = 0;
                        let smoltcp::wire::IpAddress::Ipv4(ipv4) = addr;
                        ipv4
                    }
                }
                Err(e) => {
                    error!(
                        "mqtt: DNS resolution failed: {:?}, retrying in {}s",
                        defmt::Debug2Format(&e),
                        backoff_secs
                    );
                    dns_failure_count = dns_failure_count.saturating_add(1);

                    // After repeated DNS failures, attempt to parse as IP address
                    if dns_failure_count >= DNS_FAILURE_THRESHOLD {
                        warn!(
                            "mqtt: DNS failed {} times, attempting to parse '{}' as IP address",
                            dns_failure_count, mqtt_broker_host
                        );
                        match mqtt_broker_host.parse::<smoltcp::wire::Ipv4Address>() {
                            Ok(ip) => {
                                info!("mqtt: using IP address directly: {}", ip);
                                dns_failure_count = 0;
                                ip
                            }
                            Err(_) => {
                                error!(
                                    "mqtt: '{}' is not a valid IP address, cannot fallback",
                                    mqtt_broker_host
                                );
                                Timer::after(Duration::from_secs(backoff_secs)).await;
                                backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                                continue;
                            }
                        }
                    } else {
                        Timer::after(Duration::from_secs(backoff_secs)).await;
                        backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                        continue;
                    }
                }
            };

            // Stage 3: Allocate TCP socket buffers (must outlive the socket and client)
            // Buffer sizing optimized for MQTT usage patterns:
            // - TCP RX/TX: 2048 bytes each (sufficient for MQTT packets + TCP overhead)
            // - MQTT recv/write: 2048 bytes each (discovery ~1KB, telemetry <100 bytes)
            // These buffers are stack-allocated and must remain valid for the entire
            // connection lifetime (until the loop restarts on error).
            let mut tcp_rx_buffer = [0u8; 2048];
            let mut tcp_tx_buffer = [0u8; 2048];
            let mut mqtt_recv_buffer = [0u8; 2048];
            let mut mqtt_write_buffer = [0u8; 2048];

            let mut tcp_socket =
                embassy_net::tcp::TcpSocket::new(*stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);

            tcp_socket.set_timeout(Some(Duration::from_secs(10)));

            let remote_endpoint = (broker_addr, mqtt_broker_port);
            info!(
                "mqtt: connecting TCP to {}:{}...",
                broker_addr, mqtt_broker_port
            );
            match tcp_socket.connect(remote_endpoint).await {
                Ok(()) => {
                    info!("mqtt: TCP connected");
                }
                Err(e) => {
                    // TCP connection failures can occur due to:
                    // - Network unreachable (Wi-Fi disconnected, routing issues)
                    // - Connection refused (broker not running, firewall blocking)
                    // - Timeout (broker overloaded, network congestion)
                    // Apply exponential backoff and retry
                    error!(
                        "mqtt: TCP connection failed: {:?}, retrying in {}s",
                        defmt::Debug2Format(&e),
                        backoff_secs
                    );
                    Timer::after(Duration::from_secs(backoff_secs)).await;
                    backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                    continue;
                }
            }

            let transport = EmbassyNetTransport::new(tcp_socket);

            let lwt_topic = build_availability_topic(DEVICE_ID);
            let lwt_payload = b"offline";

            let mqtt_config = MqttClientConfig {
                client_id: DEVICE_ID,
                keep_alive_secs: MQTT_KEEP_ALIVE_SECS,
                session_expiry_secs: MQTT_SESSION_EXPIRY_SECS,
                username: mqtt_username,
                password: mqtt_password,
                lwt_topic: lwt_topic.as_str(),
                lwt_payload,
                lwt_retain: true,
            };

            match init_rust_mqtt_client(
                transport,
                mqtt_config,
                &mut mqtt_recv_buffer,
                &mut mqtt_write_buffer,
            )
            .await
            {
                Ok(mut client) => {
                    info!("mqtt: connected successfully (MQTT v5)");

                    backoff_secs = 2;
                    MQTT_CONNECTION_HEALTHY.store(true, Ordering::Relaxed);

                    info!("mqtt: publishing availability status (online)...");
                    let availability_topic = build_availability_topic(DEVICE_ID);
                    if let Err(e) = client
                        .publish(
                            availability_topic.as_str(),
                            b"online",
                            MqQos::AtLeastOnce,
                            true,
                        )
                        .await
                    {
                        error!("mqtt: availability publish failed: {:?}", e);
                        MQTT_CONNECTION_HEALTHY.store(false, Ordering::Relaxed);
                        MQTT_CONNECTED.signal(false);
                        info!("mqtt: availability publish failed, will reconnect...");
                        continue;
                    }

                    info!("mqtt: publishing discovery messages...");
                    if let Err(e) = publish_discovery(&mut client, DEVICE_ID, SENSOR_ID).await {
                        error!("mqtt: discovery publish failed: {:?}", e);
                        MQTT_CONNECTION_HEALTHY.store(false, Ordering::Relaxed);
                        MQTT_CONNECTED.signal(false);
                        info!("mqtt: discovery failed, will reconnect...");
                        continue;
                    }

                    MQTT_CONNECTED.signal(true);
                    info!("mqtt: ready for telemetry publishing");

                    // Connection maintenance and publishing loop
                    //
                    // Error Handling Notes:
                    // - PINGREQ/PINGRESP: rust-mqtt handles keepalive internally based on keep_alive_secs.
                    //   If keepalive timeout occurs, subsequent operations will fail and we'll reconnect.
                    // - PUBACK reason codes: rust-mqtt v0.3 does not expose PUBACK reason codes to the caller.
                    //   Publish failures are detected as generic errors, triggering reconnection.
                    // - DISCONNECT packets: rust-mqtt does not expose server-initiated DISCONNECT packets.
                    //   Connection loss is detected when operations fail, triggering reconnection.
                    // - All errors break the loop and trigger automatic reconnection with exponential backoff.
                    loop {
                        // Use select to handle both sensor readings and periodic health checks
                        match embassy_futures::select::select(
                            sensor_receiver.receive(),
                            Timer::after(Duration::from_secs(30)),
                        )
                        .await
                        {
                            embassy_futures::select::Either::First(reading) => {
                                info!(
                                    "mqtt: publishing - moisture={}%, raw={}, timestamp={}ms",
                                    reading.moisture, reading.raw, reading.timestamp
                                );

                                let moisture_topic =
                                    build_state_topic(DEVICE_ID, SENSOR_ID, "moisture");
                                let raw_topic = build_state_topic(DEVICE_ID, SENSOR_ID, "raw");

                                let mut moisture_payload = String::<16>::new();
                                if write!(&mut moisture_payload, "{}", reading.moisture).is_err() {
                                    error!("mqtt: failed to format moisture payload");
                                    continue;
                                }

                                let mut raw_payload = String::<16>::new();
                                if write!(&mut raw_payload, "{}", reading.raw).is_err() {
                                    error!("mqtt: failed to format raw payload");
                                    continue;
                                }

                                if let Err(e) = client
                                    .publish(
                                        moisture_topic.as_str(),
                                        moisture_payload.as_bytes(),
                                        MqQos::AtMostOnce,
                                        false,
                                    )
                                    .await
                                {
                                    error!("mqtt: moisture publish failed: {:?}", e);
                                    MQTT_CONNECTION_HEALTHY.store(false, Ordering::Relaxed);
                                    break;
                                }

                                if let Err(e) = client
                                    .publish(
                                        raw_topic.as_str(),
                                        raw_payload.as_bytes(),
                                        MqQos::AtMostOnce,
                                        false,
                                    )
                                    .await
                                {
                                    error!("mqtt: raw publish failed: {:?}", e);
                                    MQTT_CONNECTION_HEALTHY.store(false, Ordering::Relaxed);
                                    break;
                                }

                                info!("mqtt: telemetry published successfully");
                            }
                            embassy_futures::select::Either::Second(_) => {
                                // Periodic health check timeout
                                // rust-mqtt handles keepalive internally.
                                // If the keepalive timeout is exceeded, subsequent operations will fail.
                                // We just continue the loop and wait for the next event.
                            }
                        }
                    }

                    // If we break from the maintenance loop, attempt to publish offline status
                    info!("mqtt: publishing availability status (offline)...");
                    let availability_topic = build_availability_topic(DEVICE_ID);
                    if let Err(e) = client
                        .publish(
                            availability_topic.as_str(),
                            b"offline",
                            MqQos::AtLeastOnce,
                            true,
                        )
                        .await
                    {
                        warn!(
                            "mqtt: offline availability publish failed (expected if connection broken): {:?}",
                            e
                        );
                    }

                    MQTT_CONNECTED.signal(false);
                    info!("mqtt: disconnected, will reconnect...");
                }
                Err(e) => {
                    // Provide detailed error diagnostics for MQTT v5 CONNACK reason codes
                    let reason_str = interpret_connack_reason(&e);
                    error!(
                        "mqtt: MQTT connection failed - reason: {} ({:?}), retrying in {}s",
                        reason_str,
                        defmt::Debug2Format(&e),
                        backoff_secs
                    );

                    // Log specific guidance for common errors
                    match e {
                        rust_mqtt::packet::v5::reason_codes::ReasonCode::BadUserNameOrPassword => {
                            error!("mqtt: Check MQTT_USERNAME and MQTT_PASSWORD configuration");
                        }
                        rust_mqtt::packet::v5::reason_codes::ReasonCode::NotAuthorized => {
                            error!("mqtt: Client not authorized - check broker ACL configuration");
                        }
                        rust_mqtt::packet::v5::reason_codes::ReasonCode::ServerUnavailable
                        | rust_mqtt::packet::v5::reason_codes::ReasonCode::ServerBusy => {
                            warn!("mqtt: Broker temporarily unavailable, will retry");
                        }
                        rust_mqtt::packet::v5::reason_codes::ReasonCode::ClientIdNotValid => {
                            error!(
                                "mqtt: Invalid client ID '{}' - check DEVICE_ID configuration",
                                DEVICE_ID
                            );
                        }
                        _ => {}
                    }

                    MQTT_CONNECTED.signal(false);
                    Timer::after(Duration::from_secs(backoff_secs)).await;
                    backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                }
            }
        }
    }

    // In default build (no mqtt feature), use a log-only publisher to exercise the flow.
    #[cfg(not(feature = "mqtt"))]
    {
        let mut publog = LoggerPublisher;
        let _ = publish_discovery(&mut publog, DEVICE_ID, SENSOR_ID).await;
        MQTT_CONNECTED.signal(true);
        info!("mqtt: log-only mode active (enable mqtt feature for real client)");

        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
}

/// Embassy-net runner task: runs the network stack to process packets
/// This task must be spawned for embassy-net to function properly.
#[cfg(feature = "mqtt")]
#[embassy_executor::task]
async fn embassy_net_task(
    mut runner: embassy_net::Runner<'static, esp_radio::wifi::WifiDevice<'static>>,
) -> ! {
    runner.run().await
}

/// Network task: manages Wi-Fi connection and coordinates with embassy-net stack
/// When mqtt feature is enabled, this task coordinates between
/// the Wi-Fi controller (physical layer) and embassy-net stack (network layer).
#[embassy_executor::task]
async fn network_task(
    mut wifi: esp_radio::wifi::WifiController<'static>,
    client_config: esp_radio::wifi::ClientConfig,
    #[cfg(feature = "mqtt")] stack: &'static Stack<'static>,
) {
    if let Err(e) = wifi.set_config(&esp_radio::wifi::ModeConfig::Client(client_config)) {
        error!("wifi set_config failed: {:?}", e);
        return;
    }

    if let Err(e) = wifi.start() {
        error!("wifi start failed: {:?}", e);
        return;
    }

    info!("wifi: started STA mode");

    if let Err(e) = wifi.connect() {
        error!("wifi connect failed: {:?}", e);
        return;
    }

    info!("wifi: connecting...");

    loop {
        if wifi.is_connected().unwrap_or(false) {
            info!("wifi: connected!");
            break;
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    // When embassy-net is enabled, wait for DHCP to assign an IP address
    #[cfg(feature = "mqtt")]
    {
        info!("network: waiting for DHCP IP assignment...");
        loop {
            if stack.is_config_up()
                && let Some(config) = stack.config_v4()
            {
                info!(
                    "network: DHCP assigned IP: {}, gateway: {}, DNS: {:?}",
                    config.address, config.gateway, config.dns_servers
                );
                break;
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        NETWORK_READY.signal(());
        info!("network: embassy-net stack ready");
    }

    // Without embassy-net, signal ready immediately after Wi-Fi connection
    #[cfg(not(feature = "mqtt"))]
    {
        NETWORK_READY.signal(());
        info!("network: ready, TCP/IP stack available via esp-radio Interfaces");
    }

    loop {
        Timer::after(Duration::from_secs(5)).await;

        if !wifi.is_connected().unwrap_or(true) {
            warn!("wifi: disconnected, attempting reconnect...");

            if let Err(e) = wifi.connect() {
                error!("wifi reconnect failed: {:?}", e);
            } else {
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

    // Initialize sensor reading channel for inter-task communication
    // Must be done before wifi block so both wifi and non-wifi paths can use it
    let sensor_channel = SENSOR_CHANNEL
        .try_init_with(Channel::new)
        .unwrap_or_else(|| {
            use alloc::boxed::Box;
            Box::leak(Box::new(Channel::new()))
        });
    let sensor_sender = sensor_channel.sender();
    let sensor_receiver = sensor_channel.receiver();

    if !ssid.is_empty() {
        match esp_radio::init() {
            Ok(radio_init) => {
                use alloc::boxed::Box;
                let radio_init: &'static _ = Box::leak(Box::new(radio_init));

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

                // --- embassy-net Stack Initialization (MQTT v5 Feature) ---
                // embassy-net provides a high-level async network stack wrapping smoltcp.
                // It handles TCP/IP, DHCP, DNS, and socket management with an async API.
                // This is required for rust-mqtt v0.3 MQTT v5 client integration.
                //
                // Architecture:
                // - Stack: High-level API for creating sockets, DNS queries, etc.
                // - Runner: Background task that processes packets, timers, and state machines
                // - Device: esp-radio's Wi-Fi device (ifaces.sta) provides link layer
                //
                // The Stack and Runner are split to allow the Runner to run in a separate
                // task while the Stack handle can be shared across multiple tasks.
                #[cfg(feature = "mqtt")]
                let stack = {
                    // Initialize stack resources (3 sockets: DHCP, DNS, MQTT)
                    // StackResources allocates memory for socket buffers and state.
                    // Must be stored in static memory for 'static lifetime requirement.
                    let resources = STACK_RESOURCES.init(StackResources::new());

                    // Create embassy-net stack with DHCP configuration
                    // - Device: ifaces.sta (Wi-Fi STA device from esp-radio)
                    // - Config: DHCP v4 for automatic IP assignment
                    // - Resources: Socket buffers and state
                    // - Seed: Timestamp seed for random number generation
                    //
                    // Returns (Stack, Runner) tuple:
                    // - Stack: Handle for creating sockets, DNS queries (can be cloned/shared)
                    // - Runner: Must be spawned as a task to process packets
                    let (stack, runner) = embassy_net::new(
                        ifaces.sta,
                        NetConfig::dhcpv4(Default::default()),
                        resources,
                        embassy_time::Instant::MIN.as_millis(),
                    );

                    // Store stack in static cell for 'static lifetime
                    // The stack handle will be passed to network_task and mqtt_connection_task
                    let stack = NET_STACK.init(stack);

                    // Spawn the runner task to handle packet processing
                    // The runner must run continuously to:
                    // - Process incoming/outgoing packets
                    // - Handle DHCP lease acquisition and renewal
                    // - Process DNS queries
                    // - Manage TCP connection state machines
                    // - Handle socket timeouts and retransmissions
                    spawner.spawn(embassy_net_task(runner)).ok();
                    info!("network: embassy-net stack initialized with DHCP");

                    stack
                };

                #[cfg(feature = "mqtt")]
                spawner.spawn(network_task(wifi, client, stack)).ok();

                #[cfg(not(feature = "mqtt"))]
                spawner.spawn(network_task(wifi, client)).ok();

                info!("wifi: network task spawned, waiting for connection...");

                #[cfg(feature = "mqtt")]
                spawner
                    .spawn(mqtt_connection_task(stack, sensor_receiver))
                    .ok();

                #[cfg(not(feature = "mqtt"))]
                spawner.spawn(mqtt_connection_task()).ok();
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
    let a0 = adc1_cfg.enable_pin(peripherals.GPIO0, Attenuation::_6dB);
    let adc1 = Adc::new(peripherals.ADC1, adc1_cfg);

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
