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

// MQTT imports - will be used when full TCP integration is implemented
// use rust_mqtt::client::client::MqttClient;
// use rust_mqtt::client::client_config::ClientConfig as MqttClientConfig;
// use rust_mqtt::packet::v5::publish_packet::QualityOfService;
// use rust_mqtt::utils::rng_generator::CountingRng;
// use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

// Optional local secrets support
#[cfg(feature = "local_secrets")]
mod secrets;
#[cfg(feature = "local_secrets")]
use secrets::{WIFI_PASS as LOCAL_PASS, WIFI_SSID as LOCAL_SSID};

extern crate alloc;

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

// Topic alias constants for MQTT 5.0 bandwidth optimization
// Topic aliases map long topic strings to small integers, saving bandwidth
// First publish: full topic + alias property (e.g., "fevicol/fevicol-01/moisture-1/moisture" + alias=1)
// Subsequent publishes: alias only (e.g., alias=1) - saves ~40+ bytes per message
const ALIAS_MOISTURE: u16 = 1;
const ALIAS_RAW: u16 = 2;
// Reserve aliases 3-10 for future sensors

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

// ============================================================================
// MQTT Topic Alias State Tracking
// ============================================================================

/// Tracks whether topic aliases have been established for the current MQTT session
/// Topic aliases are per-session and must be re-established after reconnection
struct TopicAliasState {
    established: bool,
    connection_generation: u32,
}

static TOPIC_ALIAS_STATE: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, TopicAliasState> =
    embassy_sync::mutex::Mutex::new(TopicAliasState {
        established: false,
        connection_generation: 0,
    });

// ============================================================================
// Smoltcp TCP Socket Adapter for embedded-io-async
// ============================================================================
//
// TODO: Implement TCP socket adapter for rust-mqtt integration
//
// The adapter needs to:
// 1. Wrap smoltcp's TCP socket to implement embedded-io-async traits (Read + Write)
// 2. Handle async polling of the smoltcp stack
// 3. Manage socket lifecycle (connect, disconnect, error handling)
//
// Example implementation approach:
//
// struct SmoltcpSocketAdapter<'a> {
//     socket: smoltcp::socket::tcp::Socket<'a>,
//     ifaces: &'a esp_radio::wifi::Interfaces<'a>,
// }
//
// impl<'a> embedded_io_async::ErrorType for SmoltcpSocketAdapter<'a> {
//     type Error = smoltcp::socket::tcp::RecvError; // or custom error type
// }
//
// impl<'a> embedded_io_async::Read for SmoltcpSocketAdapter<'a> {
//     async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
//         // Poll until data available, then read
//     }
// }
//
// impl<'a> embedded_io_async::Write for SmoltcpSocketAdapter<'a> {
//     async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
//         // Poll until can send, then write
//     }
// }
//
// Alternative: Use embedded-nal-async which might provide built-in adapters

/// Publish Home Assistant discovery messages for all sensors
/// This function should be called after successful MQTT connection
///
/// Note: This is currently a placeholder implementation that demonstrates the structure.
/// Full MQTT publishing with rust-mqtt will be implemented in a future step.
/// Discovery messages must be published with retain=true and QoS 0 or 1.
async fn publish_discovery(device_id: &str, sensor_id: &str) {
    info!("mqtt: publishing Home Assistant discovery messages...");

    // Publish availability topic first (online status)
    let availability_topic = build_availability_topic(device_id);
    info!(
        "mqtt: [PLACEHOLDER] would publish to '{}' payload='online' (retain=true)",
        availability_topic.as_str()
    );

    // Small delay between publishes to avoid overwhelming the broker
    Timer::after(Duration::from_millis(100)).await;

    // Publish moisture sensor discovery
    let moisture_topic = build_discovery_topic("sensor", device_id, "moisture");
    let moisture_payload = create_moisture_discovery_payload(device_id, sensor_id);
    info!(
        "mqtt: [PLACEHOLDER] would publish discovery to '{}'",
        moisture_topic.as_str()
    );
    info!("mqtt:   payload length: {} bytes", moisture_payload.len());
    info!("mqtt:   payload: {}", moisture_payload.as_str());

    // Small delay between publishes
    Timer::after(Duration::from_millis(100)).await;

    // Publish raw ADC sensor discovery
    let raw_topic = build_discovery_topic("sensor", device_id, "raw");
    let raw_payload = create_raw_discovery_payload(device_id, sensor_id);
    info!(
        "mqtt: [PLACEHOLDER] would publish discovery to '{}'",
        raw_topic.as_str()
    );
    info!("mqtt:   payload length: {} bytes", raw_payload.len());
    info!("mqtt:   payload: {}", raw_payload.as_str());

    info!("mqtt: Home Assistant discovery complete");

    // TODO: In actual implementation, this function will:
    // 1. Take MQTT client reference as parameter
    // 2. Publish availability topic with "online" (retain=true)
    // 3. Publish moisture discovery config (retain=true, QoS 0 or 1)
    // 4. Publish raw ADC discovery config (retain=true, QoS 0 or 1)
    // 5. Return Result<(), Error> for error handling
    // 6. Set Last Will Testament (LWT) to publish "offline" on disconnect
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
/// Manages connection lifecycle and topic alias reset on reconnection
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

    // Connection loop with generation tracking for alias management
    let mut connection_attempt = 0u32;

    loop {
        connection_attempt += 1;
        info!("mqtt: connection attempt #{}", connection_attempt);

        // TODO: Full TCP/MQTT connection implementation:
        // ==========================================
        // 1. Create TCP socket using esp-radio's Interfaces (smoltcp stack):
        //
        //    let mut socket_set = SocketSet::new(&mut socket_storage);
        //    let tcp_handle = socket_set.add(tcp_socket);
        //    let socket = socket_set.get_mut::<tcp::Socket>(tcp_handle);
        //
        //    // Connect to broker
        //    let remote_endpoint = (MQTT_BROKER_HOST.parse().unwrap(), MQTT_BROKER_PORT);
        //    socket.connect(cx, remote_endpoint, local_port)?;
        //
        // 2. Wrap socket in SmoltcpSocketAdapter (or use embedded-nal-async):
        //
        //    let adapter = SmoltcpSocketAdapter::new(socket, &ifaces);
        //
        // 3. Create rust-mqtt client:
        //
        //    let mut mqtt_config = MqttClientConfig::new(
        //        rust_mqtt::client::client_config::MqttVersion::MQTTv5,
        //        CountingRng(20000)
        //    );
        //    mqtt_config.add_max_subscribe_qos(QualityOfService::QoS1);
        //    mqtt_config.add_client_id(DEVICE_ID);
        //    mqtt_config.keep_alive = MQTT_KEEP_ALIVE_SECS;
        //
        //    // Set Last Will Testament (LWT) - publish "offline" if disconnected
        //    let availability_topic = build_availability_topic(DEVICE_ID);
        //    mqtt_config.add_last_will(
        //        availability_topic.as_str(),
        //        b"offline",
        //        QualityOfService::QoS0,
        //        true  // retain
        //    );
        //
        //    let mut mqtt_client = MqttClient::new(
        //        adapter,
        //        &mut write_buffer,
        //        256,  // write buffer size
        //        &mut recv_buffer,
        //        256,  // receive buffer size
        //        mqtt_config
        //    );
        //
        // 4. Connect to broker:
        //
        //    mqtt_client.connect_to_broker().await?;
        //
        // 5. Publish availability as "online" (with retain):
        //
        //    mqtt_client.publish(
        //        availability_topic.as_str(),
        //        b"online",
        //        QualityOfService::QoS0,
        //        true,  // retain
        //        None   // no alias for availability
        //    ).await?;

        // Increment connection generation to trigger alias re-establishment
        {
            let mut alias_state = TOPIC_ALIAS_STATE.lock().await;
            alias_state.connection_generation = connection_attempt;
            alias_state.established = false;
            info!(
                "mqtt: connection generation updated to {}",
                alias_state.connection_generation
            );
        }

        // Publish Home Assistant discovery messages
        // This should happen after successful MQTT connection
        publish_discovery(DEVICE_ID, SENSOR_ID).await;

        // Signal that MQTT is connected
        MQTT_CONNECTED.signal(true);
        info!(
            "mqtt: simulated connection established (generation {})",
            connection_attempt
        );
        info!("mqtt: topic aliases will be re-established on next publish");

        // For now, just stay "connected" - in real implementation, monitor connection health
        // TODO: Real implementation would:
        // - Monitor connection state
        // - Send PINGREQ for keep-alive
        // - Detect disconnection and reconnect with exponential backoff (2s → 30s max)
        // - Re-publish discovery on reconnection (HA might have restarted)
        // - Reset aliases_established flag when disconnected

        loop {
            Timer::after(Duration::from_secs(60)).await;
            // TODO: Check if still connected, break if disconnected to retry
        }
    }
}

/// MQTT publishing task: receives sensor readings and publishes to broker
/// Implements MQTT 5.0 topic aliases for bandwidth optimization
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

    // Track current connection generation for alias management
    let mut current_generation = 0u32;
    let mut publish_count = 0u32;

    // Build topic strings (used for first publish to establish aliases)
    let moisture_topic = build_state_topic(device_id, sensor_id, "moisture");
    let raw_topic = build_state_topic(device_id, sensor_id, "raw");

    info!("mqtt: moisture topic = '{}'", moisture_topic.as_str());
    info!("mqtt: raw topic = '{}'", raw_topic.as_str());

    loop {
        // Receive sensor reading from channel (blocks until available)
        let reading = receiver.receive().await;

        // Check if we need to re-establish aliases (connection changed)
        let mut alias_state = TOPIC_ALIAS_STATE.lock().await;
        let aliases_established =
            alias_state.established && alias_state.connection_generation == current_generation;

        if !aliases_established {
            // New connection or first publish - establish aliases
            info!(
                "mqtt: establishing topic aliases for connection generation {}",
                alias_state.connection_generation
            );

            // Format payloads
            let mut moisture_payload = String::<16>::new();
            let mut raw_payload = String::<16>::new();
            write!(moisture_payload, "{}", reading.moisture).ok();
            write!(raw_payload, "{}", reading.raw).ok();

            // TODO: Actual MQTT publishing will use rust-mqtt client here
            // First publish with full topic + alias property to establish the mapping:
            //
            // mqtt_client.publish(
            //     &moisture_topic,           // Full topic string
            //     moisture_payload.as_bytes(),
            //     QualityOfService::QoS0,
            //     false,                     // retain = false (or true for last value)
            //     Some(ALIAS_MOISTURE)       // Topic alias property
            // ).await?;
            //
            // This tells the broker: "map this topic to alias 1"

            info!(
                "mqtt: [FIRST PUBLISH] topic='{}' payload='{}' alias={} (establishes alias)",
                moisture_topic.as_str(),
                moisture_payload.as_str(),
                ALIAS_MOISTURE
            );

            info!(
                "mqtt: [FIRST PUBLISH] topic='{}' payload='{}' alias={} (establishes alias)",
                raw_topic.as_str(),
                raw_payload.as_str(),
                ALIAS_RAW
            );

            // Mark aliases as established for this connection
            current_generation = alias_state.connection_generation;
            alias_state.established = true;
            drop(alias_state);

            info!("mqtt: topic aliases established successfully");
        } else {
            // Aliases already established - use alias-only publish (bandwidth optimized)
            // Format payloads
            let mut moisture_payload = String::<16>::new();
            let mut raw_payload = String::<16>::new();
            write!(moisture_payload, "{}", reading.moisture).ok();
            write!(raw_payload, "{}", reading.raw).ok();

            // TODO: Actual MQTT publishing with alias only (no topic string):
            //
            // mqtt_client.publish_with_alias(
            //     ALIAS_MOISTURE,              // Alias only (2 bytes vs ~50 bytes)
            //     moisture_payload.as_bytes(),
            //     QualityOfService::QoS0,
            //     false                        // retain
            // ).await?;
            //
            // This saves ~48 bytes per publish!

            // Only log every 12th publish to reduce spam (once per minute at 5s intervals)
            publish_count += 1;
            if publish_count % 12 == 0 {
                info!(
                    "mqtt: [PUBLISH VIA ALIAS] moisture={}% (alias={}), raw={} (alias={}) - published {} times",
                    reading.moisture, ALIAS_MOISTURE, reading.raw, ALIAS_RAW, publish_count
                );
            }
        }

        // TODO: Error handling for publish failures:
        // - If publish fails due to network: log error, continue (reading is lost but sensor continues)
        // - If publish fails due to protocol error: may need to re-establish connection
        // - Never panic - robustness is key for embedded systems
    }
}

/// Network task: manages Wi-Fi connection
/// Note: esp-radio 0.17.0 provides smoltcp stack integration via Interfaces
/// Future implementation will use Interfaces directly for TCP socket creation
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

    // Signal that network is ready
    // Note: esp-radio's Interfaces provides the smoltcp stack
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
