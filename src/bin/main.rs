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
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

#[cfg(feature = "mqtt")]
use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(feature = "mqtt")]
use embassy_net::{Config as NetConfig, Stack, StackResources};

use static_cell::StaticCell;

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
// MQTT client abstractions moved to crate module (src/mqtt/client.rs)
// ----------------------------------------------------------------------------

#[cfg(not(feature = "mqtt"))]
use fevicol::mqtt::LoggerPublisher;
#[cfg(not(feature = "mqtt"))]
use fevicol::mqtt::publish_discovery;
#[cfg(feature = "mqtt")]
use fevicol::mqtt::{
    EmbassyNetTransport, MqQos, MqttClientConfig, MqttPublish, build_availability_topic,
    build_state_topic, init_rust_mqtt_client, interpret_connack_reason, publish_discovery,
};
use fevicol::sensor::{
    MOISTURE_THRESHOLD, SENSOR_DRY, SENSOR_WET, SensorReading, moisture_sensor_task,
};

#[cfg(feature = "mqtt")]
use core::fmt::Write;
#[cfg(feature = "mqtt")]
use heapless::String;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

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
// Discovery helpers moved to src/mqtt/discovery.rs

// Signal to notify when network is ready
static NETWORK_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Channel for passing sensor readings from sensor task to MQTT publisher
// Capacity of 20 readings allows buffering ~100 seconds of data during network outages
static SENSOR_CHANNEL: StaticCell<Channel<NoopRawMutex, SensorReading, 20>> = StaticCell::new();

// MQTT state management: signal to notify when MQTT client is connected
static MQTT_CONNECTED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// Connection health flag: set to false when disconnection is detected
// Used by publish task to signal errors and by connection task to detect disconnection
#[cfg(feature = "mqtt")]
static MQTT_CONNECTION_HEALTHY: AtomicBool = AtomicBool::new(false);

// embassy-net stack resources (for rust-mqtt integration)
#[cfg(feature = "mqtt")]
static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
#[cfg(feature = "mqtt")]
static NET_STACK: StaticCell<Stack<'static>> = StaticCell::new();

// ----------------------------------------------------------------------------
// MQTT Session Error Types
// ----------------------------------------------------------------------------

/// Unified error type for MQTT session lifecycle stages
#[cfg(feature = "mqtt")]
#[derive(defmt::Format)]
enum MqttSessionError {
    /// DNS resolution failed (no addresses or query error)
    DnsResolutionFailed,
    /// DNS returned no addresses
    DnsNoAddresses,
    /// Failed to parse broker host as IP address (when DNS fallback is needed)
    InvalidIpAddress,
    /// TCP connection failed
    TcpConnectionFailed,
    /// MQTT CONNECT handshake failed with a reason code
    MqttConnectFailed(rust_mqtt::packet::v5::reason_codes::ReasonCode),
    /// Publishing availability status failed
    AvailabilityPublishFailed,
    /// Publishing discovery messages failed
    DiscoveryPublishFailed,
    /// Publishing telemetry failed (moisture or raw reading)
    TelemetryPublishFailed,
}

// ----------------------------------------------------------------------------
// MQTT Helper Functions
// ----------------------------------------------------------------------------

/// Resolves the MQTT broker hostname to an IPv4 address
///
/// This function attempts DNS resolution and falls back to parsing the hostname
/// as an IP address after 5 consecutive DNS failures.
///
/// **Parameters**:
/// - `stack`: embassy-net stack for DNS queries
/// - `broker_host`: hostname or IP address as a string
/// - `dns_failure_count`: mutable reference to track consecutive DNS failures
///
/// **Returns**:
/// - `Ok(Ipv4Address)`: Successfully resolved or parsed IPv4 address
/// - `Err(MqttSessionError)`: DNS resolution failed and fallback unsuccessful
///
/// **Error Handling**:
/// - DNS query errors: Returns `DnsResolutionFailed`
/// - Empty DNS results: Returns `DnsNoAddresses`
/// - Invalid IP address (when fallback triggered): Returns `InvalidIpAddress`
///
/// **DNS Fallback Strategy**:
/// After 5 consecutive failures, attempts to parse `broker_host` as an IP address.
/// This allows operation when DNS is unavailable but an IP is configured.
#[cfg(feature = "mqtt")]
async fn resolve_broker_address(
    stack: &Stack<'static>,
    broker_host: &str,
    dns_failure_count: &mut u8,
) -> Result<smoltcp::wire::Ipv4Address, MqttSessionError> {
    const DNS_FAILURE_THRESHOLD: u8 = 5;

    info!("mqtt: resolving broker hostname '{}'...", broker_host);

    match stack
        .dns_query(broker_host, embassy_net::dns::DnsQueryType::A)
        .await
    {
        Ok(addrs) => {
            if addrs.is_empty() {
                error!("mqtt: DNS resolution returned no addresses");
                *dns_failure_count = dns_failure_count.saturating_add(1);

                // After repeated DNS failures, attempt to parse as IP address
                if *dns_failure_count >= DNS_FAILURE_THRESHOLD {
                    warn!(
                        "mqtt: DNS failed {} times, attempting to parse '{}' as IP address",
                        dns_failure_count, broker_host
                    );
                    match broker_host.parse::<smoltcp::wire::Ipv4Address>() {
                        Ok(ip) => {
                            info!("mqtt: using IP address directly: {}", ip);
                            *dns_failure_count = 0;
                            Ok(ip)
                        }
                        Err(_) => {
                            error!(
                                "mqtt: '{}' is not a valid IP address, cannot fallback",
                                broker_host
                            );
                            Err(MqttSessionError::InvalidIpAddress)
                        }
                    }
                } else {
                    Err(MqttSessionError::DnsNoAddresses)
                }
            } else {
                let addr = addrs[0];
                info!("mqtt: resolved '{}' to {}", broker_host, addr);
                *dns_failure_count = 0;
                let smoltcp::wire::IpAddress::Ipv4(ipv4) = addr;
                Ok(ipv4)
            }
        }
        Err(e) => {
            error!("mqtt: DNS resolution failed: {:?}", defmt::Debug2Format(&e));
            *dns_failure_count = dns_failure_count.saturating_add(1);

            // After repeated DNS failures, attempt to parse as IP address
            if *dns_failure_count >= DNS_FAILURE_THRESHOLD {
                warn!(
                    "mqtt: DNS failed {} times, attempting to parse '{}' as IP address",
                    dns_failure_count, broker_host
                );
                match broker_host.parse::<smoltcp::wire::Ipv4Address>() {
                    Ok(ip) => {
                        info!("mqtt: using IP address directly: {}", ip);
                        *dns_failure_count = 0;
                        Ok(ip)
                    }
                    Err(_) => {
                        error!(
                            "mqtt: '{}' is not a valid IP address, cannot fallback",
                            broker_host
                        );
                        Err(MqttSessionError::InvalidIpAddress)
                    }
                }
            } else {
                Err(MqttSessionError::DnsResolutionFailed)
            }
        }
    }
}

/// Establishes a TCP connection to the MQTT broker
///
/// This function creates a TCP socket and connects it to the specified broker address.
///
/// **Parameters**:
/// - `stack`: embassy-net stack for TCP socket operations
/// - `broker_addr`: resolved IPv4 address of the broker
/// - `broker_port`: TCP port number of the broker
/// - `tcp_rx_buffer`: mutable reference to RX buffer (must outlive the socket)
/// - `tcp_tx_buffer`: mutable reference to TX buffer (must outlive the socket)
///
/// **Returns**:
/// - `Ok(TcpSocket)`: Successfully connected TCP socket
/// - `Err(MqttSessionError)`: TCP connection failed
///
/// **Error Handling**:
/// - Connection failures can occur due to network unreachability, connection refused,
///   or timeout. The error is logged with details and wrapped in `TcpConnectionFailed`.
///
/// **Configuration**:
/// - Timeout: 10 seconds for connection establishment
#[cfg(feature = "mqtt")]
async fn establish_tcp_connection<'a>(
    stack: &'a Stack<'static>,
    broker_addr: smoltcp::wire::Ipv4Address,
    broker_port: u16,
    tcp_rx_buffer: &'a mut [u8],
    tcp_tx_buffer: &'a mut [u8],
) -> Result<embassy_net::tcp::TcpSocket<'a>, MqttSessionError> {
    let mut tcp_socket = embassy_net::tcp::TcpSocket::new(*stack, tcp_rx_buffer, tcp_tx_buffer);
    tcp_socket.set_timeout(Some(Duration::from_secs(10)));

    let remote_endpoint = (broker_addr, broker_port);
    info!("mqtt: connecting TCP to {}:{}...", broker_addr, broker_port);

    match tcp_socket.connect(remote_endpoint).await {
        Ok(()) => {
            info!("mqtt: TCP connected");
            Ok(tcp_socket)
        }
        Err(e) => {
            // TCP connection failures can occur due to:
            // - Network unreachable (Wi-Fi disconnected, routing issues)
            // - Connection refused (broker not running, firewall blocking)
            // - Timeout (broker overloaded, network congestion)
            error!("mqtt: TCP connection failed: {:?}", defmt::Debug2Format(&e));
            Err(MqttSessionError::TcpConnectionFailed)
        }
    }
}

/// Connects to the MQTT broker and publishes initial availability/discovery messages
///
/// This function performs the MQTT CONNECT handshake, publishes availability as "online",
/// and publishes Home Assistant discovery messages.
///
/// **Parameters**:
/// - `tcp_socket`: connected TCP socket
/// - `mqtt_username`: MQTT username for authentication (can be empty, must have lifetime 'a)
/// - `mqtt_password`: MQTT password for authentication (can be empty, must have lifetime 'a)
/// - `lwt_topic`: Last Will Testament topic (must have lifetime 'a)
/// - `mqtt_recv_buffer`: mutable reference to MQTT receive buffer
/// - `mqtt_write_buffer`: mutable reference to MQTT write buffer
///
/// **Returns**:
/// - `Ok(RustMqttPublisher)`: Successfully connected MQTT client ready for telemetry
/// - `Err(MqttSessionError)`: Connection, availability, or discovery publishing failed
///
/// **Error Handling**:
/// - MQTT CONNECT failures: Returns `MqttConnectFailed` with reason code
/// - Availability publish failures: Returns `AvailabilityPublishFailed`
/// - Discovery publish failures: Returns `DiscoveryPublishFailed`
///
/// **Side Effects**:
/// - Sets `MQTT_CONNECTION_HEALTHY` atomic flag to true on successful connection
/// - Signals `MQTT_CONNECTED` to notify other tasks (set to false on error)
#[cfg(feature = "mqtt")]
async fn connect_mqtt_client<'a>(
    tcp_socket: embassy_net::tcp::TcpSocket<'a>,
    mqtt_username: &'a str,
    mqtt_password: &'a str,
    lwt_topic: &'a str,
    mqtt_recv_buffer: &'a mut [u8],
    mqtt_write_buffer: &'a mut [u8],
) -> Result<fevicol::mqtt::RustMqttPublisher<'a, EmbassyNetTransport<'a>>, MqttSessionError> {
    let transport = EmbassyNetTransport::new(tcp_socket);

    let lwt_payload = b"offline";

    let mqtt_config = MqttClientConfig {
        client_id: DEVICE_ID,
        keep_alive_secs: MQTT_KEEP_ALIVE_SECS,
        session_expiry_secs: MQTT_SESSION_EXPIRY_SECS,
        username: mqtt_username,
        password: mqtt_password,
        lwt_topic,
        lwt_payload,
        lwt_retain: true,
    };

    // Stage 5: MQTT CONNECT handshake
    let mut client =
        init_rust_mqtt_client(transport, mqtt_config, mqtt_recv_buffer, mqtt_write_buffer)
            .await
            .map_err(|e| {
                let reason_str = interpret_connack_reason(&e);
                error!(
                    "mqtt: MQTT connection failed - reason: {} ({:?})",
                    reason_str,
                    defmt::Debug2Format(&e)
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

                MqttSessionError::MqttConnectFailed(e)
            })?;

    info!("mqtt: connected successfully (MQTT v5)");
    MQTT_CONNECTION_HEALTHY.store(true, Ordering::Relaxed);

    // Stage 6: Publish availability status (online)
    info!("mqtt: publishing availability status (online)...");
    let availability_topic = build_availability_topic(DEVICE_ID);
    client
        .publish(
            availability_topic.as_str(),
            b"online",
            MqQos::AtLeastOnce,
            true,
        )
        .await
        .map_err(|e| {
            error!("mqtt: availability publish failed: {:?}", e);
            MQTT_CONNECTION_HEALTHY.store(false, Ordering::Relaxed);
            MQTT_CONNECTED.signal(false);
            MqttSessionError::AvailabilityPublishFailed
        })?;

    // Stage 7: Publish Home Assistant discovery messages
    info!("mqtt: publishing discovery messages...");
    publish_discovery(&mut client, DEVICE_ID, SENSOR_ID)
        .await
        .map_err(|e| {
            error!("mqtt: discovery publish failed: {:?}", e);
            MQTT_CONNECTION_HEALTHY.store(false, Ordering::Relaxed);
            MQTT_CONNECTED.signal(false);
            MqttSessionError::DiscoveryPublishFailed
        })?;

    MQTT_CONNECTED.signal(true);
    info!("mqtt: ready for telemetry publishing");

    Ok(client)
}

/// Runs the telemetry publishing loop
///
/// This function receives sensor readings from a channel and publishes them via MQTT.
/// It also performs periodic health checks to detect connection issues.
///
/// **Parameters**:
/// - `client`: mutable reference to connected MQTT client
/// - `sensor_receiver`: channel receiver for sensor readings
///
/// **Returns**:
/// - `Ok(())`: Loop exited normally (should not happen in normal operation)
/// - `Err(MqttSessionError)`: Publishing failed, connection should be reestablished
///
/// **Error Handling**:
/// - Payload formatting errors: logged and skipped (continue loop)
/// - MQTT publish errors: Returns `TelemetryPublishFailed` to trigger reconnection
///
/// **Side Effects**:
/// - Sets `MQTT_CONNECTION_HEALTHY` to false on publish failure
///
/// **Loop Behavior**:
/// - Uses `embassy_futures::select` to handle both sensor readings and 30s health check timeout
/// - On sensor reading: publishes moisture and raw ADC values
/// - On timeout: continues (keepalive handled by rust-mqtt internally)
#[cfg(feature = "mqtt")]
async fn run_telemetry_loop(
    client: &mut fevicol::mqtt::RustMqttPublisher<'_, EmbassyNetTransport<'_>>,
    sensor_receiver: &embassy_sync::channel::Receiver<'_, NoopRawMutex, SensorReading, 20>,
) -> Result<(), MqttSessionError> {
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

                let moisture_topic = build_state_topic(DEVICE_ID, SENSOR_ID, "moisture");
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
                    return Err(MqttSessionError::TelemetryPublishFailed);
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
                    return Err(MqttSessionError::TelemetryPublishFailed);
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

    #[cfg(feature = "local_secrets")]
    let (mqtt_username, mqtt_password) = (LOCAL_MQTT_USERNAME, LOCAL_MQTT_PASSWORD);
    #[cfg(not(feature = "local_secrets"))]
    let (mqtt_username, mqtt_password) = ("", "");

    #[cfg(feature = "local_secrets")]
    let (mqtt_broker_host, mqtt_broker_port) = (LOCAL_MQTT_BROKER_HOST, LOCAL_MQTT_BROKER_PORT);
    #[cfg(not(feature = "local_secrets"))]
    let (mqtt_broker_host, mqtt_broker_port) = ("192.168.0.245", 1883u16); // Default fallback values

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

        loop {
            info!("mqtt: attempting connection...");

            // Stage 2: DNS resolution with fallback to IP parsing
            let broker_addr =
                match resolve_broker_address(stack, mqtt_broker_host, &mut dns_failure_count).await
                {
                    Ok(addr) => addr,
                    Err(e) => {
                        error!(
                            "mqtt: broker address resolution failed: {:?}, retrying in {}s",
                            e, backoff_secs
                        );
                        Timer::after(Duration::from_secs(backoff_secs)).await;
                        backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                        continue;
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

            // Stage 4: Establish TCP connection
            let tcp_socket = match establish_tcp_connection(
                stack,
                broker_addr,
                mqtt_broker_port,
                &mut tcp_rx_buffer,
                &mut tcp_tx_buffer,
            )
            .await
            {
                Ok(socket) => socket,
                Err(e) => {
                    error!(
                        "mqtt: TCP connection failed: {:?}, retrying in {}s",
                        e, backoff_secs
                    );
                    Timer::after(Duration::from_secs(backoff_secs)).await;
                    backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                    continue;
                }
            };

            // Stage 5: Connect MQTT client, publish availability and discovery
            let lwt_topic = build_availability_topic(DEVICE_ID);
            let mut client = match connect_mqtt_client(
                tcp_socket,
                mqtt_username,
                mqtt_password,
                lwt_topic.as_str(),
                &mut mqtt_recv_buffer,
                &mut mqtt_write_buffer,
            )
            .await
            {
                Ok(client) => {
                    backoff_secs = 2;
                    client
                }
                Err(e) => {
                    error!(
                        "mqtt: client connection failed: {:?}, retrying in {}s",
                        e, backoff_secs
                    );
                    Timer::after(Duration::from_secs(backoff_secs)).await;
                    backoff_secs = (backoff_secs * 2).min(MAX_BACKOFF_SECS);
                    continue;
                }
            };

            // Stage 6: Run telemetry publishing loop
            let _ = run_telemetry_loop(&mut client, &sensor_receiver).await;

            // If we exit from the telemetry loop, attempt to publish offline status
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

    #[cfg(feature = "local_secrets")]
    let (ssid, pass) = (LOCAL_SSID, LOCAL_PASS);
    #[cfg(not(feature = "local_secrets"))]
    let (ssid, pass) = (
        option_env!("WIFI_SSID").unwrap_or(""),
        option_env!("WIFI_PASS").unwrap_or(""),
    );

    // Initialize sensor reading channel for inter-task communication
    // Must be done before wifi block so both wifi and non-wifi paths can use it
    let sensor_channel = SENSOR_CHANNEL
        .try_init_with(Channel::new)
        .unwrap_or_else(|| {
            use alloc::boxed::Box;
            Box::leak(Box::new(Channel::new()))
        });
    let sensor_sender = sensor_channel.sender();
    #[cfg(feature = "mqtt")]
    let sensor_receiver = sensor_channel.receiver();

    if !ssid.is_empty() {
        match esp_radio::init() {
            Ok(radio_init) => {
                use alloc::boxed::Box;
                let radio_init: &'static _ = Box::leak(Box::new(radio_init));

                let wifi_cfg = esp_radio::wifi::Config::default();
                #[cfg(feature = "mqtt")]
                let (wifi, ifaces) =
                    match esp_radio::wifi::new(radio_init, peripherals.WIFI, wifi_cfg) {
                        Ok(v) => v,
                        Err(e) => {
                            error!("wifi new() failed: {:?}", e);
                            panic!("wifi initialization failed");
                        }
                    };

                #[cfg(not(feature = "mqtt"))]
                let (wifi, _ifaces) =
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
