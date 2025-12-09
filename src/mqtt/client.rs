//! MQTT client abstraction and rust-mqtt integration.
//!
//! This file is extracted from src/bin/main.rs (Phase 1 of refactor).
//! It centralizes MQTT-specific types and feature-gated code to reduce
//! cfg-noise in the application entrypoint.

extern crate alloc;

use defmt::info;

// ----------------------------------------------------------------------------
// MQTT publishing abstraction (crate-agnostic)
// ----------------------------------------------------------------------------

/// MQTT QoS mapping for a minimal, crate-agnostic publish interface.
#[derive(Clone, Copy, Debug)]
pub enum MqQos {
    /// QoS 0 — At most once
    AtMostOnce,
    /// QoS 1 — At least once
    AtLeastOnce,
}

/// Minimal MQTT publish trait to decouple app code from a specific client crate.
#[allow(async_fn_in_trait)]
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
    pub(crate) client: MqttClient<'a, T, 5, CountingRng>,
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

        match self.client.send_message(topic, payload, mqtt_qos, retain).await {
            Ok(()) => Ok(()),
            Err(ReasonCode::NoMatchingSubscribers) => {
                // This is a success-message published but no subscribers
                Ok(())
            }
            Err(e) => Err(e),
        }
    }
}

/// Interpret MQTT v5 CONNACK reason codes for better error diagnostics.
/// Returns a human-readable description of the reason code.
#[cfg(feature = "mqtt")]
pub fn interpret_connack_reason(
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
/// Parameters:
/// - `transport`: EmbassyNetTransport wrapping a connected TCP socket
/// - `config`: MQTT client configuration (credentials, keep-alive, LWT, etc.)
/// - `recv_buffer`: Buffer for receiving MQTT packets
/// - `write_buffer`: Buffer for writing MQTT packets
///
/// Returns: A configured RustMqttPublisher ready for publishing messages.
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

    let mut client_config =
        ClientConfig::new(rust_mqtt::client::client_config::MqttVersion::MQTTv5, rng);

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
            defmt::info!("rust-mqtt: Connected to broker");
            Ok(RustMqttPublisher { client })
        }
        Err(e) => {
            defmt::error!(
                "rust-mqtt: Connection failed: {:?}",
                defmt::Debug2Format(&e)
            );
            Err(e)
        }
    }
}
