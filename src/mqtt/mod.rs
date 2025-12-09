//! MQTT module: client abstraction and Home Assistant discovery helpers.
//! This module groups MQTT-related components behind feature flags.

pub mod client;
pub mod discovery;

// Re-exports for cleaner imports from crate::mqtt
pub use client::{LoggerPublisher, MqQos, MqttPublish};
pub use discovery::{build_availability_topic, build_state_topic, publish_discovery};

#[cfg(feature = "mqtt")]
pub use client::{
    EmbassyNetTransport, MqttClientConfig, RustMqttPublisher, init_rust_mqtt_client,
    interpret_connack_reason,
};
