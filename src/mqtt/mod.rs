//! MQTT module: client abstraction and (later) discovery helpers.
//! This module groups MQTT-related components behind feature flags.

pub mod client;

// Re-exports for cleaner imports from crate::mqtt
pub use client::{MqQos, MqttPublish, LoggerPublisher};

#[cfg(feature = "mqtt")]
pub use client::{EmbassyNetTransport, MqttClientConfig, RustMqttPublisher, init_rust_mqtt_client};
