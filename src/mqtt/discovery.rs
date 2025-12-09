//! Home Assistant MQTT Discovery module.
//!
//! This module provides helpers for publishing Home Assistant MQTT Discovery messages
//! for automatic sensor entity creation. Discovery messages follow the Home Assistant
//! MQTT Discovery protocol with retained messages and device metadata.
//!
//! Reference: https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery

use core::fmt::Write;
use embassy_time::{Duration, Timer};
use heapless::String;

use super::client::{MqQos, MqttPublish};

/// Project version from Cargo.toml for device metadata
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Home Assistant device information (shared across all entities)
pub const DEVICE_NAME: &str = "Fevicol Plant Monitor";
pub const DEVICE_MANUFACTURER: &str = "Fevicol Project";
pub const DEVICE_MODEL: &str = "ESP32-C6 Moisture Sensor";

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
pub fn build_state_topic(device_id: &str, sensor_id: &str, metric: &str) -> String<128> {
    let mut topic = String::new();
    write!(topic, "fevicol/{}/{}/{}", device_id, sensor_id, metric).ok();
    topic
}

/// Build availability topic for device online/offline status
/// Format: fevicol/{device_id}/status
pub fn build_availability_topic(device_id: &str) -> String<64> {
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
pub async fn publish_discovery<C: MqttPublish + ?Sized>(
    client: &mut C,
    device_id: &str,
    sensor_id: &str,
) -> Result<(), C::Err> {
    defmt::info!("mqtt: publishing Home Assistant discovery messages...");

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

    defmt::info!("mqtt: Home Assistant discovery published");
    Ok(())
}
