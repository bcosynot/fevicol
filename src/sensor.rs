//! Moisture sensor module
//!
//! This module contains the moisture sensor logic for resistive soil moisture sensors.
//! It includes calibration constants, ADC reading conversion, and the sensor task.

use defmt::{error, info, warn};
use embassy_executor::task;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::analog::adc::{Adc, AdcPin};

// Type aliases for moisture sensor task parameters
pub type MoistureAdc = Adc<'static, esp_hal::peripherals::ADC1<'static>, esp_hal::Blocking>;
pub type MoistureAdcPin =
    AdcPin<esp_hal::peripherals::GPIO0<'static>, esp_hal::peripherals::ADC1<'static>>;

// Calibration constants from sensor calibration routine
pub const SENSOR_DRY: u16 = 2188; // ADC value when sensor is in air (0% moisture)
pub const SENSOR_WET: u16 = 4095; // ADC value when sensor is in water (100% moisture)

// Watering threshold: trigger pump when moisture falls below this percentage
pub const MOISTURE_THRESHOLD: u8 = 30; // 30% moisture

/// Sensor reading data structure for inter-task communication
#[derive(Clone, Copy, defmt::Format)]
pub struct SensorReading {
    /// Moisture percentage (0-100%)
    pub moisture: u8,
    /// Raw ADC value (0-4095)
    pub raw: u16,
    /// Timestamp in milliseconds since boot
    pub timestamp: u64,
}

/// Convert raw ADC reading to moisture percentage (0-100%)
pub fn raw_to_moisture_percent(raw: u16) -> u8 {
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

/// Moisture sensor task: reads ADC periodically and sends readings to MQTT publisher
#[task]
pub async fn moisture_sensor_task(
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
