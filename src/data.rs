use serde::Serialize;

use crate::error::ThingError;
use crate::Bme680;

/// Data structure to publish
#[derive(Debug, Serialize)]
pub struct Data {
    #[serde(rename = "temp")]
    pub temperature: f32,
    #[serde(rename = "hum")]
    pub humidity: f32,
    #[serde(rename = "press")]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pressure: Option<f32>,
    #[serde(rename = "gas")]
    pub gas_resistance: f32,
}

pub fn measure(sensor: &mut Bme680) -> Result<Data, ThingError> {
    let data = sensor.measure_default();

    match data {
        Ok(Some(data)) => Ok(Data {
            temperature: data.temperature,
            humidity: data.humidity,
            pressure: data.pressure,
            gas_resistance: data.gas_resistance,
        }),
        _ => Err(ThingError::FailedToMeasure),
    }
}
