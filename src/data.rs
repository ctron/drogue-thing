use serde::Serialize;

use crate::error::ThingError;
use crate::Bme680;

/// Data structure to publish
#[derive(Debug, Serialize)]
pub struct Data {
    #[serde(rename = "temp")]
    pub temperature: f32,
}

/// Mock measurement
pub fn measure(sensor: &mut Bme680) -> Result<Data, ThingError> {
    let data = sensor.measure_default();

    match data {
        Ok(Some(data)) => Ok(Data {
            temperature: data.temperature,
        }),
        _ => Err(ThingError::FailedToMeasure),
    }
}
