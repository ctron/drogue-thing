use serde::Serialize;

/// Data structure to publish
#[derive(Debug, Serialize)]
pub struct Data {
    #[serde(rename = "temp")]
    pub temperature: f64,
}

/// Mock measurement
pub fn measure() -> Result<Data, ()> {
    Ok(Data { temperature: 1.2 })
}
