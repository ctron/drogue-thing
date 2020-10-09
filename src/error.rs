use snafu::Snafu;

#[derive(Debug, Snafu)]
pub enum ThingError {
    #[snafu(display("Initialization failed"))]
    InitializationFailed,
    #[snafu(display("Not (yet) initialized"))]
    NotInitialized,
    #[snafu(display("Failed to publish"))]
    FailedToPublish,
    #[snafu(display("Failed to measure"))]
    FailedToMeasure,
}
