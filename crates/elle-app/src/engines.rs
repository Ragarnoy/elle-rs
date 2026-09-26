//! Engine command output from the control loops to the DShot task.

use elle_hardware::dshot::DSHOT_THROTTLE;
use elle_system::FlightController;

/// Publish the controller's engine output as eRPM targets for the DShot task
/// (the governor converts eRPM back to DShot). Both targets are always sent; the
/// single-engine task reads only the left one.
pub fn publish_engine_output(fc: &FlightController<'_>) {
    let (engine_l, engine_r) = fc.engine_output();
    let to_erpm = |dshot: u16| {
        (u32::from(dshot) * elle_config::MAX_ERPM) / u32::from(elle_config::DSHOT_THROTTLE_MAX)
    };
    DSHOT_THROTTLE.signal((to_erpm(engine_l), to_erpm(engine_r)));
}
