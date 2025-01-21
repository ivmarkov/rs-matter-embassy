//! Epoch: an `epoch` function implementation based on `embassy-time`

/// Get the current epoch time
pub fn epoch() -> core::time::Duration {
    core::time::Duration::from_millis(embassy_time::Instant::now().as_millis())
}
