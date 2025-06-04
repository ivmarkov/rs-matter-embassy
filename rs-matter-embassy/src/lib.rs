#![no_std]
#![allow(async_fn_in_trait)]
#![allow(unknown_lints)]
#![allow(renamed_and_removed_lints)]
#![allow(clippy::declare_interior_mutable_const)]
#![allow(clippy::uninlined_format_args)]
#![warn(clippy::large_futures)]
//#![warn(clippy::large_stack_frames)]
#![warn(clippy::large_types_passed_by_value)]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

pub mod ble;
#[cfg(feature = "embassy-net")]
pub mod enet;
pub mod epoch;
pub mod error;
#[cfg(all(feature = "rs-matter-stack", feature = "embassy-net"))]
pub mod eth;
pub mod matter;
#[cfg(feature = "openthread")]
pub mod ot;
#[cfg(feature = "rs-matter-stack")]
pub mod persist;
pub mod rand;
#[cfg(feature = "rs-matter-stack")]
pub mod stack;
pub mod wifi;
#[cfg(feature = "rs-matter-stack")]
pub mod wireless;
