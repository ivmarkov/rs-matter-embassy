#![no_std]
#![allow(async_fn_in_trait)]
#![allow(unknown_lints)]
#![allow(renamed_and_removed_lints)]
#![allow(clippy::declare_interior_mutable_const)]
#![warn(clippy::large_futures)]
#![warn(clippy::large_stack_frames)]
#![warn(clippy::large_types_passed_by_value)]

#[cfg(feature = "rs-matter-stack")]
pub use eth::*;
pub mod ble;
pub mod epoch;
pub mod error;
#[cfg(feature = "rs-matter-stack")]
pub mod eth;
pub mod matter;
pub mod nal;
pub mod netif;
#[cfg(feature = "rs-matter-stack")]
pub mod persist;
#[cfg(feature = "rs-matter-stack")]
pub mod stack;
#[cfg(feature = "rs-matter-stack")]
pub mod wireless;
