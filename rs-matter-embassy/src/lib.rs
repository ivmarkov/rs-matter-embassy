#![no_std]
#![allow(async_fn_in_trait)]
#![allow(unknown_lints)]
#![allow(renamed_and_removed_lints)]
#![allow(unexpected_cfgs)]
#![allow(clippy::declare_interior_mutable_const)]
#![warn(clippy::large_futures)]
#![warn(clippy::large_stack_frames)]
#![warn(clippy::large_types_passed_by_value)]

use core::net::Ipv6Addr;

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

/// Create a link-local IPv6 address from a MAC address.
pub fn create_link_local_ipv6(mac: &[u8; 6]) -> Ipv6Addr {
    Ipv6Addr::new(
        0xfe80,
        0,
        0,
        0,
        u16::from_be_bytes([mac[0] ^ 0x02, mac[1]]),
        u16::from_be_bytes([mac[2], 0xff]),
        u16::from_be_bytes([0xfe, mac[3]]),
        u16::from_be_bytes([mac[4], mac[5]]),
    )
}

#[cfg(test)]
mod test {
    #[test]
    fn test() {
        assert_eq!(
            super::create_link_local_ipv6(&[0x52, 0x74, 0xf2, 0xb1, 0xa8, 0x7f]).octets(),
            [0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0x50, 0x74, 0xf2, 0xff, 0xfe, 0xb1, 0xa8, 0x7f]
        );
    }
}
