//! UDP: A `UdpBind` trait implementation for `embassy-net`

/// Re-export the `edge-nal-embassy` crate
pub use edge_nal_embassy::*;

/// Re-export the `embassy_net` crate
pub mod net {
    pub use ::embassy_net::*;
}
