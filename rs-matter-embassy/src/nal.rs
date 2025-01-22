//! UDP: A `UdpBind` trait implementation for `embassy-net`

use core::net::Ipv6Addr;

/// Re-export the `edge-nal-embassy` crate
pub use edge_nal_embassy::*;

use embassy_net::driver::{Driver, HardwareAddress};
use embassy_net::{Config, ConfigV6, Ipv6Cidr, Runner, Stack, StackResources, StaticConfigV6};

use rs_matter_stack::matter::transport::network::{MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE};

/// Re-export the `embassy_net` crate
pub mod net {
    pub use ::embassy_net::*;
}

/// The minimum number of sockets that should be configured in the `embassy-net` `StackResources`:
/// The two UDP sockets used by the Matter stack, plus extra 2 for DHCP + DNS
pub const MIN_SOCKET_SET: usize = MAX_SOCKETS + 2;

/// A type alias for the `UdpBuffers` type configured with the minimum number of UDP socket buffers
/// sufficient for the operation of the Matter stack
pub type MatterUdpBuffers =
    UdpBuffers<MAX_SOCKETS, MAX_TX_PACKET_SIZE, MAX_RX_PACKET_SIZE, MAX_META_DATA>;

/// A type alias for the `StackResources` type configured with the minimum number of sockets
/// sufficient for the operation of the Matter stack
pub type MatterStackResources = StackResources<MIN_SOCKET_SET>;

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
/// - Another, for the UDP socket used by the mDNS responder
const MAX_SOCKETS: usize = 2;
/// The max number of meta data buffers that the Matter stack would use
const MAX_META_DATA: usize = 4;

/// Create an `embassy-net` stack suitable for the `rs-matter` stack
pub fn create_net_stack<'d, const N: usize, D: Driver>(
    driver: D,
    seed: u64,
    resources: &'d mut StackResources<N>,
) -> (Stack<'d>, Runner<'d, D>) {
    let config = create_net_config(&driver);

    net::new(driver, config, resources, seed)
}

/// Create a `Config` instance suitable for the `rs-matter` stack:
/// - Ipv6 enabled with a static configuration that uses the link-local address derived from the MAC address
/// - Ipv4 enabled with DHCPv4; structly speaking this is not necessary for the Matter stack, but it is
///   useful in that the `rs-matter` mDNS responder would also answer ipv4 queries
pub fn create_net_config<D: Driver>(driver: &D) -> Config {
    let HardwareAddress::Ethernet(mac) = driver.hardware_address() else {
        unreachable!();
    };

    let mut config = Config::dhcpv4(Default::default());
    config.ipv6 = ConfigV6::Static(StaticConfigV6 {
        address: Ipv6Cidr::new(create_link_local_ipv6(&mac), 10),
        gateway: None,
        dns_servers: heapless::Vec::new(),
    });

    config
}

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
