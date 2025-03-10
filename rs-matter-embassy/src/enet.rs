//! Network interface: `EnetNetif` - a `Netif` trait implementation for `embassy-net`
//! UDP: A `UdpBind` trait implementation for `embassy-net`

use core::cell::Cell;
use core::net::Ipv6Addr;

use embassy_futures::select::select;
use embassy_net::driver::{Driver, HardwareAddress as DriverHardwareAddress};
use embassy_net::{
    Config, ConfigV6, HardwareAddress, Ipv6Cidr, Runner, Stack, StackResources, StaticConfigV6,
};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Timer};

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::transport::network::{MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE};
use rs_matter_stack::netif::{Netif, NetifConf};

/// Re-export the `edge-nal-embassy` crate
pub use edge_nal_embassy::*;

/// Re-export the `embassy_net` crate
pub mod net {
    pub use ::embassy_net::*;
}

/// The minimum number of sockets that should be configured in the `embassy-net` `StackResources`:
/// The two UDP sockets used by the Matter stack, plus extra 2 for DHCP + DNS
pub const ENET_MIN_SOCKET_SET: usize = ENET_MAX_SOCKETS + 2;

/// A type alias for the `UdpBuffers` type configured with the minimum number of UDP socket buffers
/// sufficient for the operation of the Matter stack
pub type EnetMatterUdpBuffers =
    UdpBuffers<ENET_MAX_SOCKETS, MAX_TX_PACKET_SIZE, MAX_RX_PACKET_SIZE, ENET_MAX_META_DATA>;

/// A type alias for the `StackResources` type configured with the minimum number of sockets
/// sufficient for the operation of the Matter stack
pub type EnetMatterStackResources = StackResources<ENET_MIN_SOCKET_SET>;

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
/// - Another, for the UDP socket used by the mDNS responder
const ENET_MAX_SOCKETS: usize = 2;
/// The max number of meta data buffers that the Matter stack would use
const ENET_MAX_META_DATA: usize = 4;

/// The MAC address used for mDNS multicast queries over IPv4
///
/// Useful with wifi stack implementations (i.e. cyw43) that require explicit
/// allowlisting of the multicast MAC addresses they should be listening on.
pub const MDNS_MULTICAST_MAC_IPV4: [u8; 6] = [0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb];

/// The MAC address used for mDNS multicast queries over IPv6
///
/// Useful with wifi stack implementations (i.e. cyw43) that require explicit
/// allowlisting of the multicast MAC addresses they should be listening on.
pub const MDNS_MULTICAST_MAC_IPV6: [u8; 6] = [0x33, 0x33, 0x00, 0x00, 0x00, 0xfb];

const TIMEOUT_PERIOD_SECS: u8 = 5;

/// A `Netif` trait implementation for `embassy-net`
pub struct EnetNetif<'d> {
    stack: Stack<'d>,
    up: Mutex<CriticalSectionRawMutex, Cell<bool>>,
}

impl<'d> EnetNetif<'d> {
    /// Create a new `EmbassyNetif` instance
    pub const fn new(stack: Stack<'d>) -> Self {
        Self {
            stack,
            up: Mutex::new(Cell::new(false)),
        }
    }

    fn get_conf(&self) -> Option<NetifConf> {
        let v4 = self.stack.config_v4()?;
        let v6 = self.stack.config_v6()?;

        let conf = NetifConf {
            ipv4: v4.address.address(),
            ipv6: v6.address.address(),
            interface: 0,
            mac: {
                #[allow(irrefutable_let_patterns)]
                let HardwareAddress::Ethernet(addr) = self.stack.hardware_address() else {
                    panic!("Invalid hardware address");
                };

                addr.0
            },
        };

        Some(conf)
    }

    async fn wait_conf_change(&self) {
        // Embassy does have a `wait_config_up/down` but no `wait_config_change`
        // Use a timer as a workaround

        if self.up.lock(|up| up.get()) {
            select(
                self.stack.wait_config_down(),
                Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)),
            )
            .await;
        } else {
            select(
                self.stack.wait_config_up(),
                Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _)),
            )
            .await;
        }

        self.up.lock(|up| up.set(self.stack.is_config_up()));
    }
}

impl Netif for EnetNetif<'_> {
    async fn get_conf(&self) -> Result<Option<NetifConf>, Error> {
        Ok(EnetNetif::get_conf(self))
    }

    async fn wait_conf_change(&self) -> Result<(), Error> {
        EnetNetif::wait_conf_change(self).await;

        Ok(())
    }
}

/// Create an `embassy-net` stack suitable for the `rs-matter` stack
pub fn create_enet_stack<const N: usize, D: Driver>(
    driver: D,
    seed: u64,
    resources: &mut StackResources<N>,
) -> (Stack<'_>, Runner<'_, D>) {
    let config = create_enet_config(&driver);

    net::new(driver, config, resources, seed)
}

/// Create a `Config` instance suitable for the `rs-matter` stack:
/// - Ipv6 enabled with a static configuration that uses the link-local address derived from the MAC address
/// - Ipv4 enabled with DHCPv4; structly speaking this is not necessary for the Matter stack, but it is
///   useful in that the `rs-matter` mDNS responder would also answer ipv4 queries
pub fn create_enet_config<D: Driver>(driver: &D) -> Config {
    let DriverHardwareAddress::Ethernet(mac) = driver.hardware_address() else {
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

/// Get the multicast MAC address corresponding to the given IPv6 link-local address.
///
/// Useful with Wifi stack implementations (i.e. cyw43) that require explicit
/// allowlisting of the multicast MAC addresses they should be listening on.
///
/// Note that the provided IP should be a link-local IP (fe80::/10) or else this
/// function would return a bogus result.
pub fn multicast_mac_for_link_local_ipv6(ip: &Ipv6Addr) -> [u8; 6] {
    let mut mac = [0x33, 0x33, 0xff, 0, 0, 0];
    mac[3..].copy_from_slice(&ip.octets()[13..]);

    mac
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
