//! Network interface: `OtNetif` - a `Netif` trait implementation for `openthread`

use core::net::{Ipv4Addr, Ipv6Addr};

use openthread::{OpenThread, OtSrpResources, OtUdpResources};

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::transport::network::MAX_RX_PACKET_SIZE;
use rs_matter_stack::netif::{Netif, NetifConf};

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
const MAX_SOCKETS: usize = 1;

const MAX_SRP_RECORDS: usize = 4;
const SRP_BUF_SZ: usize = 128;

/// A type alias for the `OtUdpResources` type configured with the minimum number of UDP socket buffers
/// sufficient for the operation of the Matter stack
pub type MatterUdpResources = OtUdpResources<MAX_SOCKETS, MAX_RX_PACKET_SIZE>;

/// A type alias for the `OtSrpResources` type configured with the minimum number of SRP record buffers
/// sufficient for the operation of the Matter stack
pub type MatterSrpResources = OtSrpResources<MAX_SRP_RECORDS, SRP_BUF_SZ>;

/// A `Netif` trait implementation for `openthread`
pub struct OtNetif<'d>(OpenThread<'d>);

impl<'d> OtNetif<'d> {
    /// Create a new `OtNetif` instance
    pub fn new(ot: OpenThread<'d>) -> Self {
        Self(ot)
    }

    fn get_conf(&self) -> Option<NetifConf> {
        let mut ipv6 = Ipv6Addr::UNSPECIFIED;

        let _ = self.0.ipv6_addrs(|addr| {
            if let Some((addr, _)) = addr {
                ipv6 = addr;
            }

            Ok(())
        });

        let conf = NetifConf {
            ipv4: Ipv4Addr::UNSPECIFIED,
            ipv6,
            interface: 0,
            mac: [0, 0, 0, 0, 0, 0], // TODO
        };

        Some(conf)
    }

    async fn wait_conf_change(&self) {
        self.0.wait_changed().await;
    }
}

impl Netif for OtNetif<'_> {
    async fn get_conf(&self) -> Result<Option<NetifConf>, Error> {
        Ok(OtNetif::get_conf(self))
    }

    async fn wait_conf_change(&self) -> Result<(), Error> {
        OtNetif::wait_conf_change(self).await;

        Ok(())
    }
}
