//! Network interface: `OtNetif` - a `Netif` trait implementation for `openthread`
//! mDNS impl: `OtMdns` - an mDNS trait implementation for `openthread` using Thread SRP

use core::fmt::Write;
use core::net::{Ipv4Addr, Ipv6Addr};

use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

use embassy_time::{Duration, Timer};
use log::info;

use openthread::{
    OpenThread, OtError, OtResources, OtSrpResources, OtUdpResources, SrpConf, SrpService,
};

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::mdns::ServiceMode;
use rs_matter_stack::matter::transport::network::MAX_RX_PACKET_SIZE;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::mdns::MatterMdnsServices;
use rs_matter_stack::netif::{Netif, NetifConf};

/// Re-export the `openthread` crate
pub mod openthread {
    pub use openthread::*;
}

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
const OT_MAX_SOCKETS: usize = 1;

const OT_MAX_SRP_RECORDS: usize = 4;
const OT_SRP_BUF_SZ: usize = 512;

/// A struct that holds all the resources required by the OpenThread stack,
/// as used by Matter.
pub struct OtMatterResources {
    /// The OpenThread main resources
    pub ot: OtResources,
    /// The OpenThread UDP resources
    pub udp: OtUdpResources<OT_MAX_SOCKETS, MAX_RX_PACKET_SIZE>,
    /// The OpenThread SRP resources
    pub srp: OtSrpResources<OT_MAX_SRP_RECORDS, OT_SRP_BUF_SZ>,
}

impl OtMatterResources {
    /// Create a new `OtMatterResources` instance
    pub const fn new() -> Self {
        Self {
            ot: OtResources::new(),
            udp: OtUdpResources::new(),
            srp: OtSrpResources::new(),
        }
    }

    /// Return an in-place initializer for `OtMatterResources`
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ot: OtResources::new(),
            udp: OtUdpResources::new(),
            srp: OtSrpResources::new(),
        })
    }
}

impl Default for OtMatterResources {
    fn default() -> Self {
        Self::new()
    }
}

/// A `Netif` trait implementation for `openthread`
pub struct OtNetif<'d>(OpenThread<'d>);

impl<'d> OtNetif<'d> {
    /// Create a new `OtNetif` instance
    pub const fn new(ot: OpenThread<'d>) -> Self {
        Self(ot)
    }

    fn get_conf(&self) -> Option<NetifConf> {
        let status = self.0.net_status();

        if status.ip6_enabled && status.role.is_connected() {
            let mut ipv6 = Ipv6Addr::UNSPECIFIED;

            let _ = self.0.ipv6_addrs(|addr| {
                if let Some((addr, prefix)) = addr {
                    info!("Got addr {addr}/{prefix}");

                    if ipv6.is_unspecified()
                        || ipv6.is_unicast_link_local()
                        || ipv6.is_unique_local()
                            && !addr.is_unicast_link_local()
                            && !addr.is_unique_local()
                    {
                        // Do not prefer link-local and unique-local addresses as they are not addressable in the Thread mesh
                        ipv6 = addr;
                    }
                }

                Ok(())
            });

            self.0
                .srp_conf(|conf, state, empty| {
                    info!("SRP conf: {conf:?}, state: {state}, empty: {empty}");

                    Ok(())
                })
                .unwrap();

            self.0
                .srp_services(|service| {
                    if let Some((service, state, slot)) = service {
                        info!("SRP service: {service}, state: {state}, slot: {slot}");
                    }
                })
                .unwrap();

            let conf = NetifConf {
                ipv4: Ipv4Addr::UNSPECIFIED,
                ipv6,
                interface: 0,
                // TODO: Fix this in `rs-matter-stack`
                mac: self.0.ieee_eui64()[..6].try_into().unwrap(),
            };

            Some(conf)
        } else {
            None
        }
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

pub struct OtMdns<'d> {
    ot: OpenThread<'d>,
    services: &'d MatterMdnsServices<'d, NoopRawMutex>,
}

impl<'d> OtMdns<'d> {
    /// Create a new `OtMdns` instance
    pub fn new(
        ot: OpenThread<'d>,
        services: &'d MatterMdnsServices<'d, NoopRawMutex>,
    ) -> Result<Self, OtError> {
        Ok(Self { ot, services })
    }

    pub async fn run(&self) -> Result<(), OtError> {
        self.services.broadcast_signal().reset();

        loop {
            // TODO: Not very efficient to remove and re-add everything
            let _ = self.ot.srp_remove_all(false);

            // TODO: Something is still not quite right with the SRP
            // We seem to get stuck here
            while !self.ot.srp_is_empty()? {
                info!("Waiting for SRP records to be removed...");
                select(
                    Timer::after(Duration::from_secs(1)),
                    self.ot.srp_wait_changed(),
                )
                .await;
            }

            let ieee_eui64 = self.ot.ieee_eui64();

            let mut hostname = heapless::String::<16>::new();
            write!(
                hostname,
                "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                ieee_eui64[0],
                ieee_eui64[1],
                ieee_eui64[2],
                ieee_eui64[3],
                ieee_eui64[4],
                ieee_eui64[5],
                ieee_eui64[6],
                ieee_eui64[7]
            )
            .unwrap();

            self.ot.srp_set_conf(&SrpConf {
                host_name: hostname.as_str(),
                ..Default::default()
            })?;

            info!("Registered SRP host {hostname}");

            self.services
                .visit_services(|matter_mode, matter_service| {
                    let name = if matches!(matter_mode, ServiceMode::Commissioned) {
                        "_matter._tcp"
                    } else {
                        "_matterc._udp"
                    };

                    self.ot
                        .srp_add_service(&SrpService {
                            name,
                            instance_name: matter_service.name,
                            port: matter_service.port,
                            subtype_labels: matter_service.service_subtypes.iter().cloned(),
                            txt_entries: matter_service
                                .txt_kvs
                                .iter()
                                .cloned()
                                .filter(|(k, _)| !k.is_empty())
                                .map(|(k, v)| (k, v.as_bytes())),
                            priority: 0,
                            weight: 0,
                            lease_secs: 0,
                            key_lease_secs: 0,
                        })
                        .unwrap(); // TODO

                    info!("Added service {} of type {name}", matter_service.name);

                    Ok(())
                })
                .unwrap();

            self.services.broadcast_signal().wait().await;
        }
    }
}
