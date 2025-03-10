//! Network interface: `OtNetif` - a `Netif` trait implementation for `openthread`
//! mDNS impl: `OtMdns` - an mDNS trait implementation for `openthread` using Thread SRP

use core::net::{Ipv4Addr, Ipv6Addr};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;

use openthread::{
    OpenThread, OtError, OtSrpResources, OtUdpResources, SrpConf, SrpService, SrpServiceId,
};

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::mdns::ServiceMode;
use rs_matter_stack::matter::transport::network::MAX_RX_PACKET_SIZE;
use rs_matter_stack::mdns::MatterMdnsServices;
use rs_matter_stack::netif::{Netif, NetifConf};

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
const OT_MAX_SOCKETS: usize = 1;

const OT_MAX_SRP_RECORDS: usize = 4;
const OT_SRP_BUF_SZ: usize = 128;

/// A type alias for the `OtUdpResources` type configured with the minimum number of UDP socket buffers
/// sufficient for the operation of the Matter stack
pub type OtMatterUdpResources = OtUdpResources<OT_MAX_SOCKETS, MAX_RX_PACKET_SIZE>;

/// A type alias for the `OtSrpResources` type configured with the minimum number of SRP record buffers
/// sufficient for the operation of the Matter stack
pub type OtMatterSrpResources = OtSrpResources<OT_MAX_SRP_RECORDS, OT_SRP_BUF_SZ>;

/// A `Netif` trait implementation for `openthread`
pub struct OtNetif<'d>(OpenThread<'d>);

impl<'d> OtNetif<'d> {
    /// Create a new `OtNetif` instance
    pub const fn new(ot: OpenThread<'d>) -> Self {
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
        ot.srp_set_conf(&SrpConf {
            host_name: "todo",
            ..Default::default()
        })?;

        ot.srp_autostart()?;

        Ok(Self { ot, services })
    }

    pub async fn run(&self) -> Result<(), OtError> {
        loop {
            self.services.broadcast_signal().wait().await;

            // TODO: Not very efficient to remove and re-add everything

            while let Some(service_id) = self.get_one()? {
                self.ot.srp_remove_service(service_id, true)?;
            }

            self.services
                .visit_services(|matter_mode, matter_service| {
                    self.ot
                        .srp_add_service(&SrpService {
                            name: matter_service.name,
                            instance_name: if matches!(matter_mode, ServiceMode::Commissioned) {
                                "_matter._tcp"
                            } else {
                                "_matterc._udp"
                            },
                            port: matter_service.port,
                            subtype_labels: matter_service.service_subtypes.iter().cloned(),
                            txt_entries: matter_service
                                .txt_kvs
                                .iter()
                                .cloned()
                                .map(|(k, v)| (k, v.as_bytes())),
                            priority: 0,
                            weight: 0,
                            lease_secs: 0,
                            key_lease_secs: 0,
                        })
                        .unwrap(); // TODO

                    Ok(())
                })
                .unwrap();
        }
    }

    fn get_one(&self) -> Result<Option<SrpServiceId>, OtError> {
        let mut id = None;
        self.ot.srp_services(|service| {
            if let Some((_, _, service_id)) = service {
                id = Some(service_id);
            }
        })?;

        Ok(id)
    }
}
