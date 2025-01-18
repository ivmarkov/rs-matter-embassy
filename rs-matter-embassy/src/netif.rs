use embassy_futures::select::select;
use embassy_net::Stack;
use embassy_time::{Duration, Timer};

use rs_matter::error::Error;
use rs_matter_stack::netif::{Netif, NetifConf};

use crate::error::to_net_error;

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
/// - Another, for the UDP socket used by the mDNS responder
pub const MAX_SOCKETS: usize = 2;

pub const MAX_META_DATA: usize = 4;

const TIMEOUT_PERIOD_SECS: u8 = 5;

/// A `Netif` trait implementation for `embassy-net`
pub struct EmbassyNetif<'d> {
    stack: Stack<'d>,
}

impl<'d> EmbassyNetif<'d> {
    /// Create a new `EmbassyNetif` instance
    pub fn new(stack: Stack<'d>) -> Self {
        Self { stack }
    }

    fn get_conf(&self) -> Result<NetifConf, ()> {
        let Some(v4) = self.stack.config_v4() else {
            return Err(());
        };

        let Some(v6) = self.stack.config_v6() else {
            return Err(());
        };

        let conf = NetifConf {
            ipv4: v4.address.address(),
            ipv6: v6.address.address(),
            interface: 0,
            mac: [0; 6], // TODO
        };

        Ok(conf)
    }

    async fn wait_conf_change(&self) -> Result<(), ()> {
        // Embassy does have a `wait_config_up` but no `wait_config_change` or `wait_config_down`
        // Use a timer as a workaround

        let wait_up = self.stack.wait_config_up();
        let timer = Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _));

        select(wait_up, timer).await;

        Ok(())
    }
}

impl Netif for EmbassyNetif<'_> {
    async fn get_conf(&self) -> Result<Option<NetifConf>, Error> {
        Ok(EmbassyNetif::get_conf(self).ok())
    }

    async fn wait_conf_change(&self) -> Result<(), Error> {
        EmbassyNetif::wait_conf_change(self)
            .await
            .map_err(to_net_error)?;

        Ok(())
    }
}
