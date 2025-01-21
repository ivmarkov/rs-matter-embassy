//! Network interface: `EmbassyNetif - a `Netif` trait implementation for `embassy-net`

use core::cell::Cell;

use embassy_futures::select::select;
use embassy_net::{HardwareAddress, Stack};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Timer};

use rs_matter::error::Error;
use rs_matter_stack::netif::{Netif, NetifConf};

/// The maximum number of sockets that the Matter stack would use:
/// - One, for the UDP socket used by the Matter protocol
/// - Another, for the UDP socket used by the mDNS responder
pub const MAX_SOCKETS: usize = 2;

pub const MAX_META_DATA: usize = 4;

const TIMEOUT_PERIOD_SECS: u8 = 5;

/// A `Netif` trait implementation for `embassy-net`
pub struct EmbassyNetif<'d> {
    stack: Stack<'d>,
    up: Mutex<CriticalSectionRawMutex, Cell<bool>>,
}

impl<'d> EmbassyNetif<'d> {
    /// Create a new `EmbassyNetif` instance
    pub fn new(stack: Stack<'d>) -> Self {
        Self { stack, up: Mutex::new(Cell::new(false)) }
    }

    fn get_conf(&self) -> Option<NetifConf> {
        let Some(v4) = self.stack.config_v4() else {
            return None;
        };

        let Some(v6) = self.stack.config_v6() else {
            return None;
        };

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
            }
        };
        
        Some(conf)
    }

    async fn wait_conf_change(&self) {
        // Embassy does have a `wait_config_up/down` but no `wait_config_change`
        // Use a timer as a workaround

        if self.up.lock(|up| up.get()) {
            select(self.stack.wait_config_down(), Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _))).await;
        } else {
            select(self.stack.wait_config_up(), Timer::after(Duration::from_secs(TIMEOUT_PERIOD_SECS as _))).await;
        }

        self.up.lock(|up| up.set(self.stack.is_config_up()));
    }

}

impl Netif for EmbassyNetif<'_> {
    async fn get_conf(&self) -> Result<Option<NetifConf>, Error> {
        Ok(EmbassyNetif::get_conf(self))
    }

    async fn wait_conf_change(&self) -> Result<(), Error> {
        EmbassyNetif::wait_conf_change(self).await;

        Ok(())
    }
}
