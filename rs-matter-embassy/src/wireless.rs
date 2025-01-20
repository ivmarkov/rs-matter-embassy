use edge_nal_embassy::UdpBuffers;

use embassy_net::StackResources;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rs_matter::error::Error;
use rs_matter::transport::network::{MAX_RX_PACKET_SIZE, MAX_TX_PACKET_SIZE};
use rs_matter::utils::init::{init, Init};

use rs_matter::utils::sync::IfMutex;
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::persist::KvBlobBuf;
use rs_matter_stack::wireless::traits::{Ble, BleTask};
use rs_matter_stack::{MatterStack, WirelessBle};

use crate::ble::{BleControllerProvider, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::netif::{MAX_META_DATA, MAX_SOCKETS};

pub use wifi::*;

/// A type alias for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.
pub type EmbassyWirelessMatterStack<'a, T, E = ()> = MatterStack<'a, EmbassyWirelessBle<T, E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<T, E = ()> =
    WirelessBle<CriticalSectionRawMutex, T, KvBlobBuf<EmbassyGatt<E>>>;

/// An embedding of the Trouble Gatt peripheral context for the `WirelessBle` network type from `rs-matter-stack`.
///
/// Allows the memory of this context to be statically allocated and cost-initialized.
///
/// Usage:
/// ```no_run
/// MatterStack<WirelessBle<CriticalSectionRawMutex, Wifi, KvBlobBuf<EmbassyGatt<C, E>>>>::new(...);
/// ```
///
/// ... where `E` can be a next-level, user-supplied embedding or just `()` if the user does not need to embed anything.
pub struct EmbassyGatt<E = ()> {
    btp_gatt_context: TroubleBtpGattContext<CriticalSectionRawMutex>,
    enet_context: EmbassyNetContext,
    embedding: E,
}

impl<E> EmbassyGatt<E>
where
    E: Embedding,
{
    /// Creates a new instance of the `EspGatt` embedding.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    const fn new() -> Self {
        Self {
            btp_gatt_context: TroubleBtpGattContext::new(),
            enet_context: EmbassyNetContext::new(),
            embedding: E::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            btp_gatt_context <- TroubleBtpGattContext::init(),
            enet_context: EmbassyNetContext::new(),
            embedding <- E::init(),
        })
    }

    /// Return a reference to the Bluedroid Gatt peripheral context.
    pub fn ble_context(&self) -> &TroubleBtpGattContext<CriticalSectionRawMutex> {
        &self.btp_gatt_context
    }

    pub fn enet_context(&self) -> &EmbassyNetContext {
        &self.enet_context
    }

    /// Return a reference to the embedding.
    pub fn embedding(&self) -> &E {
        &self.embedding
    }
}

impl<E> Embedding for EmbassyGatt<E>
where
    E: Embedding,
{
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EmbassyGatt::init()
    }
}

pub struct EmbassyNetContext {
    inner: IfMutex<CriticalSectionRawMutex, EmbassyNetContextInner>,
}

impl EmbassyNetContext {
    pub const fn new() -> Self {
        Self {
            inner: IfMutex::new(EmbassyNetContextInner::new()),
        }
    }
}

impl Default for EmbassyNetContext {
    fn default() -> Self {
        Self::new()
    }
}

struct EmbassyNetContextInner {
    resources: StackResources<MAX_SOCKETS>,
    buffers: UdpBuffers<MAX_SOCKETS, MAX_TX_PACKET_SIZE, MAX_RX_PACKET_SIZE, MAX_META_DATA>,
}

impl EmbassyNetContextInner {
    pub const fn new() -> Self {
        Self {
            resources: StackResources::new(),
            buffers: UdpBuffers::new(),
        }
    }
}

pub struct EmbassyBle<'a, T> {
    provider: T,
    context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
}

impl<'a, T> EmbassyBle<'a, T>
where
    T: BleControllerProvider,
{
    /// Create a new instance of the `EmbassyWifi` type.
    pub fn new<E>(provider: T, stack: &'a EmbassyWifiMatterStack<'a, E>) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            provider,
            stack.network().embedding().embedding().ble_context(),
        )
    }

    /// Wrap the `EmbassyWifi` type around a Wifi driver provider and a network context.
    pub const fn wrap(
        provider: T,
        context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    ) -> Self {
        Self { provider, context }
    }
}

impl<T> Ble for EmbassyBle<'_, T>
where
    T: BleControllerProvider,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleTask,
    {
        let peripheral = TroubleBtpGattPeripheral::new(&mut self.provider, self.context);

        task.run(peripheral).await
    }
}

mod wifi {
    use core::pin::pin;

    use edge_nal_embassy::Udp;
    use embassy_futures::select::select;
    use embassy_net::Config;

    use rs_matter::error::Error;
    use rs_matter::utils::select::Coalesce;

    use rs_matter_stack::network::{Embedding, Network};
    use rs_matter_stack::wireless::traits::{
        Controller, Wifi, WifiData, Wireless, WirelessTask, NC,
    };

    use crate::netif::EmbassyNetif;

    use super::{EmbassyNetContext, EmbassyWirelessMatterStack};

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    pub type EmbassyWifiMatterStack<'a, E> = EmbassyWirelessMatterStack<'a, Wifi, E>;

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    ///
    /// Unlike `EmbassyWifiMatterStack`, this type alias runs the commissioning in a non-concurrent mode,
    /// where the device runs either BLE or Wifi, but not both at the same time.
    ///
    /// This is useful to save memory by only having one of the stacks active at any point in time.
    ///
    /// Note that Alexa does not (yet) work with non-concurrent commissioning.
    pub type EmbassyWifiNCMatterStack<'a, E> = EmbassyWirelessMatterStack<'a, Wifi<NC>, E>;

    pub trait WifiDriverProvider {
        type Driver<'a>: embassy_net::driver::Driver
        where
            Self: 'a;
        type Controller<'a>: Controller<Data = WifiData>
        where
            Self: 'a;

        async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>);
    }

    impl<T> WifiDriverProvider for &mut T
    where
        T: WifiDriverProvider,
    {
        type Driver<'a>
            = T::Driver<'a>
        where
            Self: 'a;
        type Controller<'a>
            = T::Controller<'a>
        where
            Self: 'a;

        async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>) {
            (*self).provide().await
        }
    }

    /// A `Wireless` trait implementation for `embassy-net`'s Wifi stack.
    pub struct EmbassyWifi<'a, T> {
        provider: T,
        context: &'a EmbassyNetContext,
    }

    impl<'a, T> EmbassyWifi<'a, T>
    where
        T: WifiDriverProvider,
    {
        /// Create a new instance of the `EmbassyWifi` type.
        pub fn new<E>(provider: T, stack: &'a EmbassyWifiMatterStack<'a, E>) -> Self
        where
            E: Embedding + 'static,
        {
            Self::wrap(
                provider,
                stack.network().embedding().embedding().enet_context(),
            )
        }

        /// Wrap the `EmbassyWifi` type around a Wifi driver provider and a network context.
        pub const fn wrap(provider: T, context: &'a EmbassyNetContext) -> Self {
            Self { provider, context }
        }
    }

    impl<T> Wireless for EmbassyWifi<'_, T>
    where
        T: WifiDriverProvider,
    {
        type Data = WifiData;

        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: WirelessTask<Data = Self::Data>,
        {
            let mut context = self.context.inner.lock().await;
            let context = &mut *context;

            let (driver, controller) = self.provider.provide().await;

            let resources = &mut context.resources;
            let buffers = &context.buffers;

            let (stack, mut runner) =
                embassy_net::new(driver, Config::default(), resources, 0 /*TODO */);

            let netif = EmbassyNetif::new(stack);
            let udp = Udp::new(stack, buffers);

            let mut main = pin!(task.run(netif, udp, controller));
            let mut run = pin!(async {
                runner.run().await;
                #[allow(unreachable_code)]
                Ok(())
            });

            select(&mut main, &mut run).coalesce().await
        }
    }
}
