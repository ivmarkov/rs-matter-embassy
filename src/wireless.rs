use edge_nal_embassy::UdpBuffers;
use embassy_net::StackResources;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rs_matter::error::Error;
use rs_matter::tlv::{FromTLV, ToTLV};
use rs_matter::utils::init::{init, Init};

use rs_matter::utils::sync::IfMutex;
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::persist::KvBlobBuf;
use rs_matter_stack::wireless::traits::{Ble, BleTask, WirelessConfig, WirelessData};
use rs_matter_stack::{MatterStack, WirelessBle};

use crate::ble::{ControllerFactory, TroubleBtpGattContext, TroubleBtpGattPeripheral};

pub use wifi::*;

/// A type alias for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.
pub type EmbassyWirelessMatterStack<
    'a,
    const N: usize,
    const TX_SZ: usize,
    const RX_SZ: usize,
    const M: usize,
    T,
    C,
    E,
> = MatterStack<'a, EmbassyWirelessBle<N, TX_SZ, RX_SZ, M, T, C, E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<
    const N: usize,
    const TX_SZ: usize,
    const RX_SZ: usize,
    const M: usize,
    T,
    C,
    E,
> = WirelessBle<CriticalSectionRawMutex, T, KvBlobBuf<EmbassyGatt<N, TX_SZ, RX_SZ, M, C, E>>>;

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
pub struct EmbassyGatt<
    const N: usize,
    const TX_SZ: usize,
    const RX_SZ: usize,
    const M: usize,
    C,
    E = (),
> where
    C: trouble_host::Controller,
{
    btp_gatt_context: TroubleBtpGattContext<CriticalSectionRawMutex, C>,
    enet_context: EmbassyNetContext<N, TX_SZ, RX_SZ, M>,
    embedding: E,
}

impl<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize, C, E>
    EmbassyGatt<N, TX_SZ, RX_SZ, M, C, E>
where
    C: trouble_host::Controller,
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
    pub fn ble_context(&self) -> &TroubleBtpGattContext<CriticalSectionRawMutex, C> {
        &self.btp_gatt_context
    }

    pub fn enet_context(&self) -> &EmbassyNetContext<N, TX_SZ, RX_SZ, M> {
        &self.enet_context
    }

    /// Return a reference to the embedding.
    pub fn embedding(&self) -> &E {
        &self.embedding
    }
}

impl<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize, C, E> Embedding
    for EmbassyGatt<N, TX_SZ, RX_SZ, M, C, E>
where
    C: trouble_host::Controller,
    E: Embedding,
{
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EmbassyGatt::init()
    }
}

pub struct EmbassyNetContext<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize>
{
    inner: IfMutex<CriticalSectionRawMutex, EmbassyNetContextInner<N, TX_SZ, RX_SZ, M>>,
}

impl<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize> Default for EmbassyNetContext<N, TX_SZ, RX_SZ, M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize>
    EmbassyNetContext<N, TX_SZ, RX_SZ, M>
{
    pub const fn new() -> Self {
        Self {
            inner: IfMutex::new(EmbassyNetContextInner::new()),
        }
    }
}

struct EmbassyNetContextInner<
    const N: usize,
    const TX_SZ: usize,
    const RX_SZ: usize,
    const M: usize,
> {
    resources: StackResources<N>,
    buffers: UdpBuffers<N, TX_SZ, RX_SZ, M>,
}

impl<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize>
    EmbassyNetContextInner<N, TX_SZ, RX_SZ, M>
{
    pub const fn new() -> Self {
        Self {
            resources: StackResources::new(),
            buffers: UdpBuffers::new(),
        }
    }
}

/// A `Ble` trait implementation for `trouble-host`
pub struct EmbassyMatterBle<'a, C>
where
    C: ControllerFactory,
{
    factory: C,
    context: &'a TroubleBtpGattContext<CriticalSectionRawMutex, C::Controller>,
}

impl<'a, C> EmbassyMatterBle<'a, C>
where
    C: ControllerFactory + 'static,
{
    /// Create a new instance of the `EmbassyMatterBle` type.
    pub fn new<const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize, T, E>(
        factory: C,
        stack: &'a EmbassyWirelessMatterStack<N, TX_SZ, RX_SZ, M, T, C::Controller, E>,
    ) -> Self
    where
        T: WirelessConfig,
        <T::Data as WirelessData>::NetworkCredentials: Clone + for<'t> FromTLV<'t> + ToTLV,
        E: Embedding + 'static,
    {
        Self::wrap(
            factory,
            stack.network().embedding().embedding().ble_context(),
        )
    }

    /// Wrap an existing `TroubleBtpGattContext` into an `EmbassyMatterBle` instance.
    pub fn wrap(
        factory: C,
        context: &'a TroubleBtpGattContext<CriticalSectionRawMutex, C::Controller>,
    ) -> Self {
        Self { factory, context }
    }
}

impl<C> Ble for EmbassyMatterBle<'_, C>
where
    C: ControllerFactory,
{
    async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
    where
        T: BleTask,
    {
        let peripheral = TroubleBtpGattPeripheral::new(&self.factory, self.context).unwrap();

        task.run(peripheral).await
    }
}

mod wifi {
    use core::pin::pin;

    use embassy_futures::select::select;
    use embassy_net::Config;

    use rs_matter::error::Error;
    use rs_matter::utils::select::Coalesce;

    use rs_matter_stack::netif::DummyNetif;
    use rs_matter_stack::wireless::traits::{
        Controller, Wifi, WifiData, Wireless, WirelessTask, NC,
    };

    use crate::netif::EmbassyNetif;

    use super::{EmbassyNetContext, EmbassyWirelessMatterStack};

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    pub type EmbassyWifiMatterStack<
        'a,
        const N: usize,
        const TX_SZ: usize,
        const RX_SZ: usize,
        const M: usize,
        C,
        E,
    > = EmbassyWirelessMatterStack<'a, N, TX_SZ, RX_SZ, M, Wifi, C, E>;

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    ///
    /// Unlike `EmbassyWifiMatterStack`, this type alias runs the commissioning in a non-concurrent mode,
    /// where the device runs either BLE or Wifi, but not both at the same time.
    ///
    /// This is useful to save memory by only having one of the stacks active at any point in time.
    ///
    /// Note that Alexa does not (yet) work with non-concurrent commissioning.
    pub type EmbassyWifiNCMatterStack<
        'a,
        const N: usize,
        const TX_SZ: usize,
        const RX_SZ: usize,
        const M: usize,
        C,
        E,
    > = EmbassyWirelessMatterStack<'a, N, TX_SZ, RX_SZ, M, Wifi<NC>, C, E>;

    pub trait EmbassyWifiFactory {
        type Driver<'a>: embassy_net::driver::Driver
        where
            Self: 'a;
        type Controller<'a>: Controller<Data = WifiData>
        where
            Self: 'a;

        async fn create(&self) -> (Self::Driver<'_>, Self::Controller<'_>);
    }

    impl<T> EmbassyWifiFactory for &T
    where
        T: EmbassyWifiFactory,
    {
        type Driver<'a>
            = T::Driver<'a>
        where
            Self: 'a;
        type Controller<'a>
            = T::Controller<'a>
        where
            Self: 'a;

        async fn create(&self) -> (Self::Driver<'_>, Self::Controller<'_>) {
            (*self).create().await
        }
    }

    /// A `Wireless` trait implementation for `embassy-net`'s Wifi stack.
    pub struct EmbassyWifi<
        'a,
        T,
        const N: usize,
        const TX_SZ: usize,
        const RX_SZ: usize,
        const M: usize,
    > {
        factory: T,
        context: &'a EmbassyNetContext<N, TX_SZ, RX_SZ, M>,
    }

    impl<'a, T, const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize>
        EmbassyWifi<'a, T, N, TX_SZ, RX_SZ, M>
    where
        T: EmbassyWifiFactory,
    {
        /// Create a new instance of the `EmbassyWifi` type.
        pub const fn new(factory: T, context: &'a EmbassyNetContext<N, TX_SZ, RX_SZ, M>) -> Self {
            Self { factory, context }
        }
    }

    impl<T, const N: usize, const TX_SZ: usize, const RX_SZ: usize, const M: usize> Wireless
        for EmbassyWifi<'_, T, N, TX_SZ, RX_SZ, M>
    where
        T: EmbassyWifiFactory,
    {
        type Data = WifiData;

        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: WirelessTask<Data = Self::Data>,
        {
            let mut context = self.context.inner.lock().await;
            let context = &mut *context;

            let (driver, controller) = self.factory.create().await;

            let resources = &mut context.resources;
            let buffers = &context.buffers;

            let (stack, mut runner) =
                embassy_net::new(driver, Config::default(), resources, 0 /*TODO */);

            let stack = EmbassyNetif::new(stack, buffers);

            let mut main = pin!(task.run(DummyNetif::default() /* TODO */, stack, controller));
            let mut run = pin!(async {
                runner.run().await;
                #[allow(unreachable_code)]
                Ok(())
            });

            select(&mut main, &mut run).coalesce().await
        }
    }
}
