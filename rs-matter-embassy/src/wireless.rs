//! Wireless: Type aliases and state structs for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::tlv::{FromTLV, ToTLV};
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::rand::Rand;
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::persist::KvBlobBuf;
use rs_matter_stack::wireless::traits::{Ble, BleTask, WirelessConfig, WirelessData};
use rs_matter_stack::{MatterStack, WirelessBle};

use trouble_host::Controller;


use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};

/// A type alias for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.
pub type EmbassyWirelessMatterStack<'a, T, N, E = ()> =
    MatterStack<'a, EmbassyWirelessBle<T, N, E>>;

/// A type alias for an Embassy implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<T, N, E = ()> =
    WirelessBle<CriticalSectionRawMutex, T, KvBlobBuf<EmbassyGatt<N, E>>>;

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
pub struct EmbassyGatt<N, E = ()> {
    btp_gatt_context: TroubleBtpGattContext<CriticalSectionRawMutex>,
    net_context: N,
    embedding: E,
}

impl<N, E> EmbassyGatt<N, E>
where
    N: Embedding,
    E: Embedding,
{
    /// Creates a new instance of the `EspGatt` embedding.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    const fn new() -> Self {
        Self {
            btp_gatt_context: TroubleBtpGattContext::new(),
            net_context: N::INIT,
            embedding: E::INIT,
        }
    }

    /// Return an in-place initializer for the `EspGatt` embedding.
    fn init() -> impl Init<Self> {
        init!(Self {
            btp_gatt_context <- TroubleBtpGattContext::init(),
            net_context <- N::init(),
            embedding <- E::init(),
        })
    }

    /// Return a reference to the Bluedroid Gatt peripheral context.
    pub fn ble_context(&self) -> &TroubleBtpGattContext<CriticalSectionRawMutex> {
        &self.btp_gatt_context
    }

    pub fn net_context(&self) -> &N {
        &self.net_context
    }

    /// Return a reference to the embedding.
    pub fn embedding(&self) -> &E {
        &self.embedding
    }
}

impl<N, E> Embedding for EmbassyGatt<N, E>
where
    N: Embedding,
    E: Embedding,
{
    const INIT: Self = Self::new();

    fn init() -> impl Init<Self> {
        EmbassyGatt::init()
    }
}

/// A companion trait of `EmbassyBle` for providing a BLE controller.
pub trait BleControllerProvider {
    type Controller<'a>: Controller
    where
        Self: 'a;

    /// Provide a BLE controller by creating it when the Matter stack needs it
    async fn provide(&mut self) -> Self::Controller<'_>;
}

impl<T> BleControllerProvider for &mut T
where
    T: BleControllerProvider,
{
    type Controller<'a>
        = T::Controller<'a>
    where
        Self: 'a;

    async fn provide(&mut self) -> Self::Controller<'_> {
        (*self).provide().await
    }
}

/// A BLE controller provider that uses a pre-existing, already created BLE controller,
/// rather than creating one when the Matter stack needs it.
pub struct PreexistingBleController<C>(C);

impl<C> PreexistingBleController<C> {
    /// Create a new instance of the `PreexistingBleController` type.
    pub const fn new(controller: C) -> Self {
        Self(controller)
    }
}

impl<C> BleControllerProvider for PreexistingBleController<C>
where
    C: Controller,
{
    type Controller<'a>
        = ControllerRef<'a, C>
    where
        Self: 'a;

    async fn provide(&mut self) -> Self::Controller<'_> {
        ControllerRef::new(&self.0)
    }
}

/// A `Ble` trait implementation for `trouble`'s BLE stack
pub struct EmbassyBle<'a, T> {
    provider: T,
    rand: Rand,
    context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
}

impl<'a, T> EmbassyBle<'a, T>
where
    T: BleControllerProvider,
{
    /// Create a new instance of the `EmbassyBle` type.
    pub fn new<E, Q>(provider: T, stack: &'a EmbassyWirelessMatterStack<'a, Q, E>) -> Self
    where
        Q: WirelessConfig,
        <Q::Data as WirelessData>::NetworkCredentials: Clone + for<'t> FromTLV<'t> + ToTLV,
        E: Embedding + 'static,
    {
        Self::wrap(
            provider,
            stack.matter().rand(),
            stack.network().embedding().embedding().ble_context(),
        )
    }

    /// Wrap the `EmbassyBle` type around a BLE controller provider and a trouble BTP GATT context.
    pub const fn wrap(
        provider: T,
        rand: Rand,
        context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    ) -> Self {
        Self {
            provider,
            rand,
            context,
        }
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
        let controller = self.provider.provide().await;

        let peripheral = TroubleBtpGattPeripheral::new(controller, self.rand, self.context);

        task.run(&peripheral).await
    }
}

impl<'a, C> TroubleBtpGattPeripheral<'a, CriticalSectionRawMutex, C>
where
    C: Controller,
{
    pub fn new_for_stack<T, E>(
        controller: C,
        stack: &'a crate::wireless::EmbassyWirelessMatterStack<T, E>,
    ) -> Self
    where
        T: WirelessConfig,
        <T::Data as WirelessData>::NetworkCredentials: Clone + for<'t> FromTLV<'t> + ToTLV,
        E: Embedding + 'static,
    {
        Self::new(
            controller,
            stack.matter().rand(),
            stack.network().embedding().embedding().ble_context(),
        )
    }
}

#[cfg(feature = "esp")]
pub mod esp {
    use bt_hci::controller::ExternalController;

    use esp_hal::peripheral::{Peripheral, PeripheralRef};

    use esp_wifi::ble::controller::BleConnector;
    use esp_wifi::EspWifiController;

    const SLOTS: usize = 20;

    /// A `BleControllerProvider` implementation for the ESP32 family of chips.
    pub struct EspBleControllerProvider<'a, 'd> {
        controller: &'a EspWifiController<'d>,
        peripheral: PeripheralRef<'d, esp_hal::peripherals::BT>,
    }

    impl<'a, 'd> EspBleControllerProvider<'a, 'd> {
        /// Create a new instance
        ///
        /// # Arguments
        /// - `controller`: The WiFi controller instance
        /// - `peripheral`: The Bluetooth peripheral instance
        pub fn new(
            controller: &'a EspWifiController<'d>,
            peripheral: impl Peripheral<P = esp_hal::peripherals::BT> + 'd,
        ) -> Self {
            Self {
                controller,
                peripheral: peripheral.into_ref(),
            }
        }
    }

    impl super::BleControllerProvider for EspBleControllerProvider<'_, '_> {
        type Controller<'t>
            = ExternalController<BleConnector<'t>, SLOTS>
        where
            Self: 't;

        async fn provide(&mut self) -> Self::Controller<'_> {
            ExternalController::new(BleConnector::new(self.controller, &mut self.peripheral))
        }
    }
}

// Thread: Type aliases and state structs for an Embassy Matter stack running over a Thread network and BLE.
#[cfg(feature = "openthread")]
pub mod thread {
    use core::mem::MaybeUninit;
    use core::pin::pin;

    use embassy_futures::select::select3;
    use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};

    use log::error;

    use openthread::{
        Channels, OpenThread, OtError, OtResources, OtRngCore, OtRngCoreError, Radio,
    };

    use rs_matter_stack::matter::error::{Error, ErrorCode};
    use rs_matter_stack::matter::tlv::OctetsOwned;
    use rs_matter_stack::matter::utils::init::{init, Init};
    use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::matter::utils::select::Coalesce;
    use rs_matter_stack::matter::utils::sync::IfMutex;
    use rs_matter_stack::mdns::MatterMdnsServices;
    use rs_matter_stack::network::{Embedding, Network};
    use rs_matter_stack::wireless::traits::{
        ConcurrencyMode, Controller, NetworkCredentials, Thread, ThreadData, ThreadId,
        ThreadScanResult, Wireless, WirelessData, WirelessTask, NC,
    };

    use crate::ot::{OtMatterSrpResources, OtMatterUdpResources, OtMdns, OtNetif};

    use super::EmbassyWirelessMatterStack;

    /// A type alias for an Embassy Matter stack running over Thread (and BLE, during commissioning).
    pub type EmbassyThreadMatterStack<'a, E> =
        EmbassyWirelessMatterStack<'a, Thread, OtNetContext, E>;

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    ///
    /// Unlike `EmbassyWifiMatterStack`, this type alias runs the commissioning in a non-concurrent mode,
    /// where the device runs either BLE or Wifi, but not both at the same time.
    ///
    /// This is useful to save memory by only having one of the stacks active at any point in time.
    ///
    /// Note that Alexa does not (yet) work with non-concurrent commissioning.
    pub type EmbassyThreadNCMatterStack<'a, E> =
        EmbassyWirelessMatterStack<'a, Thread<NC>, OtNetContext, E>;

    /// A companion trait of `EmbassyThread` for providing a Thread radio and controller.
    pub trait ThreadRadioProvider {
        type Radio<'a>: Radio
        where
            Self: 'a;

        /// Provide a Thread radio by creating it when the Matter stack needs it
        async fn provide(&mut self) -> Self::Radio<'_>;
    }

    impl<T> ThreadRadioProvider for &mut T
    where
        T: ThreadRadioProvider,
    {
        type Radio<'a>
            = T::Radio<'a>
        where
            Self: 'a;

        async fn provide(&mut self) -> Self::Radio<'_> {
            (*self).provide().await
        }
    }

    /// A Wifi driver provider that uses a pre-existing, already created Thread radio and controller,
    /// rather than creating them when the Matter stack needs them.
    pub struct PreexistingThreadRadio<R>(R);

    impl<R> PreexistingThreadRadio<R> {
        /// Create a new instance of the `PreexistingWifiDriver` type.
        pub const fn new(radio: R) -> Self {
            Self(radio)
        }
    }

    impl<R> ThreadRadioProvider for PreexistingThreadRadio<R>
    where
        R: Radio,
    {
        type Radio<'t>
            = &'t mut R
        where
            Self: 't;

        async fn provide(&mut self) -> Self::Radio<'_> {
            &mut self.0
        }
    }

    /// A `Wireless` trait implementation for `openthread`'s Thread stack.
    pub struct EmbassyThread<'a, T> {
        provider: T,
        mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
        context: &'a OtNetContext,
        rand: Rand,
    }

    impl<'a, T> EmbassyThread<'a, T>
    where
        T: ThreadRadioProvider,
    {
        /// Create a new instance of the `EmbassyThread` type.
        pub fn new<E, M>(
            provider: T,
            mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
            stack: &'a EmbassyWirelessMatterStack<'a, Thread<M>, OtNetContext, E>,
        ) -> Self
        where
            M: ConcurrencyMode,
            E: Embedding + 'static,
        {
            Self::wrap(
                provider,
                mdns_services,
                stack.network().embedding().embedding().net_context(),
                stack.matter().rand(),
            )
        }

        /// Wrap the `EmbassyThread` type around a Thread driver provider and a network context.
        pub const fn wrap(
            provider: T,
            mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
            context: &'a OtNetContext,
            rand: Rand,
        ) -> Self {
            Self {
                provider,
                mdns_services,
                context,
                rand,
            }
        }
    }

    impl<T> Wireless for EmbassyThread<'_, T>
    where
        T: ThreadRadioProvider,
    {
        type Data = ThreadData;

        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: WirelessTask<Data = Self::Data>,
        {
            let radio = self.provider.provide().await;

            let mut resources = self.context.resources.lock().await;
            let (ot_resources, ot_udp_resources, ot_srp_resources) = &mut *resources;

            let mut rng = OtRngCoreImpl(self.rand);

            let ot = OpenThread::new_with_udp_srp(
                &mut rng,
                ot_resources,
                ot_udp_resources,
                ot_srp_resources,
            )
            .unwrap();

            let controller = OtController(ot);
            let netif = OtNetif::new(ot);
            let mdns = OtMdns::new(ot, self.mdns_services).unwrap();

            let mut main = pin!(task.run(netif, ot, controller));
            let mut radio = pin!(async {
                ot.run(radio).await;
                #[allow(unreachable_code)]
                Ok(())
            });
            let mut mdns = pin!(async {
                mdns.run().await.unwrap(); // TODO
                #[allow(unreachable_code)]
                Ok(())
            });

            select3(&mut main, &mut radio, &mut mdns).coalesce().await
        }
    }

    pub struct OtNetContext {
        resources: IfMutex<
            CriticalSectionRawMutex,
            (OtResources, OtMatterUdpResources, OtMatterSrpResources),
        >,
    }

    impl OtNetContext {
        /// Create a new instance of the `OtNetContext` type.
        pub const fn new() -> Self {
            Self {
                resources: IfMutex::new((
                    OtResources::new(),
                    OtMatterUdpResources::new(),
                    OtMatterSrpResources::new(),
                )),
            }
        }

        /// Return an in-place initializer for the `OtNetContext` type.
        pub fn init() -> impl Init<Self> {
            init!(Self {
                // Note: below will break if `Ot*Resources` stop being a bunch of `MaybeUninit`s
                resources <- IfMutex::init((
                    unsafe { MaybeUninit::<OtResources>::uninit().assume_init() },
                    unsafe { MaybeUninit::<OtMatterUdpResources>::uninit().assume_init() },
                    unsafe { MaybeUninit::<OtMatterSrpResources>::uninit().assume_init() },
                )),
            })
        }
    }

    impl Default for OtNetContext {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Embedding for OtNetContext {
        const INIT: Self = Self::new();

        fn init() -> impl Init<Self> {
            OtNetContext::init()
        }
    }

    struct OtController<'a>(OpenThread<'a>);

    impl Controller for OtController<'_> {
        type Data = ThreadData;

        async fn scan<F>(
            &mut self,
            network_id: Option<
                &<<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
            >,
            mut callback: F,
        ) -> Result<(), Error>
        where
            F: FnMut(Option<&<Self::Data as WirelessData>::ScanResult>) -> Result<(), Error>,
        {
            const SCAN_DURATION_MILLIS: u16 = 2000;

            self.0
                .scan(Channels::all(), SCAN_DURATION_MILLIS, |scan_result| {
                    if scan_result
                        .map(|sr| {
                            network_id
                                .map(|id| id.0.vec.as_slice() == sr.extended_pan_id.to_be_bytes())
                                .unwrap_or(true)
                        })
                        .unwrap_or(true)
                    {
                        let _ = callback(
                            scan_result
                                .map(|scan_result| ThreadScanResult {
                                    pan_id: scan_result.pan_id,
                                    extended_pan_id: scan_result.extended_pan_id,
                                    network_name: scan_result
                                        .network_name
                                        .try_into()
                                        .unwrap_or(heapless::String::new()),
                                    channel: scan_result.channel as _,
                                    version: scan_result.version,
                                    // TODO: Should be Vec<u8, 8>
                                    extended_address:
                                        rs_matter_stack::matter::utils::storage::Vec::from_slice(
                                            &scan_result.ext_address.to_be_bytes(),
                                        )
                                        .unwrap_or(
                                            rs_matter_stack::matter::utils::storage::Vec::new(),
                                        ),
                                    rssi: scan_result.rssi,
                                    lqi: scan_result.lqi,
                                })
                                .as_ref(),
                        );
                    }
                })
                .await
                .map_err(to_matter_err)
        }

        async fn connect(
            &mut self,
            creds: &<Self::Data as WirelessData>::NetworkCredentials,
        ) -> Result<(), Error> {
            self.0
                .set_active_dataset_tlv(&creds.op_dataset)
                .map_err(to_matter_err)
        }

        async fn connected_network(
            &mut self,
        ) -> Result<
            Option<
                <<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
            >,
            Error,
        > {
            let status = self.0.net_status();

            Ok(status
                .role
                .is_connected()
                .then_some(status.ext_pan_id)
                .flatten()
                .map(|id| {
                    ThreadId(OctetsOwned {
                        vec: rs_matter_stack::matter::utils::storage::Vec::from_slice(
                            &u64::to_be_bytes(id),
                        )
                        .unwrap(),
                    })
                }))
        }

        async fn stats(&mut self) -> Result<<Self::Data as WirelessData>::Stats, Error> {
            Ok(())
        }
    }

    fn to_matter_err(err: OtError) -> Error {
        error!("OpenThread error: {:?}", err);

        ErrorCode::NoNetworkInterface.into()
    }

    /// Adapt `rs-matter`'s `Rand` to `RngCore` necessary for OpenThread
    struct OtRngCoreImpl(Rand);

    impl OtRngCore for OtRngCoreImpl {
        fn next_u32(&mut self) -> u32 {
            let mut bytes = [0; 4];
            (self.0)(&mut bytes);

            u32::from_ne_bytes(bytes)
        }

        fn next_u64(&mut self) -> u64 {
            let mut bytes = [0; 8];
            (self.0)(&mut bytes);

            u64::from_ne_bytes(bytes)
        }

        fn fill_bytes(&mut self, dest: &mut [u8]) {
            (self.0)(dest);
        }

        fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), OtRngCoreError> {
            (self.0)(dest);

            Ok(())
        }
    }

    #[cfg(feature = "esp")]
    pub mod esp {
        use esp_hal::peripheral::{Peripheral, PeripheralRef};

        use openthread::esp::EspRadio;

        /// A `ThreadRadioProvider` implementation for the ESP32 family of chips.
        pub struct EspThreadRadioProvider<'d> {
            _radio_peripheral: PeripheralRef<'d, esp_hal::peripherals::IEEE802154>,
            _radio_clk_peripheral: PeripheralRef<'d, esp_hal::peripherals::RADIO_CLK>,
        }

        impl<'d> EspThreadRadioProvider<'d> {
            /// Create a new instance of the `EspThreadRadioDriverProvider` type.
            ///
            /// # Arguments
            /// - `peripheral` - The Thread radio peripheral instance.
            pub fn new(
                radio_peripheral: impl Peripheral<P = esp_hal::peripherals::IEEE802154> + 'd,
                radio_clk_peripheral: impl Peripheral<P = esp_hal::peripherals::RADIO_CLK> + 'd,
            ) -> Self {
                Self {
                    _radio_peripheral: radio_peripheral.into_ref(),
                    _radio_clk_peripheral: radio_clk_peripheral.into_ref(),
                }
            }
        }

        impl super::ThreadRadioProvider for EspThreadRadioProvider<'_> {
            type Radio<'t>
                = EspRadio<'t>
            where
                Self: 't;

            async fn provide(&mut self) -> Self::Radio<'_> {
                // See https://github.com/esp-rs/esp-hal/issues/3238
                //EspRadio::new(openthread::esp::Ieee802154::new(&mut self.radio_peripheral, &mut self.radio_clk_peripheral))
                EspRadio::new(openthread::esp::Ieee802154::new(
                    unsafe { esp_hal::peripherals::IEEE802154::steal() },
                    unsafe { esp_hal::peripherals::RADIO_CLK::steal() },
                ))
            }
        }
    }
}

// Wifi: Type aliases and state structs for an Embassy Matter stack running over a Wifi network and BLE.
#[cfg(feature = "embassy-net")]
pub mod wifi {
    use core::mem::MaybeUninit;
    use core::pin::pin;

    use edge_nal_embassy::Udp;

    use embassy_futures::select::select;

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    use rs_matter_stack::matter::error::Error;
    use rs_matter_stack::matter::utils::init::{init, Init};
    use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::matter::utils::select::Coalesce;
    use rs_matter_stack::matter::utils::sync::IfMutex;
    use rs_matter_stack::network::{Embedding, Network};
    use rs_matter_stack::wireless::traits::{
        ConcurrencyMode, Controller, Wifi, WifiData, Wireless, WirelessTask, NC,
    };

    use crate::enet::{
        create_enet_stack, EnetMatterStackResources, EnetMatterUdpBuffers, EnetNetif,
    };

    use super::EmbassyWirelessMatterStack;

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    pub type EmbassyWifiMatterStack<'a, E> =
        EmbassyWirelessMatterStack<'a, Wifi, EmbassyNetContext, E>;

    /// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
    ///
    /// Unlike `EmbassyWifiMatterStack`, this type alias runs the commissioning in a non-concurrent mode,
    /// where the device runs either BLE or Wifi, but not both at the same time.
    ///
    /// This is useful to save memory by only having one of the stacks active at any point in time.
    ///
    /// Note that Alexa does not (yet) work with non-concurrent commissioning.
    pub type EmbassyWifiNCMatterStack<'a, E> =
        EmbassyWirelessMatterStack<'a, Wifi<NC>, EmbassyNetContext, E>;

    /// A companion trait of `EmbassyWifi` for providing a Wifi driver and controller.
    pub trait WifiDriverProvider {
        type Driver<'a>: embassy_net::driver::Driver
        where
            Self: 'a;
        type Controller<'a>: Controller<Data = WifiData>
        where
            Self: 'a;

        /// Provide a Wifi driver and controller by creating these when the Matter stack needs them
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

    /// A Wifi driver provider that uses a pre-existing, already created Wifi driver and controller,
    /// rather than creating them when the Matter stack needs them.
    pub struct PreexistingWifiDriver<D, C>(D, C);

    impl<D, C> PreexistingWifiDriver<D, C> {
        /// Create a new instance of the `PreexistingWifiDriver` type.
        pub const fn new(driver: D, controller: C) -> Self {
            Self(driver, controller)
        }
    }

    impl<D, C> WifiDriverProvider for PreexistingWifiDriver<D, C>
    where
        D: embassy_net::driver::Driver,
        C: Controller<Data = WifiData>,
    {
        type Driver<'a>
            = &'a mut D
        where
            Self: 'a;
        type Controller<'a>
            = &'a mut C
        where
            Self: 'a;

        async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>) {
            (&mut self.0, &mut self.1)
        }
    }

    /// A `Wireless` trait implementation for `embassy-net`'s Wifi stack.
    pub struct EmbassyWifi<'a, T> {
        provider: T,
        context: &'a EmbassyNetContext,
        rand: Rand,
    }

    impl<'a, T> EmbassyWifi<'a, T>
    where
        T: WifiDriverProvider,
    {
        /// Create a new instance of the `EmbassyWifi` type.
        pub fn new<E, M>(
            provider: T,
            stack: &'a EmbassyWirelessMatterStack<'a, Wifi<M>, EmbassyNetContext, E>,
        ) -> Self
        where
            M: ConcurrencyMode,
            E: Embedding + 'static,
        {
            Self::wrap(
                provider,
                stack.network().embedding().embedding().net_context(),
                stack.matter().rand(),
            )
        }

        /// Wrap the `EmbassyWifi` type around a Wifi driver provider and a network context.
        pub const fn wrap(provider: T, context: &'a EmbassyNetContext, rand: Rand) -> Self {
            Self {
                provider,
                context,
                rand,
            }
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
            let (driver, controller) = self.provider.provide().await;

            let mut resources = self.context.resources.lock().await;
            let resources = &mut *resources;
            let buffers = &self.context.buffers;

            let mut seed = [0; core::mem::size_of::<u64>()];
            (self.rand)(&mut seed);

            let (stack, mut runner) =
                create_enet_stack(driver, u64::from_le_bytes(seed), resources);

            let netif = EnetNetif::new(stack);
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

    /// A context (storage) for the network layer of the Matter stack.
    pub struct EmbassyNetContext {
        buffers: EnetMatterUdpBuffers,
        resources: IfMutex<CriticalSectionRawMutex, EnetMatterStackResources>,
    }

    impl EmbassyNetContext {
        /// Create a new instance of the `EmbassyNetContext` type.
        pub const fn new() -> Self {
            Self {
                buffers: EnetMatterUdpBuffers::new(),
                resources: IfMutex::new(EnetMatterStackResources::new()),
            }
        }

        /// Return an in-place initializer for the `EmbassyNetContext` type.
        pub fn init() -> impl Init<Self> {
            init!(Self {
                // TODO: Implement init constructor for `UdpBuffers`
                buffers: EnetMatterUdpBuffers::new(),
                // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
                resources <- IfMutex::init(unsafe { MaybeUninit::<EnetMatterStackResources>::uninit().assume_init() }),
            })
        }
    }

    impl Default for EmbassyNetContext {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Embedding for EmbassyNetContext {
        const INIT: Self = Self::new();

        fn init() -> impl Init<Self> {
            EmbassyNetContext::init()
        }
    }

    #[cfg(feature = "rp")]
    pub mod rp {
        use cyw43::{Control, JoinOptions, ScanOptions};

        use log::{error, info};

        use crate::matter::data_model::sdm::nw_commissioning::{WiFiSecurity, WifiBand};
        use crate::matter::error::{Error, ErrorCode};
        use crate::matter::tlv::OctetsOwned;
        use crate::matter::utils::storage::Vec;
        use crate::stack::wireless::traits::{
            Controller, NetworkCredentials, WifiData, WifiScanResult, WifiSsid, WirelessData,
        };

        /// An adaptor from the `cyw43` Wifi controller API to the `rs-matter` Wifi controller API
        pub struct Cyw43WifiController<'a>(Control<'a>, Option<WifiSsid>);

        impl<'a> Cyw43WifiController<'a> {
            /// Create a new instance of the `Cyw43WifiController` type.
            ///
            /// # Arguments
            /// - `controller` - The `cyw43` Wifi controller instance.
            pub const fn new(controller: Control<'a>) -> Self {
                Self(controller, None)
            }
        }

        impl Controller for Cyw43WifiController<'_> {
            type Data = WifiData;

            async fn scan<F>(
                &mut self,
                network_id: Option<
                    &<<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
                >,
                mut callback: F,
            ) -> Result<(), Error>
            where
                F: FnMut(Option<&<Self::Data as WirelessData>::ScanResult>) -> Result<(), Error>,
            {
                info!("Wifi scan request");

                let mut scan_options = ScanOptions::default();
                //scan_options.scan_type = ScanType::Active;

                if let Some(network_id) = network_id {
                    scan_options.ssid = Some(
                        core::str::from_utf8(network_id.0.vec.as_slice())
                            .unwrap_or("???")
                            .try_into()
                            .unwrap(),
                    );
                }

                let mut scanner = self.0.scan(scan_options).await;

                info!("Wifi scan started");

                while let Some(ap) = scanner.next().await {
                    if ap.ssid_len > 0 {
                        let result = WifiScanResult {
                            ssid: WifiSsid(OctetsOwned {
                                vec: Vec::from_slice(&ap.ssid[..ap.ssid_len as _]).unwrap(),
                            }),
                            bssid: OctetsOwned {
                                vec: Vec::from_slice(&ap.bssid).unwrap(),
                            },
                            channel: ap.chanspec,
                            rssi: Some(ap.rssi as _),
                            band: Some(WifiBand::B2G4), // cyw43 only supports 2.4GHz
                            security: WiFiSecurity::WPA2_PERSONAL, // TODO
                        };

                        callback(Some(&result))?;

                        info!("Scan result {:?}", result);
                    } else {
                        info!(
                            "Skipping scan result for a hidden network {:02x?}",
                            ap.bssid
                        );
                    }
                }

                callback(None)?;

                info!("Wifi scan complete");

                Ok(())
            }

            async fn connect(
                &mut self,
                creds: &<Self::Data as WirelessData>::NetworkCredentials,
            ) -> Result<(), Error> {
                let ssid = core::str::from_utf8(creds.ssid.0.vec.as_slice()).unwrap_or("???");

                info!("Wifi connect request for SSID {ssid}");

                self.1 = None;

                self.0.leave().await;
                info!("Disconnected from current Wifi AP (if any)");

                self.0
                    .join(ssid, JoinOptions::new(creds.password.as_bytes())) // TODO: Try with something else besides Wpa2Wpa3
                    .await
                    .map_err(to_err)?;

                info!("Wifi connected");

                self.1 = Some(creds.ssid.clone());

                info!("Wifi connect complete");

                Ok(())
            }

            async fn connected_network(
                &mut self,
            ) -> Result<
                Option<
                    <<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
                >,
                Error,
            >{
                Ok(self.1.clone())
            }

            async fn stats(&mut self) -> Result<<Self::Data as WirelessData>::Stats, Error> {
                Ok(None)
            }
        }

        fn to_err(e: cyw43::ControlError) -> Error {
            error!("Wifi error: {:?}", e);
            Error::new(ErrorCode::NoNetworkInterface)
        }
    }

    // TODO:
    // This adaptor would've not been necessary, if there was a common Wifi trait aggreed upon and
    // implemented by all MCU Wifi controllers in the field.
    //
    // Perhaps it is time to dust-off `embedded_svc::wifi` and publish it as a micro-crate?
    // `embedded-wifi`?
    #[cfg(feature = "esp")]
    pub mod esp {
        use esp_hal::peripheral::{Peripheral, PeripheralRef};
        use esp_wifi::wifi::{
            AuthMethod, ClientConfiguration, Configuration, ScanConfig, WifiController, WifiDevice,
            WifiError, WifiStaDevice,
        };

        use log::{error, info};

        use crate::matter::data_model::sdm::nw_commissioning::{WiFiSecurity, WifiBand};
        use crate::matter::error::{Error, ErrorCode};
        use crate::matter::tlv::OctetsOwned;
        use crate::matter::utils::storage::Vec;
        use crate::stack::wireless::traits::{
            Controller, NetworkCredentials, WifiData, WifiScanResult, WifiSsid, WirelessData,
        };

        const MAX_NETWORKS: usize = 3;

        /// A `WifiDriverProvider` implementation for the ESP32 family of chips.
        pub struct EspWifiDriverProvider<'a, 'd> {
            controller: &'a esp_wifi::EspWifiController<'d>,
            peripheral: PeripheralRef<'d, esp_hal::peripherals::WIFI>,
        }

        impl<'a, 'd> EspWifiDriverProvider<'a, 'd> {
            /// Create a new instance of the `Esp32WifiDriverProvider` type.
            ///
            /// # Arguments
            /// - `controller` - The `esp-wifi` Wifi controller instance.
            /// - `peripheral` - The Wifi peripheral instance.
            pub fn new(
                controller: &'a esp_wifi::EspWifiController<'d>,
                peripheral: impl Peripheral<P = esp_hal::peripherals::WIFI> + 'd,
            ) -> Self {
                Self {
                    controller,
                    peripheral: peripheral.into_ref(),
                }
            }
        }

        impl super::WifiDriverProvider for EspWifiDriverProvider<'_, '_> {
            type Driver<'t>
                = WifiDevice<'t, WifiStaDevice>
            where
                Self: 't;
            type Controller<'t>
                = EspWifiController<'t>
            where
                Self: 't;

            async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>) {
                let (wifi_interface, mut controller) = esp_wifi::wifi::new_with_mode(
                    self.controller,
                    &mut self.peripheral,
                    WifiStaDevice,
                )
                .unwrap();

                // esp32c6-specific - need to boost the power to get a good signal
                controller
                    .set_power_saving(esp_wifi::config::PowerSaveMode::None)
                    .unwrap();

                (wifi_interface, EspWifiController::new(controller))
            }
        }

        /// An adaptor from the `esp-wifi` Wifi controller API to the `rs-matter` Wifi controller API
        pub struct EspWifiController<'a>(WifiController<'a>, Option<WifiSsid>);

        impl<'a> EspWifiController<'a> {
            /// Create a new instance of the `Esp32Controller` type.
            ///
            /// # Arguments
            /// - `controller` - The `esp-wifi` Wifi controller instance.
            pub const fn new(controller: WifiController<'a>) -> Self {
                Self(controller, None)
            }
        }

        impl Controller for EspWifiController<'_> {
            type Data = WifiData;

            async fn scan<F>(
                &mut self,
                network_id: Option<
                    &<<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
                >,
                mut callback: F,
            ) -> Result<(), Error>
            where
                F: FnMut(Option<&<Self::Data as WirelessData>::ScanResult>) -> Result<(), Error>,
            {
                info!("Wifi scan request");

                if !self.0.is_started().map_err(to_err)? {
                    self.0.start_async().await.map_err(to_err)?;
                    info!("Wifi started");
                }

                let mut scan_config = ScanConfig::default();
                if let Some(network_id) = network_id {
                    scan_config.ssid = Some(
                        core::str::from_utf8(network_id.0.vec.as_slice())
                            .unwrap_or("???")
                            .try_into()
                            .unwrap(),
                    );
                }

                let (aps, len) = self
                    .0
                    .scan_with_config_async::<MAX_NETWORKS>(scan_config)
                    .await
                    .map_err(to_err)?;

                info!(
                    "Wifi scan complete, reporting {} results out of {len} total",
                    aps.len()
                );

                for ap in aps {
                    let result = WifiScanResult {
                        ssid: WifiSsid(OctetsOwned {
                            vec: ap.ssid.as_bytes().try_into().unwrap(),
                        }),
                        bssid: OctetsOwned {
                            vec: Vec::from_slice(&ap.bssid).unwrap(),
                        },
                        channel: ap.channel as _,
                        rssi: Some(ap.signal_strength),
                        band: Some(WifiBand::B2G4), // TODO: Once c5 is out we can no longer hard-code this
                        security: match ap.auth_method {
                            Some(AuthMethod::None) => WiFiSecurity::UNENCRYPTED,
                            Some(AuthMethod::WEP) => WiFiSecurity::WEP,
                            Some(AuthMethod::WPA) => WiFiSecurity::WPA_PERSONAL,
                            Some(AuthMethod::WPA2Personal) => WiFiSecurity::WPA2_PERSONAL,
                            Some(AuthMethod::WPAWPA2Personal) => {
                                WiFiSecurity::WPA_PERSONAL | WiFiSecurity::WPA2_PERSONAL
                            }
                            Some(AuthMethod::WPA2WPA3Personal) => {
                                WiFiSecurity::WPA2_PERSONAL | WiFiSecurity::WPA3_PERSONAL
                            }
                            Some(AuthMethod::WPA2Enterprise) => WiFiSecurity::WPA2_PERSONAL,
                            _ => WiFiSecurity::WPA2_PERSONAL, // Best guess
                        },
                    };

                    callback(Some(&result))?;

                    info!("Scan result {:?}", result);
                }

                callback(None)?;

                info!("Wifi scan complete");

                Ok(())
            }

            async fn connect(
                &mut self,
                creds: &<Self::Data as WirelessData>::NetworkCredentials,
            ) -> Result<(), Error> {
                let ssid = core::str::from_utf8(creds.ssid.0.vec.as_slice()).unwrap_or("???");

                if self.1.as_ref() == Some(&creds.ssid) && self.0.is_connected().map_err(to_err)? {
                    info!("Wifi connect request for an already connected SSID {ssid}");
                    return Ok(());
                }

                info!("Wifi connect request for SSID {ssid}");

                self.1 = None;

                if self.0.is_started().map_err(to_err)? {
                    self.0.stop_async().await.map_err(to_err)?;
                    info!("Wifi stopped");
                }

                self.0
                    .set_configuration(&Configuration::Client(ClientConfiguration {
                        ssid: ssid.try_into().unwrap(),
                        password: creds.password.clone(),
                        ..Default::default() // TODO: Try something else besides WPA2-Personal
                    }))
                    .map_err(to_err)?;
                info!("Wifi configuration updated");

                self.0.start_async().await.map_err(to_err)?;
                info!("Wifi started");

                self.0.connect_async().await.map_err(to_err)?;

                info!("Wifi connected");

                self.1 = self
                    .0
                    .is_connected()
                    .map_err(to_err)?
                    .then_some(creds.ssid.clone());

                info!("Wifi connect complete");

                Ok(())
            }

            async fn connected_network(
                &mut self,
            ) -> Result<
                Option<
                    <<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
                >,
                Error,
            >{
                Ok(self.1.clone())
            }

            async fn stats(&mut self) -> Result<<Self::Data as WirelessData>::Stats, Error> {
                Ok(None)
            }
        }

        fn to_err(e: WifiError) -> Error {
            error!("Wifi error: {:?}", e);
            Error::new(ErrorCode::NoNetworkInterface)
        }
    }
}

pub mod thread {

    // OpenThread:
    //  ThreadController
    //  ThreadError

    use rs_matter_stack::wireless::traits::{
        Controller,
        // Wifi, WifiData,
        Thread, ThreadData,
        Wireless,
        WirelessTask,
        // NC,
    };

    // use rs_matter_stack::wireless::traits::ThreadId;
    use rs_matter_stack::network::{Embedding, Network};
    // use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::matter::error::Error;

    use super::EmbassyNetContext;
    use super::EmbassyWirelessMatterStack;

    pub type EmbassyThreadMatterStack<'a, E> = EmbassyWirelessMatterStack<'a, Thread, E>;

    pub trait ThreadDriverProvider {
        type Driver<'a>: embassy_net::driver::Driver
        where
            Self: 'a;
        type Controller<'a>: Controller<Data = ThreadData>
        where
            Self: 'a;

        /// Provide a Wifi driver and controller by creating these when the Matter stack needs them
        async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>);
    }


    impl<T> ThreadDriverProvider for &mut T
    where
        T: ThreadDriverProvider,
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

    // pub struct EmbassyThread {
    pub struct EmbassyThread<'a, T> {
        provider: T,
        context: &'a EmbassyNetContext,
        // rand: Rand,
    }

    // impl EmbassyThread
    impl<'a, T> EmbassyThread<'a, T>
    where
        T: ThreadDriverProvider,
    {
        /// Create a new instance of the `EmbassyThread` type.
        pub fn new<E>(provider: T, stack: &'a EmbassyThreadMatterStack<'a, E>) -> Self
        where
            E: Embedding + 'static,
        {
            Self::wrap(
                provider,
                stack.network().embedding().embedding().enet_context(),
                // stack.matter().rand(),
            )
        }

        /// Wrap the `EmbassyWifi` type around a Wifi driver provider and a network context.
        pub const fn wrap(provider: T, context: &'a EmbassyNetContext/*, rand: Rand */) -> Self {
            Self {
                provider,
                context,
                // rand,
            }
        }
    }

    impl<T> Wireless for EmbassyThread<'_, T>
    where
        T: ThreadDriverProvider,
    {
        type Data = ThreadData;

        async fn run<A>(&mut self, mut _task: A) -> Result<(), Error>
        where
            A: WirelessTask<Data = Self::Data>,
        {
            todo!("");
            // Ok(())
        }
    }

//     impl Wireless for EmbassyThread
//     // impl<T> Wireless for EmbassyThread<'_, T>
//     // where
//     //     T: WifiDriverProvider,
//     {
//         // type Data = WifiData;

//         // async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
//         // where
//         //     A: WirelessTask<Data = Self::Data>,
//         // {
//         //     let (driver, controller) = self.provider.provide().await;

//         //     let mut resources = self.context.resources.lock().await;
//         //     let resources = &mut *resources;
//         //     let buffers = &self.context.buffers;

//         //     let mut seed = [0; core::mem::size_of::<u64>()];
//         //     (self.rand)(&mut seed);

//         //     let (stack, mut runner) = create_net_stack(driver, u64::from_le_bytes(seed), resources);

//         //     let netif = EmbassyNetif::new(stack);
//         //     let udp = Udp::new(stack, buffers);

//         //     let mut main = pin!(task.run(netif, udp, controller));
//         //     let mut run = pin!(async {
//         //         runner.run().await;
//         //         #[allow(unreachable_code)]
//         //         Ok(())
//         //     });

//         //     select(&mut main, &mut run).coalesce().await
//         // }
//     }

    #[cfg(feature = "nrf")]
    pub mod nrf {

        // use rs_matter_stack::wireless::traits::ThreadCredentials;
        use rs_matter_stack::wireless::traits::ThreadId;

        // use crate::matter::data_model::sdm::nw_commissioning::ThreadInterfaceScanResult;// {WiFiSecurity, WifiBand};
        use crate::matter::error::{Error, ErrorCode};
        // use crate::matter::tlv::OctetsOwned;
        // use crate::matter::utils::storage::Vec;
        use crate::stack::wireless::traits::{
            Controller, NetworkCredentials, 
            ThreadData,
            // ThreadCredentials,
            // ThreadScanResult,
            // WifiData, WifiScanResult, WifiSsid,
            WirelessData,
        };
    

        pub struct NrfThreadDriverProvider {
        // pub struct NrfThreadDriverProvider<'a, 'd> {
            // controller: &'a nrf_thread::NrfThreadController<'d>,
            // peripheral: PeripheralRef<'d, esp_hal::peripherals::WIFI>,
        }

        // impl<'a, 'd> NrfThreadDriverProvider<'a, 'd> {
        impl NrfThreadDriverProvider {
                /// Create a new instance of the `NrfThreadDriverProvider` type.
            ///
            /// # Arguments
            /// - `controller` - The `esp-wifi` Wifi controller instance.
            /// - `peripheral` - The Wifi peripheral instance.
            pub fn new(
                // controller: &'a nrf_thread::NrfThreadController<'d>,
                // peripheral: impl Peripheral<P = esp_hal::peripherals::WIFI> + 'd,
            ) -> Self {
                Self {
                    // controller,
                    // peripheral: peripheral.into_ref(),
                }
            }
        }

        impl super::ThreadDriverProvider for NrfThreadDriverProvider {
        // impl super::ThreadDriverProvider for NrfThreadDriverProvider<'_, '_> {
                // type Driver<'t>
            //     = WifiDevice<'t, WifiStaDevice>
            // where
            //     Self: 't;
            // type Controller<'t>
            //     = EspWifiController<'t>
            // where
            //     Self: 't;

            async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>) {
                // let (wifi_interface, mut controller) = esp_wifi::wifi::new_with_mode(
                //     self.controller,
                //     &mut self.peripheral,
                //     WifiStaDevice,
                // )
                // .unwrap();

                // // esp32c6-specific - need to boost the power to get a good signal
                // controller
                //     .set_power_saving(esp_wifi::config::PowerSaveMode::None)
                //     .unwrap();

                // (wifi_interface, EspWifiController::new(controller))
            }
        }
        
        /// An adaptor from the OpenThread API to the `rs-matter` Thread controller API
        pub struct NrfThreadController<'a>(ThreadController<'a>, Option<ThreadId>);
        // pub struct EspWifiController<'a>(WifiController<'a>, Option<WifiSsid>);

        impl<'a> NrfThreadController<'a> {
            /// Create a new instance of the `Esp32Controller` type.
            ///
            /// # Arguments
            /// - `controller` - The `esp-wifi` Wifi controller instance.
            pub const fn new(controller: ThreadController<'a>) -> Self {
                Self(controller, None)
            }
        }

        impl Controller for NrfThreadController<'_> {
            type Data = ThreadData;

            async fn scan<F>(
                &mut self,
                _network_id: Option<
                    &<<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
                >,
                mut _callback: F,
            ) -> Result<(), Error>
            where
                F: FnMut(Option<&<Self::Data as WirelessData>::ScanResult>) -> Result<(), Error>,
            {
                // info!("Thread scan request");

                // if !self.0.is_started().map_err(to_err)? {
                //     self.0.start_async().await.map_err(to_err)?;
                //     info!("Wifi started");
                // }

                // let mut scan_config = ScanConfig::default();
                // if let Some(network_id) = network_id {
                //     scan_config.ssid = Some(
                //         core::str::from_utf8(network_id.0.vec.as_slice())
                //             .unwrap_or("???")
                //             .try_into()
                //             .unwrap(),
                //     );
                // }

                // let (aps, len) = self
                //     .0
                //     .scan_with_config_async::<MAX_NETWORKS>(scan_config)
                //     .await
                //     .map_err(to_err)?;

                // info!(
                //     "Wifi scan complete, reporting {} results out of {len} total",
                //     aps.len()
                // );

                // for ap in aps {
                //     let result = WifiScanResult {
                //         ssid: WifiSsid(OctetsOwned {
                //             vec: ap.ssid.as_bytes().try_into().unwrap(),
                //         }),
                //         bssid: OctetsOwned {
                //             vec: Vec::from_slice(&ap.bssid).unwrap(),
                //         },
                //         channel: ap.channel as _,
                //         rssi: Some(ap.signal_strength),
                //         band: Some(WifiBand::B2G4), // TODO: Once c5 is out we can no longer hard-code this
                //         security: match ap.auth_method {
                //             Some(AuthMethod::None) => WiFiSecurity::UNENCRYPTED,
                //             Some(AuthMethod::WEP) => WiFiSecurity::WEP,
                //             Some(AuthMethod::WPA) => WiFiSecurity::WPA_PERSONAL,
                //             Some(AuthMethod::WPA2Personal) => WiFiSecurity::WPA2_PERSONAL,
                //             Some(AuthMethod::WPAWPA2Personal) => {
                //                 WiFiSecurity::WPA_PERSONAL | WiFiSecurity::WPA2_PERSONAL
                //             }
                //             Some(AuthMethod::WPA2WPA3Personal) => {
                //                 WiFiSecurity::WPA2_PERSONAL | WiFiSecurity::WPA3_PERSONAL
                //             }
                //             Some(AuthMethod::WPA2Enterprise) => WiFiSecurity::WPA2_PERSONAL,
                //             _ => WiFiSecurity::WPA2_PERSONAL, // Best guess
                //         },
                //     };

                //     callback(Some(&result))?;

                //     info!("Scan result {:?}", result);
                // }

                // callback(None)?;

                // info!("Wifi scan complete");

                Ok(())
            }

            async fn connect(
                &mut self,
                _creds: &<Self::Data as WirelessData>::NetworkCredentials,
            ) -> Result<(), Error> {
                // let ssid = core::str::from_utf8(creds.ssid.0.vec.as_slice()).unwrap_or("???");

                // if self.1.as_ref() == Some(&creds.ssid) && self.0.is_connected().map_err(to_err)? {
                //     info!("Wifi connect request for an already connected SSID {ssid}");
                //     return Ok(());
                // }

                // info!("Wifi connect request for SSID {ssid}");

                // self.1 = None;

                // if self.0.is_started().map_err(to_err)? {
                //     self.0.stop_async().await.map_err(to_err)?;
                //     info!("Wifi stopped");
                // }

                // self.0
                //     .set_configuration(&Configuration::Client(ClientConfiguration {
                //         ssid: ssid.try_into().unwrap(),
                //         password: creds.password.clone(),
                //         ..Default::default() // TODO: Try something else besides WPA2-Personal
                //     }))
                //     .map_err(to_err)?;
                // info!("Wifi configuration updated");

                // self.0.start_async().await.map_err(to_err)?;
                // info!("Wifi started");

                // self.0.connect_async().await.map_err(to_err)?;

                // info!("Wifi connected");

                // self.1 = self
                //     .0
                //     .is_connected()
                //     .map_err(to_err)?
                //     .then_some(creds.ssid.clone());

                // info!("Wifi connect complete");

                Ok(())
            }

            async fn connected_network(
                &mut self,
            ) -> Result<
                Option<
                    <<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
                >,
                Error,
            >{
                Ok(self.1.clone())
            }

            async fn stats(&mut self) -> Result<<Self::Data as WirelessData>::Stats, Error> {
                // Ok(None)
                Ok(())
            }
        }

        fn to_err(e: ThreadError) -> Error {
            // error!("Thread error: {:?}", e);
            Error::new(ErrorCode::NoNetworkInterface)
        }
    }
}
