use core::pin::pin;

use embassy_futures::select::{select, select4};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_time::{Duration, Instant, Timer};

use log::{error, info};

use openthread::{Channels, OpenThread, OtError, Radio};

use rs_matter::utils::rand::Rand;
use rs_matter::utils::sync::IfMutex;
use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::tlv::OctetsOwned;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::mdns::MatterMdnsServices;
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::persist::{KvBlobStore, SharedKvBlobStore};
use rs_matter_stack::rand::MatterRngCore;
use rs_matter_stack::wireless::{
    Controller, Gatt, GattTask, NetworkCredentials, Thread, ThreadData, ThreadId, ThreadScanResult,
    Wireless, WirelessCoex, WirelessCoexTask, WirelessData, WirelessTask,
};

use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::ot::OtPersist;
use crate::ot::{OtMatterResources, OtMdns, OtNetif};

use super::{BleDriver, BleDriverTask, EmbassyWirelessMatterStack};

/// A type alias for an Embassy Matter stack running over Thread (and BLE, during commissioning).
pub type EmbassyThreadMatterStack<'a, E = ()> =
    EmbassyWirelessMatterStack<'a, Thread, OtNetContext, E>;

/// A trait representing a task that needs access to the Thread radio to perform its work
pub trait ThreadDriverTask {
    /// Run the task with the given Thread radio
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio;
}

impl<T> ThreadDriverTask for &mut T
where
    T: ThreadDriverTask,
{
    async fn run<R>(&mut self, radio: R) -> Result<(), Error>
    where
        R: Radio,
    {
        (*self).run(radio).await
    }
}

/// A trait representing a task that needs access to the Thread radio,
/// as well as to the BLE controller to perform its work
pub trait ThreadCoexDriverTask {
    /// Run the task with the given Thread radio and BLE controller
    async fn run<R, B>(&mut self, radio: R, ble_controller: B) -> Result<(), Error>
    where
        R: Radio,
        B: trouble_host::Controller;
}

impl<T> ThreadCoexDriverTask for &mut T
where
    T: ThreadCoexDriverTask,
{
    async fn run<R, B>(&mut self, radio: R, ble_controller: B) -> Result<(), Error>
    where
        R: Radio,
        B: trouble_host::Controller,
    {
        (*self).run(radio, ble_controller).await
    }
}

/// A trait for running a task within a context where the Thread radio is initialized and operable
pub trait ThreadDriver {
    /// Setup the Thread radio and run the given task with it
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadDriverTask;
}

impl<T> ThreadDriver for &mut T
where
    T: ThreadDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadDriverTask,
    {
        (*self).run(task).await
    }
}

/// A trait for running a task within a context where the Thread radio - as well as the BLE controller - are initialized and operable
pub trait ThreadCoexDriver {
    /// Setup the Thread radio and the BLE controller and run the given task with these
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadCoexDriverTask;
}

impl<T> ThreadCoexDriver for &mut T
where
    T: ThreadCoexDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: ThreadCoexDriverTask,
    {
        (*self).run(task).await
    }
}

/// A Thread radio provider that uses a pre-existing, already created Thread radio,
/// rather than creating it when the Matter stack needs it.
pub struct PreexistingThreadDriver<R, B>(R, B);

impl<R, B> PreexistingThreadDriver<R, B> {
    /// Create a new instance of the `PreexistingThreadRadio` type.
    pub const fn new(radio: R, ble: B) -> Self {
        Self(radio, ble)
    }
}

impl<R, B> ThreadDriver for PreexistingThreadDriver<R, B>
where
    R: Radio,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: ThreadDriverTask,
    {
        task.run(&mut self.0).await
    }
}

impl<R, B> ThreadCoexDriver for PreexistingThreadDriver<R, B>
where
    R: Radio,
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: ThreadCoexDriverTask,
    {
        task.run(&mut self.0, ControllerRef::new(&self.1)).await
    }
}

impl<R, B> BleDriver for PreexistingThreadDriver<R, B>
where
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        task.run(ControllerRef::new(&self.1)).await
    }
}

/// A `Wireless` trait implementation for `openthread`'s Thread stack.
pub struct EmbassyThread<'a, T, S> {
    driver: T,
    mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
    ieee_eui64: [u8; 8],
    store: &'a SharedKvBlobStore<'a, S>,
    context: &'a OtNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    rand: Rand,
}

impl<'a, T, S> EmbassyThread<'a, T, S>
where
    T: ThreadDriver,
    S: KvBlobStore,
{
    /// Create a new instance of the `EmbassyThread` type.
    pub fn new<E>(
        driver: T,
        mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
        ieee_eui64: [u8; 8],
        store: &'a SharedKvBlobStore<'a, S>,
        stack: &'a EmbassyThreadMatterStack<'a, E>,
    ) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            driver,
            mdns_services,
            ieee_eui64,
            store,
            stack.network().embedding().net_context(),
            stack.network().embedding().ble_context(),
            stack.matter().rand(),
        )
    }

    /// Wrap an existing `ThreadDriver` with the given parameters.
    pub fn wrap(
        driver: T,
        mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
        ieee_eui64: [u8; 8],
        store: &'a SharedKvBlobStore<'a, S>,
        context: &'a OtNetContext,
        ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
        rand: Rand,
    ) -> Self {
        Self {
            driver,
            mdns_services,
            ieee_eui64,
            store,
            context,
            ble_context,
            rand,
        }
    }
}

impl<T, S> Wireless for EmbassyThread<'_, T, S>
where
    T: ThreadDriver,
    S: KvBlobStore,
{
    type Data = ThreadData;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WirelessTask<Data = Self::Data>,
    {
        fn to_matter_err(_err: OtError) -> Error {
            Error::new(ErrorCode::NoNetworkInterface)
        }

        struct ThreadDriverTaskImpl<'a, A, S> {
            mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
            ieee_eui64: [u8; 8],
            rand: Rand,
            store: &'a SharedKvBlobStore<'a, S>,
            context: &'a OtNetContext,
            task: A,
        }

        impl<A, S> ThreadDriverTask for ThreadDriverTaskImpl<'_, A, S>
        where
            A: WirelessTask<Data = ThreadData>,
            S: KvBlobStore,
        {
            async fn run<R>(&mut self, radio: R) -> Result<(), Error>
            where
                R: Radio,
            {
                let mut rng = MatterRngCore::new(self.rand);
                let mut resources = self.context.resources.lock().await;
                let resources = &mut *resources;

                let persister = OtPersist::new(&mut resources.settings_buf, self.store);
                persister.load().await?;

                let mut settings = persister.settings();

                let ot = OpenThread::new_with_udp_srp(
                    self.ieee_eui64,
                    &mut rng,
                    &mut settings,
                    &mut resources.ot,
                    &mut resources.udp,
                    &mut resources.srp,
                )
                .map_err(to_matter_err)?;

                let controller = OtController(ot.clone());
                let netif = OtNetif::new(ot.clone());
                let mdns = OtMdns::new(ot.clone(), self.mdns_services).map_err(to_matter_err)?;

                let mut main = pin!(self.task.run(netif, ot.clone(), controller));
                let mut radio = pin!(async {
                    ot.run(radio).await;
                    #[allow(unreachable_code)]
                    Ok(())
                });
                let mut mdns = pin!(async { mdns.run().await.map_err(to_matter_err) });
                let mut persist = pin!(persister.run());
                ot.enable_ipv6(true).map_err(to_matter_err)?;
                ot.srp_autostart().map_err(to_matter_err)?;

                let result = select4(&mut main, &mut radio, &mut mdns, &mut persist)
                    .coalesce()
                    .await;

                let _ = ot.enable_thread(false);
                let _ = ot.srp_stop();
                let _ = ot.enable_ipv6(false);

                result
            }
        }

        self.driver
            .run(ThreadDriverTaskImpl {
                mdns_services: self.mdns_services,
                ieee_eui64: self.ieee_eui64,
                rand: self.rand,
                store: self.store,
                context: self.context,
                task,
            })
            .await
    }
}

impl<T, S> WirelessCoex for EmbassyThread<'_, T, S>
where
    T: ThreadCoexDriver,
    S: KvBlobStore,
{
    type Data = ThreadData;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WirelessCoexTask<Data = Self::Data>,
    {
        fn to_matter_err(_err: OtError) -> Error {
            Error::new(ErrorCode::NoNetworkInterface)
        }

        struct ThreadCoexDriverTaskImpl<'a, A, S> {
            mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
            ieee_eui64: [u8; 8],
            rand: Rand,
            store: &'a SharedKvBlobStore<'a, S>,
            context: &'a OtNetContext,
            ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
            task: A,
        }

        impl<A, S> ThreadCoexDriverTask for ThreadCoexDriverTaskImpl<'_, A, S>
        where
            A: WirelessCoexTask<Data = ThreadData>,
            S: KvBlobStore,
        {
            async fn run<R, B>(&mut self, radio: R, ble_controller: B) -> Result<(), Error>
            where
                R: Radio,
                B: trouble_host::Controller,
            {
                let mut rng = MatterRngCore::new(self.rand);
                let mut resources = self.context.resources.lock().await;
                let resources = &mut *resources;

                let persister = OtPersist::new(&mut resources.settings_buf, self.store);
                persister.load().await?;

                let mut settings = persister.settings();

                let ot = OpenThread::new_with_udp_srp(
                    self.ieee_eui64,
                    &mut rng,
                    &mut settings,
                    &mut resources.ot,
                    &mut resources.udp,
                    &mut resources.srp,
                )
                .map_err(to_matter_err)?;

                let controller = OtController(ot.clone());
                let netif = OtNetif::new(ot.clone());
                let mdns = OtMdns::new(ot.clone(), self.mdns_services).map_err(to_matter_err)?;

                let peripheral =
                    TroubleBtpGattPeripheral::new(ble_controller, self.rand, self.ble_context);

                let mut main = pin!(self.task.run(netif, ot.clone(), controller, peripheral));
                let mut radio = pin!(async {
                    ot.run(radio).await;
                    #[allow(unreachable_code)]
                    Ok(())
                });
                let mut mdns = pin!(async { mdns.run().await.map_err(to_matter_err) });
                let mut persist = pin!(persister.run());
                ot.enable_ipv6(true).map_err(to_matter_err)?;
                ot.srp_autostart().map_err(to_matter_err)?;

                let result = select4(&mut main, &mut radio, &mut mdns, &mut persist)
                    .coalesce()
                    .await;

                let _ = ot.enable_thread(false);
                let _ = ot.srp_stop();
                let _ = ot.enable_ipv6(false);

                result
            }
        }

        self.driver
            .run(ThreadCoexDriverTaskImpl {
                mdns_services: self.mdns_services,
                ieee_eui64: self.ieee_eui64,
                rand: self.rand,
                store: self.store,
                context: self.context,
                ble_context: self.ble_context,
                task,
            })
            .await
    }
}

impl<T, S> Gatt for EmbassyThread<'_, T, S>
where
    T: BleDriver,
    S: KvBlobStore,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: GattTask,
    {
        struct BleDriverTaskImpl<'a, A> {
            task: A,
            rand: Rand,
            context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
        }

        impl<A> BleDriverTask for BleDriverTaskImpl<'_, A>
        where
            A: GattTask,
        {
            async fn run<C>(&mut self, controller: C) -> Result<(), Error>
            where
                C: trouble_host::Controller,
            {
                let peripheral = TroubleBtpGattPeripheral::new(controller, self.rand, self.context);

                self.task.run(&peripheral).await
            }
        }

        self.driver
            .run(BleDriverTaskImpl {
                task,
                rand: self.rand,
                context: self.ble_context,
            })
            .await
    }
}

/// A network context for the `EmbassyThread` type.
pub struct OtNetContext {
    resources: IfMutex<NoopRawMutex, OtMatterResources>,
}

impl OtNetContext {
    /// Create a new instance of the `OtNetContext` type.
    pub const fn new() -> Self {
        Self {
            resources: IfMutex::new(OtMatterResources::new()),
        }
    }

    /// Return an in-place initializer for the `OtNetContext` type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            resources <- IfMutex::init(OtMatterResources::init()),
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
                                    .unwrap_or(rs_matter_stack::matter::utils::storage::Vec::new()),
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
        const TIMEOUT_SECS: u64 = 20;

        let _ = self.0.enable_thread(false);

        // NOTE: Printing the dataset is a security issue, but we do it for now for debugging purposes
        // (i.e. for running some of the pseudo-eth examples the user needs the Thread network dataset)
        use hex_slice::AsHex;
        info!(
            "Connecting to Thread network, dataset: {:02x}",
            creds.op_dataset.plain_hex(false)
        );

        self.0
            .set_active_dataset_tlv(&creds.op_dataset)
            .map_err(to_matter_err)?;

        self.0.enable_thread(true).map_err(to_matter_err)?;

        let now = Instant::now();

        while !self.0.net_status().role.is_connected() {
            if now.elapsed().as_secs() > TIMEOUT_SECS {
                let _ = self.0.enable_thread(false);

                return Err(ErrorCode::NoNetworkInterface.into());
            }

            select(self.0.wait_changed(), Timer::after(Duration::from_secs(1))).await;
        }

        Ok(())
    }

    async fn connected_network(
        &mut self,
    ) -> Result<
        Option<<<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId>,
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

#[cfg(feature = "esp")]
pub mod esp_thread {
    use bt_hci::controller::ExternalController;
    use esp_hal::peripheral::{Peripheral, PeripheralRef};

    use esp_wifi::ble::controller::BleConnector;
    use openthread::esp::EspRadio;

    use rs_matter_stack::matter::error::Error;

    use crate::wireless::SLOTS;

    /// A `ThreadRadio` implementation for the ESP32 family of chips.
    pub struct EspThreadDriver<'a, 'd> {
        controller: &'a esp_wifi::EspWifiController<'d>,
        _radio_peripheral: PeripheralRef<'d, esp_hal::peripherals::IEEE802154>,
        bt_peripheral: PeripheralRef<'d, esp_hal::peripherals::BT>,
    }

    impl<'a, 'd> EspThreadDriver<'a, 'd> {
        /// Create a new instance of the `EspThreadRadio` type.
        ///
        /// # Arguments
        /// - `peripheral` - The Thread radio peripheral instance.
        pub fn new(
            controller: &'a esp_wifi::EspWifiController<'d>,
            radio_peripheral: impl Peripheral<P = esp_hal::peripherals::IEEE802154> + 'd,
            bt_peripheral: impl Peripheral<P = esp_hal::peripherals::BT> + 'd,
        ) -> Self {
            Self {
                controller,
                _radio_peripheral: radio_peripheral.into_ref(),
                bt_peripheral: bt_peripheral.into_ref(),
            }
        }
    }

    impl super::ThreadDriver for EspThreadDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::ThreadDriverTask,
        {
            // See https://github.com/esp-rs/esp-hal/issues/3238
            //EspRadio::new(openthread::esp::Ieee802154::new(&mut self.radio_peripheral, &mut self.radio_clk_peripheral))
            let radio = EspRadio::new(openthread::esp::Ieee802154::new(
                unsafe { esp_hal::peripherals::IEEE802154::steal() },
                unsafe { esp_hal::peripherals::RADIO_CLK::steal() },
            ));

            task.run(radio).await
        }
    }

    impl super::ThreadCoexDriver for EspThreadDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::ThreadCoexDriverTask,
        {
            // See https://github.com/esp-rs/esp-hal/issues/3238
            //EspRadio::new(openthread::esp::Ieee802154::new(&mut self.radio_peripheral, &mut self.radio_clk_peripheral))
            let radio = EspRadio::new(openthread::esp::Ieee802154::new(
                unsafe { esp_hal::peripherals::IEEE802154::steal() },
                unsafe { esp_hal::peripherals::RADIO_CLK::steal() },
            ));

            let ble_controller = ExternalController::<_, SLOTS>::new(BleConnector::new(
                self.controller,
                &mut self.bt_peripheral,
            ));

            task.run(radio, ble_controller).await
        }
    }

    impl super::BleDriver for EspThreadDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::BleDriverTask,
        {
            let ble_controller = ExternalController::<_, SLOTS>::new(BleConnector::new(
                self.controller,
                &mut self.bt_peripheral,
            ));

            task.run(ble_controller).await
        }
    }
}

#[cfg(feature = "nrf")]
pub mod nrf {
    use core::pin::pin;

    use embassy_futures::select::select;

    use embassy_nrf::interrupt;
    use embassy_nrf::interrupt::typelevel::Interrupt;
    use embassy_nrf::interrupt::typelevel::{Binding, Handler};
    use embassy_nrf::into_ref;
    use embassy_nrf::peripherals::{
        PPI_CH17, PPI_CH18, PPI_CH19, PPI_CH20, PPI_CH21, PPI_CH22, PPI_CH23, PPI_CH24, PPI_CH25,
        PPI_CH26, PPI_CH27, PPI_CH28, PPI_CH29, PPI_CH30, PPI_CH31, RADIO, RTC0, TEMP, TIMER0,
    };
    use embassy_nrf::radio::InterruptHandler;
    use embassy_nrf::{Peripheral, PeripheralRef};

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    use log::info;

    pub use nrf_sdc::mpsl::{
        ClockInterruptHandler as NrfBleClockInterruptHandler,
        HighPrioInterruptHandler as NrfBleHighPrioInterruptHandler,
        LowPrioInterruptHandler as NrfBleLowPrioInterruptHandler,
    };

    use openthread::nrf::Ieee802154Peripheral;
    use openthread::nrf::NrfRadio;
    pub use openthread::ProxyRadioResources as NrfThreadRadioResources;
    use openthread::{EmbassyTimeTimer, PhyRadioRunner, ProxyRadio, ProxyRadioResources};

    use portable_atomic::{AtomicBool, Ordering};

    use rs_matter_stack::matter::error::{Error, ErrorCode};
    use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::matter::utils::sync::Signal;
    use rs_matter_stack::rand::MatterRngCore;

    use crate::ble::MAX_MTU_SIZE;

    pub struct NrfThreadRadioInterruptHandler;

    impl Handler<interrupt::typelevel::RADIO> for NrfThreadRadioInterruptHandler {
        unsafe fn on_interrupt() {
            if NRF_THREAD_RADIO_STATE.irq_enabled() {
                // Call the IEEE 802.15.4 driver interrupt handler, if the driver is enabled
                InterruptHandler::<embassy_nrf::peripherals::RADIO>::on_interrupt();
            }
        }
    }

    struct NrfThreadRadioState {
        irq_enabled: AtomicBool,
        enable: Signal<CriticalSectionRawMutex, bool>,
        state: Signal<CriticalSectionRawMutex, bool>,
    }

    impl NrfThreadRadioState {
        const fn new() -> Self {
            Self {
                irq_enabled: AtomicBool::new(false),
                enable: Signal::new(false),
                state: Signal::new(false),
            }
        }

        fn irq_enabled(&self) -> bool {
            self.irq_enabled.load(Ordering::SeqCst)
        }

        fn set_enabled(&self, enabled: bool) {
            self.irq_enabled.store(enabled, Ordering::SeqCst);
            self.enable.modify(|state| {
                if *state != enabled {
                    *state = enabled;
                    (true, ())
                } else {
                    (false, ())
                }
            });
        }

        fn set_enabled_state(&self, enabled: bool) {
            self.state.modify(|state| {
                if *state != enabled {
                    *state = enabled;
                    (true, ())
                } else {
                    (false, ())
                }
            });
        }

        async fn wait_enabled(&self, enabled: bool) {
            self.enable
                .wait(|state| (*state == enabled).then_some(()))
                .await;
        }

        pub(crate) async fn wait_enabled_state(&self, enabled: bool) {
            self.state
                .wait(|state| (*state == enabled).then_some(()))
                .await;
        }
    }

    static NRF_THREAD_RADIO_STATE: NrfThreadRadioState = NrfThreadRadioState::new();

    struct NrfThreadRadioInterrupts;

    unsafe impl Binding<<RADIO as Ieee802154Peripheral>::Interrupt, InterruptHandler<RADIO>>
        for NrfThreadRadioInterrupts
    {
    }

    /// A runner for the NRF52 PHY radio
    /// Needs to run in a high-prio execution context
    pub struct NrfThreadRadioRunner<'a, 'd> {
        runner: PhyRadioRunner<'a>,
        radio_peripheral: PeripheralRef<'d, RADIO>,
    }

    impl<'a, 'd> NrfThreadRadioRunner<'a, 'd> {
        fn new(
            runner: PhyRadioRunner<'a>,
            radio_peripheral: impl Peripheral<P = RADIO> + 'd,
        ) -> Self {
            Self {
                runner,
                radio_peripheral: radio_peripheral.into_ref(),
            }
        }

        /// Run the PHY radio
        pub async fn run(&mut self) -> ! {
            loop {
                NRF_THREAD_RADIO_STATE.wait_enabled(true).await;

                {
                    NRF_THREAD_RADIO_STATE.set_enabled_state(true);

                    info!("Thread radio started");

                    let radio = NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
                        &mut self.radio_peripheral,
                        NrfThreadRadioInterrupts,
                    ));

                    let mut cmd = pin!(NRF_THREAD_RADIO_STATE.wait_enabled(false));
                    let mut runner = pin!(self.runner.run(radio, EmbassyTimeTimer));

                    select(&mut cmd, &mut runner).await;

                    NRF_THREAD_RADIO_STATE.set_enabled_state(false);

                    info!("Thread radio stopped");
                }
            }
        }
    }

    /// A `ThreadDriver` implementation for the NRF52 family of chips.
    pub struct NrfThreadDriver<'d> {
        proxy: ProxyRadio<'d>,
        rtc0: PeripheralRef<'d, RTC0>,
        timer0: PeripheralRef<'d, TIMER0>,
        temp: PeripheralRef<'d, TEMP>,
        ppi_ch17: PeripheralRef<'d, PPI_CH17>,
        ppi_ch18: PeripheralRef<'d, PPI_CH18>,
        ppi_ch19: PeripheralRef<'d, PPI_CH19>,
        ppi_ch20: PeripheralRef<'d, PPI_CH20>,
        ppi_ch21: PeripheralRef<'d, PPI_CH21>,
        ppi_ch22: PeripheralRef<'d, PPI_CH22>,
        ppi_ch23: PeripheralRef<'d, PPI_CH23>,
        ppi_ch24: PeripheralRef<'d, PPI_CH24>,
        ppi_ch25: PeripheralRef<'d, PPI_CH25>,
        ppi_ch26: PeripheralRef<'d, PPI_CH26>,
        ppi_ch27: PeripheralRef<'d, PPI_CH27>,
        ppi_ch28: PeripheralRef<'d, PPI_CH28>,
        ppi_ch29: PeripheralRef<'d, PPI_CH29>,
        ppi_ch30: PeripheralRef<'d, PPI_CH30>,
        ppi_ch31: PeripheralRef<'d, PPI_CH31>,
        rand: Rand,
    }

    impl<'d> NrfThreadDriver<'d> {
        /// Create a new instance of the `NrfThreadRadio` type.
        ///
        /// # Arguments
        /// - `resources` - The resources for the radio proxying
        /// - `radio_peripheral` - The radio peripheral instance
        /// - `irq` - The radio interrupt binding
        #[allow(clippy::too_many_arguments)]
        pub fn new<T, I>(
            resources: &'d mut ProxyRadioResources,
            radio: impl Peripheral<P = RADIO> + 'd,
            rtc0: impl Peripheral<P = RTC0> + 'd,
            timer0: impl Peripheral<P = TIMER0> + 'd,
            temp: impl Peripheral<P = TEMP> + 'd,
            ppi_ch17: impl Peripheral<P = PPI_CH17> + 'd,
            ppi_ch18: impl Peripheral<P = PPI_CH18> + 'd,
            ppi_ch19: impl Peripheral<P = PPI_CH19> + 'd,
            ppi_ch20: impl Peripheral<P = PPI_CH20> + 'd,
            ppi_ch21: impl Peripheral<P = PPI_CH21> + 'd,
            ppi_ch22: impl Peripheral<P = PPI_CH22> + 'd,
            ppi_ch23: impl Peripheral<P = PPI_CH23> + 'd,
            ppi_ch24: impl Peripheral<P = PPI_CH24> + 'd,
            ppi_ch25: impl Peripheral<P = PPI_CH25> + 'd,
            ppi_ch26: impl Peripheral<P = PPI_CH26> + 'd,
            ppi_ch27: impl Peripheral<P = PPI_CH27> + 'd,
            ppi_ch28: impl Peripheral<P = PPI_CH28> + 'd,
            ppi_ch29: impl Peripheral<P = PPI_CH29> + 'd,
            ppi_ch30: impl Peripheral<P = PPI_CH30> + 'd,
            ppi_ch31: impl Peripheral<P = PPI_CH31> + 'd,
            rand: Rand,
            _irqs: I,
        ) -> (Self, NrfThreadRadioRunner<'d, 'd>)
        where
            T: Interrupt,
            I: Binding<T, NrfBleLowPrioInterruptHandler>
                + Binding<interrupt::typelevel::RADIO, NrfBleHighPrioInterruptHandler>
                + Binding<interrupt::typelevel::TIMER0, NrfBleHighPrioInterruptHandler>
                + Binding<interrupt::typelevel::RTC0, NrfBleHighPrioInterruptHandler>
                + Binding<interrupt::typelevel::CLOCK_POWER, NrfBleClockInterruptHandler>
                + Binding<
                    <embassy_nrf::peripherals::RADIO as Ieee802154Peripheral>::Interrupt,
                    NrfThreadRadioInterruptHandler,
                >,
        {
            let caps = openthread::Capabilities::empty();
            // TODO: A bit dirty as we create it and then drop it immediately
            // NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
            //     &mut radio_peripheral,
            //     NrfThreadRadioInterrupts,
            // ))
            // .caps();

            let (proxy, proxy_runner) = ProxyRadio::new(caps, resources);

            let runner = NrfThreadRadioRunner::new(proxy_runner, radio);

            into_ref!(
                rtc0, timer0, temp, ppi_ch17, ppi_ch18, ppi_ch19, ppi_ch20, ppi_ch21, ppi_ch22,
                ppi_ch23, ppi_ch24, ppi_ch25, ppi_ch26, ppi_ch27, ppi_ch28, ppi_ch29, ppi_ch30,
                ppi_ch31
            );

            (
                Self {
                    proxy,
                    rtc0,
                    timer0,
                    temp,
                    ppi_ch17,
                    ppi_ch18,
                    ppi_ch19,
                    ppi_ch20,
                    ppi_ch21,
                    ppi_ch22,
                    ppi_ch23,
                    ppi_ch24,
                    ppi_ch25,
                    ppi_ch26,
                    ppi_ch27,
                    ppi_ch28,
                    ppi_ch29,
                    ppi_ch30,
                    ppi_ch31,
                    rand,
                },
                runner,
            )
        }
    }

    impl super::ThreadDriver for NrfThreadDriver<'_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::ThreadDriverTask,
        {
            info!("About to enable Thread radio");

            let _guard = scopeguard::guard((), |_| NRF_THREAD_RADIO_STATE.set_enabled(false));

            NRF_THREAD_RADIO_STATE.set_enabled(true);
            NRF_THREAD_RADIO_STATE.wait_enabled_state(true).await;

            info!("Running Thread radio task");

            task.run(&mut self.proxy).await
        }
    }

    impl super::BleDriver for NrfThreadDriver<'_> {
        async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
        where
            T: super::BleDriverTask,
        {
            NRF_THREAD_RADIO_STATE.set_enabled(false);
            NRF_THREAD_RADIO_STATE.wait_enabled_state(false).await;

            let mpsl_p = nrf_sdc::mpsl::Peripherals::new(
                &mut self.rtc0,
                &mut self.timer0,
                &mut self.temp,
                &mut self.ppi_ch19,
                &mut self.ppi_ch30,
                &mut self.ppi_ch31,
            );

            let sdc_p = nrf_sdc::Peripherals::new(
                &mut self.ppi_ch17,
                &mut self.ppi_ch18,
                &mut self.ppi_ch20,
                &mut self.ppi_ch21,
                &mut self.ppi_ch22,
                &mut self.ppi_ch23,
                &mut self.ppi_ch24,
                &mut self.ppi_ch25,
                &mut self.ppi_ch26,
                &mut self.ppi_ch27,
                &mut self.ppi_ch28,
                &mut self.ppi_ch29,
            );

            let lfclk_cfg = nrf_sdc::mpsl::raw::mpsl_clock_lfclk_cfg_t {
                source: nrf_sdc::mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
                rc_ctiv: nrf_sdc::mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
                rc_temp_ctiv: nrf_sdc::mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
                accuracy_ppm: nrf_sdc::mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
                skip_wait_lfclk_started: nrf_sdc::mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED
                    != 0,
            };

            let mpsl = nrf_sdc::mpsl::MultiprotocolServiceLayer::new::<
                interrupt::typelevel::EGU0_SWI0,
                _,
            >(mpsl_p, NrfBleControllerInterrupts, lfclk_cfg)
            .map_err(to_matter_err)?;

            // TODO: Externalize as resources
            // Mem is roughly MAC_CONNECTIONS * MAX_MTU_SIZE * L2CAP_TXQ * L2CAP_RXQ
            let mut sdc_mem = nrf_sdc::Mem::<3084>::new();

            let mut rng = MatterRngCore::new(self.rand);

            let controller = nrf_sdc::Builder::new()
                .map_err(to_matter_err)?
                .support_adv()
                .map_err(to_matter_err)?
                .support_peripheral()
                .map_err(to_matter_err)?
                .peripheral_count(1)
                .map_err(to_matter_err)?
                .buffer_cfg(MAX_MTU_SIZE as u8, MAX_MTU_SIZE as u8, L2CAP_TXQ, L2CAP_RXQ)
                .map_err(to_matter_err)?
                .build(sdc_p, &mut rng, &mpsl, &mut sdc_mem)
                .map_err(to_matter_err)?;

            task.run(controller).await
        }
    }

    /// How many outgoing L2CAP buffers per link
    const L2CAP_TXQ: u8 = 3;
    /// How many incoming L2CAP buffers per link
    const L2CAP_RXQ: u8 = 3;

    struct NrfBleControllerInterrupts;

    unsafe impl<T> Binding<T, NrfBleLowPrioInterruptHandler> for NrfBleControllerInterrupts where
        T: interrupt::typelevel::Interrupt
    {
    }

    unsafe impl Binding<interrupt::typelevel::RADIO, NrfBleHighPrioInterruptHandler>
        for NrfBleControllerInterrupts
    {
    }
    unsafe impl Binding<interrupt::typelevel::TIMER0, NrfBleHighPrioInterruptHandler>
        for NrfBleControllerInterrupts
    {
    }
    unsafe impl Binding<interrupt::typelevel::RTC0, NrfBleHighPrioInterruptHandler>
        for NrfBleControllerInterrupts
    {
    }
    unsafe impl Binding<interrupt::typelevel::CLOCK_POWER, NrfBleClockInterruptHandler>
        for NrfBleControllerInterrupts
    {
    }

    fn to_matter_err<E>(_: E) -> Error {
        Error::new(ErrorCode::BtpError)
    }
}
