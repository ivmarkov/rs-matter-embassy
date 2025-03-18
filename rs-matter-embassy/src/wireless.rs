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

/// A trait representing a task that needs access to the BLE controller to perform its work
pub trait BleControllerTask {
    /// Run the task with the given BLE controller
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: Controller;
}

impl<T> BleControllerTask for &mut T
where
    T: BleControllerTask,
{
    async fn run<C>(&mut self, controller: C) -> Result<(), Error>
    where
        C: Controller,
    {
        (*self).run(controller).await
    }
}

/// A trait for running a task within a context where the BLE Controller is initialized and operable
/// (e.g. in a commissioning workflow)
pub trait BleController {
    /// Setup the BLE controller and run the given task with it
    async fn run<T>(&mut self, task: T) -> Result<(), Error>
    where
        T: BleControllerTask;
}

impl<T> BleController for &mut T
where
    T: BleController,
{
    async fn run<U>(&mut self, task: U) -> Result<(), Error>
    where
        U: BleControllerTask,
    {
        (*self).run(task).await
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

impl<C> BleController for PreexistingBleController<C>
where
    C: Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleControllerTask,
    {
        let controller = ControllerRef::new(&self.0);

        task.run(controller).await
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
    T: BleController,
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
    T: BleController,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: BleTask,
    {
        struct BleControllerTaskImpl<'a, A> {
            task: A,
            rand: Rand,
            context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
        }

        impl<A> BleControllerTask for BleControllerTaskImpl<'_, A>
        where
            A: BleTask,
        {
            async fn run<C>(&mut self, controller: C) -> Result<(), Error>
            where
                C: Controller,
            {
                let peripheral = TroubleBtpGattPeripheral::new(controller, self.rand, self.context);

                self.task.run(&peripheral).await
            }
        }

        self.provider
            .run(BleControllerTaskImpl {
                task,
                rand: self.rand,
                context: self.context,
            })
            .await
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

    use rs_matter_stack::matter::error::Error;

    const SLOTS: usize = 20;

    /// A `BleController` implementation for the ESP32 family of chips.
    pub struct EspBleController<'a, 'd> {
        controller: &'a EspWifiController<'d>,
        peripheral: PeripheralRef<'d, esp_hal::peripherals::BT>,
    }

    impl<'a, 'd> EspBleController<'a, 'd> {
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

    impl super::BleController for EspBleController<'_, '_> {
        async fn run<T>(&mut self, mut task: T) -> Result<(), Error>
        where
            T: super::BleControllerTask,
        {
            let controller = ExternalController::<_, SLOTS>::new(BleConnector::new(
                self.controller,
                &mut self.peripheral,
            ));

            task.run(controller).await
        }
    }
}

#[cfg(feature = "nrf")]
pub mod nrf {
    use embassy_nrf::interrupt::typelevel::Interrupt;
    use embassy_nrf::interrupt::{self, typelevel::Binding};
    use embassy_nrf::into_ref;
    use embassy_nrf::peripherals::{
        PPI_CH17, PPI_CH18, PPI_CH19, PPI_CH20, PPI_CH21, PPI_CH22, PPI_CH23, PPI_CH24, PPI_CH25,
        PPI_CH26, PPI_CH27, PPI_CH28, PPI_CH29, PPI_CH30, PPI_CH31, RADIO, RTC0, TEMP, TIMER0,
    };
    use embassy_nrf::Peripheral;
    use embassy_nrf::PeripheralRef;

    pub use nrf_sdc::mpsl::{
        ClockInterruptHandler as NrfBleClockInterruptHandler,
        HighPrioInterruptHandler as NrfBleHighPrioInterruptHandler,
        LowPrioInterruptHandler as NrfBleLowPrioInterruptHandler,
    };

    use rs_matter_stack::matter::error::{Error, ErrorCode};
    use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::rand::MatterRngCore;

    use super::BleController;

    /// How many outgoing L2CAP buffers per link
    const L2CAP_TXQ: u8 = 3;
    /// How many incoming L2CAP buffers per link
    const L2CAP_RXQ: u8 = 3;
    /// Size of L2CAP packets
    const L2CAP_MTU: usize = 127; // TODO: 27 does not work

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

    pub struct NrfBleController<'d> {
        _radio: PeripheralRef<'d, RADIO>,
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

    impl<'d> NrfBleController<'d> {
        #[allow(clippy::too_many_arguments)]
        pub fn new<T, I>(
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
        ) -> Self
        where
            T: Interrupt,
            I: Binding<T, NrfBleLowPrioInterruptHandler>
                + Binding<interrupt::typelevel::RADIO, NrfBleHighPrioInterruptHandler>
                + Binding<interrupt::typelevel::TIMER0, NrfBleHighPrioInterruptHandler>
                + Binding<interrupt::typelevel::RTC0, NrfBleHighPrioInterruptHandler>
                + Binding<interrupt::typelevel::CLOCK_POWER, NrfBleClockInterruptHandler>,
        {
            into_ref!(
                radio, rtc0, timer0, temp, ppi_ch17, ppi_ch18, ppi_ch19, ppi_ch20, ppi_ch21,
                ppi_ch22, ppi_ch23, ppi_ch24, ppi_ch25, ppi_ch26, ppi_ch27, ppi_ch28, ppi_ch29,
                ppi_ch30, ppi_ch31
            );

            Self {
                _radio: radio,
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
            }
        }
    }

    impl<'d> BleController for NrfBleController<'d> {
        async fn run<T>(&mut self, mut task: T) -> Result<(), rs_matter_stack::matter::error::Error>
        where
            T: super::BleControllerTask,
        {
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
            let mut sdc_mem = nrf_sdc::Mem::<3312>::new();

            let mut rng = MatterRngCore::new(self.rand);

            let controller = nrf_sdc::Builder::new()
                .map_err(to_matter_err)?
                .support_adv()
                .map_err(to_matter_err)?
                .support_peripheral()
                .map_err(to_matter_err)?
                .peripheral_count(1)
                .map_err(to_matter_err)?
                .buffer_cfg(L2CAP_MTU as u8, L2CAP_MTU as u8, L2CAP_TXQ, L2CAP_RXQ)
                .map_err(to_matter_err)?
                .build(sdc_p, &mut rng, &mpsl, &mut sdc_mem)
                .map_err(to_matter_err)?;

            task.run(controller).await
        }
    }

    fn to_matter_err<E>(_: E) -> Error {
        Error::new(ErrorCode::BtpError)
    }
}

// Thread: Type aliases and state structs for an Embassy Matter stack running over a Thread network and BLE.
#[cfg(feature = "openthread")]
pub mod thread {
    use core::pin::pin;

    use embassy_futures::select::{select, select3};
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    use embassy_time::{Duration, Instant, Timer};
    use log::error;

    use openthread::{Channels, OpenThread, OtError, Radio};

    use rs_matter_stack::matter::error::{Error, ErrorCode};
    use rs_matter_stack::matter::tlv::OctetsOwned;
    use rs_matter_stack::matter::utils::init::{init, Init};
    use rs_matter_stack::matter::utils::select::Coalesce;
    use rs_matter_stack::mdns::MatterMdnsServices;
    use rs_matter_stack::network::Embedding;
    use rs_matter_stack::rand::MatterRngCore;
    use rs_matter_stack::wireless::traits::{
        Controller, NetworkCredentials, Thread, ThreadData, ThreadId, ThreadScanResult, Wireless,
        WirelessData, WirelessTask, NC,
    };

    use crate::ot::{OtMatterResources, OtMdns, OtNetif};

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

    /// A trait representing a task that needs access to the Thread radio to perform its work
    pub trait ThreadRadioTask {
        /// Run the task with the given Thread radio
        async fn run<R>(&mut self, radio: R) -> Result<(), Error>
        where
            R: Radio;
    }

    impl<T> ThreadRadioTask for &mut T
    where
        T: ThreadRadioTask,
    {
        async fn run<R>(&mut self, radio: R) -> Result<(), Error>
        where
            R: Radio,
        {
            (*self).run(radio).await
        }
    }

    /// A companion trait of `EmbassyThread` for providing a Thread radio and controller.
    pub trait ThreadRadio {
        /// Setup the Thread radio and run the given task with it
        async fn run<A>(&mut self, task: A) -> Result<(), Error>
        where
            A: ThreadRadioTask;
    }

    impl<T> ThreadRadio for &mut T
    where
        T: ThreadRadio,
    {
        async fn run<A>(&mut self, task: A) -> Result<(), Error>
        where
            A: ThreadRadioTask,
        {
            (*self).run(task).await
        }
    }

    /// A Thread radio provider that uses a pre-existing, already created Thread radio,
    /// rather than creating it when the Matter stack needs it.
    pub struct PreexistingThreadRadio<R>(R);

    impl<R> PreexistingThreadRadio<R> {
        /// Create a new instance of the `PreexistingThreadRadio` type.
        pub const fn new(radio: R) -> Self {
            Self(radio)
        }
    }

    impl<R> ThreadRadio for PreexistingThreadRadio<R>
    where
        R: Radio,
    {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: ThreadRadioTask,
        {
            task.run(&mut self.0).await
        }
    }

    /// A `Wireless` trait implementation for `openthread`'s Thread stack.
    pub struct EmbassyThread<'a, T> {
        provider: T,
        mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
        ot: OpenThread<'a>,
        mac: [u8; 6],
    }

    impl<'a, T> EmbassyThread<'a, T>
    where
        T: ThreadRadio,
    {
        /// Create a new instance of the `EmbassyThread` type.
        pub fn new(
            provider: T,
            mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
            resources: &'a mut OtMatterResources,
            rng: &'a mut MatterRngCore,
            mac: [u8; 6],
        ) -> Result<Self, OtError> {
            let ot = OpenThread::new_with_udp_srp(
                rng,
                &mut resources.ot,
                &mut resources.udp,
                &mut resources.srp,
            )?;

            Ok(Self {
                provider,
                mdns_services,
                ot,
                mac,
            })
        }
    }

    impl<T> Wireless for EmbassyThread<'_, T>
    where
        T: ThreadRadio,
    {
        type Data = ThreadData;

        async fn run<A>(&mut self, task: A) -> Result<(), Error>
        where
            A: WirelessTask<Data = Self::Data>,
        {
            fn to_matter_err(_err: OtError) -> Error {
                Error::new(ErrorCode::NoNetworkInterface)
            }

            struct ThreadRadioTaskImpl<'a, A> {
                mdns_services: &'a MatterMdnsServices<'a, NoopRawMutex>,
                ot: OpenThread<'a>,
                mac: [u8; 6],
                task: A,
            }

            impl<'a, A> ThreadRadioTask for ThreadRadioTaskImpl<'a, A>
            where
                A: WirelessTask<Data = ThreadData>,
            {
                async fn run<R>(&mut self, radio: R) -> Result<(), Error>
                where
                    R: Radio,
                {
                    let controller = OtController(self.ot);
                    let netif = OtNetif::new(self.ot, self.mac);
                    let mdns = OtMdns::new(self.ot, self.mdns_services, self.mac)
                        .map_err(to_matter_err)?;

                    let mut main = pin!(self.task.run(netif, self.ot, controller));
                    let mut radio = pin!(async {
                        self.ot.run(radio).await;
                        #[allow(unreachable_code)]
                        Ok(())
                    });
                    let mut mdns = pin!(async { mdns.run().await.map_err(to_matter_err) });

                    self.ot.enable_ipv6(true).map_err(to_matter_err)?;
                    self.ot.srp_autostart().map_err(to_matter_err)?;

                    let result = select3(&mut main, &mut radio, &mut mdns).coalesce().await;

                    let _ = self.ot.enable_thread(false);
                    let _ = self.ot.srp_stop();
                    let _ = self.ot.enable_ipv6(false);

                    result
                }
            }

            self.provider
                .run(ThreadRadioTaskImpl {
                    mdns_services: self.mdns_services,
                    ot: self.ot,
                    mac: self.mac,
                    task,
                })
                .await
        }
    }

    /// A network context for the `EmbassyThread` type.
    pub struct OtNetContext {
        _resources: (),
    }

    impl OtNetContext {
        /// Create a new instance of the `OtNetContext` type.
        pub const fn new() -> Self {
            Self { _resources: () }
        }

        /// Return an in-place initializer for the `OtNetContext` type.
        pub fn init() -> impl Init<Self> {
            init!(Self { _resources: () })
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
            const TIMEOUT_SECS: u64 = 20;

            let _ = self.0.enable_thread(false);

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

    #[cfg(feature = "esp")]
    pub mod esp {
        use esp_hal::peripheral::{Peripheral, PeripheralRef};

        use openthread::esp::EspRadio;

        use rs_matter_stack::matter::error::Error;

        /// A `ThreadRadio` implementation for the ESP32 family of chips.
        pub struct EspThreadRadio<'d> {
            _radio_peripheral: PeripheralRef<'d, esp_hal::peripherals::IEEE802154>,
            _radio_clk_peripheral: PeripheralRef<'d, esp_hal::peripherals::RADIO_CLK>,
        }

        impl<'d> EspThreadRadio<'d> {
            /// Create a new instance of the `EspThreadRadio` type.
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

        impl super::ThreadRadio for EspThreadRadio<'_> {
            async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
            where
                A: super::ThreadRadioTask,
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
    }

    #[cfg(feature = "nrf")]
    pub mod nrf {
        use core::cell::Cell;

        use embassy_nrf::interrupt;
        use embassy_nrf::interrupt::typelevel::{Binding, Handler};
        use embassy_nrf::radio::InterruptHandler;
        use embassy_nrf::{Peripheral, PeripheralRef};

        use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
        use embassy_sync::blocking_mutex::Mutex;

        use openthread::nrf::Ieee802154Peripheral;
        use openthread::nrf::NrfRadio;
        use openthread::{PhyRadioRunner, ProxyRadio, ProxyRadioResources, Radio};

        use rs_matter_stack::matter::error::Error;

        pub use openthread::ProxyRadioResources as NrfThreadRadioResources;

        pub struct NrfThreadRadioInterruptHandler;

        impl Handler<interrupt::typelevel::RADIO> for NrfThreadRadioInterruptHandler {
            unsafe fn on_interrupt() {
                if IEEE802154_ENABLED.lock(Cell::get) {
                    // Call the IEEE 802.15.4 driver interrupt handler, if the driver is enabled
                    InterruptHandler::<embassy_nrf::peripherals::RADIO>::on_interrupt();
                }
            }
        }

        static IEEE802154_ENABLED: Mutex<CriticalSectionRawMutex, Cell<bool>> =
            Mutex::new(Cell::new(false));

        struct NrfThreadRadioInterrupts;

        unsafe impl
            Binding<
                <embassy_nrf::peripherals::RADIO as Ieee802154Peripheral>::Interrupt,
                InterruptHandler<embassy_nrf::peripherals::RADIO>,
            > for NrfThreadRadioInterrupts
        {
        }

        /// A runner for the NRF52 PHY radio
        /// Needs to run in a high-prio execution context
        pub struct NrfThreadRadioRunner<'a, 'd> {
            runner: PhyRadioRunner<'a>,
            radio_peripheral: PeripheralRef<'d, embassy_nrf::peripherals::RADIO>,
        }

        impl<'a, 'd> NrfThreadRadioRunner<'a, 'd> {
            fn new(
                runner: PhyRadioRunner<'a>,
                radio_peripheral: impl Peripheral<P = embassy_nrf::peripherals::RADIO> + 'd,
            ) -> Self {
                Self {
                    runner,
                    radio_peripheral: radio_peripheral.into_ref(),
                }
            }

            /// Run the PHY radio
            pub async fn run(&mut self) -> ! {
                let radio = NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
                    &mut self.radio_peripheral,
                    NrfThreadRadioInterrupts,
                ));

                self.runner.run(radio, embassy_time::Delay).await
            }
        }

        /// A `ThreadRadio` implementation for the NRF52 family of chips.
        pub struct NrfThreadRadio<'a>(ProxyRadio<'a>);

        impl<'a> NrfThreadRadio<'a> {
            /// Create a new instance of the `NrfThreadRadio` type.
            ///
            /// # Arguments
            /// - `resources` - The resources for the radio proxying
            /// - `radio_peripheral` - The radio peripheral instance
            /// - `irq` - The radio interrupt binding
            pub fn new<'d, I>(
                resources: &'a mut ProxyRadioResources,
                mut radio_peripheral: impl Peripheral<P = embassy_nrf::peripherals::RADIO> + 'd,
                _irq: I,
            ) -> (Self, NrfThreadRadioRunner<'a, 'd>)
            where
                I: Binding<
                    <embassy_nrf::peripherals::RADIO as Ieee802154Peripheral>::Interrupt,
                    NrfThreadRadioInterruptHandler,
                >,
            {
                // TODO: A bit dirty as we create it and then drop it immediately
                let caps = NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(
                    &mut radio_peripheral,
                    NrfThreadRadioInterrupts,
                ))
                .caps();

                let (proxy, proxy_runner) = ProxyRadio::new(caps, resources);

                let runner = NrfThreadRadioRunner::new(proxy_runner, radio_peripheral);

                (Self(proxy), runner)
            }
        }

        impl super::ThreadRadio for NrfThreadRadio<'_> {
            async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
            where
                A: super::ThreadRadioTask,
            {
                let _guard = scopeguard::guard((), |_| IEEE802154_ENABLED.lock(|s| s.set(false)));

                IEEE802154_ENABLED.lock(|s| s.set(true));

                task.run(&mut self.0).await
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

    pub trait WifiDriverTask {
        async fn run<D, C>(&mut self, driver: D, controller: C) -> Result<(), Error>
        where
            D: embassy_net::driver::Driver,
            C: Controller<Data = WifiData>;
    }

    impl<T> WifiDriverTask for &mut T
    where
        T: WifiDriverTask,
    {
        async fn run<D, C>(&mut self, driver: D, controller: C) -> Result<(), Error>
        where
            D: embassy_net::driver::Driver,
            C: Controller<Data = WifiData>,
        {
            (*self).run(driver, controller).await
        }
    }

    /// A companion trait of `EmbassyWifi` for providing a Wifi driver and controller.
    pub trait WifiDriver {
        /// Provide a Wifi driver and controller by creating these when the Matter stack needs them
        async fn run<A>(&mut self, task: A) -> Result<(), Error>
        where
            A: WifiDriverTask;
    }

    impl<T> WifiDriver for &mut T
    where
        T: WifiDriver,
    {
        async fn run<A>(&mut self, task: A) -> Result<(), Error>
        where
            A: WifiDriverTask,
        {
            (*self).run(task).await
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

    impl<D, C> WifiDriver for PreexistingWifiDriver<D, C>
    where
        D: embassy_net::driver::Driver,
        C: Controller<Data = WifiData>,
    {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: WifiDriverTask,
        {
            task.run(&mut self.0, &mut self.1).await
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
        T: WifiDriver,
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
        T: WifiDriver,
    {
        type Data = WifiData;

        async fn run<A>(&mut self, task: A) -> Result<(), Error>
        where
            A: WirelessTask<Data = Self::Data>,
        {
            struct WifiDriverTaskImpl<'a, A> {
                context: &'a EmbassyNetContext,
                rand: Rand,
                task: A,
            }

            impl<'a, A> WifiDriverTask for WifiDriverTaskImpl<'a, A>
            where
                A: WirelessTask<Data = WifiData>,
            {
                async fn run<D, C>(&mut self, driver: D, controller: C) -> Result<(), Error>
                where
                    D: embassy_net::driver::Driver,
                    C: Controller<Data = WifiData>,
                {
                    let mut resources = self.context.resources.lock().await;
                    let resources = &mut *resources;
                    let buffers = &self.context.buffers;

                    let mut seed = [0; core::mem::size_of::<u64>()];
                    (self.rand)(&mut seed);

                    let (stack, mut runner) =
                        create_enet_stack(driver, u64::from_le_bytes(seed), resources);

                    let netif = EnetNetif::new(stack);
                    let udp = Udp::new(stack, buffers);

                    let mut main = pin!(self.task.run(netif, udp, controller));
                    let mut run = pin!(async {
                        runner.run().await;
                        #[allow(unreachable_code)]
                        Ok(())
                    });

                    select(&mut main, &mut run).coalesce().await
                }
            }

            self.provider
                .run(WifiDriverTaskImpl {
                    context: self.context,
                    rand: self.rand,
                    task,
                })
                .await
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
            AuthMethod, ClientConfiguration, Configuration, ScanConfig, WifiController, WifiError,
            WifiStaDevice,
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

        /// A `WifiDriver` implementation for the ESP32 family of chips.
        pub struct EspWifiDriver<'a, 'd> {
            controller: &'a esp_wifi::EspWifiController<'d>,
            peripheral: PeripheralRef<'d, esp_hal::peripherals::WIFI>,
        }

        impl<'a, 'd> EspWifiDriver<'a, 'd> {
            /// Create a new instance of the `Esp32WifiDriver` type.
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

        impl super::WifiDriver for EspWifiDriver<'_, '_> {
            async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
            where
                A: super::WifiDriverTask,
            {
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

                task.run(wifi_interface, EspWifiController::new(controller))
                    .await
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
