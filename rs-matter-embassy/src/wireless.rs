//! Wireless: Type aliases and state structs for an Embassy Matter stack running over a wireless network (Wifi or Thread) and BLE.

use core::mem::MaybeUninit;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rs_matter::tlv::{FromTLV, ToTLV};
use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::rand::Rand;
use rs_matter_stack::matter::utils::sync::IfMutex;
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::persist::KvBlobBuf;
use rs_matter_stack::wireless::traits::{Ble, BleTask, WirelessConfig, WirelessData};
use rs_matter_stack::{MatterStack, WirelessBle};

use trouble_host::Controller;


use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::nal::{MatterStackResources, MatterUdpBuffers};

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
            enet_context <- EmbassyNetContext::init(),
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

/// A context (storage) for the network layer of the Matter stack.
pub struct EmbassyNetContext {
    buffers: MatterUdpBuffers,
    resources: IfMutex<CriticalSectionRawMutex, MatterStackResources>,
}

impl EmbassyNetContext {
    /// Create a new instance of the `EmbassyNetContext` type.
    pub const fn new() -> Self {
        Self {
            buffers: MatterUdpBuffers::new(),
            resources: IfMutex::new(MatterStackResources::new()),
        }
    }

    /// Return an in-place initializer for the `EmbassyNetContext` type.
    pub fn init() -> impl Init<Self> {
        init!(Self {
            // TODO: Implement init constructor for `UdpBuffers`
            buffers: MatterUdpBuffers::new(),
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources <- IfMutex::init(unsafe { MaybeUninit::<MatterStackResources>::uninit().assume_init() }),
        })
    }
}

impl Default for EmbassyNetContext {
    fn default() -> Self {
        Self::new()
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

// Wifi: Type aliases and state structs for an Embassy Matter stack running over a Wifi network and BLE.
pub mod wifi {
    use core::pin::pin;

    use edge_nal_embassy::Udp;

    use embassy_futures::select::select;

    use rs_matter_stack::matter::error::Error;
    use rs_matter_stack::matter::utils::rand::Rand;
    use rs_matter_stack::matter::utils::select::Coalesce;
    use rs_matter_stack::network::{Embedding, Network};
    use rs_matter_stack::wireless::traits::{
        Controller, Wifi, WifiData, Wireless, WirelessTask, NC,
    };

    use crate::nal::create_net_stack;
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
        pub fn new<E>(provider: T, stack: &'a EmbassyWifiMatterStack<'a, E>) -> Self
        where
            E: Embedding + 'static,
        {
            Self::wrap(
                provider,
                stack.network().embedding().embedding().enet_context(),
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

            let (stack, mut runner) = create_net_stack(driver, u64::from_le_bytes(seed), resources);

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
