use core::pin::pin;

use edge_nal_embassy::Udp;

use embassy_futures::select::select;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::utils::rand::Rand;
use rs_matter_stack::matter::utils::select::Coalesce;
use rs_matter_stack::network::{Embedding, Network};
use rs_matter_stack::wireless::{
    Controller, Gatt, GattTask, Wifi, WifiData, Wireless, WirelessCoex, WirelessCoexTask,
    WirelessTask,
};

use crate::ble::{ControllerRef, TroubleBtpGattContext, TroubleBtpGattPeripheral};
use crate::enet::{create_enet_stack, EnetNetif};
use crate::eth::EmbassyNetContext;

use super::{BleDriver, BleDriverTask, EmbassyWirelessMatterStack};

/// A type alias for an Embassy Matter stack running over Wifi (and BLE, during commissioning).
pub type EmbassyWifiMatterStack<'a, E = ()> =
    EmbassyWirelessMatterStack<'a, Wifi, EmbassyNetContext, E>;

/// A trait representing a task that needs access to the Wifi driver and controller to perform its work
pub trait WifiDriverTask {
    /// Run the task with the given Wifi driver and controller
    async fn run<D, C>(&mut self, driver: D, controller: C) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: Controller<Data = WifiData>;
}

impl<T> WifiDriverTask for &mut T
where
    T: WifiDriverTask,
{
    /// Run the task with the given Wifi driver and Wifi controller
    async fn run<D, C>(&mut self, driver: D, controller: C) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: Controller<Data = WifiData>,
    {
        (*self).run(driver, controller).await
    }
}

/// A trait representing a task that needs access to the Wifi driver and controller,
/// as well as to the BLe controller to perform its work
pub trait WifiCoexDriverTask {
    /// Run the task with the given Wifi driver, Wifi controller and BLE controller
    async fn run<D, C, B>(
        &mut self,
        wifi_driver: D,
        wifi_controller: C,
        ble_controller: B,
    ) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: Controller<Data = WifiData>,
        B: trouble_host::Controller;
}

impl<T> WifiCoexDriverTask for &mut T
where
    T: WifiCoexDriverTask,
{
    async fn run<D, C, B>(
        &mut self,
        wifi_driver: D,
        wifi_controller: C,
        ble_controller: B,
    ) -> Result<(), Error>
    where
        D: embassy_net::driver::Driver,
        C: Controller<Data = WifiData>,
        B: trouble_host::Controller,
    {
        (*self)
            .run(wifi_driver, wifi_controller, ble_controller)
            .await
    }
}

/// A trait for running a task within a context where the Wifi radio is initialized and operable
pub trait WifiDriver {
    /// Setup the Wifi driver and controller and run the given task with these
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

/// A trait for running a task within a context where the Wifi radio - as well as the BLE controller - are initialized and operable
pub trait WifiCoexDriver {
    /// Setup the Wifi driver and controller, as well as the BLE controller run the given task with these
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiCoexDriverTask;
}

impl<T> WifiCoexDriver for &mut T
where
    T: WifiCoexDriver,
{
    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WifiCoexDriverTask,
    {
        (*self).run(task).await
    }
}

/// A Wifi driver provider that uses a pre-existing, already created Wifi driver and controller,
/// rather than creating them when the Matter stack needs them.
pub struct PreexistingWifiDriver<D, C, B>(D, C, B);

impl<D, C, B> PreexistingWifiDriver<D, C, B> {
    /// Create a new instance of the `PreexistingWifiDriver` type.
    pub const fn new(enet_wifi_driver: D, wifi_controller: C, ble_controller: B) -> Self {
        Self(enet_wifi_driver, wifi_controller, ble_controller)
    }
}

impl<D, C, B> WifiDriver for PreexistingWifiDriver<D, C, B>
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

impl<D, C, B> BleDriver for PreexistingWifiDriver<D, C, B>
where
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: BleDriverTask,
    {
        task.run(ControllerRef::new(&self.2)).await
    }
}

impl<D, C, B> WifiCoexDriver for PreexistingWifiDriver<D, C, B>
where
    D: embassy_net::driver::Driver,
    C: Controller<Data = WifiData>,
    B: trouble_host::Controller,
{
    async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
    where
        A: WifiCoexDriverTask,
    {
        task.run(&mut self.0, &mut self.1, ControllerRef::new(&self.2))
            .await
    }
}

/// A `Wireless` trait implementation for `embassy-net`'s Wifi stack.
pub struct EmbassyWifi<'a, T> {
    driver: T,
    context: &'a EmbassyNetContext,
    ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
    rand: Rand,
}

impl<'a, T> EmbassyWifi<'a, T> {
    /// Create a new instance of the `EmbassyWifi` type.
    pub fn new<E>(driver: T, stack: &'a EmbassyWifiMatterStack<'a, E>) -> Self
    where
        E: Embedding + 'static,
    {
        Self::wrap(
            driver,
            stack.network().embedding().net_context(),
            stack.network().embedding().ble_context(),
            stack.matter().rand(),
        )
    }

    /// Wrap the `EmbassyWifi` type around a Wifi Driver and BLE controller runner and a network context.
    pub const fn wrap(
        driver: T,
        context: &'a EmbassyNetContext,
        ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
        rand: Rand,
    ) -> Self {
        Self {
            driver,
            context,
            ble_context,
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

        impl<A> WifiDriverTask for WifiDriverTaskImpl<'_, A>
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

        self.driver
            .run(WifiDriverTaskImpl {
                context: self.context,
                rand: self.rand,
                task,
            })
            .await
    }
}

impl<T> WirelessCoex for EmbassyWifi<'_, T>
where
    T: WifiCoexDriver,
{
    type Data = WifiData;

    async fn run<A>(&mut self, task: A) -> Result<(), Error>
    where
        A: WirelessCoexTask<Data = Self::Data>,
    {
        struct WifiCoexDriverTaskImpl<'a, A> {
            context: &'a EmbassyNetContext,
            ble_context: &'a TroubleBtpGattContext<CriticalSectionRawMutex>,
            rand: Rand,
            task: A,
        }

        impl<A> WifiCoexDriverTask for WifiCoexDriverTaskImpl<'_, A>
        where
            A: WirelessCoexTask<Data = WifiData>,
        {
            async fn run<D, C, B>(
                &mut self,
                wifi_driver: D,
                wifi_controller: C,
                ble_controller: B,
            ) -> Result<(), Error>
            where
                D: embassy_net::driver::Driver,
                C: Controller<Data = WifiData>,
                B: trouble_host::Controller,
            {
                let mut resources = self.context.resources.lock().await;
                let resources = &mut *resources;
                let buffers = &self.context.buffers;

                let mut seed = [0; core::mem::size_of::<u64>()];
                (self.rand)(&mut seed);

                let (stack, mut runner) =
                    create_enet_stack(wifi_driver, u64::from_le_bytes(seed), resources);

                let netif = EnetNetif::new(stack);
                let udp = Udp::new(stack, buffers);

                let peripheral =
                    TroubleBtpGattPeripheral::new(ble_controller, self.rand, self.ble_context);

                let mut main = pin!(self.task.run(netif, udp, wifi_controller, peripheral));
                let mut run = pin!(async {
                    runner.run().await;
                    #[allow(unreachable_code)]
                    Ok(())
                });

                select(&mut main, &mut run).coalesce().await
            }
        }

        self.driver
            .run(WifiCoexDriverTaskImpl {
                context: self.context,
                ble_context: self.ble_context,
                rand: self.rand,
                task,
            })
            .await
    }
}

impl<T> Gatt for EmbassyWifi<'_, T>
where
    T: BleDriver,
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

#[cfg(feature = "rp")]
pub mod rp {
    use cyw43::{Control, JoinOptions, ScanOptions};

    use crate::fmt::Bytes;
    use crate::matter::data_model::sdm::nw_commissioning::{WiFiSecurity, WifiBand};
    use crate::matter::error::{Error, ErrorCode};
    use crate::matter::tlv::OctetsOwned;
    use crate::matter::utils::storage::Vec;
    use crate::stack::wireless::{
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
                scan_options.ssid =
                    Some(unwrap!(core::str::from_utf8(network_id.0.vec.as_slice())
                        .unwrap_or("???")
                        .try_into()));
            }

            let mut scanner = self.0.scan(scan_options).await;

            info!("Wifi scan started");

            while let Some(ap) = scanner.next().await {
                if ap.ssid_len > 0 {
                    let result = WifiScanResult {
                        ssid: WifiSsid(OctetsOwned {
                            vec: unwrap!(Vec::from_slice(&ap.ssid[..ap.ssid_len as _])),
                        }),
                        bssid: OctetsOwned {
                            vec: unwrap!(Vec::from_slice(&ap.bssid)),
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
                        "Skipping scan result for a hidden network {}",
                        Bytes(&ap.bssid)
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

            info!("Wifi connect request for SSID {}", ssid);

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
        > {
            Ok(self.1.clone())
        }

        async fn stats(&mut self) -> Result<<Self::Data as WirelessData>::Stats, Error> {
            Ok(None)
        }
    }

    fn to_err(e: cyw43::ControlError) -> Error {
        error!("Wifi error: {:?}", debug2format!(e));
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
pub mod esp_wifi {
    use bt_hci::controller::ExternalController;

    use esp_hal::peripheral::{Peripheral, PeripheralRef};

    use esp_wifi::ble::controller::BleConnector;
    use esp_wifi::wifi::{
        AuthMethod, ClientConfiguration, Configuration, ScanConfig, WifiController, WifiError,
        WifiStaDevice,
    };

    use crate::matter::data_model::sdm::nw_commissioning::{WiFiSecurity, WifiBand};
    use crate::matter::error::{Error, ErrorCode};
    use crate::matter::tlv::OctetsOwned;
    use crate::matter::utils::storage::Vec;
    use crate::stack::wireless::{
        Controller, NetworkCredentials, WifiData, WifiScanResult, WifiSsid, WirelessData,
    };
    use crate::wireless::SLOTS;

    const MAX_NETWORKS: usize = 3;

    /// A `WifiDriver` implementation for the ESP32 family of chips.
    pub struct EspWifiDriver<'a, 'd> {
        controller: &'a esp_wifi::EspWifiController<'d>,
        wifi_peripheral: PeripheralRef<'d, esp_hal::peripherals::WIFI>,
        bt_peripheral: PeripheralRef<'d, esp_hal::peripherals::BT>,
    }

    impl<'a, 'd> EspWifiDriver<'a, 'd> {
        /// Create a new instance of the `Esp32WifiDriver` type.
        ///
        /// # Arguments
        /// - `controller` - The `esp-wifi` Wifi controller instance.
        /// - `peripheral` - The Wifi peripheral instance.
        pub fn new(
            controller: &'a esp_wifi::EspWifiController<'d>,
            wifi_peripheral: impl Peripheral<P = esp_hal::peripherals::WIFI> + 'd,
            bt_peripheral: impl Peripheral<P = esp_hal::peripherals::BT> + 'd,
        ) -> Self {
            Self {
                controller,
                wifi_peripheral: wifi_peripheral.into_ref(),
                bt_peripheral: bt_peripheral.into_ref(),
            }
        }
    }

    impl super::WifiDriver for EspWifiDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::WifiDriverTask,
        {
            let (wifi_interface, mut controller) = unwrap!(esp_wifi::wifi::new_with_mode(
                self.controller,
                &mut self.wifi_peripheral,
                WifiStaDevice,
            ));

            // esp32c6-specific - need to boost the power to get a good signal
            unwrap!(controller.set_power_saving(esp_wifi::config::PowerSaveMode::None));

            task.run(wifi_interface, EspWifiController::new(controller))
                .await
        }
    }

    impl super::WifiCoexDriver for EspWifiDriver<'_, '_> {
        async fn run<A>(&mut self, mut task: A) -> Result<(), Error>
        where
            A: super::WifiCoexDriverTask,
        {
            let (wifi_interface, mut controller) = unwrap!(esp_wifi::wifi::new_with_mode(
                self.controller,
                &mut self.wifi_peripheral,
                WifiStaDevice,
            ));

            // esp32c6-specific - need to boost the power to get a good signal
            unwrap!(controller.set_power_saving(esp_wifi::config::PowerSaveMode::None));

            let ble_controller = ExternalController::<_, SLOTS>::new(BleConnector::new(
                self.controller,
                &mut self.bt_peripheral,
            ));

            task.run(
                wifi_interface,
                EspWifiController::new(controller),
                ble_controller,
            )
            .await
        }
    }

    impl super::BleDriver for EspWifiDriver<'_, '_> {
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
                scan_config.ssid = Some(unwrap!(core::str::from_utf8(network_id.0.vec.as_slice())
                    .unwrap_or("???")
                    .try_into()));
            }

            let (aps, _len) = self
                .0
                .scan_with_config_async::<MAX_NETWORKS>(scan_config)
                .await
                .map_err(to_err)?;

            info!(
                "Wifi scan complete, reporting {} results out of {_len} total",
                aps.len()
            );

            for ap in aps {
                let result = WifiScanResult {
                    ssid: WifiSsid(OctetsOwned {
                        vec: unwrap!(ap.ssid.as_bytes().try_into()),
                    }),
                    bssid: OctetsOwned {
                        vec: unwrap!(Vec::from_slice(&ap.bssid)),
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
                    ssid: unwrap!(ssid.try_into()),
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
        > {
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
