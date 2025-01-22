//! An example utilizing the `EmbassyWifiMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Wifi as the main transport,
//! and thus BLE for commissioning.
//!
//! If you want to use Ethernet, utilize `EspEthMatterStack` instead.
//! If you want to use non-concurrent commissioning, utilize `EmbassyWifiNCMatterStack` instead
//! (Alexa does not work (yet) with non-concurrent commissioning).
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::pin::pin;

use bt_hci::controller::ExternalController;

use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};

use esp_hal::rng::Rng;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};

use esp_wifi::ble::controller::BleConnector;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};
use esp_wifi::EspWifiController;

use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::data_model::cluster_basic_information::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::cluster_on_off;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{Dataver, Endpoint, HandlerCompat, Node};
use rs_matter_embassy::matter::data_model::system_model::descriptor;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::utils::sync::blocking::Mutex;
use rs_matter_embassy::stack::persist::DummyPersist;
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::MdnsType;
use rs_matter_embassy::wireless::{EmbassyBle, EmbassyWifi, EmbassyWifiMatterStack};

macro_rules! mk_static {
    ($t:ty) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit();
        x
    }};
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    info!("Starting...");

    // == Step 1: ==
    // Necessary `esp-hal` and `esp-wifi` initialization boilerplate

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    // For Wifi and for the only Matter dependency which needs (~4KB) alloc - `x509`
    esp_alloc::heap_allocator!(100 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = esp_hal::rng::Rng::new(peripherals.RNG);

    // ... To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to store the `Rng` in a global variable
    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<Rng>>> =
        Mutex::new(RefCell::new(None));
    RAND.lock(|r| *r.borrow_mut() = Some(rng));

    let init = esp_wifi::init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap();

    #[cfg(not(feature = "esp32"))]
    {
        esp_hal_embassy::init(
            esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0,
        );
    }
    #[cfg(feature = "esp32")]
    {
        esp_hal_embassy::init(timg0.timer1);
    }

    // == Step 2: ==
    // Statically allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyWifiMatterStack<()>).init_with(EmbassyWifiMatterStack::init(
        &BasicInfoConfig {
            vid: TEST_VID,
            pid: TEST_PID,
            hw_ver: 2,
            sw_ver: 1,
            sw_ver_str: "1",
            serial_no: "aabbccdd",
            device_name: "MyLight",
            product_name: "ACME Light",
            vendor_name: "ACME",
        },
        TEST_BASIC_COMM_DATA,
        &TEST_DEV_ATT,
        MdnsType::Builtin,
        epoch,
        |buf| {
            RAND.lock(|rng| {
                let mut rng = rng.borrow_mut();

                buf.iter_mut()
                    .for_each(|byte| *byte = rng.as_mut().unwrap().random() as _);
            })
        },
    ));

    // == Step 3: ==
    // Our "light" on-off cluster.
    // Can be anything implementing `rs_matter::data_model::AsyncHandler`
    let on_off = cluster_on_off::OnOffCluster::new(Dataver::new_rand(stack.matter().rand()));

    // Chain our endpoint clusters with the
    // (root) Endpoint 0 system clusters in the final handler
    let handler = stack
        .root_handler()
        // Our on-off cluster, on Endpoint 1
        .chain(
            LIGHT_ENDPOINT_ID,
            cluster_on_off::ID,
            HandlerCompat(&on_off),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            LIGHT_ENDPOINT_ID,
            descriptor::ID,
            HandlerCompat(descriptor::DescriptorCluster::new(Dataver::new_rand(
                stack.matter().rand(),
            ))),
        );

    // == Step 4: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let mut matter = pin!(stack.run(
        // The Matter stack needs to instantiate an `embassy-net` `Driver` and `Controller`
        EmbassyWifi::new(WifiDriverProvider(&init, peripherals.WIFI), stack),
        // The Matter stack needs BLE
        EmbassyBle::new(BleControllerProvider(&init, peripherals.BT), stack),
        // The Matter stack needs a persister to store its state
        // `EmbassyPersist`+`EmbassyKvBlobStore` saves to a user-supplied NOR Flash region
        // However, for this demo and for simplicity, we use a dummy persister that does nothing
        DummyPersist,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        core::future::pending(),
    ));

    // Just for demoing purposes:
    //
    // Run a sample loop that simulates state changes triggered by the HAL
    // Changes will be properly communicated to the Matter controllers
    // (i.e. Google Home, Alexa) and other Matter devices thanks to subscriptions
    let mut device = pin!(async {
        loop {
            // Simulate user toggling the light with a physical switch every 5 seconds
            Timer::after(Duration::from_secs(5)).await;

            // Toggle
            on_off.set(!on_off.get());

            // Let the Matter stack know that we have changed
            // the state of our Light device
            stack.notify_changed();

            info!("Light toggled");
        }
    });

    // Schedule the Matter run & the device loop together
    select(&mut matter, &mut device).coalesce().await.unwrap();
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyWifiMatterStack::<()>::root_metadata(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: &[DEV_TYPE_ON_OFF_LIGHT],
            clusters: &[descriptor::CLUSTER, cluster_on_off::CLUSTER],
        },
    ],
};

/// If you are OK with having the BLE BTP peripheral instantiated all the time and find this too verbose, scrap it!
/// Use `PreexistingBle::new` instead when instantiating the Matter stack.
struct BleControllerProvider<'a, 'd>(&'a EspWifiController<'d>, esp_hal::peripherals::BT);

impl rs_matter_embassy::ble::BleControllerProvider for BleControllerProvider<'_, '_> {
    type Controller<'t>
        = ExternalController<BleConnector<'t>, 20>
    where
        Self: 't;

    async fn provide(&mut self) -> Self::Controller<'_> {
        ExternalController::new(BleConnector::new(self.0, &mut self.1))
    }
}

/// If you are OK with having Wifi running all the time and find this too verbose, scrap it!
/// Use `PreexistingWireless::new` instead when instantiating the Matter stack.
///
/// Note that this won't you save much in terms of LOCs after all, as you still have to do the
/// equivalent of `WifiDriverProvider::provide` except in your `main()` function.
struct WifiDriverProvider<'a, 'd>(&'a EspWifiController<'d>, esp_hal::peripherals::WIFI);

impl rs_matter_embassy::wireless::WifiDriverProvider for WifiDriverProvider<'_, '_> {
    type Driver<'t>
        = WifiDevice<'t, WifiStaDevice>
    where
        Self: 't;
    type Controller<'t>
        = wifi_controller::EspController<'t>
    where
        Self: 't;

    async fn provide(&mut self) -> (Self::Driver<'_>, Self::Controller<'_>) {
        let (wifi_interface, controller) =
            esp_wifi::wifi::new_with_mode(self.0, &mut self.1, WifiStaDevice).unwrap();

        (
            wifi_interface,
            wifi_controller::EspController(controller, None),
        )
    }
}

// TODO:
// This adaptor would've not been necessary, if there was a common Wifi trait aggreed upon and
// implemented by all MCU Wifi controllers in the field.
//
// Perhaps it is time to dust-off `embedded_svc::wifi` and publish it as a micro-crate?
// `embedded-wifi`?
mod wifi_controller {
    use esp_wifi::wifi::{
        AuthMethod, ClientConfiguration, Configuration, ScanConfig, WifiController, WifiError,
    };

    use rs_matter_embassy::matter::data_model::sdm::nw_commissioning::WiFiSecurity;
    use rs_matter_embassy::matter::error::{Error, ErrorCode};
    use rs_matter_embassy::matter::tlv::OctetsOwned;
    use rs_matter_embassy::matter::utils::storage::Vec;
    use rs_matter_embassy::stack::wireless::traits::{
        Controller, NetworkCredentials, WifiData, WifiScanResult, WifiSsid, WirelessData,
    };

    const MAX_NETWORKS: usize = 3;

    pub struct EspController<'a>(pub WifiController<'a>, pub Option<WifiSsid>);

    impl Controller for EspController<'_> {
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
            if !self.0.is_started().map_err(to_err)? {
                self.0.start_async().await.map_err(to_err)?;
            }

            let mut scan_config = ScanConfig::default();
            if let Some(network_id) = network_id {
                scan_config.ssid = Some(network_id.0.as_str());
            }

            let (aps, _) = self
                .0
                .scan_with_config_async::<MAX_NETWORKS>(scan_config)
                .await
                .map_err(to_err)?;

            for ap in aps {
                callback(Some(&WifiScanResult {
                    ssid: WifiSsid(ap.ssid),
                    bssid: OctetsOwned {
                        vec: Vec::from_slice(&ap.bssid).unwrap(),
                    },
                    channel: ap.channel as _,
                    rssi: Some(ap.signal_strength),
                    band: None,
                    security: match ap.auth_method {
                        Some(AuthMethod::None) => WiFiSecurity::Unencrypted,
                        Some(AuthMethod::WEP) => WiFiSecurity::Wep,
                        Some(AuthMethod::WPA) => WiFiSecurity::WpaPersonal,
                        Some(AuthMethod::WPA3Personal) => WiFiSecurity::Wpa3Personal,
                        _ => WiFiSecurity::Wpa2Personal,
                    },
                }))?;
            }

            callback(None)?;

            Ok(())
        }

        async fn connect(
            &mut self,
            creds: &<Self::Data as WirelessData>::NetworkCredentials,
        ) -> Result<(), Error> {
            self.1 = None;

            if self.0.is_started().map_err(to_err)? {
                self.0.stop_async().await.map_err(to_err)?;
            }

            self.0
                .set_configuration(&Configuration::Client(ClientConfiguration {
                    ssid: creds.ssid.0.clone(),
                    password: creds.password.clone(),
                    ..Default::default()
                }))
                .map_err(to_err)?;

            self.0.start_async().await.map_err(to_err)?;
            self.0.connect_async().await.map_err(to_err)?;

            self.1 = self
                .0
                .is_connected()
                .map_err(to_err)?
                .then_some(creds.ssid.clone());

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

    fn to_err(_: WifiError) -> Error {
        Error::new(ErrorCode::NoNetworkInterface)
    }
}
