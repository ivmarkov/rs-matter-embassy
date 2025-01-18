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

use rs_matter_embassy::ble::{GPHostResources, TroubleBtpGattPeripheral, QOS};
use rs_matter_embassy::matter::data_model::cluster_basic_information::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::cluster_on_off;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{Dataver, Endpoint, HandlerCompat, Node};
use rs_matter_embassy::matter::data_model::system_model::descriptor;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::utils::sync::blocking::Mutex;
use rs_matter_embassy::stack::network::Network;
use rs_matter_embassy::stack::persist::DummyPersist;
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::wireless::traits::PreexistingBle;
use rs_matter_embassy::stack::MdnsType;
use rs_matter_embassy::wireless::{EmbassyWifi, EmbassyWifiMatterStack};

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
    esp_println::logger::init_logger_from_env();

    info!("Starting...");

    // First some minimal `esp-hal` and `esp-wifi` initialization boilerplate

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(80 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = esp_hal::rng::Rng::new(peripherals.RNG);

    // ... To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to store the `Rng` in a global variable
    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<Rng>>> =
        Mutex::new(RefCell::new(None));
    RAND.lock(|r| *r.borrow_mut() = Some(rng.clone()));

    // TrouBLE needs the BLE controller to be static, which
    // means the `EspWifiController` must be static too.
    let init = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng, peripherals.RADIO_CLK,).unwrap()
    );

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

    // Now, initialize the Matter stack (can be done only once),
    // as we'll run it in this thread
    // The Matter stack is allocated statically to avoid program stack blowups.
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
        || core::time::Duration::from_millis(embassy_time::Instant::now().as_millis()),
        |buf| {
            RAND.lock(|rng| {
                let mut rng = rng.borrow_mut();

                buf.iter_mut()
                    .for_each(|byte| *byte = rng.as_mut().unwrap().random() as _);
            })
        },
    ));

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

    // Currently (and unlike `embassy-net`) TroubBLE does NOT support dynamic stack creation/teardown
    // Therefore, we need to create the BLE controller (and the Matter BTP stack!) once and it would live forever
    let controller = ExternalController::<_, 20>::new(BleConnector::new(&init, peripherals.BT));
    let btp_peripheral = TroubleBtpGattPeripheral::new(
        controller,
        &stack.network().embedding().embedding().ble_context(),
        mk_static!(
            GPHostResources<ExternalController<BleConnector<'static>, 20>>,
            GPHostResources::new(QOS)
        ),
    )
    .unwrap();

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    let mut matter = pin!(stack.run(
        // The Matter stack needs to instantiate an `embassy-net` `Driver` and `Controller`
        EmbassyWifi::new(WifiDriverProvider(&init, peripherals.WIFI), &stack),
        // The Matter stack needs BLE
        PreexistingBle::new(btp_peripheral),
        // The Matter stack needs a persister to store its state
        // `EmbassyPersist`+`EspKvBlobStore` saves to a user-supplied NVS partition
        // under namespace `esp-idf-matter`
        DummyPersist,
        //EspPersist::new_wifi_ble(EspKvBlobStore::new_default(nvs.clone())?, stack),
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

/// If you are OK with having Wifi running all the time and find this too verbose, scrap it!
/// Use `PreexistingWireless::new` instead when instantiating the Matter stack.
///
/// Note that this won't you save much in terms of LOCs after all, as you still have to do the
/// equivalent of `WifiDriverProvider::provide` except in your `main()` function.
struct WifiDriverProvider<'a, 'd>(&'a EspWifiController<'d>, esp_hal::peripherals::WIFI);

impl<'a, 'd> rs_matter_embassy::wireless::WifiDriverProvider for WifiDriverProvider<'a, 'd> {
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

        (wifi_interface, wifi_controller::EspController(controller))
    }
}

// TODO:
// This adaptor would've not been necessary, if there was a common Wifi trait aggreed upon and
// implemented by all MCU Wifi controllers in the field.
//
// Perhaps it is time to dust-off `embedded_svc::wifi` and publish it as a micro-crate?
// `embedded-wifi`?
mod wifi_controller {
    use esp_wifi::wifi::WifiController;

    use rs_matter_embassy::matter::error::Error;
    use rs_matter_embassy::stack::wireless::traits::{
        Controller, NetworkCredentials, WifiData, WirelessData,
    };

    pub struct EspController<'a>(pub WifiController<'a>);

    impl Controller for EspController<'_> {
        type Data = WifiData;

        async fn scan<F>(
            &mut self,
            network_id: Option<
                &<<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
            >,
            callback: F,
        ) -> Result<(), Error>
        where
            F: FnMut(Option<&<Self::Data as WirelessData>::ScanResult>) -> Result<(), Error>,
        {
            todo!()
        }

        async fn connect(
            &mut self,
            creds: &<Self::Data as WirelessData>::NetworkCredentials,
        ) -> Result<(), Error> {
            todo!()
        }

        async fn connected_network(
            &mut self,
        ) -> Result<
            Option<
                <<Self::Data as WirelessData>::NetworkCredentials as NetworkCredentials>::NetworkId,
            >,
            Error,
        > {
            todo!()
        }

        async fn stats(&mut self) -> Result<<Self::Data as WirelessData>::Stats, Error> {
            todo!()
        }
    }
}
