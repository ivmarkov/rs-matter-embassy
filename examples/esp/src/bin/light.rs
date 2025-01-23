//! An example utilizing the `EmbassyWifiMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Wifi as the main transport,
//! and thus BLE for commissioning.
//!
//! If you want to use Ethernet, utilize `EmbassyEthMatterStack` instead.
//! If you want to use non-concurrent commissioning, utilize `EmbassyWifiNCMatterStack` instead
//! (Note: Alexa does not work (yet) with non-concurrent commissioning.)
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::{Duration, Timer};

use esp_backtrace as _;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};

use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::data_model::cluster_basic_information::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::cluster_on_off;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{Dataver, Endpoint, HandlerCompat, Node};
use rs_matter_embassy::matter::data_model::system_model::descriptor;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::rand::esp::{esp_init_rand, esp_rand};
use rs_matter_embassy::stack::persist::DummyPersist;
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::MdnsType;
use rs_matter_embassy::wireless::esp::EspBleControllerProvider;
use rs_matter_embassy::wireless::wifi::esp::EspWifiDriverProvider;
use rs_matter_embassy::wireless::wifi::{EmbassyWifi, EmbassyWifiMatterStack};
use rs_matter_embassy::wireless::EmbassyBle;

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

    // To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to initialize the global `rand` fn once
    esp_init_rand(rng);

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
    let stack = &*mk_static!(EmbassyWifiMatterStack<()>).init_with(EmbassyWifiMatterStack::init(
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
        esp_rand,
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
        EmbassyWifi::new(EspWifiDriverProvider::new(&init, peripherals.WIFI), stack),
        // The Matter stack needs BLE
        EmbassyBle::new(EspBleControllerProvider::new(&init, peripherals.BT), stack),
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
