//! An example utilizing the `EspEthMatterStack` struct.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as ESP32s don't have Ethernet ports out of the box.
//! Instead, we utilize Wifi, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Wifi network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).

use core::pin::pin;

use embassy_futures::select::select;
use embassy_time::{Duration, Timer};

use esp_idf_matter::netif::EspMatterNetif;
use esp_idf_matter::{init_async_io, EspEthMatterStack};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::block_on;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{esp, esp_netif_create_ip6_linklocal};
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::{self, AsyncWifi, EspWifi};

use log::{error, info};

use rs_matter::data_model::cluster_basic_information::BasicInfoConfig;
use rs_matter::data_model::cluster_on_off;
use rs_matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter::data_model::objects::{Dataver, Endpoint, HandlerCompat, Node};
use rs_matter::data_model::system_model::descriptor;
use rs_matter::utils::init::InitMaybeUninit;
use rs_matter::utils::select::Coalesce;

use rs_matter::BasicCommData;
use rs_matter_stack::persist::DummyPersist;

use static_cell::StaticCell;

#[path = "dev_att/dev_att.rs"]
mod dev_att;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");

fn main() -> Result<(), anyhow::Error> {
    EspLogger::initialize_default();

    info!("Starting...");

    // Run in a higher-prio thread to avoid issues with `async-io` getting
    // confused by the low priority of the ESP IDF main task
    // Also allocate a very large stack (for now) as `rs-matter` futures do occupy quite some space
    let thread = std::thread::Builder::new()
        .stack_size(60 * 1024)
        .spawn(|| {
            // Eagerly initialize `async-io` to minimize the risk of stack blowups later on
            init_async_io()?;

            run()
        })
        .unwrap();

    thread.join().unwrap()
}

#[inline(never)]
#[cold]
fn run() -> Result<(), anyhow::Error> {
    let result = block_on(matter());

    if let Err(e) = &result {
        error!("Matter aborted execution with error: {:?}", e);
    }
    {
        info!("Matter finished execution successfully");
    }

    result
}

async fn matter() -> Result<(), anyhow::Error> {
    // Initialize the Matter stack (can be done only once),
    // as we'll run it in this thread
    let stack = MATTER_STACK
        .uninit()
        .init_with(EspEthMatterStack::init_default(
            &BasicInfoConfig {
                vid: 0xFFF1,
                pid: 0x8000,
                hw_ver: 2,
                sw_ver: 1,
                sw_ver_str: "1",
                serial_no: "aabbccdd",
                device_name: "MyLight",
                product_name: "ACME Light",
                vendor_name: "ACME",
            },
            BasicCommData {
                password: 20202021,
                discriminator: 3840,
            },
            &DEV_ATT,
        ));

    // Take some generic ESP-IDF stuff we'll need later
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let peripherals = Peripherals::take()?;

    // Configure and start the Wifi first
    let mut wifi = Box::new(AsyncWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs.clone()))?,
        sysloop.clone(),
        EspTaskTimerService::new()?,
    )?);
    wifi.set_configuration(&wifi::Configuration::Client(wifi::ClientConfiguration {
        ssid: WIFI_SSID.try_into().unwrap(),
        password: WIFI_PASS.try_into().unwrap(),
        ..Default::default()
    }))?;
    wifi.start().await?;
    wifi.connect().await?;

    // Matter needs an IPv6 address to work
    esp!(unsafe { esp_netif_create_ip6_linklocal(wifi.wifi().sta_netif().handle() as _) })?;

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

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    let mut matter = pin!(stack.run(
        // The Matter stack need access to the netif on which we'll operate
        // Since we are pretending to use a wired Ethernet connection - yet -
        // we are using a Wifi STA, provide the Wifi netif here
        EspMatterNetif::new(wifi.wifi().sta_netif(), sysloop),
        // The Matter stack needs a persister to store its state
        // `EspPersist`+`EspKvBlobStore` saves to a user-supplied NVS partition
        // under namespace `esp-idf-matter`
        DummyPersist,
        //EspPersist::new_eth(EspKvBlobStore::new_default(nvs.clone())?, stack),
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
    select(&mut matter, &mut device).coalesce().await?;

    Ok(())
}

/// The Matter stack is allocated statically to avoid
/// program stack blowups.
static MATTER_STACK: StaticCell<EspEthMatterStack<()>> = StaticCell::new();

static DEV_ATT: dev_att::HardCodedDevAtt = dev_att::HardCodedDevAtt::new();

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EspEthMatterStack::<()>::root_metadata(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: &[DEV_TYPE_ON_OFF_LIGHT],
            clusters: &[descriptor::CLUSTER, cluster_on_off::CLUSTER],
        },
    ],
};
