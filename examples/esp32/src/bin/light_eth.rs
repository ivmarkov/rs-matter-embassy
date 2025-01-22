//! An example utilizing the `EmbassyEthMatterStack` struct.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as ESP32s don't have Ethernet ports out of the box.
//! Instead, we utilize Wifi, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Wifi network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::env;
use core::pin::pin;

use embassy_executor::Spawner;
use embassy_futures::select::select4;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};

use esp_backtrace as _;
use esp_hal::rng::Rng;
use esp_hal::{clock::CpuClock, timer::timg::TimerGroup};
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiEvent, WifiStaDevice, WifiState,
};

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
use rs_matter_embassy::nal::{create_net_stack, MatterStackResources, MatterUdpBuffers, Udp};
use rs_matter_embassy::netif::EmbassyNetif;
use rs_matter_embassy::stack::persist::DummyPersist;
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::utils::futures::IntoFaillble;
use rs_matter_embassy::stack::MdnsType;
use rs_matter_embassy::EmbassyEthMatterStack;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASS: &str = env!("WIFI_PASS");

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
    //esp_println::logger::init_logger(log::LevelFilter::Info);
    logger::init_logger(true, log::LevelFilter::Info);

    info!("Starting...");

    // == Step 1: ==
    // Necessary `esp-hal` and `esp-wifi` initialization boilerplate

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    // For Wifi and for the only Matter dependency which needs (~4KB) alloc - `x509`
    esp_alloc::heap_allocator!(80 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);

    // ... To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to store the `Rng` in a global variable
    static RAND: Mutex<CriticalSectionRawMutex, RefCell<Option<Rng>>> =
        Mutex::new(RefCell::new(None));
    RAND.lock(|r| *r.borrow_mut() = Some(rng.clone()));

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

    let stack = mk_static!(EmbassyEthMatterStack<()>).init_with(EmbassyEthMatterStack::init(
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

    // Configure and start the Wifi first
    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();

    let (net_stack, mut net_runner) = create_net_stack(
        wifi_interface,
        (rng.random() as u64) << 32 | rng.random() as u64,
        mk_static!(MatterStackResources, MatterStackResources::new()),
    );

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
        // The Matter stack needs access to the netif so as to detect network going up/down
        EmbassyNetif::new(net_stack),
        // The Matter stack needs to open two UDP sockets
        Udp::new(
            net_stack,
            mk_static!(MatterUdpBuffers, MatterUdpBuffers::new())
        ),
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
    select4(
        &mut matter,
        &mut device,
        connection(controller).into_fallible(),
        async {
            net_runner.run().await;
            Ok(())
        },
    )
    .coalesce()
    .await
    .unwrap();
}

async fn connection(mut controller: WifiController<'_>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: WIFI_SSID.try_into().unwrap(),
                password: WIFI_PASS.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                info!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyEthMatterStack::<()>::root_metadata(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: &[DEV_TYPE_ON_OFF_LIGHT],
            clusters: &[descriptor::CLUSTER, cluster_on_off::CLUSTER],
        },
    ],
};

/// The default `EspLogger` from `esp_println` is not used as it does not
/// print a timestamp and the module name. This custom logger is used instead.
mod logger {
    #![allow(unexpected_cfgs)]

    use core::cell::Cell;

    use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
    use esp_println::println;

    static COLORED: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(true));

    /// Initialize the logger with the given maximum log level.
    pub fn init_logger(colored: bool, level: log::LevelFilter) {
        COLORED.lock(|c| c.set(colored));

        unsafe { log::set_logger_racy(&EspIdfLikeLogger) }.unwrap();
        unsafe {
            log::set_max_level_racy(level);
        }
    }

    struct EspIdfLikeLogger;

    impl log::Log for EspIdfLikeLogger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        #[allow(unused)]
        fn log(&self, record: &log::Record) {
            if !self.enabled(record.metadata()) {
                return;
            }

            const RESET: &str = "\u{001B}[0m";
            const RED: &str = "\u{001B}[31m";
            const GREEN: &str = "\u{001B}[32m";
            const YELLOW: &str = "\u{001B}[33m";
            const BLUE: &str = "\u{001B}[34m";
            const CYAN: &str = "\u{001B}[35m";

            let colored = COLORED.lock(|c| c.get());

            let (color, reset) = if colored {
                (
                    match record.level() {
                        log::Level::Error => RED,
                        log::Level::Warn => YELLOW,
                        log::Level::Info => GREEN,
                        log::Level::Debug => BLUE,
                        log::Level::Trace => CYAN,
                    },
                    RESET,
                )
            } else {
                ("", "")
            };

            println!(
                //"{}{:.1} ({}) {}: {}{}",
                "{}{:.1} {}: {}{}",
                color,
                record.level(),
                //UtcTimestamp::new((&NOW).now()),
                record.metadata().target(),
                record.args(),
                reset
            );
        }

        fn flush(&self) {}
    }
}
