//! An example utilizing the `EmbassyThreadMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Thread as the main transport,
//! and thus BLE for commissioning, in non-concurrent commissioning mode
//! (the IEEE802154 radio and BLE cannot not run at the same time yet with `esp-hal`).
//!
//! If you want to use Ethernet, utilize `EmbassyEthMatterStack` instead.
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::pin::pin;

use alloc::boxed::Box;
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
use rs_matter_embassy::matter::{BasicCommData, MATTER_PORT};
use rs_matter_embassy::rand::esp::{esp_init_rand, esp_rand};
use rs_matter_embassy::stack::mdns::MatterMdnsServices;
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::stack::rand::RngCore;
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::MdnsType;
use rs_matter_embassy::wireless::esp::EspThreadDriver;
use rs_matter_embassy::wireless::{EmbassyThread, EmbassyThreadMatterStack};

use tinyrlibc as _;

extern crate alloc;

#[esp_hal_embassy::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    info!("Starting...");

    // Heap strictly necessary only for BLE and for the only Matter dependency which needs (~4KB) alloc - `x509`
    // However since `esp32` specifically has a disjoint heap which causes bss size troubles, it is easier
    // to allocate the statics once from heap as well
    init_heap();

    // == Step 1: ==
    // Necessary `esp-hal` initialization boilerplate

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);

    // Use a random/unique Matter discriminator for this session,
    // in case there are left-overs from our previous registrations in Thread SRP
    let discriminator = (rng.next_u32() & 0xfff) as u16;

    // TODO
    let mut ieee_eui64 = [0; 8];
    rng.fill_bytes(&mut ieee_eui64);

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
    // Replace the built-in Matter mDNS responder with a bridge that delegates
    // all mDNS work to the OpenThread SRP client.
    // Thread is not friendly to IpV6 multicast, so we have to use SRP instead.
    let mdns_services = &*Box::leak(Box::new_uninit())
        .init_with(MatterMdnsServices::init(&TEST_BASIC_INFO, MATTER_PORT));

    // == Step 3: ==
    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = &*Box::leak(Box::new_uninit()).init_with(EmbassyThreadMatterStack::<()>::init(
        &TEST_BASIC_INFO,
        BasicCommData {
            password: TEST_BASIC_COMM_DATA.password,
            discriminator,
        },
        &TEST_DEV_ATT,
        MdnsType::Provided(mdns_services),
        epoch,
        esp_rand,
    ));

    // == Step 4: ==
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

    // == Step 5: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let store = stack.create_shared_store(DummyKvBlobStore);
    let mut matter = pin!(stack.run(
        // The Matter stack needs to instantiate an `openthread` Radio
        EmbassyThread::new(
            EspThreadDriver::new(&init, peripherals.IEEE802154, peripherals.BT),
            mdns_services,
            ieee_eui64,
            &store,
            stack,
        ),
        // The Matter stack needs a persister to store its state
        &store,
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

/// Basic info about our device
/// Both the matter stack as well as out mDNS-to-SRP bridge need this, hence extracted out
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    vid: TEST_VID,
    pid: TEST_PID,
    hw_ver: 2,
    hw_ver_str: "2",
    sw_ver: 1,
    sw_ver_str: "1",
    serial_no: "aabbccdd",
    device_name: "MyLight",
    product_name: "ACME Light",
    vendor_name: "ACME",
    sai: Some(500),
    sii: None,
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyThreadMatterStack::<()>::root_metadata(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: &[DEV_TYPE_ON_OFF_LIGHT],
            clusters: &[descriptor::CLUSTER, cluster_on_off::CLUSTER],
        },
    ],
};

#[allow(static_mut_refs)]
fn init_heap() {
    fn add_region<const N: usize>(region: &'static mut MaybeUninit<[u8; N]>) {
        unsafe {
            esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
                region.as_mut_ptr() as *mut u8,
                N,
                esp_alloc::MemoryCapability::Internal.into(),
            ));
        }
    }

    #[cfg(feature = "esp32")]
    {
        // The esp32 has two disjoint memory regions for heap
        // Also, it has 64KB reserved for the BT stack in the first region, so we can't use that

        static mut HEAP1: MaybeUninit<[u8; 30 * 1024]> = MaybeUninit::uninit();
        #[link_section = ".dram2_uninit"]
        static mut HEAP2: MaybeUninit<[u8; 96 * 1024]> = MaybeUninit::uninit();

        add_region(unsafe { &mut HEAP1 });
        add_region(unsafe { &mut HEAP2 });
    }

    #[cfg(not(feature = "esp32"))]
    {
        static mut HEAP: MaybeUninit<[u8; 186 * 1024]> = MaybeUninit::uninit();

        add_region(unsafe { &mut HEAP });
    }
}
