//! An example utilizing the `EthMatterStack` struct directly from `rs-matter-stack`.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as ESP32s don't have Ethernet ports out of the box.
//! Instead, we utilize Thread, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Thread network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::env;
use core::mem::MaybeUninit;
use core::pin::pin;

use alloc::boxed::Box;

use embassy_executor::Spawner;
use embassy_futures::select::select4;
use embassy_time::{Duration, Timer};

use esp_backtrace as _;
use esp_ieee802154::Ieee802154;

use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::data_model::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{
    Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node,
};
use rs_matter_embassy::matter::data_model::on_off::{self, ClusterHandler as _};
use rs_matter_embassy::matter::data_model::system_model::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::{clusters, devices, BasicCommData, MATTER_PORT};
use rs_matter_embassy::ot::openthread::esp::EspRadio;
use rs_matter_embassy::ot::openthread::{OpenThread, RamSettings};
use rs_matter_embassy::ot::{OtMatterResources, OtMdns, OtNetif};
use rs_matter_embassy::rand::esp::{esp_init_rand, esp_rand};
use rs_matter_embassy::stack::eth::EthMatterStack;
use rs_matter_embassy::stack::matter::test_device::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use rs_matter_embassy::stack::mdns::MatterMdnsServices;
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::stack::rand::{MatterRngCore, RngCore};
use rs_matter_embassy::stack::MdnsType;

use tinyrlibc as _;

extern crate alloc;

const THREAD_DATASET: &str = env!("THREAD_DATASET");

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(_s: Spawner) {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    info!("Starting...");

    // == Step 1: ==
    // Necessary `esp-hal` and `esp-wifi` initialization boilerplate

    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Heap strictly necessary only for Wifi and for the only Matter dependency which needs (~4KB) alloc - `x509`
    // However since `esp32` specifically has a disjoint heap which causes bss size troubles, it is easier
    // to allocate the statics once from heap as well
    init_heap();

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);

    // Use a random/unique Matter discriminator for this session,
    // in case there are left-overs from our previous registrations in Thread SRP
    let discriminator = (rng.next_u32() & 0xfff) as u16;

    // TODO
    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(&mut rng, &mut ieee_eui64);

    // To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to initialize the global `rand` fn once
    esp_init_rand(rng);

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
    let stack = Box::leak(Box::new_uninit()).init_with(EthMatterStack::<()>::init(
        &TEST_BASIC_INFO,
        BasicCommData {
            password: TEST_DEV_COMM.password,
            discriminator,
        },
        &TEST_DEV_ATT,
        MdnsType::Provided(mdns_services),
        epoch,
        esp_rand,
    ));

    let mut ot_rng = MatterRngCore::new(stack.matter().rand());
    let ot_resources = Box::leak(Box::new_uninit()).init_with(OtMatterResources::init());

    let mut ot_settings = RamSettings::new(&mut ot_resources.settings_buf);

    let ot = OpenThread::new_with_udp_srp(
        ieee_eui64,
        &mut ot_rng,
        &mut ot_settings,
        &mut ot_resources.ot,
        &mut ot_resources.udp,
        &mut ot_resources.srp,
    )
    .unwrap();

    let ot_mdns = OtMdns::new(ot.clone(), mdns_services).unwrap();

    let mut ot_runner = pin!(async {
        ot.run(EspRadio::new(Ieee802154::new(
            peripherals.IEEE802154,
            peripherals.RADIO_CLK,
        )))
        .await;
        #[allow(unreachable_code)]
        Ok(())
    });
    let mut ot_mdns_runner = pin!(async {
        ot_mdns.run().await.unwrap();
        #[allow(unreachable_code)]
        Ok(())
    });

    ot.srp_autostart().unwrap();

    ot.set_active_dataset_tlv_hexstr(THREAD_DATASET).unwrap();
    ot.enable_ipv6(true).unwrap();
    ot.enable_thread(true).unwrap();

    // == Step 4: ==
    // Our "light" on-off cluster.
    // Can be anything implementing `rs_matter::data_model::AsyncHandler`
    let on_off = on_off::OnOffHandler::new(Dataver::new_rand(stack.matter().rand()));

    // Chain our endpoint clusters
    let handler = EmptyHandler
        // Our on-off cluster, on Endpoint 1
        .chain(
            EpClMatcher::new(
                Some(LIGHT_ENDPOINT_ID),
                Some(on_off::OnOffHandler::CLUSTER.id),
            ),
            Async(on_off::HandlerAdaptor(&on_off)),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(desc::DescHandler::CLUSTER.id)),
            Async(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
        );

    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    let store = stack.create_shared_store(DummyKvBlobStore);
    let mut matter = pin!(stack.run_preex(
        // The Matter stack needs to open two UDP sockets
        ot.clone(),
        // The Matter stack needs access to the netif so as to detect network going up/down
        OtNetif::new(ot.clone()),
        // The Matter stack needs a persister to store its state
        // `EmbassyPersist`+`EmbassyKvBlobStore` saves to a user-supplied NOR Flash region
        // However, for this demo and for simplicity, we use a dummy persister that does nothing
        &store,
        // Our `AsyncHandler` + `AsyncMetadata` impl
        (NODE, handler),
        // No user future to run
        (),
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
        &mut ot_runner,
        &mut ot_mdns_runner,
    )
    .coalesce()
    .await
    .unwrap();
}

/// Basic info about our device
/// Both the matter stack as well as our mDNS-to-SRP bridge need this, hence extracted out
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    sai: Some(1000),
    ..TEST_DEV_DET
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EthMatterStack::<()>::root_endpoint(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: devices!(DEV_TYPE_ON_OFF_LIGHT),
            clusters: clusters!(desc::DescHandler::CLUSTER, on_off::OnOffHandler::CLUSTER),
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
