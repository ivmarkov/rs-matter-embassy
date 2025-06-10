//! An example utilizing the `EthMatterStack` struct directly from `rs-matter-stack`.
//! As the name suggests, this Matter stack assembly uses Ethernet as the main transport, as well as for commissioning.
//!
//! Notice thart we actually don't use Ethernet for real, as NRFs don't have Ethernet ports out of the box.
//! Instead, we utilize Thread, which - from the POV of Matter - is indistinguishable from Ethernet as long as the Matter
//! stack is not concerned with connecting to the Thread network, managing its credentials etc. and can assume it "pre-exists".
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]
#![recursion_limit = "256"]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

use embassy_futures::select::select4;

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::peripherals::RNG;
use embassy_nrf::radio::InterruptHandler;
use embassy_nrf::{bind_interrupts, rng};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap;

use defmt::{info, unwrap};

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::data_model::basic_info::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{
    Async, Dataver, EmptyHandler, Endpoint, EpClMatcher, Node,
};
use rs_matter_embassy::matter::data_model::on_off::{self, ClusterHandler as _};
use rs_matter_embassy::matter::data_model::system_model::desc::{self, ClusterHandler as _};
use rs_matter_embassy::matter::test_device::{TEST_DEV_ATT, TEST_DEV_COMM, TEST_DEV_DET};
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::{clusters, devices, BasicCommData, MATTER_PORT};
use rs_matter_embassy::ot::openthread::nrf::NrfRadio;
use rs_matter_embassy::ot::openthread::{
    Capabilities, EmbassyTimeTimer, OpenThread, PhyRadioRunner, ProxyRadio, ProxyRadioResources,
    RamSettings,
};
use rs_matter_embassy::ot::{OtMatterResources, OtMdns, OtNetif};
use rs_matter_embassy::rand::nrf::{nrf_init_rand, nrf_rand};
use rs_matter_embassy::stack::eth::EthMatterStack;
use rs_matter_embassy::stack::mdns::MatterMdnsServices;
use rs_matter_embassy::stack::persist::DummyKvBlobStore;
use rs_matter_embassy::stack::rand::{MatterRngCore, RngCore};
use rs_matter_embassy::stack::MdnsType;

use panic_rtt_target as _;

use tinyrlibc as _;

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

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    RADIO => InterruptHandler<embassy_nrf::peripherals::RADIO>;
});

#[interrupt]
unsafe fn EGU1_SWI1() {
    RADIO_EXECUTOR.on_interrupt()
}

static RADIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

const THREAD_DATASET: &str = env!("THREAD_DATASET");

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
const LOG_RINGBUF_SIZE: usize = 2048;

#[embassy_executor::main]
async fn main(_s: Spawner) {
    // `rs-matter` uses the `x509` crate which (still) needs a few kilos of heap space
    {
        const HEAP_SIZE: usize = 8192;

        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    // == Step 1: ==
    // Necessary `nrf-hal` initialization boilerplate

    rtt_target::rtt_init_defmt!(rtt_target::ChannelMode::NoBlockSkip, LOG_RINGBUF_SIZE);

    info!("Starting...");

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    let mut rng = rng::Rng::new_blocking(p.RNG);

    // Use a random/unique Matter discriminator for this session,
    // in case there are left-overs from our previous registrations in Thread SRP
    let discriminator = (rng.next_u32() & 0xfff) as u16;

    // TODO
    let mut ieee_eui64 = [0; 8];
    RngCore::fill_bytes(&mut rng, &mut ieee_eui64);

    // To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to initialize the global `rand` fn once
    nrf_init_rand(rng);

    // == Step 2: ==
    // Replace the built-in Matter mDNS responder with a bridge that delegates
    // all mDNS work to the OpenThread SRP client.
    // Thread is not friendly to IpV6 multicast, so we have to use SRP instead.
    let mdns_services = &*mk_static!(MatterMdnsServices<'static, NoopRawMutex>)
        .init_with(MatterMdnsServices::init(&TEST_BASIC_INFO, MATTER_PORT));

    // == Step 3: ==
    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EthMatterStack).init_with(EthMatterStack::init(
        &TEST_BASIC_INFO,
        BasicCommData {
            password: TEST_DEV_COMM.password,
            discriminator,
        },
        &TEST_DEV_ATT,
        MdnsType::Provided(mdns_services),
        epoch,
        nrf_rand,
    ));

    let (radio_proxy, radio_runner) = ProxyRadio::new(
        Capabilities::empty(),
        mk_static!(ProxyRadioResources, ProxyRadioResources::new()),
    );
    let radio = NrfRadio::new(embassy_nrf::radio::ieee802154::Radio::new(p.RADIO, Irqs));

    // High-priority executor: EGU1_SWI1, priority level 6
    interrupt::EGU1_SWI1.set_priority(Priority::P6);

    // The NRF radio needs to run in a high priority executor
    // because it is lacking hardware MAC-filtering and ACK caps,
    // hence these are emulated in software, so low latency is crucial
    unwrap!(RADIO_EXECUTOR
        .start(interrupt::EGU1_SWI1)
        .spawn(run_radio(radio_runner, radio)));

    let mut ot_rng = MatterRngCore::new(stack.matter().rand());
    let ot_resources = mk_static!(OtMatterResources).init_with(OtMatterResources::init());

    let mut ot_settings = RamSettings::new(&mut ot_resources.settings_buf);

    let ot = unwrap!(OpenThread::new_with_udp_srp(
        ieee_eui64,
        &mut ot_rng,
        &mut ot_settings,
        &mut ot_resources.ot,
        &mut ot_resources.udp,
        &mut ot_resources.srp,
    ));

    let ot_mdns = unwrap!(OtMdns::new(ot.clone(), mdns_services));

    let mut ot_runner = pin!(async {
        ot.run(radio_proxy).await;
        #[allow(unreachable_code)]
        Ok(())
    });
    let mut ot_mdns_runner = pin!(async {
        unwrap!(ot_mdns.run().await);
        #[allow(unreachable_code)]
        Ok(())
    });

    unwrap!(ot.srp_autostart());

    unwrap!(ot.set_active_dataset_tlv_hexstr(THREAD_DATASET));
    unwrap!(ot.enable_ipv6(true));
    unwrap!(ot.enable_thread(true));

    // == Step 4: ==
    // Our "light" on-off cluster.
    // Can be anything implementing `rs_matter::data_model::AsyncHandler`
    let on_off = on_off::OnOffHandler::new(Dataver::new_rand(stack.matter().rand()));

    // Chain our endpoint clusters
    let handler = EmptyHandler
        // Our on-off cluster, on Endpoint 1
        .chain(
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(on_off::OnOffHandler::CLUSTER.id)),
            Async(on_off::HandlerAdaptor(&on_off)),
        )
        // Each Endpoint needs a Descriptor cluster too
        // Just use the one that `rs-matter` provides out of the box
        .chain(
            EpClMatcher::new(Some(LIGHT_ENDPOINT_ID), Some(desc::DescHandler::CLUSTER.id)),
            Async(desc::DescHandler::new(Dataver::new_rand(stack.matter().rand())).adapt()),
        );

    // == Step 5: ==
    // Run the Matter stack with our handler
    // Using `pin!` is completely optional, but saves some memory due to `rustc`
    // not being very intelligent w.r.t. stack usage in async functions
    //
    // This step can be repeated in that the stack can be stopped and started multiple times, as needed.
    let store = stack.create_shared_store(DummyKvBlobStore);
    let mut matter = pin!(stack.run_preex(
        // The Matter stack needs to open two UDP sockets
        ot.clone(),
        // The Matter stack needs access to the netif so as to detect network going up/down
        OtNetif::new(ot.clone()),
        // The Matter stack needs a persister to store its state
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
    unwrap!(
        select4(
            &mut matter,
            &mut device,
            &mut ot_runner,
            &mut ot_mdns_runner,
        )
        .coalesce()
        .await
    );
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

#[embassy_executor::task]
async fn run_radio(
    mut runner: PhyRadioRunner<'static>,
    radio: NrfRadio<'static, embassy_nrf::peripherals::RADIO>,
) -> ! {
    runner.run(radio, EmbassyTimeTimer).await
}
