//! An example utilizing the `EmbassyThreadMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Thread as the main transport,
//! and thus BLE for commissioning).
//! TODO: below is not implemented for Nordic yet
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::pin::pin;
use core::ptr::addr_of_mut;

use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, rng};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap;

use log::info;

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::data_model::cluster_basic_information::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::cluster_on_off;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{Dataver, Endpoint, HandlerCompat, Node};
use rs_matter_embassy::matter::data_model::system_model::descriptor;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::matter::MATTER_PORT;
use rs_matter_embassy::ot::OtMatterResources;
use rs_matter_embassy::rand::nrf::{nrf_init_rand, nrf_rand};
use rs_matter_embassy::stack::mdns::MatterMdnsServices;
use rs_matter_embassy::stack::persist::DummyPersist;
use rs_matter_embassy::stack::rand::{MatterRngCore, RngCore};
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::MdnsType;
use rs_matter_embassy::wireless::nrf::NrfBleController;
use rs_matter_embassy::wireless::thread::nrf::{
    NrfThreadRadio, NrfThreadRadioInterruptHandler, NrfThreadRadioResources, NrfThreadRadioRunner,
};
use rs_matter_embassy::wireless::thread::{EmbassyThread, EmbassyThreadNCMatterStack};
use rs_matter_embassy::wireless::EmbassyBle;

use panic_rtt_target as _;

use tinyrlibc as _;

use rtt_target::rtt_init_log;

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
    EGU0_SWI0 => rs_matter_embassy::wireless::nrf::NrfBleLowPrioInterruptHandler;
    CLOCK_POWER => rs_matter_embassy::wireless::nrf::NrfBleClockInterruptHandler;
    RADIO => rs_matter_embassy::wireless::nrf::NrfBleHighPrioInterruptHandler, NrfThreadRadioInterruptHandler;
    TIMER0 => rs_matter_embassy::wireless::nrf::NrfBleHighPrioInterruptHandler;
    RTC0 => rs_matter_embassy::wireless::nrf::NrfBleHighPrioInterruptHandler;
});

#[interrupt]
unsafe fn EGU1_SWI1() {
    RADIO_EXECUTOR.on_interrupt()
}

static RADIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
const LOG_RINGBUF_SIZE: usize = 4096;

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

    rtt_init_log!(
        log::LevelFilter::Info,
        rtt_target::ChannelMode::NoBlockSkip,
        LOG_RINGBUF_SIZE
    );

    info!("Starting...");

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    let mut rng = rng::Rng::new(p.RNG, Irqs);

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
    let mdns_services = mk_static!(MatterMdnsServices<'static, NoopRawMutex>)
        .init_with(MatterMdnsServices::init(&TEST_BASIC_INFO, MATTER_PORT));

    // == Step 3: ==
    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack =
        mk_static!(EmbassyThreadNCMatterStack<()>).init_with(EmbassyThreadNCMatterStack::init(
            &TEST_BASIC_INFO,
            TEST_BASIC_COMM_DATA,
            &TEST_DEV_ATT,
            MdnsType::Builtin,
            epoch,
            nrf_rand,
        ));

    let nrf_ble_controller = NrfBleController::new(
        p.RADIO,
        p.RTC0,
        p.TIMER0,
        p.TEMP,
        p.PPI_CH17,
        p.PPI_CH18,
        p.PPI_CH19,
        p.PPI_CH20,
        p.PPI_CH21,
        p.PPI_CH22,
        p.PPI_CH23,
        p.PPI_CH24,
        p.PPI_CH25,
        p.PPI_CH26,
        p.PPI_CH27,
        p.PPI_CH28,
        p.PPI_CH29,
        p.PPI_CH30,
        p.PPI_CH31,
        stack.matter().rand(),
        Irqs,
    );

    // TODO: Share the radio peripheral between the BLE and Thread stacks
    let (nrf_radio, nrf_radio_runner) = NrfThreadRadio::new(
        mk_static!(NrfThreadRadioResources, NrfThreadRadioResources::new()),
        unsafe { embassy_nrf::peripherals::RADIO::steal() },
        Irqs,
    );

    // High-priority executor: EGU1_SWI1, priority level 6
    interrupt::EGU1_SWI1.set_priority(Priority::P6);

    // The NRF radio needs to run in a high priority executor
    // because it is lacking hardware MAC-filtering and ACK caps,
    // hence these are emulated in software, so low latency is crucial
    RADIO_EXECUTOR
        .start(interrupt::EGU1_SWI1)
        .spawn(run_radio(nrf_radio_runner))
        .unwrap();

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
    let mut ot_rng = MatterRngCore::new(stack.matter().rand());
    let ot_resources = mk_static!(OtMatterResources).init_with(OtMatterResources::init());
    let mut matter = pin!(stack.run(
        // The Matter stack needs to instantiate `openthread`
        EmbassyThread::new(
            nrf_radio,
            mdns_services,
            ot_resources,
            ieee_eui64,
            &mut ot_rng
        )
        .unwrap(),
        // The Matter stack needs BLE
        EmbassyBle::new(nrf_ble_controller, stack),
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

/// Basic info about our device
/// Both the matter stack as well as out mDNS-to-SRP bridge need this, hence extracted out
const TEST_BASIC_INFO: BasicInfoConfig = BasicInfoConfig {
    vid: TEST_VID,
    pid: TEST_PID,
    hw_ver: 2,
    sw_ver: 1,
    sw_ver_str: "1",
    serial_no: "aabbccdd",
    device_name: "MyLight",
    product_name: "ACME Light",
    vendor_name: "ACME",
};

/// Endpoint 0 (the root endpoint) always runs
/// the hidden Matter system clusters, so we pick ID=1
const LIGHT_ENDPOINT_ID: u16 = 1;

/// The Matter Light device Node
const NODE: Node = Node {
    id: 0,
    endpoints: &[
        EmbassyThreadNCMatterStack::<()>::root_metadata(),
        Endpoint {
            id: LIGHT_ENDPOINT_ID,
            device_types: &[DEV_TYPE_ON_OFF_LIGHT],
            clusters: &[descriptor::CLUSTER, cluster_on_off::CLUSTER],
        },
    ],
};

#[embassy_executor::task]
async fn run_radio(mut runner: NrfThreadRadioRunner<'static, 'static>) -> ! {
    runner.run().await
}
