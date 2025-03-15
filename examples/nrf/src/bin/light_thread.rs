//! An example utilizing the `EmbassyThreadMatterStack` struct.
//!
//! As the name suggests, this Matter stack assembly uses Thread as the main transport,
//! and thus BLE for commissioning).
//! TODO: below is not implemented for Nordic yet
//!
//! The example implements a fictitious Light device (an On-Off Matter cluster).
#![no_std]
#![no_main]

// use core::mem::MaybeUninit;
use core::pin::pin;
// use core::future::Future;

use static_cell::StaticCell;
// use anyhow::Error;
// use alloc::boxed::Box;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap;
// use embassy_futures::block_on;
// use embassy_executor::Executor;

use log::info;
// use defmt::{error, info, unwrap};

use rs_matter_embassy::epoch::epoch;
use rs_matter_embassy::matter::data_model::cluster_basic_information::BasicInfoConfig;
use rs_matter_embassy::matter::data_model::cluster_on_off;
use rs_matter_embassy::matter::data_model::device_types::DEV_TYPE_ON_OFF_LIGHT;
use rs_matter_embassy::matter::data_model::objects::{Dataver, Endpoint, HandlerCompat, Node};
use rs_matter_embassy::matter::data_model::system_model::descriptor;
use rs_matter_embassy::matter::utils::init::InitMaybeUninit;
use rs_matter_embassy::matter::utils::select::Coalesce;
use rs_matter_embassy::stack::persist::DummyPersist;
use rs_matter_embassy::stack::test_device::{
    TEST_BASIC_COMM_DATA, TEST_DEV_ATT, TEST_PID, TEST_VID,
};
use rs_matter_embassy::stack::MdnsType;
// use rs_matter_embassy::wireless::nrf::NrfBleControllerProvider;
use rs_matter_embassy::wireless::nrf::SoftdeviceExternalController;
use rs_matter_embassy::wireless::thread::nrf::NrfThreadDriverProvider;
use rs_matter_embassy::wireless::thread::{EmbassyThread, EmbassyThreadMatterStack};
use rs_matter_embassy::wireless::{EmbassyBle, PreexistingBleController};
// use rs_matter_embassy::wireless::EmbassyBle;

use rs_matter_embassy::rand::nrf::{nrf_init_rand, nrf_rand};

use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};

use bt_hci::controller::ExternalController;

// use bt_hci::{ControllerToHostPacket, FromHciBytes, HostToControllerPacket, PacketKind, WriteHci};

// use embassy_nrf::rng::Rng;
// use embassy_nrf::peripherals;
// use embassy_nrf::{bind_interrupts, peripherals, rng};
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, rng};
// use rand::Rng as _;

// extern crate alloc;
// use nrf_matter::persist;
// use log::{error, info};
// use static_cell::StaticCell;

// use {defmt_rtt as _, panic_probe as _};
// use rand::Rng as _;

use rtt_target::rtt_init_log;

// static EXECUTOR: StaticCell<Executor> = StaticCell::new();

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 20;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 20;

/// Size of L2CAP packets
const L2CAP_MTU: usize = 27;

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

// bind_interrupts!(struct Irqs {
//     RNG => rng::InterruptHandler<peripherals::RNG>;
// });

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
});

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();
// #[global_allocator]
// static HEAP: Heap = Heap::empty();

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    //defmt::error!("panicked");
    exit()
}

pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<RNG>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()?
        .support_peripheral()?
        .peripheral_count(1)?
        .buffer_cfg(L2CAP_MTU as u8, L2CAP_MTU as u8, L2CAP_TXQ, L2CAP_RXQ)?
        .build(p, rng, mpsl, mem)
}

/// We need a bigger log ring-buffer or else the device QR code printout is half-lost
const LOG_RINGBUF_SIZE: usize = 4096;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // `rs-matter` uses the `x509` crate which (still) needs a few kilos of heap space
    {
        const HEAP_SIZE: usize = 8192;

        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    // == Step 1: ==
    // Necessary `nrf-hal` and `mrf-thread` initialization boilerplate

    rtt_init_log!(
        log::LevelFilter::Info,
        rtt_target::ChannelMode::NoBlockSkip,
        LOG_RINGBUF_SIZE
    );

    info!("Starting...");

    // let peripherals = mrf_hal::init({
    //     let mut config = nrf_hal::Config::default();
    //     config.cpu_clock = CpuClock::max();
    //     config
    // });

    // let timg0 = TimerGroup::new(peripherals.TIMG0);
    // let rng = esp_hal::rng::Rng::new(peripherals.RNG);

    // let peripherals = embassy_nrf::init(Default::default());

    let p = embassy_nrf::init(Default::default());
    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.must_spawn(mpsl_task(&*mpsl));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let mut sdc_mem = sdc::Mem::<3312>::new();
    let sdc = build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem).unwrap();

    let controller: ExternalController<_, 20> =
        ExternalController::new(SoftdeviceExternalController(sdc));

    // To erase generics, `Matter` takes a rand `fn` rather than a trait or a closure,
    // so we need to initialize the global `rand` fn once
    // nrf_init_rand(rng);
    nrf_init_rand(p.RNG);

    // let init = nrf_thread::init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap();

    // nrf_hal_embassy::init(timg0.timer1);

    // == Step 2: ==
    // Allocate the Matter stack.
    // For MCUs, it is best to allocate it statically, so as to avoid program stack blowups (its memory footprint is ~ 35 to 50KB).
    // It is also (currently) a mandatory requirement when the wireless stack variation is used.
    let stack = mk_static!(EmbassyThreadMatterStack<()>).init_with(EmbassyThreadMatterStack::init(
        // let stack = &*Box::leak(Box::new_uninit()).init_with(EmbassyThreadMatterStack::<()>::init(
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
        nrf_rand,
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
        EmbassyThread::new(
            NrfThreadDriverProvider::new(
            // &init, p.THREAD
        ),
            stack
        ),
        // The Matter stack needs BLE
        // EmbassyBle::new(NrfBleControllerProvider::new(&init, peripherals.BT), stack),
        EmbassyBle::new(PreexistingBleController::new(controller), stack),
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
    // select(&mut matter, &mut device).coalesce().await?;
    // Ok(())
}

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
