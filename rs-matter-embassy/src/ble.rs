// BLE: `TroubleBtpGattPeripheral` - an implementation of the `GattPeripheral` trait from `rs-matter`.

#![allow(clippy::useless_conversion)] // https://github.com/embassy-rs/trouble/issues/248

use core::future::Future;
use core::mem::MaybeUninit;

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::data::{AclPacket, IsoPacket, SyncPacket};
use bt_hci::ControllerToHostPacket;

use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::RawMutex;

use embedded_io::ErrorType;

use log::{info, warn};

use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID,
    MATTER_BLE_SERVICE_UUID16, MAX_BTP_SESSIONS,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::rand::Rand;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::IfMutex;

use trouble_host::att::{AttReq, AttRsp};
use trouble_host::prelude::*;
use trouble_host::{self, Address, BleHostError, Controller, HostResources};

const MAX_CONNECTIONS: usize = MAX_BTP_SESSIONS;
const MAX_MTU_SIZE: usize = 251; // For now 512; // TODO const L2CAP_MTU: usize = 251;
const MAX_CHANNELS: usize = 2;
const ADV_SETS: usize = 1;

pub type GPHostResources = HostResources<MAX_CONNECTIONS, MAX_CHANNELS, MAX_MTU_SIZE, ADV_SETS>;

/// A companion trait of `EmbassyBle` for providing a BLE controller.
// TODO: Move to `wireless` once https://github.com/embassy-rs/bt-hci/issues/32 is resolved
pub trait BleControllerProvider {
    type Controller<'a>: Controller
    where
        Self: 'a;

    /// Provide a BLE controller by creating it when the Matter stack needs it
    async fn provide(&mut self) -> Self::Controller<'_>;
}

impl<T> BleControllerProvider for &mut T
where
    T: BleControllerProvider,
{
    type Controller<'a>
        = T::Controller<'a>
    where
        Self: 'a;

    async fn provide(&mut self) -> Self::Controller<'_> {
        (*self).provide().await
    }
}

pub struct Preexisting<C>(pub C);

impl<C> BleControllerProvider for Preexisting<C>
where
    C: Controller,
{
    type Controller<'a> = ControllerRef<'a, C> where Self: 'a;

    async fn provide(&mut self) -> Self::Controller<'_> {
        ControllerRef::new(&self.0)
    }
}

type External = [u8; 0];

// GATT Server definition
#[gatt_server]
struct Server {
    matter_service: MatterService,
}

/// Matter service
#[gatt_service(uuid = MATTER_BLE_SERVICE_UUID16)]
struct MatterService {
    #[characteristic(uuid = Uuid::Uuid128(C1_CHARACTERISTIC_UUID.to_le_bytes()), write)]
    c1: External,
    #[characteristic(uuid = Uuid::Uuid128(C2_CHARACTERISTIC_UUID.to_le_bytes()), write, indicate)]
    c2: External,
}

#[derive(Debug)]
struct IndBuffer {
    addr: BtAddr,
    data: Vec<u8, MAX_MTU_SIZE>,
}

impl IndBuffer {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            addr: BtAddr([0; 6]),
            data: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            addr: BtAddr([0; 6]),
            data <- Vec::init(),
        })
    }
}

/// The `'static` state of the `TroubleBtpGattPeripheral` struct.
/// Isolated as a separate struct to allow for `const fn` construction
/// and static allocation.
pub struct TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    ind: IfMutex<M, IndBuffer>,
    resources: IfMutex<M, GPHostResources>,
}

impl<M> TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            ind: IfMutex::new(IndBuffer::new()),
            resources: IfMutex::new(GPHostResources::new()),
        }
    }

    /// Return an in-place initializer for the type.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ind <- IfMutex::init(IndBuffer::init()),
            // Note: below will break if `HostResources` stops being a bunch of `MaybeUninit`s
            resources <- IfMutex::init(unsafe { MaybeUninit::<GPHostResources>::uninit().assume_init() }),
        })
    }

    // pub(crate) fn reset(&self) -> Result<(), ()> {
    //     self.ind
    //         .try_lock()
    //         .map(|mut ind| {
    //             ind.data.clear();
    //         })
    //         .unwrap(); // TODO

    //     Ok(())
    // }
}

impl<M> Default for TroubleBtpGattContext<M>
where
    M: RawMutex,
{
    // TODO
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    fn default() -> Self {
        Self::new()
    }
}

/// A GATT peripheral implementation for the BTP protocol in `rs-matter` via `trouble-host`.
/// Implements the `GattPeripheral` trait.
pub struct TroubleBtpGattPeripheral<'a, M, C>
where
    M: RawMutex,
    C: Controller,
{
    // TODO: Ideally this should be the controller itself, but this is not possible
    // until `bt-hci` is updated with `impl<C: Controller>` Controller for &C {}`
    controller: IfMutex<M, C>,
    rand: Rand,
    context: &'a TroubleBtpGattContext<M>,
}

impl<'a, M, C> TroubleBtpGattPeripheral<'a, M, C>
where
    M: RawMutex,
    C: Controller,
{
    /// Create a new instance.
    ///
    /// Creation might fail if the GATT context cannot be reset, so user should ensure
    /// that there are no other GATT peripherals running before calling this function.
    // TODO: change `provider` to `controller` once https://github.com/embassy-rs/bt-hci/issues/32 is resolved
    pub const fn new(controller: C, rand: Rand, context: &'a TroubleBtpGattContext<M>) -> Self {
        Self {
            controller: IfMutex::new(controller),
            rand,
            context,
        }
    }

    /// Run the GATT peripheral.
    pub async fn run<F>(
        &self,
        service_name: &str,
        service_adv_data: &AdvData,
        mut callback: F,
    ) -> Result<(), ()>
    where
        F: FnMut(GattPeripheralEvent) + Send,
    {
        info!("Starting advertising and GATT service");

        let controller = self.controller.lock().await;
        let mut resources = self.context.resources.lock().await;

        let controller = ControllerRef::new(&*controller);

        let mut address = [0; 6];
        (self.rand)(&mut address);

        let address = Address::random(address);
        info!("GATT address = {:?}", address);

        let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

        let Host {
            mut peripheral,
            runner,
            ..
        } = stack.build();

        let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: "TrouBLE",                                             // TODO
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE, // TODO
        }))
        .unwrap();

        let _ = join(Self::run_ble(runner), async {
            loop {
                match Self::advertise(service_name, service_adv_data, &mut peripheral).await {
                    Ok(conn) => {
                        let events = Self::handle_events(&server, &conn, &mut callback);
                        let indications =
                            Self::handle_indications(&server, &conn, &self.context.ind);

                        select(events, indications).await;
                    }
                    Err(e) => {
                        // #[cfg(feature = "defmt")]
                        // let e = defmt::Debug2Format(&e);
                        panic!("[adv] error: {:?}", e);
                    }
                }
            }
        })
        .await;

        Ok(())
    }

    async fn run_ble<CC>(mut runner: Runner<'_, CC>)
    where
        CC: Controller,
    {
        loop {
            if let Err(e) = runner.run().await {
                // #[cfg(feature = "defmt")]
                // let e = defmt::Debug2Format(&e);
                panic!("[ble_task] error: {:?}", e);
            }
        }
    }

    async fn handle_indications(
        server: &Server<'_>,
        conn: &Connection<'_>,
        ind: &IfMutex<M, IndBuffer>,
    ) -> Result<(), trouble_host::Error> {
        loop {
            let mut ind = ind.lock_if(|ind| !ind.data.is_empty()).await;

            server
                .matter_service
                .c2
                .notify_raw(server, conn, &ind.data)
                .await?;

            ind.data.clear();
        }
    }

    /// Stream Events until the connection closes.
    ///
    /// This function will handle the GATT events and process them.
    /// This is how we interact with read and write requests.
    async fn handle_events<F>(
        server: &Server<'_>,
        conn: &Connection<'_>,
        mut callback: F,
    ) -> Result<(), Error>
    where
        F: FnMut(GattPeripheralEvent) + Send,
    {
        fn to_bt_addr(addr: &BdAddr) -> BtAddr {
            let raw = addr.raw();
            BtAddr([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]])
        }

        loop {
            match conn.next().await {
                ConnectionEvent::Disconnected { reason } => {
                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                        &conn.peer_address(),
                    )));
                    info!("GATT: Disconnected: {:?}", reason);
                    break;
                }
                ConnectionEvent::Gatt { data } => {
                    let request = data.request();

                    info!("GATT: Got event: {:?}", request);

                    if let AttReq::Write {
                        handle,
                        data: bytes,
                    } = request
                    {
                        if handle == server.matter_service.c1.handle {
                            info!("GATT: Write {:?} / MTU {}", bytes, conn.att_mtu());

                            callback(GattPeripheralEvent::Write {
                                address: to_bt_addr(&conn.peer_address()),
                                data: bytes,
                                gatt_mtu: Some(conn.att_mtu()),
                            });

                            data.reply(AttRsp::Write).await.unwrap();

                            continue;
                        }
                    }

                    match data.process(server).await {
                        Ok(Some(GattEvent::Write(event))) => {
                            if Some(event.handle()) == server.matter_service.c2.cccd_handle {
                                let data = event.data();
                                let subscribed = data[0] != 0;

                                info!("GATT: Write Event to CCC Characteristic: {:?}", data);

                                if subscribed {
                                    callback(GattPeripheralEvent::NotifySubscribed(to_bt_addr(
                                        &conn.peer_address(),
                                    )));
                                } else {
                                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                                        &conn.peer_address(),
                                    )));
                                }
                            }
                        }
                        Ok(_) => {}
                        Err(e) => {
                            warn!("GATT: Error processing event: {:?}", e);
                        }
                    }
                }
            }
        }

        info!("GATT: Task finished");

        Ok(())
    }

    /// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
    async fn advertise<'p, CC>(
        service_name: &str,
        service_adv_data: &AdvData,
        peripheral: &mut Peripheral<'p, CC>,
    ) -> Result<Connection<'p>, BleHostError<CC::Error>>
    where
        CC: Controller,
    {
        let service_adv_enc_data = service_adv_data
            .service_payload_iter()
            .collect::<Vec<_, 8>>();

        let adv_data = [
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceData16 {
                uuid: MATTER_BLE_SERVICE_UUID16,
                data: &service_adv_enc_data,
            },
            AdStructure::CompleteLocalName(service_name.as_bytes()),
        ];

        let mut adv_enc_data = [0; 31];
        let len = AdStructure::encode_slice(&adv_data, &mut adv_enc_data)?;

        let advertiser = peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &adv_enc_data[..len],
                    scan_data: &[],
                },
            )
            .await?;

        info!("GATT: Advertising");

        let conn = advertiser.accept().await?;

        info!("GATT: Connection established");

        Ok(conn)
    }

    /// Indicate new data on characteristic `C2` to a remote peer.
    pub async fn indicate(&self, data: &[u8], address: BtAddr) {
        self.context
            .ind
            .with(|ind| {
                if ind.data.is_empty() {
                    ind.data.extend_from_slice(data).unwrap();
                    ind.addr = address;

                    Some(())
                } else {
                    None
                }
            })
            .await;
    }
}

impl<M, C> GattPeripheral for TroubleBtpGattPeripheral<'_, M, C>
where
    M: RawMutex,
    C: Controller,
{
    async fn run<F>(&self, service_name: &str, adv_data: &AdvData, callback: F) -> Result<(), Error>
    where
        F: FnMut(GattPeripheralEvent) + Send + Clone + 'static,
    {
        TroubleBtpGattPeripheral::run(self, service_name, adv_data, callback)
            .await
            .map_err(|_| ErrorCode::BtpError)?;

        Ok(())
    }

    async fn indicate(&self, data: &[u8], address: BtAddr) -> Result<(), Error> {
        TroubleBtpGattPeripheral::indicate(self, data, address).await;

        Ok(())
    }
}

pub struct ControllerRef<'a, C>(&'a C);

impl<'a, C> ControllerRef<'a, C> {
    pub const fn new(controller: &'a C) -> Self {
        Self(controller)
    }
}

impl<C> ErrorType for ControllerRef<'_, C>
where
    C: ErrorType,
{
    type Error = C::Error;
}

impl<C> bt_hci::controller::Controller for ControllerRef<'_, C>
where
    C: bt_hci::controller::Controller,
{
    fn write_acl_data(&self, packet: &AclPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_acl_data(packet)
    }

    fn write_sync_data(&self, packet: &SyncPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_sync_data(packet)
    }

    fn write_iso_data(&self, packet: &IsoPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_iso_data(packet)
    }

    fn read<'a>(&self, buf: &'a mut [u8]) -> impl Future<Output = Result<ControllerToHostPacket<'a>, Self::Error>> {
        self.0.read(buf)
    }
}

impl<C> bt_hci::controller::blocking::Controller for ControllerRef<'_, C>
where
    C: bt_hci::controller::blocking::Controller,
{
    fn write_acl_data(&self, packet: &AclPacket) -> Result<(), Self::Error> {
        self.0.write_acl_data(packet)
    }

    fn write_sync_data(&self, packet: &SyncPacket) -> Result<(), Self::Error> {
        self.0.write_sync_data(packet)
    }

    fn write_iso_data(&self, packet: &IsoPacket) -> Result<(), Self::Error> {
        self.0.write_iso_data(packet)
    }

    fn try_write_acl_data(&self, packet: &AclPacket) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_acl_data(packet)
    }

    fn try_write_sync_data(&self, packet: &SyncPacket) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_sync_data(packet)
    }

    fn try_write_iso_data(&self, packet: &IsoPacket) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_iso_data(packet)
    }

    fn read<'a>(&self, buf: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        self.0.read(buf)
    }

    fn try_read<'a>(&self, buf: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_read(buf)
    }
}

impl<C, Q> ControllerCmdSync<Q> for ControllerRef<'_, C>
where
    C: ControllerCmdSync<Q>,
    Q: SyncCmd + ?Sized,
{
    fn exec(&self, cmd: &Q) -> impl Future<Output = Result<Q::Return, bt_hci::cmd::Error<Self::Error>>> {
        self.0.exec(cmd)
    }
}

impl<C, Q> ControllerCmdAsync<Q> for ControllerRef<'_, C>
where
    C: ControllerCmdAsync<Q>,
    Q: AsyncCmd + ?Sized,
{
    fn exec(&self, cmd: &Q) -> impl Future<Output = Result<(), bt_hci::cmd::Error<Self::Error>>> {
        self.0.exec(cmd)
    }
}

#[cfg(feature = "esp")]
pub mod esp {
    use bt_hci::controller::ExternalController;

    use esp_hal::peripheral::{Peripheral, PeripheralRef};

    use esp_wifi::ble::controller::BleConnector;
    use esp_wifi::EspWifiController;

    const SLOTS: usize = 20;

    /// A `BleControllerProvider` implementation for the ESP32 family of chips.
    pub struct EspBleControllerProvider<'a, 'd> {
        controller: &'a EspWifiController<'d>,
        peripheral: PeripheralRef<'d, esp_hal::peripherals::BT>,
    }

    impl<'a, 'd> EspBleControllerProvider<'a, 'd> {
        /// Create a new instance
        ///
        /// # Arguments
        /// - `controller`: The WiFi controller instance
        /// - `peripheral`: The Bluetooth peripheral instance
        pub fn new(
            controller: &'a EspWifiController<'d>,
            peripheral: impl Peripheral<P = esp_hal::peripherals::BT> + 'd,
        ) -> Self {
            Self {
                controller,
                peripheral: peripheral.into_ref(),
            }
        }
    }

    impl super::BleControllerProvider for EspBleControllerProvider<'_, '_> {
        type Controller<'t>
            = ExternalController<BleConnector<'t>, SLOTS>
        where
            Self: 't;

        async fn provide(&mut self) -> Self::Controller<'_> {
            ExternalController::new(BleConnector::new(self.controller, &mut self.peripheral))
        }
    }
}
