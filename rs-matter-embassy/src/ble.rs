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

use log::{debug, info, warn};

use rs_matter_stack::matter::error::{Error, ErrorCode};
use rs_matter_stack::matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID,
    MATTER_BLE_SERVICE_UUID16,
};
use rs_matter_stack::matter::transport::network::BtAddr;
use rs_matter_stack::matter::utils::init::{init, Init};
use rs_matter_stack::matter::utils::rand::Rand;
use rs_matter_stack::matter::utils::storage::Vec;
use rs_matter_stack::matter::utils::sync::IfMutex;

use trouble_host::att::{AttCfm, AttClient, AttReq, AttRsp, AttUns};
use trouble_host::prelude::*;
use trouble_host::{self, Address, BleHostError, Controller, HostResources};

const MAX_CONNECTIONS: usize = 1;
// Issue with esp32c6: we can't go lower than 255 on it
// Issue with esp32: we can't go lower than 251 on it
// TODO: Make the MTU size a feature in future
#[cfg(any(target_arch = "riscv32", target_arch = "xtensa"))]
const MAX_MTU_SIZE: usize = 255;
#[cfg(not(any(target_arch = "riscv32", target_arch = "xtensa")))]
const MAX_MTU_SIZE: usize = 131;
const MAX_CHANNELS: usize = 2;
const ADV_SETS: usize = 1;

pub type GPHostResources = HostResources<MAX_CONNECTIONS, MAX_CHANNELS, MAX_MTU_SIZE, ADV_SETS>;

type External = [u8; 0];

// GATT Server definition
#[gatt_server]
struct Server {
    matter_service: MatterService,
}

/// Matter service
#[gatt_service(uuid = MATTER_BLE_SERVICE_UUID16)]
struct MatterService {
    #[characteristic(uuid = C1_CHARACTERISTIC_UUID, write)]
    c1: External,
    #[characteristic(uuid = C2_CHARACTERISTIC_UUID, write, indicate)]
    c2: External,
}

#[derive(Debug)]
struct IndBuffer {
    addr: BtAddr,
    data: Vec<u8, MAX_MTU_SIZE>,
    in_flight: bool,
}

impl IndBuffer {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            addr: BtAddr([0; 6]),
            data: Vec::new(),
            in_flight: false,
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            addr: BtAddr([0; 6]),
            data <- Vec::init(),
            in_flight: false,
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
                        let events =
                            Self::handle_events(&server, &conn, &self.context.ind, &mut callback);
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
            let mut ind = ind
                .lock_if(|ind| !ind.data.is_empty() && !ind.in_flight)
                .await;

            ind.in_flight = true;

            GattData::send_unsolicited(
                conn,
                AttUns::Indicate {
                    handle: server.matter_service.c2.handle,
                    data: &ind.data,
                },
            )
            .await?;

            debug!("GATT: Indicate {:02x?} len {}", ind.data, ind.data.len());
        }
    }

    /// Stream Events until the connection closes.
    ///
    /// This function will handle the GATT events and process them.
    /// This is how we interact with read and write requests.
    async fn handle_events<F>(
        server: &Server<'_>,
        conn: &Connection<'_>,
        ind: &IfMutex<M, IndBuffer>,
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
                    info!("GATT: Disconnect {:?}", reason);

                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                        &conn.peer_address(),
                    )));
                    break;
                }
                ConnectionEvent::Gatt { data } => {
                    let incoming = data.incoming();

                    match incoming {
                        AttClient::Request(AttReq::Write {
                            handle,
                            data: bytes,
                        }) => {
                            if handle == server.matter_service.c1.handle {
                                debug!(
                                    "GATT: C1 Write {:02x?} len {} / MTU {}",
                                    bytes,
                                    bytes.len(),
                                    conn.att_mtu()
                                );

                                callback(GattPeripheralEvent::Write {
                                    address: to_bt_addr(&conn.peer_address()),
                                    data: bytes,
                                    gatt_mtu: Some(conn.att_mtu()),
                                });

                                data.reply(AttRsp::Write).await.unwrap();

                                continue;
                            } else if Some(handle) == server.matter_service.c2.cccd_handle {
                                let subscribed = bytes[0] != 0;

                                debug!("GATT: Write to C2 CCC descriptor: {:?}", bytes);

                                if subscribed {
                                    callback(GattPeripheralEvent::NotifySubscribed(to_bt_addr(
                                        &conn.peer_address(),
                                    )));
                                } else {
                                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                                        &conn.peer_address(),
                                    )));
                                }

                                data.reply(AttRsp::Write).await.unwrap();

                                continue;
                            }
                        }
                        AttClient::Confirmation(AttCfm::ConfirmIndication) => {
                            debug!("GATT: Confirm indication");

                            ind.with(|ind| {
                                assert!(!ind.data.is_empty() && ind.in_flight);

                                ind.data.clear();
                                ind.in_flight = false;

                                Some(())
                            })
                            .await;

                            continue;
                        }
                        _ => (),
                    }

                    if let Err(e) = data.process(server).await {
                        warn!("GATT: Error processing event: {:?}", e);
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

/// A newtype allowing to use a bt_hci `&Controller` as a `Controller`
/// A workaround for:
/// https://github.com/embassy-rs/bt-hci/issues/32
pub struct ControllerRef<'a, C>(&'a C);

impl<'a, C> ControllerRef<'a, C> {
    /// Create a new instance.
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

    fn write_sync_data(
        &self,
        packet: &SyncPacket,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_sync_data(packet)
    }

    fn write_iso_data(&self, packet: &IsoPacket) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.write_iso_data(packet)
    }

    fn read<'a>(
        &self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<ControllerToHostPacket<'a>, Self::Error>> {
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

    fn try_write_acl_data(
        &self,
        packet: &AclPacket,
    ) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_acl_data(packet)
    }

    fn try_write_sync_data(
        &self,
        packet: &SyncPacket,
    ) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_sync_data(packet)
    }

    fn try_write_iso_data(
        &self,
        packet: &IsoPacket,
    ) -> Result<(), bt_hci::controller::blocking::TryError<Self::Error>> {
        self.0.try_write_iso_data(packet)
    }

    fn read<'a>(&self, buf: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        self.0.read(buf)
    }

    fn try_read<'a>(
        &self,
        buf: &'a mut [u8],
    ) -> Result<ControllerToHostPacket<'a>, bt_hci::controller::blocking::TryError<Self::Error>>
    {
        self.0.try_read(buf)
    }
}

impl<C, Q> ControllerCmdSync<Q> for ControllerRef<'_, C>
where
    C: ControllerCmdSync<Q>,
    Q: SyncCmd + ?Sized,
{
    fn exec(
        &self,
        cmd: &Q,
    ) -> impl Future<Output = Result<Q::Return, bt_hci::cmd::Error<Self::Error>>> {
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


#[cfg(feature = "nrf")]
pub mod nrf {

use core::future::Future;

use nrf_sdc::SoftdeviceController;

use bt_hci::ControllerToHostPacket;
use bt_hci::HostToControllerPacket;
use bt_hci::transport::WithIndicator;
use bt_hci::WriteHci;

use embedded_io::{Error, ErrorType};

// struct MyType(FooType);
// 
// impl BarTrait for MyType {
//     fn bar(&self) {
//         // use `self.0` here
//     }
// }

pub struct SoftdeviceExternalController<'d>(pub SoftdeviceController<'d>);

// impl<'d> SoftdeviceExternalController<'d> {
//     pub fn new() -> Self {
//         //..
//     }
// }

impl<'d> bt_hci::transport::Transport for SoftdeviceExternalController<'d> {
    fn read<'a>(&self, buf: &'a mut [u8]) -> impl Future<Output = Result<ControllerToHostPacket<'a>, Self::Error>> {
        async {
            let kind = self.0.hci_get(buf).await.unwrap();// ?;
            bt_hci::ControllerToHostPacket::from_hci_bytes_with_kind(kind, buf)
                .map(|(x, _)| x)
                .map_err(|err| match err {
                    bt_hci::FromHciBytesError::InvalidSize => SoftdeviceExternalControllerError::Unknown,
                    bt_hci::FromHciBytesError::InvalidValue => SoftdeviceExternalControllerError::Unknown,
                })
        }
    }

    /// Write a complete HCI packet from the tx buffer
    fn write<T: HostToControllerPacket>(&self, val: &T) -> impl Future<Output = Result<(), Self::Error>> {
        async {
            let buf: [u8; 259] = [0; 259];
            let w = WithIndicator::new(val);
            let len = w.size();

            self.0.hci_data_put(&buf[..len])
                .map_err(|err| match err {
                    _Error => SoftdeviceExternalControllerError::Unknown
                })
        }
    }
}

#[derive(Debug)]
pub enum SoftdeviceExternalControllerError {
    Unknown,
}

impl Error for SoftdeviceExternalControllerError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl ErrorType for SoftdeviceExternalController<'_> {
    type Error = SoftdeviceExternalControllerError;
}
}
