use embassy_futures::join::join;
use embassy_futures::select::select;

use embassy_sync::blocking_mutex::raw::RawMutex;

use log::{info, warn};

use rs_matter::error::ErrorCode;
use rs_matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C2_CHARACTERISTIC_UUID,
    MATTER_BLE_SERVICE_UUID16, MAX_BTP_SESSIONS,
};
use rs_matter::transport::network::BtAddr;
use rs_matter::utils::cell::RefCell;
use rs_matter::utils::init::{init, Init};
use rs_matter::utils::storage::Vec;
use rs_matter::utils::sync::blocking::Mutex;
use rs_matter::utils::sync::{IfMutex, Signal};

use trouble_host::att::{AttReq, AttRsp};
use trouble_host::prelude::*;
use trouble_host::{
    Address, BdAddr, BleHostError, Controller, Error, HostResources, PacketQos, Peripheral,
};

const MAX_CONNECTIONS: usize = MAX_BTP_SESSIONS;
const MAX_MTU_SIZE: usize = 27; // For now 512; // TODO const L2CAP_MTU: usize = 251;
const MAX_CHANNELS: usize = 2;
const ADV_SETS: usize = 1;

type GPHostResources<C> = HostResources<C, MAX_CONNECTIONS, MAX_CHANNELS, MAX_MTU_SIZE, ADV_SETS>;

type External = [u8; 0];

// GATT Server definition
#[gatt_server]
struct Server {
    matter_service: MatterService,
}

/// Matter service
#[gatt_service(uuid = MATTER_BLE_SERVICE_UUID16)]
struct MatterService {
    #[characteristic(uuid = Uuid::Uuid128(C1_CHARACTERISTIC_UUID.to_be_bytes()), write)]
    c1: External,
    #[characteristic(uuid = Uuid::Uuid128(C2_CHARACTERISTIC_UUID.to_be_bytes()), write, indicate)]
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
pub struct TroubleBtpGattContext<M, C>
where
    M: RawMutex,
    C: Controller,
{
    ind: IfMutex<M, IndBuffer>,
    ind_in_flight: Signal<M, bool>,
    resources: IfMutex<M, GPHostResources<C>>,
}

impl<M, C> TroubleBtpGattContext<M, C>
where
    M: RawMutex,
    C: Controller,
{
    /// Create a new instance.
    #[allow(clippy::large_stack_frames)]
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            ind: IfMutex::new(IndBuffer::new()),
            ind_in_flight: Signal::new(false),
            resources: IfMutex::new(GPHostResources::new(PacketQos::Fair)),
        }
    }

    /// Return an in-place initializer for the type.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            ind <- IfMutex::init(IndBuffer::init()),
            ind_in_flight: Signal::new(false),
            resources <- IfMutex::init(GPHostResources::new(PacketQos::Fair)), // TODO
        })
    }

    // pub(crate) fn reset(&self) -> Result<(), ()> {
    //     self.ind_in_flight.modify(|ind_inf_flight| {
    //         *ind_inf_flight = false;
    //         (false, ())
    //     });

    //     self.ind
    //         .try_lock()
    //         .map(|mut ind| {
    //             ind.data.clear();
    //         })
    //         .unwrap(); // TODO

    //     Ok(())
    // }
}

impl<M, C> Default for TroubleBtpGattContext<M, C>
where
    M: RawMutex,
    C: Controller,
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
    controller: Mutex<M, RefCell<Option<C>>>,
    context: &'a TroubleBtpGattContext<M, C>,
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
    pub fn new(controller: C, context: &'a TroubleBtpGattContext<M, C>) -> Result<Self, ()>
    where
        C: Controller,
    {
        Ok(Self {
            controller: Mutex::new(RefCell::new(Some(controller))),
            context,
        })
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
        let mut resources = self.context.resources.lock().await;

        let controller = self.controller.lock(|c| c.take().unwrap()); // TODO XXX FIXME

        let builder = trouble_host::new(controller, &mut resources);

        let address = Address::random([0x41, 0x5A, 0xE3, 0x1E, 0x83, 0xE7]); // TODO
        info!("Our address = {:?}", address);

        let (_stack, mut peripheral, _central, runner) =
            builder.set_random_address(address).build();

        info!("Starting advertising and GATT service");

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
                        #[cfg(feature = "defmt")]
                        let e = defmt::Debug2Format(&e);
                        panic!("[adv] error: {:?}", e);
                    }
                }
            }
        })
        .await;

        Ok(())
    }

    async fn run_ble(mut runner: Runner<'_, C>) {
        loop {
            if let Err(e) = runner.run().await {
                #[cfg(feature = "defmt")]
                let e = defmt::Debug2Format(&e);
                panic!("[ble_task] error: {:?}", e);
            }
        }
    }

    async fn handle_indications(
        server: &Server<'_>,
        conn: &Connection<'_>,
        ind: &IfMutex<M, IndBuffer>,
    ) -> Result<(), Error> {
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
                    info!("[gatt] disconnected: {:?}", reason);
                    break;
                }
                ConnectionEvent::Gatt { data } => {
                    if let AttReq::Write {
                        handle,
                        data: bytes,
                    } = data.request()
                    {
                        if handle == server.matter_service.c1.handle {
                            callback(GattPeripheralEvent::Write {
                                address: to_bt_addr(&conn.peer_address()),
                                data: bytes,
                                gatt_mtu: Some(conn.att_mtu()),
                            });

                            info!("[gatt] Write {:?}", bytes);

                            data.reply(AttRsp::Write).await.unwrap();

                            continue;
                        }
                    }

                    match data.process(server).await {
                        Ok(Some(GattEvent::Write(event))) => {
                            if Some(event.handle()) == server.matter_service.c2.cccd_handle {
                                let data = event.data();
                                let subscribed = data[0] == 1;

                                if subscribed {
                                    callback(GattPeripheralEvent::NotifySubscribed(to_bt_addr(
                                        &conn.peer_address(),
                                    )));
                                } else {
                                    callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                                        &conn.peer_address(),
                                    )));
                                }

                                info!("[gatt] Write Event to CCC Characteristic: {:?}", data);
                            }
                        }
                        Ok(_) => {}
                        Err(e) => {
                            warn!("[gatt] error processing event: {:?}", e);
                        }
                    }
                }
            }
        }
        info!("[gatt] task finished");
        Ok(())
    }

    /// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
    async fn advertise<'p>(
        service_name: &str,
        service_adv_data: &AdvData,
        peripheral: &mut Peripheral<'p, C>,
    ) -> Result<Connection<'p>, BleHostError<C::Error>> {
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
        info!("[adv] advertising");
        let conn = advertiser.accept().await?;
        info!("[adv] connection established");
        Ok(conn)
    }

    /// Indicate new data on characteristic `C2` to a remote peer.
    pub async fn indicate(
        &self,
        data: &[u8],
        address: BtAddr,
    ) -> Result<(), BleHostError<C::Error>> {
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

        Ok(())
    }
}

impl<M, C> GattPeripheral for TroubleBtpGattPeripheral<'_, M, C>
where
    M: RawMutex,
    C: Controller,
{
    async fn run<F>(
        &self,
        service_name: &str,
        adv_data: &AdvData,
        callback: F,
    ) -> Result<(), rs_matter::error::Error>
    where
        F: FnMut(GattPeripheralEvent) + Send + Clone + 'static,
    {
        TroubleBtpGattPeripheral::run(self, service_name, adv_data, callback)
            .await
            .map_err(|_| ErrorCode::BtpError)?;

        Ok(())
    }

    async fn indicate(&self, data: &[u8], address: BtAddr) -> Result<(), rs_matter::error::Error> {
        TroubleBtpGattPeripheral::indicate(self, data, address)
            .await
            .map_err(|_| ErrorCode::BtpError)?;

        Ok(())
    }
}
