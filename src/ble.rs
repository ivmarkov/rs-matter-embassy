use core::pin::pin;

use embassy_futures::select::select4;
use embassy_sync::blocking_mutex::raw::RawMutex;

use log::{info, warn};

use rs_matter::error::ErrorCode;
use rs_matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C1_MAX_LEN,
    C2_CHARACTERISTIC_UUID, C2_MAX_LEN, MATTER_BLE_SERVICE_UUID16, MAX_BTP_SESSIONS,
};
use rs_matter::transport::network::BtAddr;
use rs_matter::utils::init::{init, Init};
use rs_matter::utils::storage::Vec;
use rs_matter::utils::sync::{IfMutex, Signal};

use trouble_host::gatt::{GattEvent, GattServer};
use trouble_host::prelude::{
    AdStructure, Advertisement, AttributeTable, Characteristic, CharacteristicProp, Runner,
    Service, Uuid, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
};
use trouble_host::{
    Address, BdAddr, BleHostError, Controller, HostResources, PacketQos, Peripheral,
};

const MAX_CONNECTIONS: usize = MAX_BTP_SESSIONS;
const MAX_MTU_SIZE: usize = 512; // TODO const L2CAP_MTU: usize = 251;
const MAX_ATTRIBUTES: usize = 2;
const MAX_CHANNELS: usize = 2;
const ADV_SETS: usize = 1;

type GPHostResources<C> = HostResources<C, MAX_CONNECTIONS, MAX_CHANNELS, MAX_MTU_SIZE, ADV_SETS>;
type GPGattServer<'a, 'v, C, M> = GattServer<'a, 'v, C, M, MAX_ATTRIBUTES, MAX_MTU_SIZE>;
type GPAttributeTable<'a, M> = AttributeTable<'a, M, MAX_ATTRIBUTES>;

// #[derive(Debug, Clone)]
// struct Connection {
//     peer: BdAddr,
//     conn_id: Handle,
//     subscribed: bool,
//     mtu: Option<u16>,
// }

struct State<C>
where
    C: Controller,
{
    resources: GPHostResources<C>,
    c1_data: Vec<u8, C1_MAX_LEN>,
    c2_data: Vec<u8, C2_MAX_LEN>,
}

impl<C> State<C>
where
    C: Controller,
{
    #[inline(always)]
    const fn new() -> Self {
        Self {
            resources: GPHostResources::new(PacketQos::None),
            c1_data: Vec::new(),
            c2_data: Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            // TODO: Cannot efficiently in-place initialize because of the pesky PacketQos enum
            resources: GPHostResources::new(PacketQos::None),
            c1_data <- Vec::init(),
            c2_data <- Vec::init(),
        })
    }
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
    state: IfMutex<M, State<C>>,
    ind: IfMutex<M, IndBuffer>,
    ind_in_flight: Signal<M, bool>,
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
            state: IfMutex::new(State::new()),
            ind: IfMutex::new(IndBuffer::new()),
            ind_in_flight: Signal::new(false),
        }
    }

    /// Return an in-place initializer for the type.
    #[allow(clippy::large_stack_frames)]
    pub fn init() -> impl Init<Self> {
        init!(Self {
            state <- IfMutex::init(State::init()),
            ind <- IfMutex::init(IndBuffer::init()),
            ind_in_flight: Signal::new(false),
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
    C: Controller + 'static,
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
    C: Controller + 'static,
{
    context: &'a TroubleBtpGattContext<M, &'static C>,
    controller: C,
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
    pub fn new(controller: C, context: &'a TroubleBtpGattContext<M, &'static C>) -> Result<Self, ()>
    where
        C: Controller,
    {
        Ok(Self {
            context,
            controller,
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
        let mut state = self.context.state.lock().await;

        let resources = &mut state.resources;
        let resources = unsafe { core::mem::transmute::<_, &mut GPHostResources<&C>>(resources) };

        let address = Address::random([0x41, 0x5A, 0xE3, 0x1E, 0x83, 0xE7]); // TODO
        info!("Our address = {:?}", address);

        let (stack, peripheral, _, runner) = trouble_host::new(&self.controller, resources)
            .set_random_address(address)
            .build();

        // Ideally, this should be in the context, but due to the mutable borrows, that's not possible
        let mut table: GPAttributeTable<'_, M> = GPAttributeTable::new();

        // Generic Access Service (mandatory)
        let mut svc = table.add_service(Service::new(0x1800));
        let _ = svc.add_characteristic_ro(0x2a00, service_name.as_bytes());
        let _ = svc.add_characteristic_ro(0x2a01, &[0x80, 0x07]);
        svc.build();

        // Generic attribute service (mandatory)
        table.add_service(Service::new(0x1801));

        let state = &mut *state;

        // Matter service
        let mut sb = table.add_service(Service::new(MATTER_BLE_SERVICE_UUID16));

        let c1 = sb
            .add_characteristic(
                Uuid::new_long(C1_CHARACTERISTIC_UUID.to_be_bytes()),
                &[CharacteristicProp::Write],
                &mut state.c1_data,
            )
            .build();

        let c2 = sb
            .add_characteristic(
                Uuid::new_long(C2_CHARACTERISTIC_UUID.to_be_bytes()),
                &[CharacteristicProp::Indicate],
                &mut state.c2_data,
            )
            .build();

        let _service = sb.build();

        let server = GPGattServer::<&C, M>::new(stack, &table);

        let mut server_task = pin!(self.run_gatt(&server, &table, c1, c2, &mut callback));
        let mut runner_task = pin!(self.run_ble(runner));
        let mut ind_task = pin!(self.run_ind(&server, c2));
        let mut adv_task = pin!(self.run_adv(peripheral, service_name, service_adv_data));

        select4(
            &mut server_task,
            &mut runner_task,
            &mut ind_task,
            &mut adv_task,
        )
        .await;

        Ok(())
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

    async fn run_ble(&self, mut runner: Runner<'_, &C>) -> Result<(), BleHostError<C::Error>> {
        runner.run().await
    }

    async fn run_gatt<F>(
        &self,
        server: &GPGattServer<'_, '_, &C, M>,
        table: &GPAttributeTable<'_, M>,
        c1: Characteristic,
        c2: Characteristic,
        mut callback: F,
    ) -> BleHostError<C::Error>
    where
        F: FnMut(GattPeripheralEvent) + Send,
    {
        fn to_bt_addr(addr: &BdAddr) -> BtAddr {
            let raw = addr.raw();
            BtAddr([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]])
        }

        loop {
            let event = server.next().await.unwrap();

            if let GattEvent::Write {
                connection,
                handle,
                offset,
                len,
            } = event
            {
                if handle == c1 {
                    table
                        .get(handle, |data| {
                            callback(GattPeripheralEvent::Write {
                                address: to_bt_addr(&connection.peer_address()),
                                data: &data[offset as usize..len as usize],
                                gatt_mtu: Some(connection.att_mtu()),
                            });
                        })
                        .unwrap(); // TODO
                } else if handle == c2 {
                    table
                        .get(handle, |data| {
                            if data[1] == 0 {
                                callback(GattPeripheralEvent::NotifySubscribed(to_bt_addr(
                                    &connection.peer_address(),
                                )));
                            } else {
                                callback(GattPeripheralEvent::NotifyUnsubscribed(to_bt_addr(
                                    &connection.peer_address(),
                                )));
                            }
                        })
                        .unwrap(); // TODO
                }
            }
        }
    }

    async fn run_ind(
        &self,
        server: &GPGattServer<'_, '_, &C, M>,
        c2: Characteristic,
    ) -> BleHostError<C::Error> {
        loop {
            let mut ind = self.context.ind.lock_if(|ind| !ind.data.is_empty()).await;

            let conn = server.get_connection(&BdAddr::new(ind.addr.0));

            let result = if let Some(conn) = conn {
                Some(server.notify(c2, &conn, &ind.data).await)
            } else {
                warn!("Connection to addr {:?} not found", ind.addr.0);

                None
            };

            ind.data.clear();

            if let Some(result) = result {
                result.unwrap(); // TODO
            }
        }
    }

    async fn run_adv(
        &self,
        mut peripheral: Peripheral<'_, &C>,
        service_name: &str,
        service_adv_data: &AdvData,
    ) -> Result<(), BleHostError<C::Error>> {
        let adv_srv_enc_data = service_adv_data
            .service_payload_iter()
            .collect::<Vec<_, 8>>();
        let adv_data = [
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceData16 {
                uuid: MATTER_BLE_SERVICE_UUID16,
                data: &adv_srv_enc_data,
            },
            AdStructure::CompleteLocalName(service_name.as_bytes()),
        ];

        let mut adv_enc_data = [0; 31];

        let len = AdStructure::encode_slice(&adv_data, &mut adv_enc_data)?;

        loop {
            info!("Advertising");
            let mut advertiser = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_enc_data[..len],
                        scan_data: &[],
                    },
                )
                .await?;

            let _conn = advertiser.accept().await?;
            info!("Connection established");
        }
    }
}

impl<'a, M, C> GattPeripheral for TroubleBtpGattPeripheral<'a, M, C>
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
