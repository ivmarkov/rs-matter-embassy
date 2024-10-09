use core::pin::pin;

use embassy_futures::select::select4;
use embassy_sync::blocking_mutex::raw::RawMutex;

use log::{info, warn};

use rs_matter::error::ErrorCode;
use rs_matter::transport::network::btp::{
    AdvData, GattPeripheral, GattPeripheralEvent, C1_CHARACTERISTIC_UUID, C1_MAX_LEN,
    C2_CHARACTERISTIC_UUID, MATTER_BLE_SERVICE_UUID16, MAX_BTP_SESSIONS,
};
use rs_matter::transport::network::BtAddr;
use rs_matter::utils::init::{init, Init};
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
const MAX_ATTRIBUTES: usize = 10;

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
    resources: HostResources<C, MAX_CONNECTIONS, 2, MAX_MTU_SIZE, 1>,
    c1_data: rs_matter::utils::storage::Vec<u8, C1_MAX_LEN>,
}

impl<C> State<C>
where
    C: Controller,
{
    #[inline(always)]
    const fn new() -> Self {
        Self {
            resources: HostResources::new(PacketQos::None),
            c1_data: rs_matter::utils::storage::Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            // TODO: Cannot efficiently in-place initialize because of the pesky PacketQos enum
            resources: HostResources::new(PacketQos::None),
            c1_data <- rs_matter::utils::storage::Vec::init(),
        })
    }
}

#[derive(Debug)]
struct IndBuffer {
    addr: BtAddr,
    data: rs_matter::utils::storage::Vec<u8, MAX_MTU_SIZE>,
}

impl IndBuffer {
    #[inline(always)]
    const fn new() -> Self {
        Self {
            addr: BtAddr([0; 6]),
            data: rs_matter::utils::storage::Vec::new(),
        }
    }

    fn init() -> impl Init<Self> {
        init!(Self {
            addr: BtAddr([0; 6]),
            data <- rs_matter::utils::storage::Vec::init(),
        })
    }
}

/// The `'static` state of the `EspBtpGattPeripheral` struct.
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

    /// Return an in-place initializer for `EspBtpGattContext`.
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
    // app_id: u16,
    // driver: BtDriver<'d, M>,
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
        let resources = unsafe {
            core::mem::transmute::<_, &mut HostResources<&C, MAX_CONNECTIONS, 2, MAX_MTU_SIZE, 1>>(
                resources,
            )
        };

        let address = Address::random([0x41, 0x5A, 0xE3, 0x1E, 0x83, 0xE7]); // TODO
        info!("Our address = {:?}", address);

        let (stack, peripheral, _, runner) = trouble_host::new(&self.controller, resources)
            .set_random_address(address)
            .build();

        let mut table: AttributeTable<'_, M, MAX_ATTRIBUTES> = AttributeTable::new();

        // Generic Access Service (mandatory)
        // TODO: Figure this out
        let id = b"BT";
        let appearance = [0x80, 0x07];

        // Generic attribute service (mandatory)
        table.add_service(Service::new(0x1801));

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
                &[CharacteristicProp::Notify],
                &mut [],
            )
            .build();

        let _service = sb.build();

        let server = GattServer::<&C, M, MAX_ATTRIBUTES, MAX_MTU_SIZE>::new(stack, &table);

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
        server: &GattServer<'_, '_, &C, M, MAX_ATTRIBUTES, MAX_MTU_SIZE>,
        table: &AttributeTable<'_, M, MAX_ATTRIBUTES>,
        c1: Characteristic,
        c2: Characteristic,
        mut callback: F,
    ) -> BleHostError<C::Error>
    where
        F: FnMut(GattPeripheralEvent) + Send,
    {
        loop {
            let event = server.next().await.unwrap();

            if let GattEvent::Write {
                connection,
                handle,
                offset,
                len,
                mtu,
            } = event
            {
                if handle == c1 {
                    table
                        .get(handle, |data| {
                            callback(GattPeripheralEvent::Write {
                                address: to_bt_addr(&connection.peer_address()),
                                data: &data[offset as usize..len as usize],
                                gatt_mtu: Some(mtu),
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
        server: &GattServer<'_, '_, &C, M, MAX_ATTRIBUTES, MAX_MTU_SIZE>,
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
        service_adv_data: &AdvData, // TODO
    ) -> Result<(), BleHostError<C::Error>> {
        let mut adv_data = [0; 31];

        AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x0f, 0x18])]),
                AdStructure::CompleteLocalName(service_name.as_bytes()),
            ],
            &mut adv_data[..],
        )?;

        loop {
            info!("[adv] advertising");
            let mut advertiser = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..],
                        scan_data: &[],
                    },
                )
                .await?;

            let _conn = advertiser.accept().await?;
            info!("[adv] connection established");
        }
    }
}

fn to_bt_addr(addr: &BdAddr) -> BtAddr {
    let raw = addr.raw();
    BtAddr([raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]])
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
