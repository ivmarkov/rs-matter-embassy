use bt_hci::controller::ExternalController;

pub type EspWirelessMatterStack<'a, T, E> = MatterStack<'a, EspWirelessBle<T, E>>;

/// A type alias for an ESP-IDF implementation of the `Network` trait for a Matter stack running over
/// BLE during commissioning, and then over either WiFi or Thread when operating.
pub type EmbassyWirelessBle<T, E> = WirelessBle<CriticalSectionRawMutex, T, KvBlobBuf<EmbassyGatt<ExternalController<BleConnector, 20>, E>>>;
