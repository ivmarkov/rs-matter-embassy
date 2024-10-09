# (WIP) Run [rs-matter](https://github.com/project-chip/rs-matter) on bare-metal MCUs with [Embassy](https://github.com/embassy-rs/embassy)

[![CI](https://github.com/ivmarkov/rs-matter-embassy/actions/workflows/ci.yml/badge.svg)](https://github.com/ivmarkov/rs-matter-embassy/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rs-matter-embassy.svg)](https://crates.io/crates/rs-matter-embassy)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://ivmarkov.github.io/rs-matter-embassy/esp_idf_matter/index.html)
[![Matrix](https://img.shields.io/matrix/ivmarkov:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)

## Overview

Everything necessary to run [`rs-matter`](https://github.com/project-chip/rs-matter) with Embassy:
* Implementation of `rs-matter`'s `GattPeripheral` for BLE comissioning support - based on [`trouble`](https://github.com/embassy-rs/trouble).
* [`rs-matter-stack`](https://github.com/ivmarkov/rs-matter-stack) traits support:
  * `Netif` - Implementation based on [`embassy-net`](https://github.com/embassy-rs/embassy/tree/main/embassy-net)
  * `Ble` - Based on [`trouble`](https://github.com/embassy-rs/trouble), as mentioned
  * `Wireless` - TBD - custom implementations for each MCU
  * `KvBlobStore` - [`sequential-storage`](https://github.com/tweedegolf/sequential-storage)

## Example

(See also [All examples](#all-examples))

TBD

## Future

* Thread networking
* Device Attestation data support using secure flash storage
* Setting system time via Matter
* Matter OTA support

## Build Prerequisites

TBD

## All examples

TBD
