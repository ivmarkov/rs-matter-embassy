# (WIP) Run [rs-matter](https://github.com/project-chip/rs-matter) on bare-metal MCUs with [Embassy](https://github.com/embassy-rs/embassy)

[![CI](https://github.com/ivmarkov/rs-matter-embassy/actions/workflows/ci.yml/badge.svg)](https://github.com/ivmarkov/rs-matter-embassy/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rs-matter-embassy.svg)](https://crates.io/crates/rs-matter-embassy)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://ivmarkov.github.io/rs-matter-embassy/esp_idf_matter/index.html)
[![Matrix](https://img.shields.io/matrix/ivmarkov:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)

## Overview

Everything necessary to run [`rs-matter`](https://github.com/project-chip/rs-matter) with Embassy:
* Implementation of `rs-matter`'s `GattPeripheral` for BLE comissioning support - based on [trouble]().
* [`rs-matter-stack`](https://github.com/ivmarkov/rs-matter-stack) traits support:
  * `Netif` - Implementation based on [`embassy-net`]()
  * `Ble` - Based on `trouble`, as mentioned
  * `Wireless` - TBD - custom implementations for each MCU
  * `KvBlobStore` - [`sequential-storage`]()

## Example

(See also [All examples](#all-examples))

TBD

## Future

* Thread networking (for ESP32H2 and ESP32C6)
* Device Attestation data support using secure flash storage
* Setting system time via Matter
* Matter OTA support based on the ESP IDF OTA API

## Build Prerequisites

TBD

## All examples

The examples could be built and flashed conveniently with [`cargo-espflash`](https://github.com/esp-rs/espflash/). To run e.g. `light` on an e.g. ESP32-C3:
(Swap the Rust target and example name with the target corresponding for your ESP32 MCU and with the example you would like to build)

with `cargo-espflash`:
```sh
$ MCU=esp32c3 cargo espflash flash --target riscv32imc-esp-espidf --example light --features examples --monitor
```

| MCU | "--target" |
| --- | ------ |
| esp32c2 | riscv32imc-esp-espidf |
| esp32c3| riscv32imc-esp-espidf |
| esp32c6| riscv32imac-esp-espidf |
| esp32h2 | riscv32imac-esp-espidf |
| esp32p4 | riscv32imafc-esp-espidf |
| esp32 | xtensa-esp32-espidf |
| esp32s2 | xtensa-esp32s2-espidf |
| esp32s3 | xtensa-esp32s3-espidf |
