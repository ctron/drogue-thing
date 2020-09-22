# A "Drogue IoT" Thing

An example device, implemented with Drogue IoT in Rust.

## Pre-requisites

* STM32F723E-DISCOVERY board
* ESP-01 (plugged in the Wi-Fi slot)

## Build

    env CC=/usr/bin/clang cargo embed --release
