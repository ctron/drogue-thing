# A "Drogue IoT" Thing

An example device, implemented with Drogue IoT in Rust.

NOTE: This is mostly my personal example project and playground.

## Pre-requisites

* STM32F723E-DISCOVERY board
  * ESP-01 (plugged in the Wi-Fi slot)
* STM32F411 Nucleo 64
  * ESP-01 wired up to UART1
  * BME680 sensor connected to I2C1

## Build

For STM32F411:

    env CC=/usr/bin/clang cargo embed --release --features stm32f411

For STM32F723:

    env CC=/usr/bin/clang cargo embed --release --features stm32f723

## Wi-Fi credentials

You can put your Wi-Fi credentials into the following files:

* `src/wifi.ssid.txt`
* `src/wifi.password.txt`

When you enable the feature `custom_wifi`, it will use the information from these files.

## Display

The example allows to plug in a display as well. As there is a huge varietey of models and drivers, it is
necessary to adapt the code to yours. That is why this is an additional feature `display`, which needs to
be activated explicitly.

When enabled, by default, it expects an "ssd1351" display having a size of 128x128, with RGB565 color format.
As display support is implemented using `embedded-graphics`, it should be relatively simple to implement a different
display type.
