# YOYO Firmware
This app controls the firmware for a microcontroller connected to a LED strip, based on the Zephyr RTOS. The code is designed to run on a Seeed Studio Xiao BLE Sense, connected to a custom PCB with an external button and a WS2812 LED strip.
## BUILD
To build, run the normal west build command in the top directory, specifying the target board of a nRF52 Sense.
```
west build -p always -b xiao_ble/nrf52840/sense
```
## FLASH
Flashing also uses the west command, specifying a urf2 bootloader.
```
west flash -r uf2
```