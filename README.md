# YOYO Firmware
This app controls the firmware for a microcontroller connected to a LED strip, based on the Zephyr RTOS (version 4.0.99). The code is designed to run on a Seeed Studio Xiao nRF52840 BLE Sense, connected to a custom PCB with an external button and a WS2812 LED strip.
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
## CODE Description
Most operations are handled in main. Main initializes all the peripherals, GPIOs, Bluetooth, etc, then runs a while loop. The loop constantly checks the state, then updates the LEDs accordingly. In main, first flash memory is read to check if there is a previously assigned color for bluetooth that it can grab, and checks to see if a name for bluetooth advertising was assigned. If not, a default blue value and Yoyo advertising name is used. In the while loop, the static colors assign once then wait, in an attempt to free up processing time for bluetooth. Bluetooth is turned off before the more advanced patterns are shown. 

The ble starting files describe individual ble characteristics. Name has a writtable characteristic to set a new name, while led has a characteristic to write a new color. In each, a recieve method is described that just copies the written data to another buffer. Besides that, there are some getter methods to return the buffer, and to determine whether the data has changed recently.

hsv2rgb has copied and pasted code that switches HSV to RGB. HSV allows for the colors to be cycled through in a more natural way compared to the RGB color space.

Orientation handles estimating orientation. The 6 DOF IMU has a sampling frequency of 104Hz, and will update the angles in orientation. A complementary filter is used, adding a weighted average of both the previous angle, current angle based on the accelerometer, and an estimate from the gyroscope. The Z (yaw) axis has no accelerometer estimate, so is much more inaccurate than the other 2. When the angles are updated, a new heading is estimated, and a change in angle estimated to create an angular acceleration. This is used to detect motion, for the IMU based color change.
## Issues
A lot of issues and hacky solutions arose. The first was exiting sleep. The button registers an interrupt on rising edges when switching between states during normal operation. However, this interrupt would not wake the nRF52 from sleep. Another interrupt for the button being active needed to be added before the microcontroller goes into system off to wake up the MCU from sleep.

The second was after BLE was added. Initially BLE would not advertise, and the issue seemed to be twofold. It seemed refreshing the LED strip (specficially calling led_strip_update_rgb) would cause BLE to not advertise. Sometimes this would cause the LEDs to also fail to change, button interrupts from triggering, and the console from printing. In general, everything would break. The work around was to make the BLE state have minimal strip updates, only updating when an update was recieved. Explicitly enabling assert, multithreading, and SMP (symmetric multiprocessing), and adding a thread sleep after the initialization phase but before the while loop seemed to also help this reliability.

These changes made BLE advertising more consistent, but with too many LED updates while BLE was enabled still caused crashes. The solution to this was explicitly disabling BLE after the BLE state was passed. However, disabling BLE without disconnecting the connected device also caused crash. The device then first needs to physically be disconnected before BLE could be disabled. If BLE disable is called right after the BLE disconnect, the MCU doesn't have enough time to disconnect, causing another crash. Therefore, the disconnecting and turning off BLE was handled in 2 phases. The first half, disconnecting, takes place in the FIXED state, while turning off happens in the WAVE state.

Overall, a lot of messy fixes were needed, and it might be an issue with Zephyr, since similar workarounds were not needed in more simple envoirnments, such as with ESP32 on Arduino (which should be based on it's FreeRTOS base).