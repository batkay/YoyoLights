/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2024 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <math.h>

#include <zephyr/logging/log.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/flash.h>


#include "orientation.h"
#include "hsv2rgb.h"
#include "bleled.h"
#include "blename.h"

// #define DEBUG

// Define Devicetree Aliases
#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif
#define SPI_FLASH_COMPAT nordic_qspi_nor

#define RX_CHARACTERISTIC  0x94, 0xF5, 0x56, 0x67, 0x86, 0x49, 0x1D, 0xA4, \
			                    0x34, 0x41, 0x32, 0x69, 0x00, 0x0D, 0x9F, 0x9D
#define RX_CHARACTERISTIC_UUID  BT_UUID_DECLARE_128(RX_CHARACTERISTIC)

static K_SEM_DEFINE(ble_init_ok, 0, 1);

bool update = false;

#define CONFIG_SAMPLE_LED_UPDATE_DELAY 50
#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define LED_ENABLE DT_ALIAS(led_enable)
#define BUTTON0 DT_ALIAS(sw0)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec enable = GPIO_DT_SPEC_GET(LED_ENABLE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON0, gpios, {0});
static const struct device *flash_dev = DEVICE_DT_GET_ONE(SPI_FLASH_COMPAT);
const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static uint32_t prevTimeMs;

static bool update_values;
static bool orientationInit;
struct bt_conn* currConn;
static char name[MAX_NAME_LENGTH + 1];

#define MAX_BRIGHTNESS 0x25
// Create RGB Colors
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb red = RGB(MAX_BRIGHTNESS, 0x00, 0x00);
static struct led_rgb bluetoothColor = RGB(0x00, 0x00, MAX_BRIGHTNESS);

static struct gpio_callback button_cb_data;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

enum STATE {
	OFF, BLUETOOTH, FIXED, WAVE, IMU
};

volatile enum STATE currentState = BLUETOOTH;
static bool bleOn = false;

static enum STATE nextState(enum STATE currentState) {
	switch(currentState){
		case OFF:
			return FIXED;
			break;
		case BLUETOOTH:
			return FIXED;
			break;
		case FIXED:
			return WAVE;
			break;
		case WAVE:
			return IMU;
			break;
		case IMU:
			return OFF;
			break;
	}
	return FIXED;
}

void buttonPressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	currentState = nextState(currentState);
}


static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
	uint32_t currTimeMs = k_uptime_get_32();
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	// x -> pitch, y -> roll, z -> yaw
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

	double accYval = sensor_value_to_double(&accel_y) * 3.1415 / 180.0;
	double accXval = sensor_value_to_double(&accel_x) * 3.1415 / 180.0;
	double accZval = sensor_value_to_double(&accel_z) * 3.1415 / 180.0;

	double pitch = atan2(accYval, accZval);
	double roll = atan2(accXval, accZval);

	if (!orientationInit) {
		initialize(pitch, roll, 0.0);
		orientationInit = true;
	}
	else {
		update_pitch(pitch, sensor_value_to_double(&gyro_x), (currTimeMs - prevTimeMs)/1000.0);
		update_roll(roll, sensor_value_to_double(&gyro_y), (currTimeMs - prevTimeMs)/1000.0);
		update_yaw(sensor_value_to_double(&gyro_z), (currTimeMs - prevTimeMs)/1000.0);
	}

	prevTimeMs = currTimeMs;

	if (update_values) {
		update_values = false;
	}
}

// BLE characteristics
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_LEN (sizeof(DEVICE_NAME)-1)

static uint8_t mfg_data[] = { 'B', 'a', 't', 'k', 'a', 'y' };

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, RX_CHARACTERISTIC),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 6),
};

static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	#ifdef DEBUG
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");
	}
	#endif
	if (!bleOn) {
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	currConn = conn;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	#ifdef DEBUG
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
	#endif

	if (get_modified_name()) {
		get_name(name, MAX_NAME_LENGTH + 1);
	}
	sd ->data = name;
	sd ->data_len = strlen(name);

	// bt_le_adv_stop();

	bt_conn_unref(conn);
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	bt_conn_ref(conn);
	currConn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


// BT_GATT_SERVICE_DEFINE(led_service,
// 	BT_GATT_PRIMARY_SERVICE(RX_CHARACTERISTIC_UUID),
// 	BT_GATT_CHARACTERISTIC(LED_SERVICE_UUID,
// 			    	BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
// 			    	BT_GATT_PERM_WRITE,
// 			    	NULL, on_receive_led, NULL),
// 	BT_GATT_CHARACTERISTIC(NAME_SERVICE_UUID,
// 					BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
// 			    	BT_GATT_PERM_WRITE,
// 			    	NULL, on_receive_name, NULL),
// );

BT_GATT_SERVICE_DEFINE(led_service,
	BT_GATT_PRIMARY_SERVICE(RX_CHARACTERISTIC_UUID),
	BT_GATT_CHARACTERISTIC(LED_SERVICE_UUID,
				BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
				BT_GATT_PERM_READ | BT_GATT_PERM_WRITE | BT_GATT_PERM_PREPARE_WRITE,
				read_led, on_receive_led, &bluetoothColor),
	BT_GATT_CHARACTERISTIC(NAME_SERVICE_UUID, 
					BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
					BT_GATT_PERM_READ | BT_GATT_PERM_WRITE |
			       	BT_GATT_PERM_PREPARE_WRITE,
			       	read_name, on_receive_name, &name),
	// BT_GATT_CHARACTERISTIC(NAME_SERVICE_UUID,
	// 				BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
	// 		    	BT_GATT_PERM_WRITE,
	// 		    	NULL, on_receive_name, NULL),
);

static void bt_ready()
{
	int err = 0;

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) 
	{
		#ifdef DEBUG
		printk("Advertising failed to start (err %d)\n", err);
		#endif
		return;
	}
	#ifdef DEBUG
	printk("Advertising successfully started\n");
	#endif

	//Initalize services
	k_sem_give(&ble_init_ok);

}

static void disconnect_device(struct bt_conn *conn, void *user_data)
{
    int err;
    struct bt_conn_info info;

	/* Check if the connection is still active */
    /* Retrieve connection information */
    err = bt_conn_get_info(conn, &info);
    if (err) {
		#ifdef DEBUG
        printk("Failed to get connection info: %d\n", err);
		#endif
        return;
    }

    /* Check if the connection is in the connected state */
    if (info.state != BT_CONN_STATE_CONNECTED || info.state != BT_CONN_STATE_CONNECTING) {
		#ifdef DEBUG
        printk("Device already disconnected or in invalid state\n");
		#endif
        return;
    }

    /* Disconnect the device */
    err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	#ifdef DEBUG
    if (err) {
        printk("Failed to disconnect device: %d\n", err);
    } else {
        printk("Device disconnected successfully\n");
    }
	#endif
}

int main(void)
{
	int err = 0;
	// flash
	if (!device_is_ready(flash_dev)) {
		#ifdef DEBUG
		printk("%s: flash not ready.\n", flash_dev->name);
		#endif
		return 0;
	}

	uint8_t prevR = 0;
	uint8_t prevG = 0;
	uint8_t prevB = MAX_BRIGHTNESS;

	memcpy(name, &"Yo-Yo", sizeof("Yo-Yo"));

	uint8_t flash_results[4 + MAX_NAME_LENGTH + 1];

	err = flash_read(flash_dev, 0, flash_results, 4 + MAX_NAME_LENGTH);
	if (err) {
		#ifdef DEBUG
		printf("Flash read failed! %d\n", err);
		#endif
		return 0;
	}

	if (flash_results[0]) {
		// flash not empty
		prevR = flash_results[1];
		prevG = flash_results[2];
		prevB = flash_results[3];

		if (strlen(&flash_results[4]) > 0) {
			flash_results[MAX_NAME_LENGTH + 4] ='\0';
			memcpy(name, &flash_results[4], MAX_NAME_LENGTH + 1);
			name[MAX_NAME_LENGTH] = '\0';

			#ifdef DEBUG
			printk("New name ");
			#endif
		}
		
	}
	else {
		#ifdef DEBUG
		printk("Flash empty");
		#endif
	}
	#ifdef DEBUG
	printk("%s", name);
	#endif

	sd ->data = name;
	sd ->data_len = strlen(name);

	err = bt_enable(bt_ready);
	if (err) {
		#ifdef DEBUG
		printk("Bluetooth init failed (err %d)\n", err);
		#endif
		return 0;
	}

	err = k_sem_take(&ble_init_ok, K_MSEC(500));

	if (err) {
		#ifdef DEBUG
		printk("BLE init did not complete in time");
		#endif
		return 0;
	}

	err = led_service_init(prevR, prevG, prevB);
	bluetoothColor.r = prevR;
	bluetoothColor.g = prevG;
	bluetoothColor.b = prevB;

	if (err) {
		#ifdef DEBUG
		printk("Failed to init lightbox (err:%d)\n", err);
		#endif
		return 0;
	}

	// setup load switch GPIO
	if (!gpio_is_ready_dt(&enable)) {
		#ifdef DEBUG
		printf("LED enable is not ready");
		#endif
		return 0;
	}

	err = gpio_pin_configure_dt(&enable, GPIO_OUTPUT_ACTIVE);
	if (err) {
		#ifdef DEBUG
		printf("LED is not configured");
		#endif
		return 0;
	}

	// Setup user push button
	err = gpio_pin_configure_dt(&button, (GPIO_INPUT));
	if (err) {
		#ifdef DEBUG
		printk("Error %d: failed to configure button", err);
		#endif
		return 0;
	}

	err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_EDGE_TO_ACTIVE));
	if (err) {
		#ifdef DEBUG
		printf("Could not configure sw0 GPIO interrupt (%d)\n", err);
		#endif
		return 0;
	}

	gpio_init_callback(&button_cb_data, buttonPressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	#ifdef DEBUG
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);
	#endif

	// // Setup LED Strip
	if (device_is_ready(strip)) {
		#ifdef DEBUG
		printf("Found LED strip device %s", strip->name);
		#endif
	} 
	else {
		#ifdef DEBUG
		printf("LED strip device %s is not ready", strip->name);
		#endif
		return 0;
	}


	// Setup IMU
	if (!device_is_ready(lsm6dsl_dev)) {
		#ifdef DEBUG
		printk("sensor: device not ready.\n");
		#endif
		return 0;
	}

	orientationInit = false;

	struct sensor_value odr_attr;
	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

	err = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (err) {
		#ifdef DEBUG
		printk("Cannot set sampling frequency for accelerometer.\n");
		#endif
		return 0;
	}

	err = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (err) {
		#ifdef DEBUG
		printk("Cannot set sampling frequency for gyro.\n");
		#endif
		return 0;
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	err = sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler);
	if (err) {
		#ifdef DEBUG
		printk("Could not set sensor type and channel\n");
		#endif
		return 0;
	}
	err = sensor_sample_fetch(lsm6dsl_dev);
	if (err) {
		#ifdef DEBUG
		printk("Sensor sample update error\n");
		#endif
		return 0;
	}

	// Makes BLE more consistent for some reason?
	k_sleep(DELAY_TIME);
	gpio_pin_set_dt(&enable, GPIO_OUTPUT_ACTIVE);
	
	int idx = 0;

	bool btOn1 = true;
	bool btOn2 = true;
	bleOn = true;
	bool sensorInit = false;
	#ifdef DEBUG
	printf("Displaying pattern on strip");
	#endif
	while (1) {		
		switch(currentState) {
			case OFF:
				gpio_pin_set_dt(&enable, GPIO_OUTPUT_INACTIVE);

				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
					memcpy(&pixels[cursor], &(struct led_rgb) RGB(0, 0, 0), sizeof(struct led_rgb));
				}
				err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (err) {
					#ifdef DEBUG
					printf("couldn't update strip: %d", err);
					#endif
				}
				err = gpio_pin_interrupt_configure_dt(&button, 	GPIO_INT_LEVEL_ACTIVE);
				if (err) {
					#ifdef DEBUG
					printf("Could not configure sw0 GPIO interrupt (%d)\n", err);
					#endif
					return 0;
				}

				err = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);	
				if (err) {
					#ifdef DEBUG
					printf("Could not suspend console (%lu)\n", RTC_EVENTS_TICK_EVENTS_TICK_Pos);
					#endif
				}
				sys_poweroff();
				break;
			case BLUETOOTH:
				if (get_updated()) {

					// get_led_data(&bluetoothColor);

					if (bluetoothColor.r > MAX_BRIGHTNESS) {
						bluetoothColor.r = MAX_BRIGHTNESS;
					}
					if (bluetoothColor.g > MAX_BRIGHTNESS) {
						bluetoothColor.g = MAX_BRIGHTNESS;
					}
					if (bluetoothColor.b > MAX_BRIGHTNESS) {
						bluetoothColor.b = MAX_BRIGHTNESS;
					}

					flash_results[0] = 1;
					flash_results[1] = bluetoothColor.r;
					flash_results[2] = bluetoothColor.g;
					flash_results[3] = bluetoothColor.b;
					#ifdef DEBUG
					printk("r: %i g: %i b: %i", bluetoothColor.r, bluetoothColor.g, bluetoothColor.b);
					#endif
					memset(&pixels, 0x00, sizeof(pixels));
					for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
						memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
					}
					update = true;

				}

				break;
			case FIXED:
				// Do nothing
				if (btOn1) {
					if (currConn) {
						err = bt_conn_disconnect(currConn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
						if (err) {
							printk("Failed to disconnect device: %d\n", err);
						} else {
							printk("Device disconnected successfully\n");
						}
					}
					bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_device, NULL);
					bt_le_adv_stop();

					bleOn = false;
					btOn1 = false;

					memset(&pixels, 0x00, sizeof(pixels));
					for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
						memcpy(&pixels[cursor], &red, sizeof(struct led_rgb));
					}
					update = true;
				}
				break;
			case WAVE:
				if (btOn2) {
					bt_disable();
					btOn2 = false;
				}

				if (flash_results[0] && (get_modified_led() || get_modified_name())) {
					err = flash_erase(flash_dev, 0, 4096);
					if (err) {
						#ifdef DEBUG
						printk("failed to erase");
						#endif
					}
					flash_results[0] = 1;
					flash_results[1] = bluetoothColor.r;
					flash_results[2] = bluetoothColor.g;
					flash_results[3] = bluetoothColor.b;
					// if (get_modified_name()) {
					// 	get_name(&flash_results[4], MAX_NAME_LENGTH + 1);
					// }
					memcpy(&flash_results[4], name, MAX_NAME_LENGTH + 1);

					err = flash_write(flash_dev, 0, flash_results, 4 + MAX_NAME_LENGTH);
					if (err) {
						#ifdef DEBUG
						printf("Flash write failed! %d\n", err);
						#endif
					}
					printf("Flash written");
					flash_results[0] = 0;
				}
				
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
					bluetoothColor = hsv2rgb((cursor + idx) % 360, 100, (MAX_BRIGHTNESS)/255.0 * 100);
					memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
				}
				update = true;
				// idx = (idx + 1) % (3 * STRIP_NUM_PIXELS);
				idx = (idx + 8) % 360;
				break;
			case IMU:
				if (!sensorInit) {
					sensorInit = true;
				}
				double rotation_magnitude = sqrt(pow(get_delta_pitch(), 2) + pow(get_delta_roll(), 2));
				
				if (rotation_magnitude > 1) {
					idx += (int) rotation_magnitude;
					idx = idx % (360);
				}

				bluetoothColor = hsv2rgb(idx, 100, (MAX_BRIGHTNESS)/255.0 * 100);

				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
					memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
				}
				update = true;

				#ifdef DEBUG
				printf("Pitch:%f,Yaw:%f,Roll:%f\n", get_delta_pitch(), get_delta_yaw(), get_delta_roll());
				#endif
				update_values = true;
				break;
		}
		
		if (update) {
			err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			if (err) {
				#ifdef DEBUG
				printf("couldn't update strip: %d", err);
				#endif
			}
			update = false;
		}

		k_sleep(DELAY_TIME);
	}

	return 0;
}