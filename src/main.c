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
#include "ledble.h"
#include "blename.h"

// Define Devicetree Aliases
#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif
#define SPI_FLASH_COMPAT nordic_qspi_nor

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 18

#define RX_CHARACTERISTIC  0xA6, 0xE8, 0xC4, 0x60, 0x7E, 0xAA, 0x41, 0x6B, \
			                    0x95, 0xD4, 0x9D, 0xCC, 0x08, 0x4F, 0xCF, 0x6A
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

// static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
// static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
static uint32_t prevTimeMs;
// static double pitch_out, yaw_out, roll_out;
static bool update_values;
static bool orientationInit;

static char name[MAX_NAME_LENGTH + 1];

#define MAX_BRIGHTNESS 0x25
// Create RGB Colors
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb red = RGB(MAX_BRIGHTNESS, 0x00, 0x00);
static struct led_rgb bluetoothColor = RGB(0x00, 0x00, MAX_BRIGHTNESS);

static struct gpio_callback button_cb_data;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

enum STATE {
	OFF, FIXED, BLUETOOTH, WAVE, IMU
};

volatile enum STATE currentState = FIXED;


static enum STATE nextState(enum STATE currentState) {
	switch(currentState){
		case OFF:
			return FIXED;
			break;
		case FIXED:
			return BLUETOOTH;
			break;
		case BLUETOOTH:
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
	// printk("Press state %i\n", currentState);
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
		// accel_x_out = accel_x;
		// accel_y_out = accel_y;
		// accel_z_out = accel_z;

		// gyro_x_out = gyro_x;
		// gyro_y_out = gyro_y;
		// gyro_z_out = gyro_z;

		// pitch_out = get_pitch();
		// yaw_out = get_yaw();
		// roll_out = get_roll();

		update_values = false;
	}
}

// BLE characteristics
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_LEN (sizeof(DEVICE_NAME)-1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, LED_SERVICE),
};

static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));

	if (get_modified_name()) {
		get_name(name, MAX_NAME_LENGTH + 1);
	}
	sd ->data = name;
	sd ->data_len = strlen(name);

	// bt_le_adv_stop();

	bt_conn_unref(conn);
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	bt_conn_ref(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


BT_GATT_SERVICE_DEFINE(led_service,
	BT_GATT_PRIMARY_SERVICE(RX_CHARACTERISTIC_UUID),
	BT_GATT_CHARACTERISTIC(LED_SERVICE_UUID,
			    	BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			    	BT_GATT_PERM_WRITE,
			    	NULL, on_receive_led, NULL),
	BT_GATT_CHARACTERISTIC(NAME_SERVICE_UUID,
					BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			    	BT_GATT_PERM_WRITE,
			    	NULL, on_receive_name, NULL),
);

static void bt_ready()
{
	int err = 0;

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) 
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
	//Initalize services
	k_sem_give(&ble_init_ok);

}

int main(void)
{
	int err = 0;
	// flash
	if (!device_is_ready(flash_dev)) {
		printk("%s: flash not ready.\n", flash_dev->name);
		return 0;
	}

	uint8_t prevR = 0;
	uint8_t prevG = 0;
	uint8_t prevB = MAX_BRIGHTNESS;

	memcpy(name, &"Yo-Yo", sizeof("Yo-Yo"));

	uint8_t flash_results[4 + MAX_NAME_LENGTH + 1];

	err = flash_read(flash_dev, 0, flash_results, 4 + MAX_NAME_LENGTH);
	if (err) {
		printf("Flash read failed! %d\n", err);
		return 0;
	}
	// printk("flash: ");
	// for (int i = 0; i < 5; ++i) {
	// 	printk("%i ", flash_results[i]);
	// }

	if (flash_results[0]) {
		// flash not empty
		prevR = flash_results[1];
		prevG = flash_results[2];
		prevB = flash_results[3];

		if (strlen(&flash_results[4]) > 0) {
			flash_results[MAX_NAME_LENGTH + 4] ='\0';
			memcpy(name, &flash_results[4], MAX_NAME_LENGTH + 1);
			name[MAX_NAME_LENGTH] = '\0';
			printk("New name ");
		}
		
	}
	else {
		printk("Flash empty");
	}

	printk("%s", name);

	sd ->data = name;
	sd ->data_len = strlen(name);

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	err = k_sem_take(&ble_init_ok, K_MSEC(500));

	if (err) {
		printk("BLE init did not complete in time");
		return 0;
	}

	err = led_service_init(prevR, prevG, prevB);

	if (err) 
	{
		printk("Failed to init lightbox (err:%d)\n", err);
		return 0;
	}

	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	// setup load switch GPIO
	if (!gpio_is_ready_dt(&enable)) {
		printf("LED enable is not ready");
		return 0;
	}

	err = gpio_pin_configure_dt(&enable, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printf("LED is not configured");
		return 0;
	}

	// Setup user push button
	err = gpio_pin_configure_dt(&button, (GPIO_INPUT));
	if (err) {
		printk("Error %d: failed to configure button", err);
		return 0;
	}

	err = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_EDGE_TO_ACTIVE));
	if (err) {
		printf("Could not configure sw0 GPIO interrupt (%d)\n", err);
		return 0;
	}

	gpio_init_callback(&button_cb_data, buttonPressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	// // Setup LED Strip
	if (device_is_ready(strip)) {
		printf("Found LED strip device %s", strip->name);
	} else {
		printf("LED strip device %s is not ready", strip->name);
		return 0;
	}


	// Setup IMU
	if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
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
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	err = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (err) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}

	if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
		printk("Sensor sample update error\n");
		return 0;
	}


	k_sleep(DELAY_TIME);
	gpio_pin_set_dt(&enable, GPIO_OUTPUT_ACTIVE);
	
	memset(&pixels, 0x00, sizeof(pixels));
	for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
		memcpy(&pixels[cursor], &red, sizeof(struct led_rgb));
	}
	update = true;
	int idx = 0;

	bool btOn = true;
	printf("Displaying pattern on strip");
	while (1) {		
		switch(currentState) {
			case OFF:
				gpio_pin_set_dt(&enable, GPIO_OUTPUT_INACTIVE);

				

				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &(struct led_rgb) RGB(0, 0, 0), sizeof(struct led_rgb));
				}
				err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (err) {
					printf("couldn't update strip: %d", err);
				}
				err = gpio_pin_interrupt_configure_dt(&button, 	GPIO_INT_LEVEL_ACTIVE);
				if (err) {
					printf("Could not configure sw0 GPIO interrupt (%d)\n", err);
					return 0;
				}

				err = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);	
				if (err) {
					printf("Could not suspend console (%lu)\n", RTC_EVENTS_TICK_EVENTS_TICK_Pos);
					// return 0;
				}
				// printf("Sleep");
				sys_poweroff();
				break;
			case FIXED:
				// gpio_pin_set_dt(&enable, GPIO_OUTPUT_ACTIVE);

				break;
			case BLUETOOTH:
				if (get_updated()) {
					// printf("New Color");

					get_led_data(&bluetoothColor);

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

					printk("r: %i g: %i b: %i", bluetoothColor.r, bluetoothColor.g, bluetoothColor.b);
					memset(&pixels, 0x00, sizeof(pixels));
					for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
						memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
					}
					// err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
					// if (err) {
					// 	printf("couldn't update strip: %d", err);
					// }
					update = true;

				}

				break;
			case WAVE:
				if (btOn) {
					bt_le_adv_stop();
					bt_disable();

					btOn = false;
				}
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < STRIP_NUM_PIXELS; cursor++) {
					bluetoothColor = hsv2rgb((cursor + idx) % 360, 100, (MAX_BRIGHTNESS)/255.0 * 100);
					memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
				}

				update = true;
				// printf("%i", flash_results[0]);
				if (flash_results[0] && (get_modified_led() || get_modified_name())) {
					err = flash_erase(flash_dev, 0, 4096);
					if (err) {
						printk("failed to erase");
					}

					if (get_modified_name()) {
						get_name(&flash_results[4], MAX_NAME_LENGTH + 1);
					}

					err = flash_write(flash_dev, 0, flash_results, 4 + MAX_NAME_LENGTH);
					if (err) {
						printf("Flash write failed! %d\n", err);
					}
					flash_results[0] = 0;
				}

				// idx = (idx + 1) % (3 * STRIP_NUM_PIXELS);
				idx = (idx + 8) % 360;
				break;
			case IMU:
				double rotation_magnitude = sqrt(pow(get_delta_pitch(), 2) + pow(get_delta_roll(), 2));
				
				if (rotation_magnitude > 1) {
					idx += (int) rotation_magnitude;
					idx = idx % (360);
				}

				bluetoothColor = hsv2rgb(idx, 100, (MAX_BRIGHTNESS)/255.0 * 100);

				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
				}
				update = true;

				// err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				// if (err) {
				// 	printf("couldn't update strip: %d", err);
				// }

				// printf("Pitch:%f,Yaw:%f,Roll:%f\n", pitch_out, yaw_out, roll_out);
				printf("Pitch:%f,Yaw:%f,Roll:%f\n", get_delta_pitch(), get_delta_yaw(), get_delta_roll());
				update_values = true;
				break;
		}
		
		if (update) {
			err = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			if (err) {
				printf("couldn't update strip: %d", err);
			}
			update = false;
		}

		k_sleep(DELAY_TIME);
	}

	return 0;
}

// K_THREAD_DEFINE(blink_id, STACKSIZE, blink, NULL, NULL, NULL,
// 		PRIORITY, 0, 0);