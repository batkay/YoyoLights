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

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/timer/nrf_grtc_timer.h>
#include <zephyr/drivers/sensor.h>

#include "orientation.h"

#define DEEP_SLEEP_TIME_S 2

// Define Devicetree Aliases
#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define CONFIG_SAMPLE_LED_UPDATE_DELAY 50
#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define LED_ENABLE DT_ALIAS(led_enable)
#define BUTTON0 DT_ALIAS(sw0)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec enable = GPIO_DT_SPEC_GET(LED_ENABLE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON0, gpios, {0});

// static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
static uint32_t prevTimeMs;
static double pitch_out, yaw_out, roll_out;
static bool update_values;
static bool orientationInit;

// static const struct gpio_dt_spec enable =
// 	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(led_enable), gpios, {0});

#define MAX_BRIGHTNESS 0x25
// Create RGB Colors
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(MAX_BRIGHTNESS, 0x00, 0x00),
	RGB(0x00, MAX_BRIGHTNESS, 0x00),
	RGB(0x00, 0x00, MAX_BRIGHTNESS)
};
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

int main(void)
{
	int ret;

	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	// setup load switch GPIO
	if (!gpio_is_ready_dt(&enable)) {
		printf("LED enable is not ready");
		return 0;
	}

	ret = gpio_pin_configure_dt(&enable, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		printf("LED is not configured");
		return 0;
	}

	// Setup user push button
	ret = gpio_pin_configure_dt(&button, (GPIO_INPUT));
	if (ret) {
		printk("Error %d: failed to configure button", ret);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_EDGE_TO_ACTIVE));
	if (ret) {
		printf("Could not configure sw0 GPIO interrupt (%d)\n", ret);
		return 0;
	}

	gpio_init_callback(&button_cb_data, buttonPressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	// Setup LED Strip
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

	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	ret = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr);
	if (ret) {
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

	int idx = 0;

	printf("Displaying pattern on strip");
	while (1) {
		switch(currentState) {
			case OFF:
				gpio_pin_set_dt(&enable, GPIO_OUTPUT_INACTIVE);

				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &(struct led_rgb) RGB(0, 0, 0), sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				ret = gpio_pin_interrupt_configure_dt(&button, 	GPIO_INT_LEVEL_ACTIVE);
				if (ret) {
					printf("Could not configure sw0 GPIO interrupt (%d)\n", ret);
					return 0;
				}

				ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);	
				if (ret) {
					printf("Could not suspend console (%lu)\n", RTC_EVENTS_TICK_EVENTS_TICK_Pos);
					// return 0;
				}
				printf("Sleep");
				sys_poweroff();
				break;
			case FIXED:
				gpio_pin_set_dt(&enable, GPIO_OUTPUT_ACTIVE);
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &red, sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				break;
			case BLUETOOTH:
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &bluetoothColor, sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				break;
			case WAVE:
				memset(&pixels, 0x00, sizeof(pixels));
				size_t cursor = 0;
				for (; cursor < (idx % STRIP_NUM_PIXELS); cursor++) {
					memcpy(&pixels[cursor], &colors[idx/STRIP_NUM_PIXELS], sizeof(struct led_rgb));
				}
				for (; cursor < STRIP_NUM_PIXELS; cursor++) {
					memcpy(&pixels[cursor], &colors[((idx)/STRIP_NUM_PIXELS + ARRAY_SIZE(colors) - 1) % ARRAY_SIZE(colors)], sizeof(struct led_rgb));
				}

				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}

				idx = (idx + 1) % (3 * STRIP_NUM_PIXELS);
				break;
			case IMU:
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &(struct led_rgb) RGB(25, 25, 0), sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}

				// printf("accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
				// 							sensor_value_to_double(&accel_x_out),
				// 							sensor_value_to_double(&accel_y_out),
				// 							sensor_value_to_double(&accel_z_out));
				// printf("x:%f,y:%f,z:%f\n",
				// 			   sensor_value_to_double(&gyro_x_out),
				// 			   sensor_value_to_double(&gyro_y_out),
				// 			   sensor_value_to_double(&gyro_z_out));

				// printf("Pitch:%f,Yaw:%f,Roll:%f\n", pitch_out, yaw_out, roll_out);
				printf("Pitch:%f,Yaw:%f,Roll:%f\n", get_delta_pitch(), get_delta_yaw(), get_delta_roll());
				update_values = true;
				break;
		}
		

		k_sleep(DELAY_TIME);
	}

	return 0;
}