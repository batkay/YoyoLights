/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2024 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

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
// static const struct gpio_dt_spec enable =
// 	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(led_enable), gpios, {0});

// Create RGB Colors
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(0x01, 0x00, 0x00), /* red */
	RGB(0x00, 0x01, 0x00), /* green */
	RGB(0x00, 0x00, 0x01), /* blue */
};

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

int main(void)
{
	int ret;
	const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	if (!gpio_is_ready_dt(&enable)) {
		printf("LED enable is not ready");
		return 0;
	}

	ret = gpio_pin_configure_dt(&enable, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		printf("LED is not configured");
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, (GPIO_INPUT));
	if (ret) {
		printk("Error %d: failed to configure button", ret);
		return 0;
	}

	// ret = gpio_pin_interrupt_configure_dt(&button, 	GPIO_INT_LEVEL_ACTIVE);
	// GPIO_INT_EDGE_TO_ACTIVE
	// GPIO_INT_WAKEUP
	ret = gpio_pin_interrupt_configure_dt(&button, ( GPIO_INT_EDGE_TO_ACTIVE));
	if (ret) {
		printf("Could not configure sw0 GPIO interrupt (%d)\n", ret);
		return 0;
	}

	gpio_init_callback(&button_cb_data, buttonPressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (device_is_ready(strip)) {
		printf("Found LED strip device %s", strip->name);
	} else {
		printf("LED strip device %s is not ready", strip->name);
		return 0;
	}


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
					printf("Could not suspend console (%d)\n", RTC_EVENTS_TICK_EVENTS_TICK_Pos);
					// return 0;
				}
				printf("Sleep");
				sys_poweroff();
				break;
			case FIXED:
				gpio_pin_set_dt(&enable, GPIO_OUTPUT_ACTIVE);
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &colors[1], sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				break;
			case BLUETOOTH:
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &colors[2], sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				break;
			case WAVE:
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &colors[0], sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				break;
			case IMU:
				memset(&pixels, 0x00, sizeof(pixels));
				for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
					memcpy(&pixels[cursor], &(struct led_rgb) RGB(1, 1, 0), sizeof(struct led_rgb));
				}
				ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
				if (ret) {
					printf("couldn't update strip: %d", ret);
				}
				break;
		}
		

		k_sleep(DELAY_TIME);
	}

	return 0;
}