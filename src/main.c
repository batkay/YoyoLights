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
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>


#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define CONFIG_SAMPLE_LED_UPDATE_DELAY 50
#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

// #define LED_ENABLE DT_ALIAS(led_enable)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(0x00, 0x00, 0x00), /* red */
	RGB(0x00, 0x01, 0x00), /* green */
	RGB(0x00, 0x00, 0x00), /* blue */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
// static const struct gpio_dt_spec enable = GPIO_DT_SPEC_GET(LED_ENABLE, gpios);
static const struct gpio_dt_spec enable =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(led_enable), gpios, {0});

int main(void)
{
	size_t color = 0;
	int rc;

	if (!gpio_is_ready_dt(&enable)) {
		printf("LED enable is not ready");
		return 0;
	}

	int ret = gpio_pin_configure_dt(&enable, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		printf("LED is not configured");
		return 0;
	}

	if (device_is_ready(strip)) {
		printf("Found LED strip device %s", strip->name);
	} else {
		printf("LED strip device %s is not ready", strip->name);
		return 0;
	}
	// printf(DT_PROP(DT_ALIAS(led_strip), delay-t1h));
	printf("Test");
	printf("Displaying pattern on strip");
	while (1) {
		// for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
		// 	memset(&pixels, 0x00, sizeof(pixels));
		// 	memcpy(&pixels[cursor], &colors[color], sizeof(struct led_rgb));

		// 	rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		// 	if (rc) {
		// 		printf("couldn't update strip: %d", rc);
		// 	}

		// 	k_sleep(DELAY_TIME);
		// }

		// color = (color + 1) % ARRAY_SIZE(colors);

		memset(&pixels, 0x00, sizeof(pixels));
		for (size_t cursor = 0; cursor < ARRAY_SIZE(pixels); cursor++) {
			memcpy(&pixels[cursor], &colors[1], sizeof(struct led_rgb));
		}
		rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		if (rc) {
			printf("couldn't update strip: %d", rc);
		}

		k_sleep(DELAY_TIME);
	}

	return 0;
}