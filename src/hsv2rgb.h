#pragma once
#include <zephyr/drivers/led_strip.h>

struct led_rgb hsv2rgb(float H, float S, float V);