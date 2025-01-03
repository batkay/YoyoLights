#pragma once


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/led_strip.h>

#define LED_SERVICE       0xd4, 0x86, 0x48, 0x24, 0x54, 0xB3, 0x43, 0xA1, \
			            0xBC, 0x20, 0x97, 0x8F, 0xC3, 0x76, 0xC2, 0x75
#define LED_SERVICE_UUID       BT_UUID_DECLARE_128(LED_SERVICE)

int led_service_init(int r, int g, int b);
void get_led_data(struct led_rgb *buf);
bool get_updated();
bool get_modified_led();
ssize_t on_receive_led(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags);
