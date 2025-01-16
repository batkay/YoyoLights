#pragma once


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/led_strip.h>

#define LED_SERVICE       0x94, 0xF5, 0x56, 0x67, 0x86, 0x49, 0x1D, 0xA4, \
						0x34, 0x41, 0x32, 0x69, 0x01, 0x0D, 0x9F, 0x9D
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
ssize_t read_led(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset);