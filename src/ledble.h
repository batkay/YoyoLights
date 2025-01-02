#pragma once
#ifndef LEDBLE
#define LEDBLE

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/led_strip.h>

#define LED_SERVICE       0xd4, 0x86, 0x48, 0x24, 0x54, 0xB3, 0x43, 0xA1, \
			            0xBC, 0x20, 0x97, 0x8F, 0xC3, 0x76, 0xC2, 0x75
#define LED_SERVICE_UUID       BT_UUID_DECLARE_128(LED_SERVICE)
// #define LED_SERVICE \
// 	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define RX_CHARACTERISTIC  0xA6, 0xE8, 0xC4, 0x60, 0x7E, 0xAA, 0x41, 0x6B, \
			                    0x95, 0xD4, 0x9D, 0xCC, 0x08, 0x4F, 0xCF, 0x6A
#define RX_CHARACTERISTIC_UUID  BT_UUID_DECLARE_128(RX_CHARACTERISTIC)

// #define RX_CHARACTERISTIC  	BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x5678, 0x56789abcdef0)
// const struct bt_uuid_128 LED_SERVICE_UUID = BT_UUID_INIT_128(
// 	LED_SERVICE);
// const struct bt_uuid_128 RX_CHARACTERISTIC_UUID = BT_UUID_INIT_128(RX_CHARACTERISTIC);

int led_service_init(void);
void get_led_data(struct led_rgb *buf);
bool get_updated();
ssize_t on_receive(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags);

#endif