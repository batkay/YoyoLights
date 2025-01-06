#pragma once


#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define NAME_SERVICE       0x94, 0xF5, 0x56, 0x67, 0x86, 0x49, 0x1D, 0xA4, \
							0x34, 0x41, 0x32, 0x69, 0x02, 0x0D, 0x9F, 0x9D
#define NAME_SERVICE_UUID       BT_UUID_DECLARE_128(NAME_SERVICE)

#define MAX_NAME_LENGTH 20

void get_name(char* name_buf, int len_buf);
bool get_modified_name();
ssize_t on_receive_name(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags);