#include "bleled.h"

#include <zephyr/drivers/led_strip.h>


struct led_rgb saved_color;
static bool updated;
static bool modified;

int led_service_init(int r, int g, int b)
{
    int err = 0;

    saved_color.b = b;
    saved_color.r = r;
    saved_color.g = g;
    updated = true;
    modified = false;

    return err;
}

void get_led_data(struct led_rgb *buf) {
    // memcpy(buf, &saved_color, sizeof(struct led_rgb));
    buf->r = saved_color.r;
    buf->g = saved_color.g;
    buf->b = saved_color.b;
}

bool get_updated() {
    if (updated) {
        updated = false;
        return true;
    }
    return false;
}

bool get_modified_led() {
    return modified;
}

ssize_t on_receive_led(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags)
{
    const uint8_t *value = attr->user_data;
    if (len != sizeof(saved_color)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    // // memcpy(&saved_color, value, sizeof(saved_color));
    // saved_color.r = value[1];
    // saved_color.g = value[2];
    // saved_color.b = value[3];

    // uint8_t *value = attr->user_data;

	memcpy(value + offset, buf, len);
	// value[offset + len] = 0;

    updated = true;
    modified = true;

    return len;
}

ssize_t read_led(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(struct led_rgb));
}