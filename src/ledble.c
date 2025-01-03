#include "ledble.h"

#include <zephyr/drivers/led_strip.h>


static volatile struct led_rgb saved_color;
static bool updated;

int led_service_init(int r, int g, int b)
{
    int err = 0;

    saved_color.b = b;
    saved_color.r = r;
    saved_color.g = g;
    updated = true;

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


ssize_t on_receive(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags)
{
    uint8_t *value = buf;
    if (len != sizeof(saved_color)) {
        return len;
    }
    // memcpy(&saved_color, value, sizeof(saved_color));
    saved_color.r = value[1];
    saved_color.g = value[2];
    saved_color.b = value[3];
    updated = true;
    return len;
}

