#include "ledble.h"

#include <zephyr/drivers/led_strip.h>


struct led_rgb saved_color;
static bool updated;

int led_service_init(void)
{
    int err = 0;

    saved_color.b = 0x25;
    saved_color.r = 0;
    saved_color.g = 0;
    updated = false;

    return err;
}

void get_led_data(struct led_rgb *buf) {
    memcpy(buf, &saved_color, sizeof(struct led_rgb));
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
    uint8_t *value = attr->user_data;
    
    memcpy(value, &saved_color, len);
    updated = true;
    return len;
}
