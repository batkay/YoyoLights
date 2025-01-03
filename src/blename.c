#include "blename.h"

#include <zephyr/drivers/led_strip.h>


static char* name[MAX_NAME_LENGTH + 1];
static bool initialized = false;

void get_name(char* name_buf, int len_buf) {
    if (len_buf < MAX_NAME_LENGTH + 1) {
        return;
    }

    if (!initialized) {
        return;
    }

    memcpy(name_buf, name, MAX_NAME_LENGTH + 1);
}

bool get_modified_name() {
    return initialized;
}


ssize_t on_receive_name(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len,
			  uint16_t offset,
			  uint8_t flags)
{
    if (len > MAX_NAME_LENGTH) {
        return len;
    }

    memcpy(name, buf, len);
    name[len + 1] = '\0';
    name[MAX_NAME_LENGTH] = '\0';

    initialized = true;

    return len;
}

