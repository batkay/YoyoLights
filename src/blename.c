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
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset + len > MAX_NAME_LENGTH) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
    
    value[len + 1] = '\0';
	value[MAX_NAME_LENGTH + 1] = '\0';
    initialized = true;

	return len;
}

ssize_t read_name(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 strlen(value));
}