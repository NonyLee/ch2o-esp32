enum
{
    IDX_SVC,
    IDX_CHAR_WIFI_SSID,
    IDX_CHAR_VAL_WIFI_SSID,

	IDX_CHAR_WIFI_PWD,
    IDX_CHAR_VAL_WIFI_PWD,

	IDX_CHAR_UPLOAD_URL,
	IDX_CHAR_VAL_UPLOAD_URL,

	IDX_CHAR_UPLOAD_INTERVAL,
	IDX_CHAR_VAL_UPLOAD_INTERVAL,

    HRS_IDX_NB,
};

typedef void(*on_ble_read_cb)(uint32_t idx, uint8_t *buffer, uint16_t offset, uint16_t *len);
typedef void(*on_ble_write_cb)(uint32_t idx, uint8_t *buffer, uint16_t len);

void set_ble_handlers(uint16_t idx, on_ble_read_cb read, on_ble_write_cb write);

void init_ble_controller(void);
