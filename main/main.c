/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "nvs_flash.h"

#include "dart_ch2o.h"
#include "wifi.h"
#include "http_sender.h"
#include "config.h"
#include "bt_controller.h"

wifi_cnf w_cnf;
sender_cnf s_cnf;

static void on_read_ble(uint16_t idx, uint8_t *buffer, uint16_t offset, uint16_t *len)
{
	char *src = w_cnf.ssid;
	switch (idx)
	{
	case IDX_CHAR_VAL_WIFI_SSID:
		src = w_cnf.ssid;
		break;
	case IDX_CHAR_VAL_WIFI_PWD:
		src = w_cnf.password;
		break;
	case IDX_CHAR_VAL_UPLOAD_URL:
		src = s_cnf.url;
		break;
	case IDX_CHAR_VAL_UPLOAD_INTERVAL:
		src = (char*)buffer;
		sprintf(src, "%d", s_cnf.interval);
		*len = strlen(src);
		return;
	}
	*len = strlen(src) - offset;
	if (*len > 499) {
		*len = 499;
	}
	memcpy(buffer, src + offset, *len);
}

static void on_write_ble(uint16_t idx, uint8_t *buffer, uint16_t len)
{
	char *dest = w_cnf.ssid;
	char *src = (char*)buffer;
	switch (idx)
	{
	case IDX_CHAR_VAL_WIFI_SSID:
		dest = w_cnf.ssid;
		break;
	case IDX_CHAR_VAL_WIFI_PWD:
		dest = w_cnf.password;
		break;
	case IDX_CHAR_VAL_UPLOAD_URL:
		dest = s_cnf.url;
		break;
	case IDX_CHAR_VAL_UPLOAD_INTERVAL:
		src = (char*)buffer;
		s_cnf.interval = atoi(src);
		set_sender_conf(&s_cnf);
		return;
	}
//	uint32_t len = strlen(buffer);
//	if (*len > 499) {
//		*len = 499;
//	}
	memset(dest, 0, strlen(dest));
	memcpy(dest, src, strlen(src));
	if (dest == s_cnf.url) {
		set_sender_conf(&s_cnf);
	} else {
		set_wifi_conf(&w_cnf);
	}
}

void app_main(void)
{

	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
	    ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	//Get config
	get_wifi_conf(&w_cnf);
	get_sender_conf(&s_cnf);

	//Init BLE
	set_ble_handlers(IDX_CHAR_VAL_WIFI_SSID, on_read_ble, on_write_ble);
	set_ble_handlers(IDX_CHAR_VAL_WIFI_PWD, on_read_ble, on_write_ble);
	set_ble_handlers(IDX_CHAR_VAL_UPLOAD_URL, on_read_ble, on_write_ble);
	set_ble_handlers(IDX_CHAR_VAL_UPLOAD_INTERVAL, on_read_ble, on_write_ble);
	init_ble_controller();

	//Init wifi
	init_wifi(w_cnf.ssid, w_cnf.password);
	vTaskDelay(3000 / portTICK_PERIOD_MS);

	init_ch2o();

	while (1) {
		short ch2o = read_ch2o();
		if (ch2o >= -1) {
			printf("CH2O: %.3f mg/m3...\n", ch2o/1000.0*1.23);
			char post_data[12] = {};
			sprintf(post_data, "ch2o=%d", ch2o);
			http_send_data(s_cnf.url, post_data);
		}
		vTaskDelay(s_cnf.interval / portTICK_PERIOD_MS);
	}

}
