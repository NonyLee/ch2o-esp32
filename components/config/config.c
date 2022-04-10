#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"

#include "config.h"

#define TAG "config"

static nvs_handle_t open_config(const char *name)
{
	nvs_handle_t h;
	esp_err_t err = nvs_open(name, NVS_READWRITE, &h);
	if (err != ESP_OK) {
		ESP_LOGI(TAG, "Open WiFi config failed");
		return 0;
	}

	return h;
}

void get_wifi_conf(wifi_cnf *cnf)
{
	size_t len;
	nvs_handle_t h = open_config("WIFI");

	if (h == 0) {
		cnf->ssid[0] = 0;
		cnf->password[0] = 0;
		return;
	}

	len = sizeof(cnf->ssid);
	nvs_get_str(h, "ssid", cnf->ssid, &len);
	len = sizeof(cnf->password);
	nvs_get_str(h, "password", cnf->password, &len);
	nvs_close(h);
}

void set_wifi_conf(wifi_cnf *cnf)
{
	nvs_handle_t h = open_config("WIFI");

	if (h == 0) {
		return;
	}

	nvs_set_str(h, "ssid", cnf->ssid);
	nvs_set_str(h, "password", cnf->password);
	nvs_commit(h);
	nvs_close(h);
}

void get_sender_conf(sender_cnf *cnf)
{
	size_t len;
	nvs_handle_t h = open_config("SENDER");

	if (h == 0) {
		strcpy(cnf->url, "http://127.0.0.1");
		cnf->interval = 600000/portTICK_PERIOD_MS;
		return;
	}

	len = sizeof(cnf->url);
	nvs_get_str(h, "url", cnf->url, &len);
	nvs_get_u32(h, "interval", &cnf->interval);
	nvs_close(h);
}

void set_sender_conf(sender_cnf *cnf)
{
	nvs_handle_t h = open_config("SENDER");

	if (h == 0) {
		return;
	}

	nvs_set_str(h, "url", cnf->url);
	nvs_set_u32(h, "interval", cnf->interval);
	nvs_close(h);
}
