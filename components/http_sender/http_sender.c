#include "esp_http_client.h"
//#include "esp_log.h"
#include "esp_system.h"

#include "http_sender.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
//static const char *TAG = "HTTP_CLIENT";

void http_send_data(const char *url, const char *data)
{
	esp_http_client_config_t config = {
	        .url = url,
			.method = HTTP_METHOD_POST,
	};
	esp_http_client_handle_t client = esp_http_client_init(&config);
	esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
	esp_http_client_set_post_field(client, data, strlen(data));
	esp_http_client_perform(client);
	/*
	esp_err_t err = esp_http_client_perform(client);
	if (err == ESP_OK) {
	    ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
	                esp_http_client_get_status_code(client),
	                esp_http_client_get_content_length(client));
	} else {
	    ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
	}
	*/
	esp_http_client_cleanup(client);
}
