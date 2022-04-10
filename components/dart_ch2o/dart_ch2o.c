#include <stdio.h>
#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"

#include "dart_ch2o.h"

#define ECHO_TEST_TXD GPIO_NUM_17
#define ECHO_TEST_RXD GPIO_NUM_16
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      UART_NUM_2
#define ECHO_UART_BAUD_RATE     (9600)

#define BUF_SIZE (128)

//static const uint8_t CMD_SWITCH_RESP[9] = {(uint8_t)0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
//static const uint8_t CMD_SWITCH_AUTO[9] = {(uint8_t)0xFF, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};
//static const uint8_t CMD_READ[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

static short s_ch2o = -100;

//计算校验和
static uint8_t calc_checksum(uint8_t *data)
{
	uint8_t sum = 0;
	for(int i = 1; i < 8; i ++)
	{
		sum += data[i];
	}
	sum = (~sum) + 1;
	return sum;
}

//计算结果值，单位ppb
static short calc_ch2o(uint8_t *data)
{
	/*
	for (int i = 0; i < 9; i++) {
		printf("%X ", data[i]);
	}
	printf("\n");
	*/
	short ch2o = data[4]*256+data[5];
	//量程为0-2ppm
	if (ch2o > 2000) ch2o = 2000;
	return ch2o;
}

//解码协议
static short decode_ch2o(uint8_t *data, int len)
{
	for (int i = 0; i < len; i++) {
		if (data[i] != 0xFF) {
			continue;
		}
		if (i + 9 > len) {
			continue;
		}
		if (data[i + 1] != 0x17) {
			continue;
		}

		if (data[i + 8] != calc_checksum(data + i)) {
			continue;
		}

		return calc_ch2o(data + i);
	}

	return -1;
}

static void read_ch2o_loop()
{
	int len;
	uint8_t data[BUF_SIZE];
	while (1) {
	    len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
	    s_ch2o = decode_ch2o(data, len);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void init_ch2o()
{
	//串口配置信息
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE*2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    //开启轮询读线程（问答模式没开启成功）
    xTaskCreate(read_ch2o_loop, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    //无效，一直未调通
    //int wlen = uart_write_bytes(ECHO_UART_PORT_NUM, CMD_SWITCH_RESP, sizeof(CMD_SWITCH_RESP));
    //uart_flush(ECHO_UART_PORT_NUM);
    //esp_err_t e1 = uart_wait_tx_done(ECHO_UART_PORT_NUM, 1000 / portTICK_RATE_MS);
    //printf("Wait tx: [%d]\n", e1);
    //xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}

short read_ch2o()
{
	//问答模式，一直没开起来
	//uint8_t data[BUF_SIZE];
	//uart_write_bytes(ECHO_UART_PORT_NUM, CMD_READ, sizeof(CMD_READ));
	//int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 50 / portTICK_RATE_MS);
	return s_ch2o;
}
