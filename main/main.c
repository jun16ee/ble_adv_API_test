#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bt_sender.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "UART_SIMPLE";

// --- 設定 ---
#define UART_PORT_NUM      UART_NUM_0
#define BUF_SIZE           1024
#define TXD_PIN            UART_PIN_NO_CHANGE
#define RXD_PIN            UART_PIN_NO_CHANGE

// 接收緩衝區
static char packet_buf[128];
static int packet_idx = 0;

void process_byte(uint8_t c) {
    // 收到換行符號，代表指令結束
    if (c == '\n') {
        packet_buf[packet_idx] = '\0'; // 補上字串結尾
    
        // 格式: cmd,delay,hex_mask
        // 例如: 160,1000000,23
        
        int cmd_in = 0;
        unsigned long delay_us = 0;
        unsigned long long target_mask = 0;

        int args = sscanf(packet_buf, "%d,%lu,%llx", &cmd_in, &delay_us, &target_mask);

        if (args == 3) {
            bt_sender_config_t burst_cfg = {
                .cmd_type = (uint8_t)cmd_in,
                .delay_us = delay_us,
                .target_mask = (uint64_t)target_mask
            };
            int sent = bt_sender_execute_burst(&burst_cfg);
            
            // 回傳 ACK
            char resp[64];
            int len = snprintf(resp, sizeof(resp), "ACK:OK,SENT:%d\n", sent);
            uart_write_bytes(UART_PORT_NUM, resp, len);
        } else {
            // 解析失敗 (可能是雜訊，或是格式不對)
            // 這裡回傳 NAK，讓 Python 知道要重傳
            uart_write_bytes(UART_PORT_NUM, "NAK:ParseError\n", 15);
        }
        
        // 重置 Buffer 索引，準備接下一行
        packet_idx = 0;
    } 
    else if (c == '\r') {
        // 忽略 \r (Carriage Return)
    }
    else if (packet_idx < sizeof(packet_buf) - 1) {
        // 正常字元，存入 Buffer
        packet_buf[packet_idx++] = (char)c;
    } 
    else {
        // Buffer 爆了還沒收到 \n，強制重置 (保護機制)
        packet_idx = 0;
        uart_write_bytes(UART_PORT_NUM, "NAK:Overflow\n", 13);
    }
}

void app_main(void)
{
    // 1. 初始化 BT Sender
    if (bt_sender_init() != ESP_OK) return;

    // 2. 配置 Native UART 驅動
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    ESP_LOGI(TAG, "UART Listening...");

    while (1) {
        // 3. 直接從硬體讀取
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                process_byte(data[i]);
            }
        }
        
        // 避免 Watchdog 叫
        if (len == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    free(data);
}