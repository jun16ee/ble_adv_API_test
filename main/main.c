#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bt_sender.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

// 定義 UART 事件 queue Handle
static QueueHandle_t uart0_queue;

void process_byte(uint8_t c) {
    // 收到換行符號，代表指令結束
    if (c == '\n') {
        packet_buf[packet_idx] = '\0'; // 補上字串結尾
    
        // 格式: cmd,delay,hex_mask
        // 例如: 160,1000000,23
        
        int cmd_in = 0;
        unsigned long delay_us = 0;
        unsigned long prep_led_us = 0;
        unsigned long long target_mask = 0;

        int args = sscanf(packet_buf, "%d,%lu,%lu,%llx", &cmd_in, &delay_us, &prep_led_us, &target_mask);

        if (args == 4) {
            bt_sender_config_t burst_cfg = {
                .cmd_type = (uint8_t)cmd_in,
                .delay_us = delay_us,
                .prep_led_us = prep_led_us,
                .target_mask = (uint64_t)target_mask
            };
            const char* ack_msg = "ACK:OK\n";
            uart_write_bytes(UART_PORT_NUM, ack_msg, strlen(ack_msg));
            bt_sender_execute_burst(&burst_cfg);
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

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);

    for(;;) {
        // 等待 UART 事件 (Block 直到有中斷發生，不會消耗 CPU)
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            
            switch(event.type) {
                // 收到資料事件
                case UART_DATA:
                    // 資料已經在 Ring Buffer 裡了，讀出來處理
                    // timeout 設為 0，因為我們知道資料已經到了
                    uart_read_bytes(UART_PORT_NUM, dtmp, event.size, 0);
                    
                    for (int i = 0; i < event.size; i++) {
                        process_byte(dtmp[i]);
                    }
                    break;

                // 處理 FIFO 溢出 (這通常是因為沒及時讀取)
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "hw fifo overflow");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart0_queue);
                    break;

                // 處理 Ring Buffer 滿
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "ring buffer full");
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart0_queue);
                    break;

                // 其他事件 (Break, Parity Error 等) 可視需求處理
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
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

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0));

    // 4. 建立 UART 事件處理 Task
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);

    ESP_LOGI(TAG, "UART Listening...");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}