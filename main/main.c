#include <stdio.h>
#include <string.h>
#include "bt_sender.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

static const char *TAG = "UART_CTRL";

void app_main(void)
{
    // 1. Initialize BT Sender API
    esp_err_t ret = bt_sender_init();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to initialize BT Sender: %s", esp_err_to_name(ret));
        return;
    }

    char line[128];

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            // Remove newline character
            line[strcspn(line, "\r\n")] = 0;

            if (strlen(line) == 0) continue;

            int cmd_in = 0;
            unsigned long delay_us = 0;
            unsigned long long target_mask = 0;

            // Parse input line
            int args = sscanf(line, "%d,%lu,%llx", &cmd_in, &delay_us, &target_mask);

            if (args == 3) {
                // Configure burst
                bt_sender_config_t burst_cfg = {
                    .cmd_type    = (uint8_t)cmd_in,
                    .delay_us    = delay_us,
                    .target_mask = (uint64_t)target_mask
                };

                // 2. Execute Burst
                int sent = bt_sender_execute_burst(&burst_cfg);
                
                // 3. Report Result
                printf("RESULT:OK,SENT:%d\n", sent);
            } else {
                ESP_LOGE(TAG, "Format Error. Received: %s", line);
                printf("RESULT:ERROR\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}