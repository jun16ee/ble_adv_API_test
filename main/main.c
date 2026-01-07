#include "bt_sender.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TEST";

#define CMD                 0xA0
#define BURST_COUNT         50
#define TARGET_DELAY_SEC    3
#define TARGET_DELAY_US     (TARGET_DELAY_SEC * 1000000)
#define PLAYER_BIT(id)      (1ULL << (id))

void app_main(void)
{
    // 1. Initialize BT Sender API
    esp_err_t ret = bt_sender_init();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to initialize BT Sender: %s", esp_err_to_name(ret));
        return;
    }

    while (1) {
        // 2. Configure Burst
        bt_sender_config_t burst_cfg = {
            .cmd_type    = CMD,
            .burst_count = BURST_COUNT,
            .delay_us    = TARGET_DELAY_US,
            .target_mask = PLAYER_BIT(0) | PLAYER_BIT(1) 
        };

        ESP_LOGI(TAG, "Sending CMD: 0x%02X | Count: %d | Delay: %d sec | Targets: 0x%llX", 
                 burst_cfg.cmd_type, burst_cfg.burst_count, TARGET_DELAY_SEC, burst_cfg.target_mask);

        // 3. Execute Burst
        int sent_count = bt_sender_execute_burst(&burst_cfg);

        ESP_LOGI(TAG, "Burst finished. Sent %d packets.", sent_count);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}