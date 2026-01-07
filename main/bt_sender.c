// bt_sender.c
#include "bt_sender.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt_hci_common.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define TX_OFFSET_US 9000 // Estimated time offset for TX in microseconds based on empirical measurements

static const char *TAG = "BT_SENDER";
static volatile int64_t last_measured_latency = 0;
static int64_t t1_start = 0;
static uint8_t hci_cmd_buf[128];
static bool is_initialized = false;

// Helper functions to send HCI commands
static void hci_cmd_send_ble_set_adv_data(uint8_t cmd_type, uint32_t delay_us, uint64_t target_mask) {
    uint8_t raw_adv_data[31];
    uint8_t idx = 0;
    
    // Flags
    raw_adv_data[idx++] = 2; raw_adv_data[idx++] = 0x01; raw_adv_data[idx++] = 0x06;
    
    // Manufacturer Specific Data
    raw_adv_data[idx++] = 16; // len
    raw_adv_data[idx++] = 0xFF; raw_adv_data[idx++] = 0xFF; raw_adv_data[idx++] = 0xFF;
    raw_adv_data[idx++] = cmd_type;

    // Bitmask for player ID (8 bytes)
    raw_adv_data[idx++] = (uint8_t)(target_mask & 0xFF);         // IDs 0-7
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 8) & 0xFF);  // IDs 8-15
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 16) & 0xFF); // IDs 16-23
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 24) & 0xFF); // IDs 24-31
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 32) & 0xFF); // IDs 32-39
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 40) & 0xFF); // IDs 40-47
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 48) & 0xFF); // IDs 48-55
    raw_adv_data[idx++] = (uint8_t)((target_mask >> 56) & 0xFF); // IDs 56-63

    // Delay info (4 bytes)
    raw_adv_data[idx++] = (delay_us >> 24) & 0xFF;
    raw_adv_data[idx++] = (delay_us >> 16) & 0xFF;
    raw_adv_data[idx++] = (delay_us >> 8)  & 0xFF;
    raw_adv_data[idx++] = (delay_us)       & 0xFF;

    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, idx, raw_adv_data);
    if (esp_vhci_host_check_send_available()) esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void) {
    uint8_t peer_addr[6] = {0};
    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf, 0x20, 0x20, 0x03, 0, 0, peer_addr, 0x07, 0);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void) {
    uint16_t sz = make_cmd_ble_set_adv_enable(hci_cmd_buf, 1);
    if (esp_vhci_host_check_send_available()) esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_stop(void) {
    uint16_t sz = make_cmd_ble_set_adv_enable(hci_cmd_buf, 0);
    if (esp_vhci_host_check_send_available()) esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_reset(void) {
    uint16_t sz = make_cmd_reset(hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void controller_rcv_pkt_ready(void) {}
static int host_rcv_pkt(uint8_t *data, uint16_t len) { return ESP_OK; }
static esp_vhci_host_callback_t vhci_host_cb = { controller_rcv_pkt_ready, host_rcv_pkt };

esp_err_t bt_sender_init(void) {
    if (is_initialized) return ESP_OK;

    // NVS Initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Controller Initialization
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_vhci_host_register_callback(&vhci_host_cb);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // HCI Commands Initialization
    hci_cmd_send_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    hci_cmd_send_ble_set_adv_param();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    is_initialized = true;
    ESP_LOGI(TAG, "BT Sender API Initialized");
    return ESP_OK;
}

int bt_sender_execute_burst(const bt_sender_config_t *config) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "Error: BT Sender not initialized! Call bt_sender_init() first.");
        return 0;
    }

    int run_count = (config->burst_count > MAX_BURST_COUNT) ? MAX_BURST_COUNT : config->burst_count;
    int64_t start_us = esp_timer_get_time();
    int64_t target_us = start_us + config->delay_us;

    for (int i = 0; i < run_count; i++) {
        int64_t now_us = esp_timer_get_time();
        int32_t remain_delay = (int32_t)(target_us - now_us - TX_OFFSET_US);
        if (remain_delay < 0) remain_delay = 0;

        // 1. Set Advertising Data
        hci_cmd_send_ble_set_adv_data(config->cmd_type, remain_delay, config->target_mask);
        esp_rom_delay_us(500);

        // 2. Start Advertising
        t1_start = esp_timer_get_time();
        hci_cmd_send_ble_adv_start();
        vTaskDelay(pdMS_TO_TICKS(10));

        // 3. Stop Advertising
        hci_cmd_send_ble_adv_stop();

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return config->burst_count;
}