// bt_sender.h
#pragma once
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize BT Sender API
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t bt_sender_init(void);

/**
 * @brief Execute a burst of advertising commands
 * @param cmd_type Command type (e.g., 0xA0)
 * @param target_delay_us Target delay in microseconds for command execution (minimum 1s)
 * @param prep_led_us Preparation LED time in microseconds
 * @param target_player_mask Bitmask representing target player IDs
 * @return int Actual number of commands executed
 */
typedef struct {
    uint8_t  cmd_type;
    uint32_t delay_us;
    uint32_t prep_led_us;
    uint64_t target_mask;
    uint8_t data[3];
} bt_sender_config_t;

int bt_sender_execute_burst(const bt_sender_config_t *config);