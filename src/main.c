/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This demo showcases BLE GATT client. It can scan BLE devices and connect to
 *one device. Run the gatt_server demo, the client demo will automatically
 *connect to the gatt_server demo. Client demo will enable gatt_server's notify
 *after connection. The two devices will then exchange data.
 *
 ****************************************************************************/

#include "nvs.h"
#include "nvs_flash.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>

#define TAG "SCAN"

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    // unit is second
    uint32_t duration = 30;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGI(TAG, "STARTING SCAN");
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // scan start complete event to indicate scan start successfully or
        // failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x %d", scan_result->scan_rst.bda[0],
                     scan_result->scan_rst.bda[1], scan_result->scan_rst.bda[2],
                     scan_result->scan_rst.bda[3], scan_result->scan_rst.bda[4],
                     scan_result->scan_rst.bda[5], scan_result->scan_rst.rssi);
            break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            esp_ble_gap_start_scanning(duration);
            ESP_LOGI(TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT\n");
            break;

        default:
            ESP_LOGI(TAG, "DEFAULT\n");
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG,
                 "update connection params status = %d, min_int = %d, max_int "
                 "= %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status, param->update_conn_params.min_int,
                 param->update_conn_params.max_int, param->update_conn_params.conn_int,
                 param->update_conn_params.latency, param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void app_main(void) {
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    static esp_ble_scan_params_t ble_scan_params = {.scan_type = BLE_SCAN_TYPE_PASSIVE,
                                                    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                                                    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
                                                    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
                                                    .scan_interval = 0x50,
                                                    .scan_window = 0x30};

    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_set_scan_params: rc=%d", ret);
        return;
    }
}