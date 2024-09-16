/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This demo showcases BLE GATT client. It can scan BLE devices and connect to one device.
* Run the gatt_server demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "led_control.h"
#include "freertos/semphr.h"




#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_hidh.h"
#include "esp_bt_device.h"

esp_gattc_char_elem_t hid_char_result;
#define GATTC_TAG "GATTC_DEMO"
#define GAPC_TAG  "GAPC_DEMO"
#define GATTC_AUTH "GATTC_AUTH"
#define REMOTE_SERVICE_UUID        0x1812
#define REMOTE_NOTIFY_CHAR_UUID    0x2902
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0
#define ADV_NAME_MAX_LEN 32
#define UUID_STR_LEN 37 

esp_event_loop_handle_t hidh_task_handle; // hidh_callback handle
QueueHandle_t command_queue;
remote_command_t command;




static char remote_device_name[ADV_NAME_MAX_LEN] = "Silverline RCU";
static bool connect    = false;
static bool get_server = false;
esp_gattc_char_elem_t char_elem_result;
esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

// Declare a mutex to protect access to the queue
static SemaphoreHandle_t queue_mutex;
void print_queue_contents(void) {
    if (queue_mutex == NULL) {
        queue_mutex = xSemaphoreCreateMutex();  // Create the mutex if it hasn't been created
    }

    // Lock the mutex to ensure exclusive access to the queue
    if (xSemaphoreTake(queue_mutex, portMAX_DELAY) == pdTRUE) {
        remote_command_t command;
        int i = 0;
        int queue_size = uxQueueMessagesWaiting(command_queue); // Get the current number of messages in the queue

        ESP_LOGI(GATTC_TAG, "Current Queue Contents:");
        for (i = 0; i < queue_size; i++) {
            // Peek at the queue to get the command without removing it
            if (xQueuePeek(command_queue, &command, 0) == pdPASS) {
                // Print the current command
                switch (command) {
                    case COMMAND_ON:
                        ESP_LOGI(GATTC_TAG, "Command [%d]: COMMAND_ON", i);
                        break;
                    case COMMAND_TOGGLE_COLOR:
                        ESP_LOGI(GATTC_TAG, "Command [%d]: COMMAND_TOGGLE", i);
                        break;
                    case COMMAND_BRIGHTNESS_UP:
                        ESP_LOGI(GATTC_TAG, "Command [%d]: COMMAND_INCREASE", i);
                        break;
                    case COMMAND_BRIGHTNESS_DOWN:
                        ESP_LOGI(GATTC_TAG, "Command [%d]: COMMAND_DECREASE", i);
                        break;
                    case COMMAND_OFF:
                        ESP_LOGI(GATTC_TAG, "Command [%d]: COMMAND_OFF", i);
                        break;
                    default:
                        ESP_LOGI(GATTC_TAG, "Command [%d]: Unknown", i);
                        break;
                }
                // Remove the item from the queue
                xQueueReceive(command_queue, &command, 0);
                // Add it back to the queue to preserve the order
                xQueueSend(command_queue, &command, 0);
            }
        }
        xSemaphoreGive(queue_mutex);  // Release the mutex after printing
    } else {
        ESP_LOGE(GATTC_TAG, "Failed to take mutex to print queue contents");
    }
}


static esp_bt_uuid_t hid_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid={
        .uuid16 = REMOTE_SERVICE_UUID,
        }
};

esp_bt_uuid_t hid_char_uuid[4] = {
	{
		.len = ESP_UUID_LEN_16,
		.uuid = {
			.uuid16 = ESP_GATT_UUID_HID_INFORMATION,
		},
	},
	{
		.len = ESP_UUID_LEN_16,
		.uuid = {
			.uuid16 = ESP_GATT_UUID_HID_REPORT_MAP,
		},
	},
	{
		.len = ESP_UUID_LEN_16,
		.uuid = {
			.uuid16 = ESP_GATT_UUID_HID_CONTROL_POINT,
		},
	},
	{
		.len = ESP_UUID_LEN_16,
		.uuid = {
			.uuid16 = ESP_GATT_UUID_HID_REPORT,
		},
	},

};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x640,
    .scan_window            = 0x50,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle[8];
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATTC application register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id,
                 ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Open successfully, mtu %u", p_data->open.mtu);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        esp_ble_set_encryption(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);

        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        //esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &hid_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &hid_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "Search result, conn_id = %x, is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d, end handle %d, current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && 
            p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "Service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID128 matched");
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "Unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "Service search complete");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
        
            
            
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
        
            if (count > 0){
                for (int i = 3; i < 4; i++)
                {
                    memset(&hid_char_result, 0, sizeof(esp_gattc_char_elem_t));

                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            param->search_cmpl.conn_id,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                            hid_char_uuid[3],
                                                            &hid_char_result,
                                                            &count);

                    // ESP_LOGI(TAG, "Get CHAR by UUID Trial with UUID %X,changed %X", (int)target_char_uuid[i].uuid.uuid16, char_elem_result_a[i].uuid.uuid.uuid16);
                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error for %d", i);
                    }
                    else
                    {
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle[i] = hid_char_result.char_handle;
                        
                        esp_err_t ret_status = esp_ble_gattc_register_for_notify(gattc_if,
                                                                                    gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                                                                    hid_char_result.char_handle);
                        if (ret_status != ESP_GATT_OK)
                        {
                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_register_for_notify error ,%s",esp_err_to_name(ret_status));
                        }
                        else
                        {
                            // ESP_LOGI(TAG, "esp_ble_gattc_register_for_notify OK to handle %X", (int)char_elem_result_a[i].char_handle);
                        }
                    }
                }
                // memset(&hid_char_result,0,sizeof(esp_gattc_char_elem_t));
                // // char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t));
                // // if (!char_elem_result){
                // //     ESP_LOGE(GATTC_TAG, "gattc no mem");
                // //     break;
                // // }else{
                
                //     status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                //                                              p_data->search_cmpl.conn_id,
                //                                              gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                //                                              gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                //                                              hid_char_uuid[3],
                //                                              &hid_char_result,
                //                                              &count);
                //     ESP_LOGE(GATTC_TAG, "COUNT: %d ", count);
                //     if (status != ESP_GATT_OK){
                //         ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                //         // free(char_elem_result);
                //         // char_elem_result = NULL;
                //         break;
                //     }

                //     /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    
                //     gl_profile_tab[PROFILE_A_APP_ID].char_handle[3] = hid_char_result.char_handle;
                //     esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, hid_char_result.char_handle);
                    
                //     ESP_LOGI(GATTC_TAG,"Registering the notification");
                // // }
                // /* free char_elem_result */
                // // free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Notification register successfully");
            

            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         p_data->reg_for_notify.handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            else{
                ESP_LOGI(GATTC_TAG, "The number of attributes found is %d", count);
            }
            uint16_t count1 = 1;
            if (count > 0){
                descr_elem_result = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count1);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        // free(descr_elem_result);
                        // descr_elem_result = NULL;
                        // break;
                    }
                    else{
                        ESP_LOGI(GATTC_TAG, "The number of descriptors found is %d", count1);
                    }
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        // esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
                        // esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
                        
                        //esp_ble_set_encryption(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
                        esp_err_t status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        descr_elem_result[0].handle,
                                                                        sizeof(notify_en),
                                                                        (uint8_t*)&notify_en,
                                                                        ESP_GATT_WRITE_TYPE_RSP,
                                                                        ESP_GATT_AUTH_REQ_NO_MITM);
                        
                        if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                        } else {
                            ESP_LOGI(GATTC_TAG, "esp_ble_gattc_write_char_descr ");
                        }
                    }

                    

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (hidh_task_handle)
        {

            esp_hidh_event_data_t *p_param = NULL;
            size_t event_data_size = sizeof(esp_hidh_event_data_t);
            if (p_data->notify.value_len && p_data->notify.value)
            {
                event_data_size += p_data->notify.value_len;
            }
            if ((p_param = (esp_hidh_event_data_t *)malloc(event_data_size)) == NULL)
            {
                ESP_LOGE(GATTC_TAG, "%s malloc event data failed!", __func__);
                return;
            }
            memset(p_param, 0, event_data_size);
            if (p_data->notify.value_len && p_data->notify.value)
            {
                memcpy(((uint8_t *)p_param) + sizeof(esp_hidh_event_data_t), p_data->notify.value,
                        p_data->notify.value_len);
            }
            p_param->input.length = p_data->notify.value_len;
            p_param->input.data = p_data->notify.value;
            ESP_LOGV(GATTC_TAG, "DATA is %d %d %d %d", p_param->input.length, p_param->input.data[2], event_data_size, sizeof(esp_hidh_event_data_t));
            esp_event_post_to(hidh_task_handle, ESP_HIDH_EVENTS, ESP_HIDH_INPUT_EVENT,
                                p_param, event_data_size, portMAX_DELAY);

            if (p_param)
            {
                free(p_param);
                p_param = NULL;
            }
        }
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Descriptor write successfully");
        // uint8_t write_char_data[35];
        // for (int i = 0; i < sizeof(write_char_data); ++i)
        // {
        //     write_char_data[i] = i % 256;
        // }
        // esp_ble_gattc_write_char( gattc_if,
        //                           gl_profile_tab[PROFILE_A_APP_ID].conn_id,
        //                           gl_profile_tab[PROFILE_A_APP_ID].char_handle,
        //                           sizeof(write_char_data),
        //                           write_char_data,
        //                           ESP_GATT_WRITE_TYPE_RSP,
        //                           ESP_GATT_AUTH_REQ_NONE);
        // break;
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(bda));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Characteristic write failed, status %x)", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Characteristic write successfully");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GAPC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GAPC_TAG, "Scanning start successfully");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL,
                                                        &adv_name_len);
            ESP_LOGI(GAPC_TAG, "Scan result, device "ESP_BD_ADDR_STR", name len %u", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), adv_name_len);
            ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GAPC_TAG, "Device found %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "Connect to "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GAPC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GAPC_TAG, "Scanning stop successfully");
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(GATTC_AUTH, "BLE GAP SEC_REQ");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_AUTH, "remote BD_ADDR: %08x%04x",\
        (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3], (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTC_AUTH, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTC_AUTH, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        break;}
    case ESP_GAP_BLE_KEY_EVT:
        // shows the ble key info share with peer device to the user.
        //ESP_LOGI(TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GAPC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GAPC_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GAPC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GAPC_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
	esp_hidh_event_t event = (esp_hidh_event_t)id;
	esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

	switch (event)
	{
	case ESP_HIDH_OPEN_EVENT:
	{
		ESP_LOGD(GATTC_TAG, "Callback: Open Event");
		if (param->open.status == ESP_OK)
		{
			const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
			ESP_LOGI(GATTC_TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
			esp_hidh_dev_dump(param->open.dev, stdout);
		}
		else
		{
			ESP_LOGE(GATTC_TAG, " OPEN failed!"); // TODO(BS): Restart searching. It stops.
		}
		break;
	}
	case ESP_HIDH_BATTERY_EVENT:
	{
		ESP_LOGD(GATTC_TAG, "Callback: Battery Event");
		const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
		ESP_LOGD(GATTC_TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
		break;
	}
	case ESP_HIDH_INPUT_EVENT:
	{        
        ESP_LOGI(GATTC_TAG, "DATA is %d", param->input.data[2]);
        remote_command_t command;
        bool valid_command = true;

        // Determine which command to send based on the input data
        switch (param->input.data[2]) {
            case COMMAND_ON:
                command = COMMAND_ON;
                break;
            case COMMAND_TOGGLE_COLOR:
                command = COMMAND_TOGGLE_COLOR;
                break;
            case COMMAND_BRIGHTNESS_UP:
                command = COMMAND_BRIGHTNESS_UP;
                break;
            case COMMAND_BRIGHTNESS_DOWN:
                command = COMMAND_BRIGHTNESS_DOWN;
                break;
            case COMMAND_OFF:
                command = COMMAND_OFF;
                break;
            default:
                ESP_LOGI(GATTC_TAG, "Unknown input value: %d", param->input.data[2]);
                valid_command = false;  // Mark command as invalid
                break;
        }

        // Send the command to the queue only if it's a valid command
        if (valid_command) {
            if (xQueueSend(command_queue, &command, portMAX_DELAY) != pdPASS) {
                ESP_LOGE(GATTC_TAG, "Failed to send command to queue");
            } else {
                ESP_LOGI(GATTC_TAG, "Command sent to queue successfully");
            }
        } else {
            ESP_LOGI(GATTC_TAG, "Command not sent to queue due to invalid input");
        }
        break;
    }
	case ESP_HIDH_FEATURE_EVENT:
	{
		ESP_LOGD(GATTC_TAG, "Callback: Feature Event");
		const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
		ESP_LOGD(GATTC_TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
				 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
				 param->feature.length);
		ESP_LOG_BUFFER_HEX(GATTC_TAG, param->feature.data, param->feature.length);
		break;
	}
	case ESP_HIDH_CLOSE_EVENT: // TODO(BS): Check when this event is triggered
	{
		ESP_LOGD(GATTC_TAG, "Callback: Close Event");
		esp_hidh_dev_free(param->close.dev);
		//message_sender(BleControllerEvent::kBleConnectionLost); // inform state machine for close event
		const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
		// esp_ble_gap_disconnect(bda);
		ESP_LOGD(GATTC_TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
		break;
	}
	default:
		ESP_LOGD(GATTC_TAG, "EVENT: %d", event);
		break;
	}
}
esp_err_t create_hidh_event_loop(esp_hidh_config_t *config)
{
	esp_err_t ret;
	esp_event_loop_args_t hidh_task_args = {
		.queue_size = 5,
		.task_name = "esp_ble_hidh_events",
		.task_priority = uxTaskPriorityGet(NULL),
		.task_stack_size = config->event_stack_size,
		.task_core_id = PRO_CPU_NUM};

	do
	{
		ret = esp_event_loop_create(&hidh_task_args, &hidh_task_handle);
		if (ret != ESP_OK)
		{
			ESP_LOGE(GATTC_TAG, "%s esp_event_loop_create failed!", __func__);
			break;
		}

		ret = esp_event_handler_register_with(hidh_task_handle, ESP_HIDH_EVENTS, ESP_EVENT_ANY_ID, config->callback,
											  config->callback_arg);
	} while (0);
	if (ret != ESP_OK)
	{
		if (hidh_task_handle)
		{
			esp_event_loop_delete(hidh_task_handle);
			esp_restart();
		}
	}
	return ret;
}
void app_main(void)
{

    
    command_queue = xQueueCreate(QUEUE_SIZE, sizeof(remote_command_t)); // Queue to hold command values
    if (command_queue == NULL) {
        ESP_LOGE("GATTC_DEMO", "Failed to create command queue");
        return;  // Exit if the queue creation failed
    } else {
        ESP_LOGI("GATTC_DEMO", "Command queue created successfully");
    }
    
    start_led_task();
 
    
    

    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(remote_device_name, esp_bluedroid_get_example_name(), sizeof(remote_device_name));
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    esp_hidh_config_t config_ = {
				.callback = hidh_callback,
				.event_stack_size = 8 * 1024,
				.callback_arg = NULL,
			};
    ret = create_hidh_event_loop(&config_);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s create hidh event loop = %x", __func__, ret);

    }
    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GAPC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    // esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    // if (local_mtu_ret){
    //     ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    // }
    
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));

 
}