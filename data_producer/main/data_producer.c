/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Zigbee Customized Data Producer Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "alarm_timer.h"

#include "esp_zigbee.h"

#include "data_stream_server.h"
#include "data_producer.h"

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

static const char *TAG = "CUSTOMIZED_DATA_PRODUCER";

// ============================================================
// AUDIO RECORDING CONFIGURATION
// ============================================================

#define I2S_BCLK GPIO_NUM_4
#define I2S_WS   GPIO_NUM_5
#define I2S_DIN  GPIO_NUM_10

#define RECORD_BUTTON GPIO_NUM_9

#define SAMPLE_RATE      16000
#define RECORD_SECONDS   2
#define RECORD_SAMPLES   (SAMPLE_RATE * RECORD_SECONDS)

// 32000 samples × 2 bytes = 64000 bytes
static int16_t audio_recording_buffer[RECORD_SAMPLES];

static volatile bool audio_ready = false;

static i2s_chan_handle_t rx_handle;


// ============================================================
// FUNCTIONS USED BY data_stream_server.c
// ============================================================

const uint8_t *audio_get_buffer_bytes(void)
{
    return (const uint8_t *)audio_recording_buffer;
}

uint32_t audio_get_buffer_size(void)
{
    return sizeof(audio_recording_buffer);
}

bool audio_recording_ready(void)
{
    return audio_ready;
}


// ============================================================
// BOOT BUTTON
// ============================================================

static void audio_button_init(void)
{
    gpio_config_t button_cfg = {
        .pin_bit_mask = (1ULL << RECORD_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&button_cfg));
}

static void wait_for_record_button(void)
{
    while (gpio_get_level(RECORD_BUTTON) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    while (gpio_get_level(RECORD_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// ============================================================
// I2S MICROPHONE INITIALIZATION
// ============================================================

static void audio_i2s_init(void)
{
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_AUTO,
            I2S_ROLE_MASTER
        );

    ESP_ERROR_CHECK(
        i2s_new_channel(
            &chan_cfg,
            NULL,
            &rx_handle
        )
    );

    i2s_std_config_t rx_cfg = {

        .clk_cfg =
            I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT,
                I2S_SLOT_MODE_MONO
            ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN,

            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    // INMP441 L/R pin connected to GND = LEFT slot
    rx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(
            rx_handle,
            &rx_cfg
        )
    );

    ESP_LOGI(TAG, "INMP441 initialized");
}


// ============================================================
// RECORD 2 SECONDS
// ============================================================

static void record_audio(void)
{
    int32_t samples[256];
    size_t bytes_read = 0;
    int write_index = 0;

    bool first_block = true;

    int32_t raw_min = INT32_MAX;
    int32_t raw_max = INT32_MIN;

    int16_t pcm_min = INT16_MAX;
    int16_t pcm_max = INT16_MIN;

    audio_ready = false;

    ESP_LOGI(TAG, "Recording audio for 2 seconds...");

    ESP_ERROR_CHECK(
        i2s_channel_enable(rx_handle)
    );

    while (write_index < RECORD_SAMPLES)
    {
        ESP_ERROR_CHECK(
            i2s_channel_read(
                rx_handle,
                samples,
                sizeof(samples),
                &bytes_read,
                portMAX_DELAY
            )
        );

        int sample_count =
            bytes_read / sizeof(int32_t);

                // Print the first few RAW microphone samples once
    if (first_block && sample_count > 0)
    {
        ESP_LOGI(TAG, "First RAW microphone samples:");

        int count = (sample_count < 8) ? sample_count : 8;

        for (int j = 0; j < count; j++)
        {   
            ESP_LOGI(
                TAG,
                "RAW[%d] = 0x%08x (%ld)",
                j,
                (unsigned int)samples[j],
                (long)samples[j]
            );
        }

        first_block = false;
    }

        int remaining =
            RECORD_SAMPLES - write_index;

        int samples_to_copy =
            (sample_count < remaining)
            ? sample_count
            : remaining;

        for (int i = 0; i < samples_to_copy; i++)
        {
            int32_t raw = samples[i];

            if (raw < raw_min) raw_min = raw;
            if (raw > raw_max) raw_max = raw;

            int16_t pcm = (int16_t)(raw >> 16);

            if (pcm < pcm_min) pcm_min = pcm;
            if (pcm > pcm_max) pcm_max = pcm;

            audio_recording_buffer[write_index + i] = pcm;
        }

        write_index += samples_to_copy;
    }

    ESP_ERROR_CHECK(
        i2s_channel_disable(rx_handle)
    );

    ESP_LOGI(
    TAG,
    "RAW range: min=%ld max=%ld",
    (long)raw_min,
    (long)raw_max
);

ESP_LOGI(
    TAG,
    "PCM16 range: min=%d max=%d",
    pcm_min,
    pcm_max
);

    audio_ready = true;

    ESP_LOGI(
        TAG,
        "Recording complete: %d samples, %u bytes",
        write_index,
        (unsigned)sizeof(audio_recording_buffer)
    );

    ESP_LOGI(TAG, "Audio ready for Zigbee transmission");
}


// ============================================================
// AUDIO TASK
// ============================================================

static void audio_record_task(void *arg)
{
    while (1)
    {
        ESP_LOGI(TAG, "Press BOOT to record voice");

        wait_for_record_button();

        record_audio();

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void esp_zigbee_alarm_bdb_commissioning(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(arg);
    esp_zigbee_lock_release();
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            if (ezb_bdb_is_factory_new()) {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ezb_bdb_open_network(180);
                ESP_LOGI(TAG, "Device reboot");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status(0x%02x), retry again", ezb_app_signal_to_string(signal_type), status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_FORMATION: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Failed to form network with status(0x%02x)", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
    } break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Network steering completed");
        } else {
            ESP_LOGW(TAG, "Failed to steering network with status(0x%02x)", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
    } break;
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        const ezb_zdo_signal_device_annce_params_t *dev_annce_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "New device commissioned or rejoined(short: 0x%04hx)", dev_annce_params->short_addr);
    } break;
    case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)ezb_app_signal_get_params(app_signal);
        if (duration) {
            ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", ezb_nwk_get_panid(), duration);
        } else {
            ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", ezb_nwk_get_panid());
        }
    } break;
    case EZB_ZDO_SIGNAL_LEAVE_INDICATION: {
        const ezb_zdo_signal_leave_indication_params_t *leave_ind_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "Zigbee Node(0x%04hx) is leaving network", leave_ind_params->short_addr);
    } break;
    default:
        ESP_LOGI(TAG, "Zigbee APP Signal: %s(type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        ezb_zcl_cmd_default_rsp_message_t *default_rsp = (ezb_zcl_cmd_default_rsp_message_t *)message;
        ESP_LOGI(TAG, "Received ZCL Default Response with status(0x%02x)", default_rsp->in.status_code);
    } break;
    default:
        ESP_LOGW(TAG, "ZCL Core Action: ID(0x%04lx)", callback_id);
        break;
    }
}

esp_err_t esp_zigbee_create_customized_data_producer_device(void)
{
    ezb_af_device_desc_t   dev_desc      = ezb_af_create_device_desc();
    ezb_af_ep_desc_t       ep_desc       = NULL;
    ezb_zcl_cluster_desc_t basic_desc    = NULL;
    ezb_zcl_cluster_desc_t identify_desc = NULL;
    ezb_zcl_cluster_desc_t custom_desc   = NULL;

    ezb_zcl_basic_cluster_server_config_t basic_cfg = {
        .zcl_version  = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
    };
    basic_desc = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);

    ezb_zcl_identify_cluster_server_config_t identify_cfg = {
        .identify_time = EZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };
    identify_desc = ezb_zcl_identify_create_cluster_desc(&identify_cfg, EZB_ZCL_CLUSTER_SERVER);

    ezb_zcl_custom_cluster_config_t custom_cfg = {
        .cluster_id  = ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_CLUSTER_ID,
        .init_func   = esp_zigbee_zcl_customized_data_stream_server_init,
        .deinit_func = esp_zigbee_zcl_customized_data_stream_server_deinit,
    };
    custom_desc = ezb_zcl_custom_create_cluster_desc(&custom_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_custom_cluster_desc_add_attr(custom_desc, ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_DEVICE_NAME_ATTR_ID,
                                         EZB_ZCL_ATTR_TYPE_STRING, EZB_ZCL_ATTR_ACCESS_READ,
                                         (void *)ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_DEVICE_NAME);
    uint16_t data_length = ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_DATA_LENGTH_DEFAULT_VALUE;
    ezb_zcl_custom_cluster_desc_add_attr(custom_desc, ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_DATA_LENGTH_ATTR_ID,
                                         EZB_ZCL_ATTR_TYPE_UINT16, EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_WRITE,
                                         (void *)&data_length);
    uint16_t data_begin = ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_DATA_BEGIN_DEFAULT_VALUE;
    ezb_zcl_custom_cluster_desc_add_attr(custom_desc, ESP_ZIGBEE_CUSTOMIZED_DATA_STREAM_DATA_BEGIN_ATTR_ID,
                                         EZB_ZCL_ATTR_TYPE_UINT16, EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_WRITE,
                                         (void *)&data_begin);
    ezb_af_ep_config_t ep_config = {
        .ep_id              = ESP_ZIGBEE_CUSTOMIZED_DATA_PRODUCER_EP_ID,
        .app_profile_id     = EZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZIGBEE_CUSTOMIZED_DATA_PRODUCER_DEVICE_ID,
        .app_device_version = 0,
    };
    ep_desc = ezb_af_create_endpoint_desc(&ep_config);

    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, basic_desc));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, identify_desc));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep_desc, custom_desc));

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));

    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);

    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));

    return ESP_OK;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());

    ESP_ERROR_CHECK(esp_zigbee_create_customized_data_producer_device());

    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize microphone
    audio_i2s_init();

    // Initialize BOOT button
    audio_button_init();

    // Start audio recording task
    xTaskCreate(
        audio_record_task,
        "audio_record_task",
        4096,
        NULL,
        4,
        NULL
    );

    // Start existing Zigbee producer
    ESP_LOGI(TAG, "Start ESP Zigbee Stack");

    xTaskCreate(
        esp_zigbee_stack_main_task,
        "Zigbee_main",
        4096,
        NULL,
        5,
        NULL
    );
}