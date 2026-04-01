/*
 * ESP32-C6 Zigbee Coordinator Firmware
 *
 * Initializes the ESP32-C6 as a Zigbee coordinator, opens the network
 * for device joining, and logs activity to UART (115200 baud).
 *
 * Wiring: No extra hardware required. Uses the built-in 802.15.4 radio.
 * Flash via: idf.py -p <PORT> flash monitor
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_core.h"

static const char *TAG = "ZB_COORD";

/* Zigbee channel mask — channel 11 (0x00000800).
   Change to CONFIG_ZB_CHANNEL_MASK for multi-channel scanning. */
#define ZB_CHANNEL          11
#define ZB_CHANNEL_MASK     (1 << ZB_CHANNEL)

/* Network open duration in seconds (0xFF = open indefinitely) */
#define NETWORK_OPEN_DURATION   0xFF

/* ---- Zigbee stack signal handler ---------------------------------------- */

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t           *p_sg_p      = signal_struct->p_app_signal;
    esp_err_t           err_status  = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Coordinator started on channel %d", ZB_CHANNEL);
            /* Open network for joining immediately after start */
            esp_zb_bdb_open_network(NETWORK_OPEN_DURATION);
            ESP_LOGI(TAG, "Network open for joining (permit join = %ds)",
                     NETWORK_OPEN_DURATION == 0xFF ? 255 : NETWORK_OPEN_DURATION);
        } else {
            ESP_LOGW(TAG, "Failed to initialize coordinator (status: %s) — retrying steering",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Network formed — PAN ID: 0x%04hx  Extended PAN ID: "
                     "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x  Channel: %d",
                     esp_zb_get_pan_id(),
                     extended_pan_id[7], extended_pan_id[6],
                     extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2],
                     extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_current_channel());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGE(TAG, "Network formation failed (status: %s)",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started — devices may join");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *dev =
            (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Device joined — short addr: 0x%04hx", dev->device_short_addr);
        break;
    }

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            uint8_t duration = *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
            if (duration) {
                ESP_LOGI(TAG, "Permit join enabled for %ds", duration);
            } else {
                ESP_LOGI(TAG, "Permit join disabled");
            }
        }
        break;

    default:
        ESP_LOGD(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/* ---- Zigbee task --------------------------------------------------------- */

static void zigbee_task(void *arg)
{
    /* Radio / platform config */
    esp_zb_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,   /* Use the built-in 802.15.4 radio */
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    /* Zigbee coordinator config */
    esp_zb_cfg_t zigbee_config = {
        .esp_zb_role  = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = 10,
        },
    };
    esp_zb_init(&zigbee_config);

    /* Set the 802.15.4 channel */
    esp_zb_set_channel_mask(ZB_CHANNEL_MASK);

    /* Primary endpoint: a minimal "Zigbee coordinator" device.
       Using Home Automation profile (0x0104), On/Off Switch cluster set
       so that Zigbee2MQTT / ZHA can interview the coordinator. */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        "Espressif"));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        "ESP32-C6-Coordinator"));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
        cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_endpoint_config_t ep_config = {
        .endpoint       = 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_config));
    ESP_ERROR_CHECK(esp_zb_device_register(ep_list));

    /* Start the Zigbee stack */
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ---- app_main ------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 Zigbee Coordinator starting...");

    /* Initialize NVS — required by the Zigbee stack for persistent storage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated — erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Run Zigbee on its own task (pinned to core 0) */
    xTaskCreate(zigbee_task, "zigbee_task", 8192, NULL, 5, NULL);
}
