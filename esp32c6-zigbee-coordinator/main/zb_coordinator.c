/*
 * zb_coordinator.c
 *
 * Zigbee coordinator layer.  Handles:
 *   - Stack initialization and network formation
 *   - Device join tracking (device table)
 *   - ZCL On/Off command sending
 *   - ZCL attribute read requests and incoming attribute reports
 *   - Periodic control_tick() scheduling via esp_zb_scheduler_alarm()
 *
 * All calls into the Zigbee stack are made from within the Zigbee task
 * context — either directly in signal/ZCL callbacks, or via scheduler alarms.
 */

#include "zb_coordinator.h"
#include "control.h"

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "ZB_COORD";

/* ---- Device table -------------------------------------------------------- */

static zb_device_t s_devices[ZB_MAX_DEVICES];
static uint8_t     s_device_count = 0;

uint8_t zb_device_count(void)
{
    return s_device_count;
}

const zb_device_t *zb_device_get(uint8_t idx)
{
    if (idx >= ZB_MAX_DEVICES || !s_devices[idx].valid) {
        return NULL;
    }
    return &s_devices[idx];
}

const zb_device_t *zb_device_find(uint16_t short_addr)
{
    for (int i = 0; i < ZB_MAX_DEVICES; i++) {
        if (s_devices[i].valid && s_devices[i].short_addr == short_addr) {
            return &s_devices[i];
        }
    }
    return NULL;
}

static void device_table_add(uint16_t short_addr)
{
    /* Ignore duplicates */
    if (zb_device_find(short_addr)) {
        return;
    }
    for (int i = 0; i < ZB_MAX_DEVICES; i++) {
        if (!s_devices[i].valid) {
            s_devices[i].short_addr = short_addr;
            s_devices[i].endpoint   = 1;   /* default; override in control.c if needed */
            s_devices[i].valid      = true;
            s_device_count++;
            ESP_LOGI(TAG, "Device table: slot %d  addr=0x%04x  (total: %d)",
                     i, short_addr, s_device_count);
            return;
        }
    }
    ESP_LOGW(TAG, "Device table full — cannot add 0x%04x", short_addr);
}

/* ---- ZCL On/Off ---------------------------------------------------------- */

void zb_send_onoff(uint16_t short_addr, uint8_t endpoint, bool on)
{
    esp_zb_zcl_on_off_cmd_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .dst_endpoint          = endpoint,
            .src_endpoint          = ZB_COORD_ENDPOINT,
        },
        .address_mode  = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID
                            : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID,
    };
    esp_err_t err = esp_zb_zcl_on_off_cmd_req(&cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "on_off_cmd to 0x%04x failed: %s", short_addr, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Sent %s to 0x%04x ep%d", on ? "ON" : "OFF", short_addr, endpoint);
    }
}

/* ---- ZCL Attribute read -------------------------------------------------- */

void zb_read_attribute(uint16_t short_addr, uint8_t endpoint,
                       uint16_t cluster_id, uint16_t attr_id)
{
    uint16_t attr_list[] = { attr_id };
    esp_zb_zcl_read_attr_cmd_t req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .dst_endpoint          = endpoint,
            .src_endpoint          = ZB_COORD_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID    = cluster_id,
        .attr_field   = attr_list,
        .attr_number  = 1,
    };
    esp_zb_zcl_read_attr_cmd_req(&req);
}

/* ---- ZCL Configure Reporting --------------------------------------------- */

void zb_configure_reporting(uint16_t short_addr, uint8_t endpoint,
                             uint16_t cluster_id, uint16_t attr_id,
                             uint16_t min_interval, uint16_t max_interval)
{
    esp_zb_zcl_config_report_record_t record = {
        .direction      = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .attributeID    = attr_id,
        .attrType       = ESP_ZB_ZCL_ATTR_TYPE_S16,  /* int16 covers temp/most sensors */
        .min_interval   = min_interval,
        .max_interval   = max_interval,
        .reportable_change = 10,  /* report if value changes by >=0.1°C (for temperature) */
    };
    esp_zb_zcl_config_report_cmd_t req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = short_addr,
            .dst_endpoint          = endpoint,
            .src_endpoint          = ZB_COORD_ENDPOINT,
        },
        .address_mode   = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID      = cluster_id,
        .record_field   = &record,
        .record_number  = 1,
    };
    esp_zb_zcl_config_report_cmd_req(&req);
    ESP_LOGI(TAG, "Configured reporting: 0x%04x ep%d cluster=0x%04x attr=0x%04x [%d–%ds]",
             short_addr, endpoint, cluster_id, attr_id, min_interval, max_interval);
}

/* ---- ZCL attribute report callback --------------------------------------- */
/*
 * Fires (within Zigbee task context) whenever a device sends an attribute
 * report or we receive a read-attribute response.
 * Forwards the data to control_on_attribute() for application processing.
 */
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    if (!message || message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t short_addr = message->info.src_address.u.short_addr;
    uint8_t  endpoint   = message->info.src_endpoint;
    uint16_t cluster_id = message->info.cluster;
    uint16_t attr_id    = message->attribute.id;

    ESP_LOGD(TAG, "Attr report: src=0x%04x ep%d cluster=0x%04x attr=0x%04x",
             short_addr, endpoint, cluster_id, attr_id);

    /* Forward to user control logic */
    control_on_attribute(short_addr, endpoint, cluster_id, attr_id,
                         message->attribute.data.type,
                         message->attribute.data.value);
    return ESP_OK;
}

/* ---- Control tick via scheduler alarm ------------------------------------ */
/*
 * esp_zb_scheduler_alarm fires within the Zigbee task context, making it
 * safe to call any Zigbee API (zb_send_onoff, zb_read_attribute, etc.) from
 * inside control_tick().
 */
static void control_tick_alarm(uint8_t param)
{
    control_tick();
    /* Reschedule for the next tick */
    esp_zb_scheduler_alarm(control_tick_alarm, 0, CONTROL_TICK_MS);
}

/* ---- BDB commissioning helper -------------------------------------------- */

static void bdb_start_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

/* ---- Zigbee app signal handler ------------------------------------------- */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t                *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t                err        = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type   = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack initialized — starting coordinator commissioning");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Coordinator booted — forming network on ch%d", ZB_CHANNEL);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGW(TAG, "Startup failed (%s) — retrying", esp_err_to_name(err));
            esp_zb_scheduler_alarm(bdb_start_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err == ESP_OK) {
            esp_zb_ieee_addr_t ext_pan;
            esp_zb_get_extended_pan_id(ext_pan);
            ESP_LOGI(TAG, "Network formed  ch=%d  PAN=0x%04x  EPAN=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     esp_zb_get_current_channel(), esp_zb_get_pan_id(),
                     ext_pan[7], ext_pan[6], ext_pan[5], ext_pan[4],
                     ext_pan[3], ext_pan[2], ext_pan[1], ext_pan[0]);

            /* Open for joining — 0xFF = indefinitely open */
            esp_zb_bdb_open_network(0xFF);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);

            /* Initialize control logic and start the periodic tick */
            control_init();
            esp_zb_scheduler_alarm(control_tick_alarm, 0, CONTROL_TICK_MS);
        } else {
            ESP_LOGE(TAG, "Network formation failed (%s) — retrying", esp_err_to_name(err));
            esp_zb_scheduler_alarm(bdb_start_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Network steering active — permit join open");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *annce =
            (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        uint16_t addr = annce->device_short_addr;
        ESP_LOGI(TAG, "Device joined: 0x%04x", addr);
        device_table_add(addr);
        const zb_device_t *dev = zb_device_find(addr);
        if (dev) {
            control_on_device_joined(dev);
        }
        break;
    }

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS: {
        uint8_t duration = *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (duration) {
            ESP_LOGI(TAG, "Permit join: %ds remaining", duration);
        } else {
            ESP_LOGI(TAG, "Permit join closed");
        }
        break;
    }

    default:
        ESP_LOGD(TAG, "ZDO signal 0x%x (%s) status: %s",
                 sig_type, esp_zb_zdo_signal_to_string(sig_type),
                 esp_err_to_name(err));
        break;
    }
}

/* ---- Coordinator start (called from main.c) ------------------------------ */

void zb_coordinator_start(void)
{
    /* Native 802.15.4 radio, no external host */
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));

    /* Coordinator role, up to ZB_MAX_DEVICES children */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg.max_children = ZB_MAX_DEVICES,
    };
    esp_zb_init(&zb_cfg);
    esp_zb_set_channel_mask(1 << ZB_CHANNEL);

    /* Build a minimal HA endpoint so the coordinator can be interviewed */
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "Espressif");
    esp_zb_basic_cluster_add_attr(basic,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  "ESP32-C6-Coordinator");

    esp_zb_cluster_list_t *clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(clusters, basic,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint           = ZB_COORD_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);
    esp_zb_device_register(ep_list);

    /* Register attribute callback — receives all incoming attribute data */
    esp_zb_core_action_handler_register(zb_attribute_handler);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();   /* does not return */
}
