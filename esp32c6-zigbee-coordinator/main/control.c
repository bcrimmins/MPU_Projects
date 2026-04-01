/*
 * control.c  —  APPLICATION CONTROL LOGIC
 *
 * This is the file you edit for each project.  The Zigbee coordinator layer
 * (zb_coordinator.c) calls the functions defined in control.h; you implement
 * them here with your specific logic.
 *
 * Two example patterns are provided below:
 *
 *   EXAMPLE A — Timer-based shutoff (e.g. soldering iron)
 *     A smart plug is turned OFF after SHUTOFF_TIMEOUT_S seconds have elapsed
 *     since the last time it was seen ON, regardless of whether anyone told it
 *     to turn off.  A button or contact sensor reset the timer.
 *
 *   EXAMPLE B — Temperature threshold control (e.g. heater / fan)
 *     A temperature sensor reports °C; a smart plug is turned ON when temp
 *     drops below TEMP_SETPOINT - TEMP_HYSTERESIS, and OFF when it rises
 *     above TEMP_SETPOINT + TEMP_HYSTERESIS.
 *
 * Uncomment the pattern you want and fill in your device addresses / setpoints.
 * Both examples use the same device table populated in control_on_device_joined().
 *
 * ---- HOW DEVICE SLOTS WORK ------------------------------------------------
 * Devices join in any order.  zb_coordinator.c assigns each joining device a
 * sequential slot index (0, 1, 2 ...) and you can retrieve them with:
 *
 *   const zb_device_t *dev = zb_device_get(idx);
 *   // dev->short_addr, dev->endpoint
 *
 * For fixed installations you can also hard-code IEEE addresses and match
 * them in control_on_device_joined() to assign named roles.
 */

#include "control.h"
#include "zb_coordinator.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "CONTROL";

/* ==========================================================================
 * SELECT YOUR PATTERN — uncomment exactly one of these:
 * ========================================================================== */
#define PATTERN_TIMER_SHUTOFF       /* Example A: soldering iron timer        */
/* #define PATTERN_TEMP_CONTROL */  /* Example B: heater/fan thermostat       */

/* ==========================================================================
 * EXAMPLE A — Timer-based shutoff
 * ========================================================================== */
#ifdef PATTERN_TIMER_SHUTOFF

/* How long (seconds) before the plug is forced OFF */
#define SHUTOFF_TIMEOUT_S   1800    /* 30 minutes */

/*
 * Device slot assignments.
 * Slot 0 = first device to join (the smart plug).
 * If you add a reset button/sensor, assign it to slot 1.
 */
#define SLOT_PLUG   0   /* smart plug that controls the iron */

static uint32_t s_elapsed_s     = 0;    /* seconds since last reset           */
static bool     s_plug_is_on    = false;
static bool     s_network_ready = false;

void control_init(void)
{
    s_elapsed_s     = 0;
    s_plug_is_on    = false;
    s_network_ready = true;
    ESP_LOGI(TAG, "[Timer] Shutoff timeout: %d s (%d min)",
             SHUTOFF_TIMEOUT_S, SHUTOFF_TIMEOUT_S / 60);
}

void control_tick(void)
{
    if (!s_network_ready) return;

    const zb_device_t *plug = zb_device_get(SLOT_PLUG);
    if (!plug) return;  /* plug hasn't joined yet */

    s_elapsed_s++;

    if (s_elapsed_s >= SHUTOFF_TIMEOUT_S && s_plug_is_on) {
        ESP_LOGW(TAG, "[Timer] %d min elapsed — forcing plug OFF", SHUTOFF_TIMEOUT_S / 60);
        zb_send_onoff(plug->short_addr, plug->endpoint, false);
        s_plug_is_on = false;
    }

    /* Log remaining time every 5 minutes */
    if (s_elapsed_s % 300 == 0 && s_plug_is_on) {
        uint32_t remaining = (SHUTOFF_TIMEOUT_S > s_elapsed_s)
                             ? SHUTOFF_TIMEOUT_S - s_elapsed_s : 0;
        ESP_LOGI(TAG, "[Timer] %lu min remaining", (unsigned long)(remaining / 60));
    }
}

void control_on_device_joined(const zb_device_t *dev)
{
    uint8_t idx = (uint8_t)(dev - zb_device_get(0)); /* derive slot index */
    ESP_LOGI(TAG, "[Timer] Device joined slot %d  addr=0x%04x", idx, dev->short_addr);

    if (idx == SLOT_PLUG) {
        ESP_LOGI(TAG, "[Timer] Smart plug identified — turning ON and starting timer");
        zb_send_onoff(dev->short_addr, dev->endpoint, true);
        s_plug_is_on = true;
        s_elapsed_s  = 0;

        /* Ask the plug to report its On/Off state every 30–300 s */
        zb_configure_reporting(dev->short_addr, dev->endpoint,
                               ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                               ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                               30, 300);
    }
}

void control_on_attribute(uint16_t short_addr, uint8_t endpoint,
                          uint16_t cluster_id, uint16_t attr_id,
                          esp_zb_zcl_attr_type_t data_type, void *data)
{
    const zb_device_t *plug = zb_device_get(SLOT_PLUG);
    if (!plug || plug->short_addr != short_addr) return;

    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        attr_id    == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        bool now_on = *(uint8_t *)data != 0;
        ESP_LOGI(TAG, "[Timer] Plug state: %s", now_on ? "ON" : "OFF");

        if (now_on && !s_plug_is_on) {
            /* Plug was turned on externally — reset the timer */
            ESP_LOGI(TAG, "[Timer] External ON detected — resetting timer");
            s_elapsed_s = 0;
        }
        s_plug_is_on = now_on;
    }
}

#endif /* PATTERN_TIMER_SHUTOFF */


/* ==========================================================================
 * EXAMPLE B — Temperature threshold / thermostat
 * ========================================================================== */
#ifdef PATTERN_TEMP_CONTROL

/*
 * Setpoint and hysteresis — tune these for your application.
 *   Heater : turn ON below (SETPOINT - HYST), OFF above (SETPOINT + HYST)
 *   Cooler : invert the logic (see the comment in control_tick below)
 */
#define TEMP_SETPOINT_C     20.0f   /* desired temperature in °C              */
#define TEMP_HYSTERESIS_C    0.5f   /* dead-band around the setpoint          */

/* Report interval — device sends temperature every MIN_RPT to MAX_RPT secs */
#define TEMP_REPORT_MIN_S   10
#define TEMP_REPORT_MAX_S   60

/* Device slot assignments */
#define SLOT_TEMP_SENSOR    0   /* temperature sensor                         */
#define SLOT_HEATER_PLUG    1   /* smart plug driving the heater              */

static float s_current_temp_c   = 0.0f;
static bool  s_heater_on        = false;
static bool  s_temp_valid       = false;
static bool  s_network_ready    = false;

void control_init(void)
{
    s_current_temp_c = 0.0f;
    s_heater_on      = false;
    s_temp_valid     = false;
    s_network_ready  = true;
    ESP_LOGI(TAG, "[Thermo] Setpoint: %.1f°C  Hysteresis: ±%.1f°C",
             TEMP_SETPOINT_C, TEMP_HYSTERESIS_C);
}

void control_tick(void)
{
    if (!s_network_ready || !s_temp_valid) return;

    const zb_device_t *heater = zb_device_get(SLOT_HEATER_PLUG);
    if (!heater) return;

    float low  = TEMP_SETPOINT_C - TEMP_HYSTERESIS_C;
    float high = TEMP_SETPOINT_C + TEMP_HYSTERESIS_C;

    /* ---- HEATER logic (invert comparisons for a cooler/fan) ---- */
    if (!s_heater_on && s_current_temp_c < low) {
        ESP_LOGI(TAG, "[Thermo] %.2f°C < %.2f°C — turning heater ON",
                 s_current_temp_c, low);
        zb_send_onoff(heater->short_addr, heater->endpoint, true);
        s_heater_on = true;
    } else if (s_heater_on && s_current_temp_c > high) {
        ESP_LOGI(TAG, "[Thermo] %.2f°C > %.2f°C — turning heater OFF",
                 s_current_temp_c, high);
        zb_send_onoff(heater->short_addr, heater->endpoint, false);
        s_heater_on = false;
    }
}

void control_on_device_joined(const zb_device_t *dev)
{
    uint8_t idx = (uint8_t)(dev - zb_device_get(0));
    ESP_LOGI(TAG, "[Thermo] Device joined slot %d  addr=0x%04x", idx, dev->short_addr);

    if (idx == SLOT_TEMP_SENSOR) {
        /* Configure the sensor to push temperature reports automatically */
        zb_configure_reporting(dev->short_addr, dev->endpoint,
                               ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                               ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                               TEMP_REPORT_MIN_S, TEMP_REPORT_MAX_S);
        ESP_LOGI(TAG, "[Thermo] Temperature sensor configured for reporting");
    }

    if (idx == SLOT_HEATER_PLUG) {
        /* Start with heater OFF */
        zb_send_onoff(dev->short_addr, dev->endpoint, false);
        ESP_LOGI(TAG, "[Thermo] Heater plug identified — starting OFF");
    }
}

void control_on_attribute(uint16_t short_addr, uint8_t endpoint,
                          uint16_t cluster_id, uint16_t attr_id,
                          esp_zb_zcl_attr_type_t data_type, void *data)
{
    const zb_device_t *sensor = zb_device_get(SLOT_TEMP_SENSOR);
    if (!sensor || sensor->short_addr != short_addr) return;

    if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT &&
        attr_id    == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
        /* Zigbee temperature is int16 in units of 0.01°C */
        int16_t raw = *(int16_t *)data;
        s_current_temp_c = raw / 100.0f;
        s_temp_valid     = true;
        ESP_LOGI(TAG, "[Thermo] Temperature: %.2f°C", s_current_temp_c);
    }
}

#endif /* PATTERN_TEMP_CONTROL */
