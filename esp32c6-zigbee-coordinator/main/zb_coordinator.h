#pragma once

/*
 * zb_coordinator.h
 *
 * Zigbee coordinator layer for standalone ESP32-C6 control projects.
 *
 * THREAD SAFETY NOTE:
 *   All functions in this header must be called from within the Zigbee task
 *   context — i.e., from inside:
 *     - esp_zb_app_signal_handler()
 *     - Any ZCL callback registered with this module
 *     - A function scheduled via esp_zb_scheduler_alarm()
 *   The control_tick() and control_on_* callbacks in control.c already satisfy
 *   this requirement. Do NOT call these from a separate RTOS task.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_zigbee_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Configuration ------------------------------------------------------- */

/* Coordinator's own endpoint */
#define ZB_COORD_ENDPOINT       1

/* 802.15.4 channel (11–26).  11 is the default; change to avoid interference. */
#define ZB_CHANNEL              11

/* How often control_tick() is called, in milliseconds */
#define CONTROL_TICK_MS         1000

/* Maximum number of Zigbee end-devices tracked simultaneously */
#define ZB_MAX_DEVICES          16

/* ---- Device table -------------------------------------------------------- */

typedef struct {
    uint16_t short_addr;        /* 16-bit network address assigned on join    */
    uint8_t  endpoint;          /* Primary endpoint (default 1)               */
    bool     valid;             /* true = slot occupied                       */
} zb_device_t;

/*
 * Return the number of currently-tracked devices.
 */
uint8_t zb_device_count(void);

/*
 * Return a pointer to the device at join-order index idx (0-based).
 * Returns NULL if idx is out of range or slot is empty.
 */
const zb_device_t *zb_device_get(uint8_t idx);

/*
 * Look up a device by its 16-bit short address.
 * Returns NULL if not found.
 */
const zb_device_t *zb_device_find(uint16_t short_addr);

/* ---- On/Off cluster ------------------------------------------------------ */

/*
 * Send an On or Off command to a smart plug / switch.
 *   short_addr : target device's 16-bit network address
 *   endpoint   : target endpoint (use zb_device_t.endpoint)
 *   on         : true = ON, false = OFF
 */
void zb_send_onoff(uint16_t short_addr, uint8_t endpoint, bool on);

/* ---- Attribute read / configure reporting -------------------------------- */

/*
 * Request a single attribute value from a remote device.
 * The result will arrive asynchronously via control_on_attribute().
 */
void zb_read_attribute(uint16_t short_addr, uint8_t endpoint,
                       uint16_t cluster_id, uint16_t attr_id);

/*
 * Ask a device to automatically report an attribute at regular intervals.
 *   min_interval : minimum seconds between reports
 *   max_interval : maximum seconds between reports (report fires even if
 *                  value unchanged)
 * The reports arrive via control_on_attribute().
 */
void zb_configure_reporting(uint16_t short_addr, uint8_t endpoint,
                             uint16_t cluster_id, uint16_t attr_id,
                             uint16_t min_interval, uint16_t max_interval);

/* ---- Coordinator init (called from main.c) ------------------------------- */

/*
 * Start the Zigbee stack.  Call once from a dedicated RTOS task; this
 * function does not return (it enters esp_zb_stack_main_loop()).
 */
void zb_coordinator_start(void);

#ifdef __cplusplus
}
#endif
