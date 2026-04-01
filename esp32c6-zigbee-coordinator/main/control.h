#pragma once

/*
 * control.h
 *
 * Interface between the Zigbee coordinator layer and your application logic.
 *
 * YOU EDIT control.c FOR EACH PROJECT.
 * This header defines the callbacks that zb_coordinator.c will call — do not
 * change the signatures here.
 *
 * All callbacks fire within the Zigbee task context, so it is safe to call
 * any zb_coordinator API (zb_send_onoff, zb_read_attribute, etc.) from them.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_zigbee_core.h"
#include "zb_coordinator.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * control_init()
 *
 * Called once after the Zigbee network is formed and the coordinator is ready.
 * Use this to set initial state, start timers, etc.
 * Devices have NOT joined yet at this point.
 */
void control_init(void);

/*
 * control_tick()
 *
 * Called every CONTROL_TICK_MS milliseconds (default: 1 s) from a scheduler
 * alarm inside the Zigbee task.  This is your main control loop.
 *
 * Typical uses:
 *   - Check elapsed timers and send On/Off commands
 *   - Evaluate thresholds and actuate outputs
 *   - Poll device attributes at a lower rate than the tick
 */
void control_tick(void);

/*
 * control_on_device_joined()
 *
 * Called when a new Zigbee device joins the network.
 * Use this to configure attribute reporting on the device, or to record
 * which join-order slot maps to which physical device.
 *
 *   dev : pointer to the new device's entry in the device table
 *         (dev->short_addr, dev->endpoint)
 */
void control_on_device_joined(const zb_device_t *dev);

/*
 * control_on_attribute()
 *
 * Called when an attribute report or read-attribute response arrives from
 * any device.  Use this to capture sensor data (temperature, occupancy, etc.)
 * into your application state variables.
 *
 *   short_addr : source device's 16-bit network address
 *   endpoint   : source endpoint
 *   cluster_id : ZCL cluster (e.g. ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT)
 *   attr_id    : attribute within the cluster
 *   data_type  : ZCL data type tag
 *   data       : pointer to the raw attribute value — cast per data_type:
 *                  int16_t  for S16 (temperature: value / 100.0 = °C)
 *                  uint8_t  for BOOL / U8
 *                  uint16_t for U16
 */
void control_on_attribute(uint16_t short_addr, uint8_t endpoint,
                          uint16_t cluster_id, uint16_t attr_id,
                          esp_zb_zcl_attr_type_t data_type, void *data);

#ifdef __cplusplus
}
#endif
