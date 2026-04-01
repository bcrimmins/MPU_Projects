/*
 * main.c — Entry point for the ESP32-C6 standalone Zigbee coordinator.
 *
 * Initializes NVS and starts the Zigbee coordinator task.
 * All Zigbee stack and control logic runs inside that task.
 * To implement your application, edit control.c.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "zb_coordinator.h"

static const char *TAG = "MAIN";

static void zigbee_task(void *arg)
{
    zb_coordinator_start();   /* does not return */
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 Zigbee Coordinator starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS truncated — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Zigbee stack must run on a dedicated task */
    xTaskCreate(zigbee_task, "zigbee", 8192, NULL, 5, NULL);
}
