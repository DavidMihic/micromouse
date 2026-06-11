#include <stdio.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_ap.h"
#include "uart_link.h"
#include "web_server.h"

static const char *TAG = "main";

void app_main(void)
{
    // NVS is required by the Wi-Fi stack
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_ap_init();     // bring up the access point
    uart_link_init();   // start reading telemetry from the STM32
    web_server_start(); // serve the dashboard

    ESP_LOGI(TAG, "Dashboard ready. Connect to the AP and open http://192.168.4.1");
}