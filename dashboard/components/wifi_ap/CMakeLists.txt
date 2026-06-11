idf_component_register(SRCS "wifi_ap.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi nvs_flash esp_netif esp_event log)