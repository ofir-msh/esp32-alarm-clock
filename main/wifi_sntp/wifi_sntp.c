#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <esp_log.h>
#include "tasks_common.h"
#include "wifi_sntp.h"

const char *TAG = "WIFI_SNTP";

static void wifi_sntp_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WIFI EVENT STA START");
            // c8:f0:9e:a4:7a:88
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WIFI EVENT STA CONNECTED");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "WIFI EVENT STA DISCONNECTED");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "STA GOT IP");
            break;
        default:
            break;
        }
    }
}

void wifi_init()
{
    // esp_log_level_set("wifi", ESP_LOG_NONE);                          // disable internal wifi logging
    ESP_ERROR_CHECK(esp_netif_init());                                // initialize network interface
    ESP_ERROR_CHECK(esp_event_loop_create_default());                 // initialize default event loop
    esp_netif_create_default_wifi_sta();                              // create default handler config for station mode
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT(); // use default values for the init config
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));                // initialize wifi module

    esp_event_handler_instance_t instance_wifi_event, instance_ip_event; // register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_sntp_event_handler, NULL, &instance_wifi_event));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_sntp_event_handler, NULL, &instance_ip_event));

    // configure wifi connection credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));         // configure wifi stack to use ram
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));               // configure ESP32 in station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // bind credentails to station mode
    ESP_ERROR_CHECK(esp_wifi_start());                               // connect to AP specified in config
}

void sntp_init()
{
}

static void wifi_sntp_task(void *params)
{
    esp_err_t ret = nvs_flash_init(); // initialize NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init();
    // sntp_init();

    for (;;)
    {
        vTaskDelay(10000);
    }
}

void wifi_sntp_start()
{
    xTaskCreatePinnedToCore(&wifi_sntp_task, "wifi_sntp", WIFI_TASK_STACK_SIZE, NULL,
                            WIFI_TASK_PRIORITY, NULL, WIFI_TASK_CORE_ID);
}