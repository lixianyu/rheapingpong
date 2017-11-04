#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "RheaPingPongConfig.h"

static const char *TAG_RHEA = "Rhea";

static void dumpBytes(const uint8_t *data, size_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (i % 16 == 0)
        {
            printf("%08x    ", i);
        }
        printf("0x%02x, ", data[i]);
        if ((i + 1) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("\r\n");
}

#ifndef RHEA_CLOSE_WIFI
static void rhea_wifi_event_str(system_event_id_t evt_id)
{
    char buf[64] = {0};
    switch (evt_id)
    {
    case SYSTEM_EVENT_WIFI_READY:           /**< ESP32 WiFi ready */
        strcpy(buf, "SYSTEM_EVENT_WIFI_READY");
        break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
        strcpy(buf, "SYSTEM_EVENT_SCAN_DONE");
        break;
    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
        strcpy(buf, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
        strcpy(buf, "SYSTEM_EVENT_STA_STOP");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
        strcpy(buf, "SYSTEM_EVENT_STA_CONNECTED");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
        strcpy(buf, "SYSTEM_EVENT_STA_DISCONNECTED");
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
        strcpy(buf, "SYSTEM_EVENT_STA_AUTHMODE_CHANGE");
        break;
    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
        strcpy(buf, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
        strcpy(buf, "SYSTEM_EVENT_STA_WPS_ER_SUCCESS");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
        strcpy(buf, "SYSTEM_EVENT_STA_WPS_ER_FAILED");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
        strcpy(buf, "SYSTEM_EVENT_STA_WPS_ER_TIMEOUT");
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
        strcpy(buf, "SYSTEM_EVENT_STA_WPS_ER_PIN");
        break;
    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
        strcpy(buf, "SYSTEM_EVENT_AP_START");
        break;
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
        strcpy(buf, "SYSTEM_EVENT_AP_STOP");
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
        strcpy(buf, "SYSTEM_EVENT_AP_STACONNECTED");
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
        strcpy(buf, "SYSTEM_EVENT_AP_STADISCONNECTED");
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
        strcpy(buf, "SYSTEM_EVENT_AP_PROBEREQRECVED");
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
        strcpy(buf, "SYSTEM_EVENT_AP_STA_GOT_IP6");
        break;
    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
        strcpy(buf, "SYSTEM_EVENT_ETH_START");
        break;
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
        strcpy(buf, "SYSTEM_EVENT_ETH_STOP");
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
        strcpy(buf, "SYSTEM_EVENT_ETH_CONNECTED");
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
        strcpy(buf, "SYSTEM_EVENT_ETH_DISCONNECTED");
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
        strcpy(buf, "SYSTEM_EVENT_ETH_GOT_IP");
        break;
    case SYSTEM_EVENT_MAX:
        strcpy(buf, "SYSTEM_EVENT_MAX");
        break;
    default:
        strcpy(buf, "OH, what event?!");
        break;
    }
    ESP_LOGW(TAG_RHEA, "event_id = %s", buf);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    ESP_LOGW(TAG_RHEA, "Enter %s(), event_id=%d", __func__, event->event_id);
    rhea_wifi_event_str(event->event_id);

    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_STOP:
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        ESP_LOGV(TAG_RHEA, "authmode=%d, ssid=%s, channel=%d", event->event_info.connected.authmode, event->event_info.connected.ssid, event->event_info.connected.channel);
        dumpBytes(event->event_info.connected.bssid, 6);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_AP_START:
        break;
    case SYSTEM_EVENT_ETH_START:
        break;
    case SYSTEM_EVENT_ETH_STOP:
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        break;
    default:
        break;
    }
    return ESP_OK;
}
#endif

static void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        gpio_set_level(BLINK_GPIO, LED_ON);
        vTaskDelay(94 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, LED_OFF);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGV(TAG_RHEA, "Enter %s", __func__);
    nvs_flash_init();
#ifndef RHEA_CLOSE_WIFI
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config =
    {
        .sta = {
            .ssid = "TYG",
            .password = "ooo1230",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
#endif
    xTaskCreate(blink_task, "blink_task", 8192, NULL, 15, NULL);

    int counts = 0;
    while (true)
    {
        ESP_LOGI(TAG_RHEA, "freememory = %d", esp_get_free_heap_size());
        printf("count %d\r\n", counts++);
        vTaskDelay(5004 / portTICK_PERIOD_MS);
    }
}

