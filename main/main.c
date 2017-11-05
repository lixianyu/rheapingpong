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
#include "sx1276.h"
#include "radio.h"

static const char *TAG_RHEA = "Rhea";

//#define RF_FREQUENCY                                470000000 // Hz
#define RF_FREQUENCY                                433000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm
#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                50000     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000 // ms
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

//States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

xQueueHandle g_led_toggle_queue;
xQueueHandle g_pingpang_queue;

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

void DelayMs( uint32_t ms )
{
    uint64_t startTick = system_get_rtc_time();
    uint64_t us = ms * 1000;
    while( ( system_get_rtc_time() - startTick ) < us );
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
static void rhea_monitor_task(void *pvParameters)
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    vTaskDelay(10004 / portTICK_PERIOD_MS);
    while (true)
    {
        uint8_t reg = 0x78;
        SX1276Read(REG_LR_VERSION, &reg);
        ESP_LOGD(TAG, "REG_LR_VERSION=0x%02X, freememory=%d", reg, esp_get_free_heap_size());
        vTaskDelay(5004 / portTICK_PERIOD_MS);
    }
}

static void led_Toggle_task(void *pvParameters)
{
    uint8_t led;
    uint8_t time;
    g_led_toggle_queue = xQueueCreate(10, sizeof(uint8_t));
    while (1)
    {
        if (xQueueReceive(g_led_toggle_queue, &time, portMAX_DELAY))
        {
            ESP_LOGI(TAG_RHEA, "led_Toggle_task:%d", time);
            gpio_set_level(LED_BLUE, LED_ON);
            if (time == 0)
            {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (time == 1)
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(LED_BLUE, LED_OFF);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(LED_BLUE, LED_ON);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            else if (time == 2)
            {
                vTaskDelay(700 / portTICK_PERIOD_MS);
            }
            else if (time == 3)
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(LED_BLUE, LED_OFF);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(LED_BLUE, LED_ON);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            gpio_set_level(LED_BLUE, LED_OFF);
        }        
    }
}

static void LedToggle(uint8_t led, uint8_t time)
{
    xQueueSend(g_led_toggle_queue, &time, 0);
}

static void LedOff(uint8_t led)
{
    gpio_set_level(led, LED_OFF);
}

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone( void )
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    Radio.Sleep( );
    States_t state = TX;
    xQueueSend(g_pingpang_queue, &state, 0);
}

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
static void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    States_t state = RX;
    xQueueSend(g_pingpang_queue, &state, 0);
}

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
static void OnTxTimeout( void )
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    Radio.Sleep( );
    States_t state = TX_TIMEOUT;
    xQueueSend(g_pingpang_queue, &state, 0);
}

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
static void OnRxTimeout( void )
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    Radio.Sleep( );
    States_t state = RX_TIMEOUT;
    xQueueSend(g_pingpang_queue, &state, 0);
}

/*!
 * \brief Function executed on Radio Rx Error event
 */
static void OnRxError( void )
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    Radio.Sleep( );
    States_t state = RX_ERROR;
    xQueueSend(g_pingpang_queue, &state, 0);
}

static void ping_pong_task(void *pvParameter)
{
    ESP_LOGW(TAG_RHEA, "Enter %s", __func__);
    bool isMaster = true;
    uint8_t i;
    States_t State = LOWPOWER;
    
    vTaskDelay(1004 / portTICK_PERIOD_MS);

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx( RX_TIMEOUT_VALUE );
    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PONG
                        //GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
                        LedToggle(LED_BLUE, 0);

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        //GpioWrite( &Led2, 1 ); // Set LED off
                        LedOff(LED_BLUE);
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        //GpioWrite( &Led1, GpioRead( &Led1 ) ^ 1 );
                        LedToggle(2);

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            //GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );
            LedToggle(1);
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        //TimerLowPowerHandler( );
        xQueueReceive(g_led_toggle_queue, &State, portMAX_DELAY)
        ESP_LOGD(TAG_RHEA, "State = %d", State);
    }
}

void app_main(void)
{
    ESP_LOGW(TAG_RHEA, "Enter %s, sw version: %s, freememory:%d", __func__, RHEA_PING_PONG_SW_VERSION, esp_get_free_heap_size());
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
    xTaskCreate(led_Toggle_task, "led_task", 2048, NULL, 1, NULL);
    g_pingpang_queue = xQueueCreate(10, sizeof(States_t));
    xTaskCreate(ping_pong_task, "ping_pong", 8192, NULL, 15, NULL);
    xTaskCreate(rhea_monitor_task, "rhea_monitor", 2048, NULL, 2, NULL);

    ESP_LOGW(TAG_RHEA, "Leave %s", __func__);
}

