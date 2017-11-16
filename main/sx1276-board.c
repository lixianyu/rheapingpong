/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
//#include "board.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

//#include <stdint.h>

#include "driver/spi_master.h"
//#include "esp_system.h"
#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "RheaPingPongConfig.h"

static const char* TAG = "sx1276-board";

//#define DIO0_INPUT_PIN_SEL  (1 << RHEA_LORA_DIO0)

xQueueHandle g_rhea_gpio_evt_queue;
spi_device_handle_t spi_handle;
static uint8_t g_shake_state = 0;
/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork
};

#if 0
/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchLf;
Gpio_t AntSwitchHf;
#endif

void SX1276IoInit( void )
{
    ESP_LOGI(TAG, "Enter %s", __func__);
    #if 0
    GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

    GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    #else
    gpio_pad_select_gpio(RHEA_LORA_RESET);
    //gpio_set_direction(RHEA_LORA_RESET, GPIO_MODE_OUTPUT);
    //gpio_set_level(RHEA_LORA_RESET, 1);
    #if 0
    gpio_pad_select_gpio(RHEA_LORA_DIO0);
    gpio_set_pull_mode(RHEA_LORA_DIO0, GPIO_PULLUP_ONLY);
    gpio_set_direction(RHEA_LORA_DIO0, GPIO_MODE_INPUT);
    
    gpio_pad_select_gpio(RHEA_LORA_DIO1);
    gpio_set_pull_mode(RHEA_LORA_DIO1, GPIO_PULLUP_ONLY);
    gpio_set_direction(RHEA_LORA_DIO1, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(RHEA_LORA_DIO2);
    gpio_set_pull_mode(RHEA_LORA_DIO2, GPIO_PULLUP_ONLY);
    gpio_set_direction(RHEA_LORA_DIO2, GPIO_MODE_INPUT);
    
    gpio_pad_select_gpio(RHEA_LORA_DIO3);
    gpio_set_pull_mode(RHEA_LORA_DIO3, GPIO_PULLUP_ONLY);
    gpio_set_direction(RHEA_LORA_DIO3, GPIO_MODE_INPUT);
    #endif
#if 0
    gpio_pad_select_gpio(RHEA_LORA_DIO4);
    gpio_set_direction(RHEA_LORA_DIO4, GPIO_MODE_INPUT);
    gpio_pad_select_gpio(RHEA_LORA_DIO5);
    gpio_set_direction(RHEA_LORA_DIO5, GPIO_MODE_INPUT);
#endif
    #endif
}

static void IRAM_ATTR rhea_gpio_isr_handler(void* arg)
{
    if (g_shake_state != 0) return;
    g_shake_state = 1;
    uint32_t gpio_num = (uint32_t) arg;
    gpio_isr_handler_remove(RHEA_LORA_DIO0);
    xQueueSendFromISR(g_rhea_gpio_evt_queue, &gpio_num, NULL);
}

static void rhea_Irq_task(void *pvParameters)
{
    ESP_LOGW(TAG, "Enter %s", __func__);
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(g_rhea_gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            ESP_LOGV(TAG, "io_num = %u", io_num);
            switch (io_num)
            {
                case RHEA_LORA_DIO0:
                    DioIrq[0]();
                    break;
                case RHEA_LORA_DIO1:
                    DioIrq[1]();
                    break;
                case RHEA_LORA_DIO2:
                    DioIrq[2]();
                    break;
                case RHEA_LORA_DIO3:
                    DioIrq[3]();
                    break;
#if 0
                case RHEA_LORA_DIO4:
                    DioIrq[4]();
                    break;
                case RHEA_LORA_DIO5:
                    DioIrq[5]();
                    break;
#endif
                default:
                    
                    break;
            }
            gpio_isr_handler_add(RHEA_LORA_DIO0, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO0);
            g_shake_state = 0;
        }
    }
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    ESP_LOGI(TAG, "Enter %s", __func__);
    #if 0
    GpioSetInterrupt( &SX1276.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0] );
    GpioSetInterrupt( &SX1276.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1] );
    GpioSetInterrupt( &SX1276.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2] );
    GpioSetInterrupt( &SX1276.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3] );
    GpioSetInterrupt( &SX1276.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4] );
    GpioSetInterrupt( &SX1276.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5] );
    #endif
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pin
    io_conf.pin_bit_mask = (1ULL << RHEA_LORA_DIO0);
    //io_conf.pin_bit_mask = DIO0_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    #if 0
    io_conf.pin_bit_mask = 1ULL << RHEA_LORA_DIO1;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1ULL << RHEA_LORA_DIO2;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1ULL << RHEA_LORA_DIO3;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1ULL << RHEA_LORA_DIO4;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1ULL << RHEA_LORA_DIO5;
    gpio_config(&io_conf);
    #endif
    //create a queue to handle gpio event from isr
    g_rhea_gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(rhea_Irq_task, "IrqTask", 4096, NULL, 9, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RHEA_LORA_DIO0, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO0);
    #if 0
    gpio_isr_handler_add(RHEA_LORA_DIO1, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO1);
    gpio_isr_handler_add(RHEA_LORA_DIO2, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO2);
    gpio_isr_handler_add(RHEA_LORA_DIO3, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO3);
    #endif
#if 0
    gpio_isr_handler_add(RHEA_LORA_DIO4, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO4);
    gpio_isr_handler_add(RHEA_LORA_DIO5, rhea_gpio_isr_handler, (void*) RHEA_LORA_DIO5);
#endif
}

void SX1276IoDeInit( void )
{
    ESP_LOGW(TAG, "Enter %s", __func__);
    #if 0
    GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    #endif
}

void SX1276SetRfTxPower( int8_t power )
{
    ESP_LOGW(TAG, "Enter %s", __func__);
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        ESP_LOGV(TAG, "...................01");
        if( power > 17 )
        {
            ESP_LOGV(TAG, "...................02");
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            ESP_LOGV(TAG, "...................03");
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            ESP_LOGV(TAG, "...................04");
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            ESP_LOGV(TAG, "...................05");
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        ESP_LOGV(TAG, "...................06");
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
    ESP_LOGW(TAG, "Leave %s", __func__);
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    ESP_LOGW(TAG, "Enter %s, channel=%d", __func__, channel);
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    //ESP_LOGW(TAG, "Enter %s", __func__);
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1276AntSwInit( );
        }
        else
        {
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    //ESP_LOGW(TAG, "Enter %s", __func__);
    #if 0
    GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    #endif
}

void SX1276AntSwDeInit( void )
{
    //ESP_LOGW(TAG, "Enter %s", __func__);
    #if 0
    GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    #endif
}

void SX1276SetAntSw( uint8_t opMode )
{
    //ESP_LOGW(TAG, "Enter %s", __func__);
    #if 0
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        GpioWrite( &AntSwitchLf, 0 );
        GpioWrite( &AntSwitchHf, 1 );
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        GpioWrite( &AntSwitchLf, 1 );
        GpioWrite( &AntSwitchHf, 0 );
        break;
    }
    #endif
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    ESP_LOGW(TAG, "Enter %s", __func__);
    // Implement check. Currently all frequencies are supported
    return true;
}


///////////////////////////////////////////////////////////////////////////////////////////
void BoardInitPeriph( void )
{
    ESP_LOGI(TAG, "Enter %s", __func__);
    gpio_pad_select_gpio(LED_BLUE);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BLUE, LED_OFF);
}

void BoardInitMcu( void )
{
    ESP_LOGI(TAG, "Enter %s", __func__);
    // SPI init.
    esp_err_t ret;
	spi_bus_config_t buscfg;
	spi_device_interface_config_t devcfg;
	memset(&buscfg, 0, sizeof(buscfg));
	memset(&devcfg, 0, sizeof(devcfg));
	buscfg.miso_io_num = RHEA_LORA_MISO;
	buscfg.mosi_io_num = RHEA_LORA_MOSI;
	buscfg.sclk_io_num = RHEA_LORA_CLK;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;

	devcfg.clock_speed_hz = 8000000;
	devcfg.command_bits = 0;
	devcfg.mode = 0;
	devcfg.spics_io_num = RHEA_LORA_SS;
	devcfg.queue_size = 8;
    // Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret == ESP_OK)
    {
		ESP_LOGI(TAG, "spi_bus_initialize sucess.");
    }
	else
	{
		ESP_LOGE(TAG, "spi_bus_initialize fail.");
	}
    // Attach the SX1278/1276 to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    if (ret == ESP_OK)
    {
		ESP_LOGI(TAG, "spi_bus_add_device sucess.");
    }
	else
	{
		ESP_LOGI(TAG, "spi_bus_add_device fail.");
	}
    
    SX1276IoInit();
}
