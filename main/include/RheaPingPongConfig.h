#ifndef __RHEA_CONFIG_LOCAL_H__
#define __RHEA_CONFIG_LOCAL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define RHEA_CLOSE_WIFI

#define RHEA_PING_PONG_SW_VERSION "0.0.B04.00"

#define LED_ON    1
#define LED_OFF   0
#define LED_BLUE     GPIO_NUM_25

// For Lora gpio.
#define RHEA_LORA_MISO GPIO_NUM_34 // VDET_1
#define RHEA_LORA_MOSI GPIO_NUM_17
#define RHEA_LORA_CLK  GPIO_NUM_14 // MTMS
#define RHEA_LORA_SS   GPIO_NUM_33 // 32K_XN

#define RHEA_LORA_RESET (GPIO_NUM_16)

#define RHEA_LORA_DIO0 GPIO_NUM_35 // VDET_2
#define RHEA_LORA_DIO1 GPIO_NUM_21
#define RHEA_LORA_DIO2 GPIO_NUM_22
#define RHEA_LORA_DIO3 GPIO_NUM_26
#define RHEA_LORA_DIO4 GPIO_NUM_19
#define RHEA_LORA_DIO5 GPIO_NUM_27


/* Strapping Pins
 GPIO_NUM_12 (MTDI): internal pull-down
 GPIO_NUM_0 : internal pull-up
 GPIO_NUM_2 : internal pull-down
 GPIO_NUM_15 (MTDO): internal pull-up
 GPIO_NUM_5 : internal pull-up
 */
 
#ifdef __cplusplus
}
#endif
#endif

