#ifndef __SYSTEM_TASKS_MANAGER_H
#define __SYSTEM_TASKS_MANAGER_H

#include <stdint.h>
#include "DD_RC.h"
#define _INTERVAL_MS 10
#define _MESSAGE_INTERVAL_MS 100

typedef enum{
  lmode_wait_for_rc = 0,
  lmode_1 = 1,
  lmode_2 = 2,
  lmode_3 = 3,
} led_mode_t;

extern volatile uint32_t g_SY_system_counter;
extern volatile uint8_t g_rc_data[RC_DATA_NUM];
extern volatile led_mode_t g_led_mode;
static volatile unsigned int count_for_rc;
extern volatile uint8_t raspi_control_rcv[8];

void Error_Handler(void);

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

#define _SCR_CURSOR_SET(x,y) MW_printf("\033[%d;%dH",(int)(y)+1,(int)(x)+1)
#define _SCR_CLEAR() MW_printf("\033[2J")

#define _MAX(x,y) ((x)>(y)?(x):(y))
#define _MIN(x,y) ((x)<(y)?(x):(y))

void SY_wait(int ms);

#endif
