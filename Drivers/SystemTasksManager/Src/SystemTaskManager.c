/* ===Kisarazu RBKN Library===
 *
 * autor          : Oishi
 * version        : v0.10
 * last update    : 20160703
 *
 * **overview***
 * システムのタスクを記述
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "SystemTaskManager.h"
#include "MW_USART.h"
#include "MW_I2C.h"
#include "MW_GPIO.h"
#include "DD_Gene.h"
#include "message.h"
#include "app.h"
#include "DD_RC.h" 
#include "MW_IWDG.h"

volatile uint32_t g_SY_system_counter;
volatile uint8_t g_rc_data[RC_DATA_NUM] = {};
static uint8_t rc_rcv[RC_DATA_NUM] = {};
volatile led_mode_t g_led_mode = lmode_1;
static volatile unsigned int count_for_rc = 0;

volatile uint8_t raspi_control_rcv[8] = {0,0,0,0,0,0,0,0};
static uint8_t raspi_control_rcv_data[8] = {0,0,0,0,0,0,0,0};
static bool raspi_control_flag = true;


static
int SY_init(void);
static
int SY_I2CConnTest(int timeout);
static
int SY_doAppTasks(void);
static
int SY_clockInit(void);
static
void SY_GPIOInit(void);


#if !_NO_DEVICE
static
int SY_doDevDriverTasks(void);
#endif

int main(void){
  int ret,i,j;

  //システムを初期化します
  ret = SY_init();
  if( ret ){
    message("err", "initialize Faild%d", ret);
    MW_waitForMessageTransitionComplete(100);
    return EXIT_FAILURE;
  }
  //(未実装)I2Cデバイスのチェックをします
  ret = SY_I2CConnTest(10);
  if( ret ){
    message("err", "I2CConnectionTest Faild%d", ret);
    MW_waitForMessageTransitionComplete(100);
    return EXIT_FAILURE;
  }

  //ここから開始します。
  g_SY_system_counter = 0;

  message("msg", "start!!\n");
  MW_printf("\033[2J\033[1;1H");
  flush();

  //アプリケーションを開始するためのループです。
  while( 1 ){
    //ウォッチドックタイマーをリセットします
    MW_IWDGClr();//reset counter of watch dog  

    //個々のアプリケーションの実行をします。
    SY_doAppTasks();
    //もしメッセージを出すタイミングであれば
    if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
#if USE_RASPI_CONTROL
      if(raspi_control_flag){
        MW_USART2ReceiveMult(8, raspi_control_rcv_data);
        raspi_control_flag = false;
      }
	    for(int i=0;i<8;i++){
	      MW_printf("[%3d]",raspi_control_rcv_data[i]);
	    }
      MW_printf("\n");
	    for(int i=0;i<8;i++){
	      raspi_control_rcv[i] = raspi_control_rcv_data[i];
	    }
#endif
      if( g_SY_system_counter % 1000 == 0){
	      MW_printf("\033[2J");
      }
      MW_printf("\033[1;1H");//カーソルを(1,1)にセットして
      DD_RCPrint((uint8_t*)g_rc_data);//RCのハンドラを表示します
      DD_print();//各デバイスハンドラを表示します
      flush(); /* out message. */
    }
    //タイミング待ちを行います
    while( g_SY_system_counter % _INTERVAL_MS != _INTERVAL_MS / 2 - 1 ){
    }
#if !_NO_DEVICE
    //デバイスがあれば、各デバイスタスクを実行します。これはハンドラに格納されているデータをMDに転送する内容などが含まれます。
#if DD_NUM_OF_LD
    if(LED_OFF_SW()){
      for(i=0;i<DD_NUM_OF_LD;i++){
	for(j=0;j<8;j++){
	  g_ld_h[i].mode[j] = D_LMOD_NONE;
	}
      }
    }
#endif
      ret = SY_doDevDriverTasks();
#endif
    //エラー処理です
    if( ret ){
      message("err", "Device Driver Tasks Faild%d", ret);
#if DD_NUM_OF_LD
      for(i=0;i<DD_NUM_OF_LD;i++){
	for(j=0;j<8;j++){
	  g_ld_h[i].mode[j] = D_LMOD_BLINK_RED;
	}
	DD_I2C1Send(g_ld_h[i].add, g_ld_h[i].mode, 8);
      }
#endif
      return EXIT_FAILURE;
    }
    //タイミング待ちを行います
    while( g_SY_system_counter % _INTERVAL_MS != 0 ){
    }
    //もし一定時間以上応答がない場合はRCが切断されたとみなし、リセットをかけます。
#if DD_USE_RC
    count_for_rc++;
    if(count_for_rc >= 20){
      message("err","RC disconnected!");
#if DD_NUM_OF_LD
      for(i=0;i<DD_NUM_OF_LD;i++){
	for(j=0;j<8;j++){
	  g_ld_h[i].mode[j] = D_LMOD_BLINK_RED;
	}
	DD_I2C1Send(g_ld_h[i].add, g_ld_h[i].mode, 8);
      }
#endif
      while(1);
    }
#endif
  }
} /* main */

void SY_wait(int ms){
  volatile uint32_t time;
  time = g_SY_system_counter;
  while(time + ms > g_SY_system_counter)
    MW_IWDGClr();//reset counter of watch dog
  MW_IWDGClr();//reset counter of watch dog
}

static
int SY_doAppTasks(void){
  return appTask();
}

#if !_NO_DEVICE
static
int SY_doDevDriverTasks(void){
  return DD_doTasks();
}
#endif

static
int SY_I2CConnTest(int timeout){
  UNUSED(timeout);
  return EXIT_SUCCESS;
}

static
int SY_init(void){
  int ret,i,j;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
  **/
  if( HAL_Init()){
    return EXIT_FAILURE;
  }

  ret = SY_clockInit();
  /* Configure the system clock */
  if( ret ){
    return EXIT_FAILURE;
  }

  /*UART initialize*/
  MW_USARTInit(USART2ID);

#if USE_RASPI_CONTROL
  MW_USARTSetBaudRate(USART3ID, 9600);
  MW_USARTInit(USART3ID);
#endif
  /*Initialize printf null transit*/
  flush();
#if !_NO_DEVICE
  ret = DD_initialize();
  if(ret){
    return ret;
  }
#endif

  /*Initialize GPIO*/
  SY_GPIOInit();

  appInit();
  
#if DD_USE_RC
  message("msg", "wait for RC connection...");
#if DD_NUM_OF_LD
  for(i=0;i<DD_NUM_OF_LD;i++){
    for(j=0;j<8;j++){
      g_ld_h[i].mode[j] = D_LMOD_BLINK_RED;
    }
    DD_I2C1Send(g_ld_h[i].add, g_ld_h[i].mode, 8);
  }
#endif
  if( DD_RCInit((uint8_t*)g_rc_data, 100000) ){
    message("err", "RC initialize faild!\n");
#if DD_NUM_OF_LD
    for(i=0;i<DD_NUM_OF_LD;i++){
      for(j=0;j<8;j++){
	g_ld_h[i].mode[j] = D_LMOD_BLINK_RED;
      }
      DD_I2C1Send(g_ld_h[i].add, g_ld_h[i].mode, 8);
    }
#endif
    return EXIT_FAILURE;
  }
#if DD_NUM_OF_LD
  for(i=0;i<DD_NUM_OF_LD;i++){
    for(j=0;j<8;j++){
      g_ld_h[i].mode[j] = D_LMOD_DIMING_BLUE;
    }
    DD_I2C1Send(g_ld_h[i].add, g_ld_h[i].mode, 8);
  }
#endif
  message("msg", "RC connected sucess");
#endif
  
  /*initialize IWDG*/
  message("msg", "IWDG initialize");
  MW_SetIWDGPrescaler(IWDG_PRESCALER_16);//clock 40kHz --> 1/16 -->2500Hz
  MW_SetIWDGReload(250);//Reload volue is 250. reset time(100ms)
  ret = MW_IWDGInit(); 
  if(ret){
    message("err", "IWDG initialize failed!\n");
    return ret;
  }
  
  appInit();

  return EXIT_SUCCESS;
} /* SY_init */

/**
   oscの設定を行います。
 */
static
int SY_clockInit(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  return EXIT_SUCCESS;
} /* SY_clockInit */

static
void SY_GPIOInit(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC8 PC9 
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12 
                           PB14 PB15 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
  UNUSED(UartHandle);
#if DD_USE_RC
  if(DD_RCTask(rc_rcv, (uint8_t*)g_rc_data)!=0)message("err","rc err");
  count_for_rc = 0;
#endif
#if USE_RASPI_CONTROL
  raspi_control_flag = true;
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){
  UNUSED(UartHandle);
  MW_messageTransitionCompletedCallBack();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
  UNUSED(hi2c);
  MW_I2C2TransitionCompletedCallBack();
  DD_receive2SS();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
  UNUSED(hi2c);
  MW_I2C2ReceptionCompletedCallBack();
  DD_receive2SS();
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}
