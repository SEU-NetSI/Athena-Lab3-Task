/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//========== 实验代码书写区开始 =============



//========== 实验代码书写区结束 =============

// Control whether the LED flashes
uint8_t work_flag = 1;
// Control the LED blinking cycle
uint32_t led_blink_timedelay = 1000;
// Serial port receive buffer structure
typedef struct {
  uint8_t buffer[256];
  uint8_t index;
}rxBuffer;
rxBuffer usart1_rx_buffer;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  //========= 实验代码书写区开始 =========



  //========= 实验代码书写区结束 =========
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	if (work_flag) {
	  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_9);
	}
	osDelay(led_blink_timedelay);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Usart_data_parse(uint8_t *data, uint16_t len)
{
  char* data_str = (char*)data;
  if(strcmp(data_str, "STOP") == 0){
    work_flag = 0;
    return ;
  }else if (strcmp(data_str, "START") == 0) {
    work_flag = 1;
    return ;
  }
  //========= 实验代码书写区开始 =========



  //========= 实验代码书写区结束 =========
}

void UartReadTask(void *argument)
{
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
  while (1) {
	//========= 实验代码书写区开始 =========
    if(LL_USART_IsActiveFlag_XXXX(USART1)) {
	//========= 实验代码书写区结束 =========
      uint8_t receivedChar = LL_USART_ReceiveData8(USART1);
      usart1_rx_buffer.buffer[usart1_rx_buffer.index++] = receivedChar;
      // 请在此处开始实现指令回显的功能(Instruction feedback)

    }
    //========= 实验代码书写区开始 =========
    if(LL_USART_IsActiveFlag_XXXX(USART1)) {
	//========= 实验代码书写区结束 =========
      LL_USART_ClearFlag_IDLE(USART1);
      usart1_rx_buffer.buffer[usart1_rx_buffer.index++] = '\0'; // Null-terminate the string
      Usart_data_parse(usart1_rx_buffer.buffer, usart1_rx_buffer.index);
      memset(usart1_rx_buffer.buffer, 0, sizeof(usart1_rx_buffer.buffer));
      usart1_rx_buffer.index = 0;
    }
  }
}
/* USER CODE END Application */

