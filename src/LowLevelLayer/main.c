/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Ctrl_Subsystem.h"
#include "guiLibs/FRAMEWIN.h"
#include "guiLibs/GUI.h"
#include "cmsis_os.h"
#include "eeprom_em.h"
#include "hardware_init.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include "queue.h"

/* USER CODE BEGIN Includes */
char Samples[5000]={0};
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId blinkTaskHandle;
osThreadId TaskHandle_10ms;
osThreadId TaskHandle_500ms;
osThreadId TaskHandle_300ms;
osSemaphoreId semHandle;
uint8_t testcnt=0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_CRC_Init(void);
void StartDefaultTask(void const * argument);

void Task_10ms(void const *argument);
void Task_100ms(void const *argument);
void Task_300ms(void const *argument);
void Task_500ms(void const *argument);
static void MX_TIM3_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
   /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  spi_init();
  MX_TIM3_Init();
  //xQueueCtrlSubsystem = xQueueCreate( 10, sizeof( struct AMessage * ) );
  EE_Init();
  initCAL();

  initctrlSystemQueue();
  initheatingSysQueue();

      /* Check the init flag in the back up register  */
  if(!(RTC->BKP0R)&1)
      {

      rtc_init();
      /* set the init flag to back up register*/
      (RTC->BKP0R)= 1;

      }

   __USART2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        asm("bkpt 255");
/*  spi_send_U8(0xAA);
  spi_send_U8(0x10);*/
   /* USER CODE BEGIN 2 */
  //GUI_TOUCH_X_MeasureX();
  GUI_Init();
  //WM_SetCreateFlags(WM_CF_MEMDEV);
  //GUI_CURSOR_Show();
  //GUI_CURSOR_Select(&GUI_CursorCrossL);
  /* USER CODE END 2 */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  //name, thread, priority, instances, stacksz
  /* Create the threads and semaphore */
  osThreadDef(task_300ms, Task_300ms, osPriorityNormal, 0, 128);
  TaskHandle_300ms = osThreadCreate(osThread(task_300ms), NULL);

 osThreadDef(task_10ms, Task_10ms, osPriorityNormal, 0, 128);
  TaskHandle_10ms = osThreadCreate(osThread(task_10ms), NULL);

  osThreadDef(task_500ms, Task_500ms, osPriorityAboveNormal, 0, 1024);
  TaskHandle_500ms = osThreadCreate(osThread(task_500ms), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}




/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Task_300ms(void const *argument)
{
    portTickType xLastWakeTime;
    const portTickType xDelay = 300 / portTICK_RATE_MS;
    // Initialise the xLastWakeTime variable with the current time.
         xLastWakeTime = xTaskGetTickCount ();
		while(1) {

		        //actualTemperature();
			//readButton(xTaskGetTickCount ());

			  // Wait for the next cycle.
			vTaskDelayUntil( &xLastWakeTime, xDelay );
		}
}

void Task_10ms(void const *argument)
    {
        portTickType xLastWakeTime;
        const portTickType xDelay = 10 / portTICK_RATE_MS;
        // Initialise the xLastWakeTime variable with the current time.
             xLastWakeTime = xTaskGetTickCount ();

    		while(1) {

    			 //readEncoder();
    			if((!__READ(TOUCH_INT))&&(!spi_TFT_busy_flag)){
    			   // testcnt++;
    			    // Touch Screen
    			    //GUI_TOUCH_Exec();
    			}
    			else {
    			    //do nothing
    			    //GUI_TOUCH_StoreState(-1,-1);
    		    }
    			  // Wait for the next cycle.
    		vTaskDelayUntil( &xLastWakeTime, xDelay );
    		}
    	}

void Task_500ms(void const *argument)
    {
        portTickType xLastWakeTime;
        const portTickType xDelay = 500 / portTICK_RATE_MS;
        uint8_t internCounter=0;
        char buffer[10]= {0};


         GUITask();
         GUI_Clear();
        //GUI_Exec();
        /*GUI_CURSOR_Show();
              GUI_CURSOR_Select(&GUI_CursorCrossL);
              GUI_SetBkColor(GUI_WHITE);
              GUI_SetColor(GUI_BLACK);
              GUI_Clear();
              GUI_DispString("Measurement of\nA/D converter values");
              */
        // Initialise the xLastWakeTime variable with the current time.
             xLastWakeTime = xTaskGetTickCount ();
    		while(1) {

    			 checkStruct();
    			 updateSollTemperature();

    			// volatile CAL_PARAM *gp = &CALinEE;
    			 //volatile uint8_t ii =  oneLevelSystem_C;
    			 heatingSysQueueSend();
    			 ctrlSystemQueueRead();
    			 Ctrl_Subsystem_step();
    			 ctrlSystemQueueSend();
    			 GUI_Exec();
    			 //run every 1 second
    			  if(internCounter==200) {

    				      //Ctrl_Subsystem_step();
    				      /*##-3- Display the updated Time and Date ################################*/
    				     // LED_Blinking((__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)))*10);

    				//      internCounter=0;
    					  //_exit();
    				     }

    			internCounter++;

    			  // Wait for the next cycle.
    			vTaskDelayUntil( &xLastWakeTime, xDelay );
    		}
    	}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}
/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* Enable CRC */
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN);

}
/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
void GUI_X_Unlock(){}
void GUI_X_Lock(){}
uint32_t GUI_X_GetTaskId(){}
void GUI_X_InitOS(){}
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
