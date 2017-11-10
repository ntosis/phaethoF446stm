


/*
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "spi.h"

bool spi_TFT_busy_flag = 0;

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void spi_init()
{
  /* STM32L1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */

  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx; //SPI1
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = 0;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 10;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;



  SpiHandle.Init.Mode = SPI_MODE_MASTER;


  if (HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    // Error_Handler();
  }
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
void spi_send_U8(uint8_t data) {
    /* enable SPE */

    SPI1->CR1 |= SPI_CR1_SPE;

    /*send 8 bits*/
    *((__IO uint8_t *)&SPI1->DR) = data;
    /* wait until RXNE flag is set */
    if(SPI_WaitOnFlagUntilTimeout(&SpiHandle, SPI_FLAG_RXNE, RESET, 2) != HAL_OK)
     {
      return;
     }
     /* Disable SPE */
     SPI1->CR1 &= ~SPI_CR1_SPE;
}
void spi_send_U16(uint16_t data) {
    /*declare local variables*/

    uint8_t temp[2]={0};

    /* enable SPE */

    SPI1->CR1 |= SPI_CR1_SPE;

    /* high bits*/
    temp[0] = (uint8_t)((data>>8));
    /* low bits */
    temp[1] = (uint8_t)((data));

    /*send the high 8 bits*/
    *((__IO uint8_t *)&SPI1->DR) = temp[0];

    /* wait until RXNE flag is set */
    if(SPI_WaitOnFlagUntilTimeout(&SpiHandle, SPI_FLAG_RXNE, RESET, 2) != HAL_OK)
     {
      return;
     }

     /* send the low 8 bits */
     *((__IO uint8_t *)&SPI1->DR) = temp[1];

     /*Wait until RXNE flag is set */
     if(SPI_WaitOnFlagUntilTimeout(&SpiHandle, SPI_FLAG_RXNE, RESET, 2) != HAL_OK)
     {
      return;
     }
     /* Disable SPE */
     SPI1->CR1 &= ~SPI_CR1_SPE;
}
uint8_t spi_send_read_U8(uint8_t *send,uint8_t *receive)
{
    //HAL_SPI_TransmitReceive(&SpiHandle,send,receive,1,5);
    /* enable SPE */

    SPI1->CR1 |= SPI_CR1_SPE;
     /* Wait until TXE flag is set to send data*/
     if(SPI_WaitOnFlagUntilTimeout(&SpiHandle, SPI_FLAG_TXE, RESET, 2000) != HAL_OK)
     {
            return -1;
     }
    /* send 8 bits */
    *((__IO uint8_t *)&SPI1->DR) = *send;

    /* wait until RXNE flag is set */
    if(SPI_WaitOnFlagUntilTimeout(&SpiHandle, SPI_FLAG_RXNE, RESET, 2000) != HAL_OK)
     {
      return -1;
     }

    /* read the spi data */
    *receive = *((__IO uint8_t *)&SPI1->DR);

     /* Disable SPE */
     SPI1->CR1 &= ~SPI_CR1_SPE;

     /*return Rx data*/
    return (uint8_t) *receive;
}

void set_spi_high_speed(SPI_HandleTypeDef *hspi)
{

    /* Disable SPI */
    /* Clear SPE */
    SPI1->CR1 &= ~SPI_CR1_SPE;

    /* Set high Speed*/
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;

    /* Update register */
    hspi->Instance->CR1 = (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize |
                         hspi->Init.CLKPolarity | hspi->Init.CLKPhase | (hspi->Init.NSS & SPI_CR1_SSM) |
                         hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit  | hspi->Init.CRCCalculation);
}

void set_spi_low_speed(SPI_HandleTypeDef *hspi)
{
    /* Disable SPI */
    /* Clear SPE */
    SPI1->CR1 &= ~SPI_CR1_SPE;

    /* set low speed*/
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    /* Update register */
    hspi->Instance->CR1 = (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize |
                         hspi->Init.CLKPolarity | hspi->Init.CLKPhase | (hspi->Init.NSS & SPI_CR1_SSM) |
                         hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit  | hspi->Init.CRCCalculation);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED2 on: Transfer in transmission/reception process is correct */
  //BSP_LED_On(LED2);
}

HAL_StatusTypeDef SPI_WaitOnFlagUntilTimeout(struct __SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status, uint32_t Timeout)
{
  uint32_t tickstart = 0;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) == RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0) || ((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_SPI_GET_FLAG(hspi, Flag) != RESET)
    {
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0) || ((HAL_GetTick() - tickstart ) > Timeout))
        {
          /* Disable the SPI and reset the CRC: the CRC value should be cleared
             on both master and slave sides in order to resynchronize the master
             and slave for their respective CRC calculation */

          /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
          __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

          /* Reset CRC Calculation */
          if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          {
            SPI_RESET_CRC(hspi);
          }

          hspi->State= HAL_SPI_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hspi);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  Error_Handler();
  /* Turn LED2 on: Transfer error in reception/transmission process */
  //BSP_LED_On(LED2);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
    //* Toogle LED2 for error
    //BSP_LED_Toggle(LED2);
    //HAL_Delay(1000);
      return;
  }
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;



}
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    MX_USART2_UART_Init();
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }

}

