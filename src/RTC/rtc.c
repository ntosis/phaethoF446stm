/*
 * rtc.c
 *
 *  Created on: Dec 4, 2017
 *      Author: unix
 */
#include "rtc.h"

    uint8_t seconds=0;

void rtc_init(void) {


  /*Local variables */
  RTC_HandleTypeDef hrtc;
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* Enable RTC */
  LL_RCC_EnableRTC();

  /**Initialize RTC and set the Time and Date */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 249;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  sTime.Hours = 1;
  sTime.Minutes = 1;
  sTime.Seconds = 1;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 2017;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /* Exit Initialization mode */
  WRITE_REG(RTC->ISR, (uint32_t)~RTC_ISR_INIT);
}
void Show_RTC_Calendar(void) {

   RTC_TimeTypeDef sTime;
   RTC_DateTypeDef sDate;
   RTC_HandleTypeDef hrtc;
   char buff[50];
   hrtc.Instance = RTC;
   HAL_RTC_GetTime(&hrtc,&sTime,FORMAT_BIN);
   HAL_RTC_GetDate(&hrtc,&sDate,FORMAT_BIN);
   seconds = sTime.Seconds;
   /*sprintf(buff,"sec: %d\r\n",(uint32_t)(hrtc.Instance->TR));
   HAL_UART_Transmit(&huart2,buff,sizeof(buff),5);*/
}
/*RTC_HandleTypeDef hrtc;

   Enable RTC
  LL_RCC_EnableRTC();
   Disable the write protection for RTC registers
  hrtc.Instance->WPR = 0xCAU;
  hrtc.Instance->WPR  = 0x53U;

   Set the Initialization mode
    hrtc.Instance->ISR = (uint32_t)RTC_INIT_MASK;

     Wait till RTC is in INIT state and if Time out is reached exit
    while((hrtc.Instance->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
    {

    } Configure the RTC PRER
    hrtc.Instance->PRER = (uint32_t)(255);
    hrtc.Instance->PRER |= (uint32_t)(127 << 16U);
    Set the RTC_TR register
    hrtc.Instance->TR = (uint32_t)(0 & RTC_TR_RESERVED_MASK);
    CLEAR_BIT(RTC->CR, RTC_CR_FMT);
    CLEAR_BIT(RTC->ISR, RTC_ISR_INIT);

    hrtc.Instance->WPR  = 0xFFU;*/

