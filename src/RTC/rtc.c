/*
 * rtc.c
 *
 *  Created on: Dec 4, 2017
 *      Author: unix
 */
#include "rtc.h"

uint8_t seconds=0;
struct timeStruct_t tm;
const char *monthName = {
 "Jan", "Feb", "Mar", "Apr", "May", "Jun",
 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

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
  getDate(__DATE__);
  getTime(__TIME__);

  sTime.Hours = tm.Hour;
  sTime.Minutes = tm.Minute;
  sTime.Seconds = tm.Second;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = tm.Month;
  sDate.Date = tm.Day;
  sDate.Year = tm.Year;

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
   tm.Second = sTime.Seconds;
   tm.Hour = sTime.Hours;
   tm.Minute = sTime.Minutes;
   /*sprintf(buff,"sec: %d\r\n",(uint32_t)(hrtc.Instance->TR));
   HAL_UART_Transmit(&huart2,buff,sizeof(buff),5);*/
}
bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
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

