/**
  ******************************************************************************
  * @file    LiveLed.c
  * @author  Margit R�bert
  * @version 0.0.1
  * @date    2019.11.19
  * @brief   Ez egy LED-et villogtat
 ********************************************************************************
Update:
- 2020.02.24: STM32CubeIDE Support...
 ********************************************************************************
*/
  
/* Includes ------------------------------------------------------------------*/
#include "LiveLed.h"

/* Private define ------------------------------------------------------------*/
uint8_t LiveLedInit(LiveLED_HnadleTypeDef *hnd)
{
  if(!hnd) 
      return LIVELED_FAIL;
  hnd->State = 1;
  hnd->Timestamp = HAL_GetTick();
  return LIVELED_OK;
}

uint8_t LiveLedTask(LiveLED_HnadleTypeDef *hnd)
{
  if(!hnd) 
    return LIVELED_FAIL;

  if(HAL_GetTick() - hnd->Timestamp > hnd->HalfPeriodTimeMs)
  {
    hnd->Timestamp = HAL_GetTick();
    if(hnd->State)
    {
      hnd->State = 0;
      hnd->LedOnFnPtr();
    }
    else
    {
      hnd->State = 1;
      hnd->LedOffFnPtr();
    }
  }
  return LIVELED_OK;
}
/************************ (C) COPYRIGHT Konvol�ci� **********END OF FILE****/
