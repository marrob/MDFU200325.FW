/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  Update: 2020.02.27 First
  ******************************************************************************
  */
#include "common.h"
/* Delay ---------------------------------------------------------------------*/
/**
  * @brief  DelayMs
  */
void DelayMs(volatile int32_t n)
{
  unsigned long timetick;
  timetick = HAL_GetTick();
  while ((HAL_GetTick() - timetick) < n);
}

/**
  * @brief  Delay Us
  */
void DelayUs(volatile int32_t us)
{  //practical limit of 25,000us
  us = us * 168 / 4;
  while (us)
  {
    us--;
  }
}
