/******************** (C) COPYRIGHT 2015 marrob Design *************************
* File Name          : LedDrv.c
* Author             : Margit R�bert
* Date First Issued  : 2014.05.10
* Description        : Generic LED Drive: On,Off,Blink 
********************************************************************************/
/********************************************************************************
Update: 
- 2015.03.25:  Blink bekapcsol�sakor vizsg�lja az eloz� allapotot, ha
               blink volt el�tte is, akkor nem bantja az idoziteseket
- 2016.11.15: Jav�tva a Timestamp t�pusa uint_16-r�l int32_t-re v�ltozott.
- 2018.12.20: Comments update
- 2020.02.24: STM32CubeIDE Support...
********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#include "Led.h"


/**
  * @brief  Led Init
  * @param hnd: Handle
  */
uint8_t LedInit(LedHandle_Type *hnd)
{
    if(!hnd) 
        return LED_FAIL;
    return LED_OK;
}

/**
  * @brief  Led Blink
  * @param hnd: Handle
  */
uint8_t LedBlink(LedHandle_Type *hnd, uint8_t id, uint16_t periodTime)
{
    if(!hnd) 
        return LED_FAIL;

    if(hnd->pLedTable[id].State.Pre != LED_STATE_BLINK)
    {
        hnd->pLedTable[id].Timestamp = HAL_GetTick();
        hnd->pLedTable[id].State.Curr = LED_STATE_BLINK;
    }
    hnd->pLedTable[id].PeriodTime = periodTime;
    hnd->pLedTable[id].Pwm = 0;
    return LED_OK;
}

/**
  * @brief  LedBlinkPwm
  * @param hnd: Handle
  */
uint8_t LedBlinkPwm(LedHandle_Type *hnd, uint8_t id, uint16_t periodTime, uint8_t pwm)
{
    if(!hnd) 
        return LED_FAIL;

    if(hnd->pLedTable[id].State.Pre != LED_STATE_BLINK)
    {
        hnd->pLedTable[id].Timestamp = HAL_GetTick();
        hnd->pLedTable[id].State.Curr = LED_STATE_BLINK;
    }
    hnd->pLedTable[id].PeriodTime = periodTime;
    hnd->pLedTable[id].Pwm = pwm;
    return LED_OK;
}

/**
  * @brief  Led On
  * @param hnd: Handle
  */
uint8_t LedOn(LedHandle_Type *hnd, uint8_t id)
{
    if(!hnd) 
        return LED_FAIL;
    hnd->pLedTable[id].State.Curr = LED_STATE_ON;
    return LED_OK;
}

/**
  * @brief  Led Off
  * @param hnd: Handle
  */
uint8_t LedOff(LedHandle_Type *hnd, uint8_t id)
{
    if(!hnd) 
        return LED_FAIL;
    hnd->pLedTable[id].State.Curr = LED_STATE_OFF;
    return LED_OK;
}

/**
  * @brief  LED Task
  * @param hnd: Handle
  */
uint8_t LedTask(LedHandle_Type *hnd)
{
    if(!hnd) 
        return LED_FAIL;

    for(uint8_t i = 0; i< hnd->Records; i++)
    {
        switch (hnd->pLedTable[i].State.Curr)
        {
            case LED_STATE_IDLE:
            {
                break;
            }
            case LED_STATE_ON:
            {
                if(hnd->pLedTable[i].State.Pre != LED_STATE_ON)
                {
                    hnd->pLedTable[i].LEDOn();
                    hnd->pLedTable[i].State.Next = LED_STATE_ON;
                }
                break;
            }
            case LED_STATE_OFF:
            {
                if(hnd->pLedTable[i].State.Pre != LED_STATE_OFF)
                {
                    hnd->pLedTable[i].LEDOff();
                    hnd->pLedTable[i].State.Next = LED_STATE_OFF;
                }
                break;
            }
            case LED_STATE_BLINK:
            {
                if(hnd->pLedTable[i].State.Pre != LED_STATE_BLINK)
                {
                    hnd->pLedTable[i].Timestamp = HAL_GetTick();
                    hnd->pLedTable[i].State.Next = LED_STATE_BLINK;
                }
                if((HAL_GetTick()- hnd->pLedTable[i].Timestamp) > hnd->pLedTable[i].PeriodTime)
                {
                    hnd->pLedTable[i].Timestamp = HAL_GetTick();
                    if(hnd->pLedTable[i].BlinkFlag)
                    {
                        if(hnd->pLedTable[i].Pwm != 0)
                        {
                            hnd->pLedTable[i].PwmValue++;
                            if( hnd->pLedTable[i].PwmValue > hnd->pLedTable[i].Pwm)
                            {
                                hnd->pLedTable[i].PwmValue = 0;
                                hnd->pLedTable[i].LEDOn();
                                hnd->pLedTable[i].BlinkFlag = 0;
                            }
                        }
                        else
                        {
                            hnd->pLedTable[i].LEDOn();
                            hnd->pLedTable[i].BlinkFlag = 0;
                        }
                    }
                    else
                    {
                        hnd->pLedTable[i].LEDOff();
                        hnd->pLedTable[i].BlinkFlag = 1;
                    }
                }
            break;
            }
        }
        hnd->pLedTable[i].State.Pre = hnd->pLedTable[i].State.Curr;
        hnd->pLedTable[i].State.Curr = hnd->pLedTable[i].State.Next;
    }
return LED_OK;
}
/******************* (C) COPYRIGHT 2015 marrob Design *****END OF FILE******/
