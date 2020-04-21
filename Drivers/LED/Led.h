/******************** (C) COPYRIGHT 2015 marrob Design *************************
* File Name          : LedDrv.c
* Author             : Margit R�bert
* Date First Issued  : 10/05/2014
* Description        : Generic LED Drive: On,Off,Blink 
********************************************************************************/
#ifndef _LED__H_ 
#define _LED__H_ 1

/* Define to prevent recursive inclusion -------------------------------------*/
#include <stdint.h>


/* Exported macro ------------------------------------------------------------*/
#define LED_OK              0
#define LED_FAIL            1
#define LED_CONFIG_ERROR    2
#define LED_HANDLE_ERROR    3

typedef enum _LedState_Type
{
    LED_STATE_IDLE,
    LED_STATE_ON,
    LED_STATE_OFF,
    LED_STATE_BLINK
}LedState_Type;

typedef struct _LedItem_Type
{   
    uint8_t Id;
    void   (*LEDOn)(void);
    void   (*LEDOff)(void);

    int32_t PeriodTime;
    int32_t Timestamp;
    uint8_t  BlinkFlag;
    uint8_t  Pwm;
    uint8_t  PwmValue;
    struct
    {
        LedState_Type Pre;
        LedState_Type Curr;
        LedState_Type Next;
    }State;
}LedItem_Type;

typedef struct _LedHandle_Type
{
    LedItem_Type    *pLedTable;
    uint8_t          Enabled;
    uint8_t          Records;
    uint8_t          IsPause;
    uint8_t          IsConfigured;
}LedHandle_Type;

/* Public functions ----------------------------------------------------------*/
extern uint32_t HAL_GetTick(void);
uint8_t LedInit(LedHandle_Type *hnd);
uint8_t LedBlink(LedHandle_Type *hnd, uint8_t id, uint16_t periodTime);
uint8_t LedBlinkPwm(LedHandle_Type *hnd, uint8_t id, uint16_t periodTime, uint8_t pwm);
uint8_t LedOn(LedHandle_Type *hnd, uint8_t id);
uint8_t LedOff(LedHandle_Type *hnd, uint8_t id);
uint8_t LedTask(LedHandle_Type *hnd);


#endif //_LEDDRV__H_
/******************* (C) COPYRIGHT 2015 marrob Design *****END OF FILE******/
