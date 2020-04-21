/**
  ******************************************************************************
  * @file    LiveLed.h
  * @author  Margit R�bert
  * @version 0.0.1
  * @date    2019.11.19
  * @brief   LiveLed header-e
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LIVELED__H_
#define _LIVELED__H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Exported macro ------------------------------------------------------------*/
#define LIVELED_OK              0
#define LIVELED_FAIL            1

/* Exported types ------------------------------------------------------------*/ 
typedef struct _LiveLED_HnadleTypeDef
{
  int32_t     HalfPeriodTimeMs;        /*!< F�l periodusido taroloja        */
  void        (*LedOnFnPtr)   (void);  /*!< Bekapcolja a LED-et             */
  void        (*LedOffFnPtr)  (void);  /*!< Kikapcolja a LED-et             */
  int32_t     Timestamp;               /*!< Idobelyeg a Task-hoz            */
  uint8_t     State;                   /*!< LED �llapot t�rol�ja            */
}LiveLED_HnadleTypeDef;

/* Public functions ----------------------------------------------------------*/
extern uint32_t HAL_GetTick(void);
uint8_t LiveLedInit(LiveLED_HnadleTypeDef *hnd);
uint8_t LiveLedTask(LiveLED_HnadleTypeDef *hnd);
#endif //_LIVE_LEDDRV__H_
