/* USER CODE BEGIN Header */
/**
  *
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "LED.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#ifdef DEBUG
  #define DEVICE_DEBUG_LEVEL  3
#else
  #define DEVICE_DEBUG_LEVEL  2
#endif


#if (DEVICE_DEBUG_LEVEL > 0)
#define  DeviceUsrLog(...)  {printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceUsrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 1)

#define  DeviceErrLog(...)  {printf(VT100_ATTR_RED);\
                             printf("ERROR.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceErrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 2)
#define  DeviceDbgLog(...)  {printf(VT100_ATTR_YELLOW);\
                             printf("DEBUG.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceDbgLog(...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAIL_LED_Pin GPIO_PIN_1
#define FAIL_LED_GPIO_Port GPIOA
#define LIVE_LED_Pin GPIO_PIN_2
#define LIVE_LED_GPIO_Port GPIOA
#define DIP1_Pin GPIO_PIN_3
#define DIP1_GPIO_Port GPIOA
#define DIP2_Pin GPIO_PIN_4
#define DIP2_GPIO_Port GPIOA
#define DIP3_Pin GPIO_PIN_5
#define DIP3_GPIO_Port GPIOA
#define DIP4_Pin GPIO_PIN_6
#define DIP4_GPIO_Port GPIOA
#define DIP5_Pin GPIO_PIN_7
#define DIP5_GPIO_Port GPIOA
#define DIP6_Pin GPIO_PIN_0
#define DIP6_GPIO_Port GPIOB
#define DIP7_Pin GPIO_PIN_1
#define DIP7_GPIO_Port GPIOB
#define DIP8_Pin GPIO_PIN_2
#define DIP8_GPIO_Port GPIOB
void   MX_CAN_Init(void);
/* USER CODE BEGIN Private defines */

#define UDS_RX_ADDR         0x600
#define UDS_TX_ADDR         0x700
#define UDS_EXT_ADDR        0
#define DEVICE_WAIT_FOR_CLIENT_SEC  5

/*--- Flahs ---*/
//#define FLASH_PAGE_SIZE           0x800
/*Start user code address: ADDR_FLASH_PAGE_10*/
#define APP_FIRST_PAGE_ADDRESS    0x08005000
/*Start address of latest flash page: ADDR_FLASH_PAGE_511*/
#define APP_LAST_PAGE_ADDRESS     0x08010000 - FLASH_PAGE_SIZE

/* Generic  -------------------------------------------------------------------*/
#define DEVICE_NAME             "MDFU200325" /*!< Eszk�z neve pl.:MDIO130204*/
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               0x0002        /*Verziszam: 0x0302:V03.02 */
#define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
#define DEVICE_PCB              "00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "KONVOLUCIO"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)
#define DEVICE_OK               0
#define DEVICE_FAIL             1
#define DEVICE_FAIL_LED         0
/* Debug ---------------------------------------------------------------------*/

/* VT100 ---------------------------------------------------------------------*/
/*
 * https://www.csie.ntu.edu.tw/~r92094/c++/VT100.html
 * http://www.termsys.demon.co.uk/vtansi.htm
 */
#define VT100_CLEARSCREEN   "\033[2J"
#define VT100_CURSORHOME    "\033[H"
#define VT100_ATTR_RESET    "\033[0m"
#define VT100_ATTR_RED      "\033[31m"
#define VT100_ATTR_GREEN    "\033[32m"
#define VT100_ATTR_YELLOW   "\033[33m"
#define VT100_CUP(__v__,__h__)    ("\033["__v__";"__h__"H") /*Cursor Position*/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
