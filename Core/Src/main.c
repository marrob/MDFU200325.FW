/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "LiveLed.h"
#include "iso15765_server.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _DebugStateTypeDef
{
  DBG_NONE,         //0

}DebugStateTypeDef;

typedef struct _CanBusSpeedType
{
  uint32_t Baud;
  uint8_t Pre;
  uint32_t BS1;
  uint32_t BS2;
  uint32_t SJW;
}CanBusSpeedTypeDef;

typedef struct _AppTypeDef
{
  uint8_t Address;
  uint16_t Version;
  Iso15765Handle_Type   Transport;
  DebugStateTypeDef DebugState;
  CanBusSpeedTypeDef *CanSpeed;

  struct _Dfu
  {
    uint32_t MemorySize;
    uint32_t FlashBytePtr;
    uint8_t CurrentSession;
    uint8_t BlockSequenceCounter;
    uint8_t TesterPresentFlag;
    uint8_t HardResetFlag;
  }Dfu;

  struct _coutners
  {
    uint32_t CanRxFrames;
  }Counters;

}DeviceTypeDef;

typedef  void (*pFunction)(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DeviceTypeDef Device;
LiveLED_HnadleTypeDef hLiveLed;
LedHandle_Type        hLed;
uint8_t SerialSendEnable = 0;
char StringBuffer[80];
CanBusSpeedTypeDef CanSpeeds[] =
/*  Baud,       Div,  BS1,            BS2,            JSW         */
{
   {  50000,    60,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
   { 100000,    30,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
   { 125000,    24,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
   { 250000,    12,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void LiveLedOff(void);
void LiveLedOn(void);
void FeilLedOn(void);
void FailLedOff(void);

void DebugPrint(char *str);
void ResetTask(void);

uint8_t GetAddress(void);
uint8_t GetSpeed(void);
static void CanInit(CanBusSpeedTypeDef *speed);
void Reset(void);
uint8_t Iso15765ReqRespCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size);
uint8_t FlashErase(void);
uint8_t FlashWrite(uint8_t *src, uint8_t *dest, uint32_t size);
uint16_t CalcCrc16Ansi(uint16_t initValue, const void* address, size_t size);
LedItem_Type LedList[1] = {
  { DEVICE_FAIL_LED,  &FeilLedOn,   &FailLedOff, },
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* UDS -----------------------------------------------------------------------*/
/**
  * @brief Request->Response Callback
  */
uint8_t Iso15765ReqRespCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{

#define UDS_SID_DIAG_CTRL                 0x10 /*DiagSesssionControl*/
#define UDS_SID_ECU_RESET                 0x11 /*Ecu Reset*/
#define UDS_SID_DID                       0x22 /*ReadDtaByIdentifier*/
#define UDS_SID_RUTINE_CTRL               0x31 /*RutineControl*/
#define UDS_SID_REQUEST_DOWNLOAD          0x34 /*RequestDownload */
#define UDS_SID_DATA_TRANSFER             0x36 /*TransferData*/
#define UDS_SID_REQUEST_TRANSFER_EXIT     0x37 /*RequestTransferExit*/
#define UDS_SID_TESTER_PRESENT            0x3E /*TesterPresent*/

#define ROUTINE_ERASE_MEM       0x01
#define ROUTINE_CHEKSUM         0x02

#define UDS_MODE_NORMAL         0x01
#define UDS_MODE_PROGRAMMING    0x02

  uint8_t sid = data[0];

  #define DID_TEST    0x0001

  switch(sid)
  {
    /*** DiagSesssionControl ***/
    case UDS_SID_DIAG_CTRL:
    {
      Device.Dfu.CurrentSession = data[1];
      if(Device.Dfu.CurrentSession == UDS_MODE_PROGRAMMING)
      {
        uint8_t resp[] = {UDS_SID_DIAG_CTRL + 0x40, Device.Dfu.CurrentSession };
        Iso15765Response(hnd, resp, sizeof(resp));
        HAL_FLASH_Unlock();
        Device.Dfu.FlashBytePtr = 0;
        Device.Dfu.BlockSequenceCounter = 0;
        DeviceDbgLog("DIAG_CTRL:Programming mode");
      }
      else if (Device.Dfu.CurrentSession == UDS_MODE_NORMAL)
      {

      }
      else
      {
        DeviceErrLog("UDS_SID_DIAG_CTRL-ISO15765_NRC_CONDITIONS_NOT_CORRECT[%s:%d]",__FILE__,__LINE__);
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
      }
      break;
    }
    /*** ReadDtaByIdentifier ***/
    case UDS_SID_DID:
    {
      DeviceDbgLog("UDS_SID_DID, Reqtuest Length %d", size);
      uint8_t did_msb = data[1];
      uint8_t did_lsb = data[2];
      uint16_t did =  did_msb << 8 | did_lsb;
      switch(did)
      {
        case DID_TEST:
        {
          uint8_t msbValue = 0x00;
          uint8_t lsbValue = 0x10;
          uint8_t temp[] = {UDS_SID_DID + 0x40, data[1], data[2], msbValue, lsbValue};
          Iso15765Response(hnd, temp, sizeof(temp));
          break;
        }
        default:
        {
          Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
        }
        break;
      }
      break;
    }
    /*** Ecu Reset ***/
    case UDS_SID_ECU_RESET:
    {
      if(data[1] == 0x01)
      { /* Hard Reset*/
        Device.Dfu.HardResetFlag = 1;
        DeviceUsrLog("UDS_ECU_RESET:HardReset");
        Device.Dfu.HardResetFlag = 1;
        uint8_t resp[] = {UDS_SID_ECU_RESET };
        Iso15765Response(hnd, resp, sizeof(resp));
      }
      else
      {
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT );
      }
      break;
    }
    /*** RutineControl ***/
    case UDS_SID_RUTINE_CTRL:
    {
      /*--- Flash Erase ---*/
      if(data[1] == 0x01 && data[2] == 0xFF)
      {
        uint8_t routine = data[3];
        if(routine == ROUTINE_ERASE_MEM)
        {
          uint8_t resp[] = {UDS_SID_RUTINE_CTRL + 0x40, routine };
          Iso15765Response(hnd, resp, sizeof(resp));
          FlashErase();
          DeviceUsrLog("RUTINE_CTRL:Erase memory complete... Free Flash: %ul bytes", APP_LAST_PAGE_ADDRESS - APP_FIRST_PAGE_ADDRESS);
        }
        else if(routine == ROUTINE_CHEKSUM)
        {
          #if (DEVICE_DEBUG_LEVEL > 0)
          uint32_t timestamp = HAL_GetTick();
          #endif
          DeviceUsrLog("RUTINE_CTRL:Checksum is calculating... please wait...");
          uint16_t crc = 0;
          crc = CalcCrc16Ansi(0, (uint8_t*)APP_FIRST_PAGE_ADDRESS, Device.Dfu.MemorySize);
          DeviceUsrLog("RUTINE_CTRL:Calc complete. Value:0x%04X, Elapsded: %lu ms", crc, HAL_GetTick() - timestamp)
          uint8_t resp[] = {UDS_SID_RUTINE_CTRL + 0x40, routine, crc, crc >>8 };
          Iso15765Response(hnd, resp, sizeof(resp));
          DeviceDbgLog("RUTINE_CTRL:Getchecksum");
        }
        else
        {
          DeviceErrLog("RUTINE_CTRL.ISO15765_NRC_CONDITIONS_NOT_CORRECT[%s:%d]",__FILE__,__LINE__);
          Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
        }
      }
      else
      {
        DeviceErrLog("RUTINE_CTRL.ISO15765_NRC_CONDITIONS_NOT_CORRECT[%s:%d]",__FILE__,__LINE__);
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT);
      }
      break;
    }
    /*** RequestDownload ***/
    case UDS_SID_REQUEST_DOWNLOAD:
    {
      if(size != 11)
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT );
      Device.Dfu.MemorySize = *((uint32_t*)(data + 7));
      DeviceUsrLog("REQUEST_DOWNLOAD:MemorySize: I will get:%lu bytes.", Device.Dfu.MemorySize);
      /*A -1-el kényszeritem a klienst hogy csak paros 255 - 2(SID, Counter) = 252 hogy csak paros szamu bajtot kuldjon*/
      uint8_t resp[] = {UDS_SID_REQUEST_DOWNLOAD + 0x40, 0x10, ISO15765_BUFFER_SIZE - 1 };
      Iso15765Response(hnd, resp, sizeof(resp));
      break;
    }
    /*** Tester Present ***/
    case UDS_SID_TESTER_PRESENT:
    {
      uint8_t resp[] = {UDS_SID_TESTER_PRESENT + 0x40 };
      Iso15765Response(hnd, resp, sizeof(resp));
      Device.Dfu.TesterPresentFlag = 5;
      DeviceUsrLog("TESTER_PRESENT.");
      break;
    }
    /*** Tester Present ***/
    case UDS_SID_DATA_TRANSFER:
    {
      uint8_t blockSequenceCounter = data[1];
      Device.Dfu.BlockSequenceCounter++;
      if(Device.Dfu.BlockSequenceCounter != blockSequenceCounter)
      {
        DeviceErrLog("DATA_TRANSFER.ISO15765_NRC_WRONG_BLOCK_SEQUNCE_COUNTER.[%s:%d]",__FILE__,__LINE__);
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_WRONG_BLOCK_SEQUNCE_COUNTER);
      }

      uint8_t blockSize = size - 2 ; /*SID + BlocSequenceCounter*/
      uint8_t blockOffset = 2; /*SID + BlocSequenceCounter*/;
      /*Ez azért kell, mert +2-offszettel nem lehet olvasni a memóriát...*/
      uint8_t block[ISO15765_BUFFER_SIZE];
      memcpy(block, data + blockOffset, blockSize );
      DeviceUsrLog("DATA_TRANSFER:Seq.Counter:0x%02X, Size:%d bytes, blockSize:%d bytes.", Device.Dfu.BlockSequenceCounter, size, blockSize );

      if(blockSize % 2)
      { /*bájt szelesseget nem tudok irni  */
        DeviceErrLog("DATA_TRANSFER.ISO15765_NRC_TRANSFER_DATA_SUSPENDED.[%s:%d]",__FILE__,__LINE__);
        Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_TRANSFER_DATA_SUSPENDED);
      }
      int srcPtr = 0;
      do
      {
        if(blockSize - srcPtr >= 8)
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_FIRST_PAGE_ADDRESS + Device.Dfu.FlashBytePtr, *(uint64_t*)(block + srcPtr)) != HAL_OK)
            {
              DeviceErrLog("DATA_TRANSFER.ISO15765_NRC_GENERAL_PROGRAMNING_FAILURE.[%s:%d]",__FILE__,__LINE__);
              Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_GENERAL_PROGRAMNING_FAILURE);
            }
            DeviceDbgLog("DATA_TRANSFER:Write: %lu byte complete...", Device.Dfu.FlashBytePtr );
            Device.Dfu.FlashBytePtr += 8;
            srcPtr +=8;
        }
        else
        {
          if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, APP_FIRST_PAGE_ADDRESS + Device.Dfu.FlashBytePtr, *(uint16_t*)(block + srcPtr)) != HAL_OK)
          {
            DeviceErrLog("DATA_TRANSFER.ISO15765_NRC_GENERAL_PROGRAMNING_FAILURE.[%s:%d]",__FILE__,__LINE__);
            Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_GENERAL_PROGRAMNING_FAILURE);
          }
          DeviceDbgLog("DATA_TRANSFER:Write: %lu byte complete...", Device.Dfu.FlashBytePtr );
          Device.Dfu.FlashBytePtr += 2;
          srcPtr +=2;
        }
      }while(srcPtr != blockSize);


      uint8_t resp[] = {UDS_SID_DATA_TRANSFER + 0x40, Device.Dfu.BlockSequenceCounter };
      Iso15765Response(hnd, resp, sizeof(resp));
      break;
    }
    /*** RequestTransferExit***/
    case UDS_SID_REQUEST_TRANSFER_EXIT:
    {
      HAL_FLASH_Lock();
      Device.Dfu.CurrentSession = UDS_MODE_NORMAL;
      Device.Dfu.FlashBytePtr = 0;
      Device.Dfu.BlockSequenceCounter = 0;
      uint8_t resp[] = {UDS_SID_REQUEST_TRANSFER_EXIT + 0x40 };
      Iso15765Response(hnd, resp, sizeof(resp));
      DeviceUsrLog("REQUEST_TRANSFER_EXIT.");
      break;
    }

    /*** Unknown ***/
    default:
    {
       Iso15765NegativeResponse(hnd, sid, ISO15765_NRC_CONDITIONS_NOT_CORRECT );
    }
  }
  return ISO15765_OK;
}
/* UDS -----------------------------------------------------------------------*/
/**
  * @brief  Bus Write Callback
  */
uint8_t Iso15765BusWriteCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{
  CAN_TxHeaderTypeDef   txHeader;
  uint32_t              txMailbox;

  txHeader.RTR = CAN_RTR_DATA;
  #if (UDS_EXT_ADDR)
    txHeader.IDE = CAN_ID_EXT;
    txHeader.ExtId = UDS_TX_ADDR | Device.Address;
  #else
    txHeader.IDE = CAN_ID_STD;
    txHeader.StdId = UDS_TX_ADDR | Device.Address;
  #endif
  txHeader.DLC = 8;
  if(HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_AddTxMessage[%s:%d]",__FILE__,__LINE__);
  }

  return ISO15765_OK;
}

/**
  * @brief  Rx Fifo 0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef   rxHeader;
  uint8_t               data[8];

  Device.Counters.CanRxFrames++;

  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_GetRxMessage[%s:%d]",__FILE__,__LINE__);
  }
  #if (UDS_EXT_ADDR)
  if ((rxHeader.ExtId == (UDS_RX_ADDR | Device.Address)) && (rxHeader.DLC == 8))
  {
    Iso15765IncomingStream(&Device.Transport, data, sizeof(data));
  }
  #else
  if ((rxHeader.StdId == (UDS_RX_ADDR | Device.Address)) && (rxHeader.DLC == 8))
  {
    Iso15765IncomingStream(&Device.Transport, data, sizeof(data));
  }
  #endif
}

void ResetTask(void)
{
  static uint8_t preResetFlag = 0;
  static uint32_t timestamp = 0;

  if(preResetFlag != Device.Dfu.HardResetFlag)
  {
    preResetFlag = Device.Dfu.HardResetFlag;
    timestamp = HAL_GetTick();
  }

  if(Device.Dfu.HardResetFlag)
  {
    if(HAL_GetTick() - timestamp > 1000)
    {
      Device.Dfu.HardResetFlag = 0;
      preResetFlag = 0;
      timestamp = HAL_GetTick();
      NVIC_SystemReset();
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

#ifdef DEBUG
  printf(VT100_ATTR_RED);
    DeviceUsrLog("This is a DEBUG version.");
  printf(VT100_ATTR_RESET);
#endif

  DeviceUsrLog("Name:%s", DEVICE_NAME);
  DeviceUsrLog("Version:%04X", DEVICE_FW);
  DeviceUsrLog("HAL_RCC_GetPCLK2Freq: %dHz",(int)HAL_RCC_GetPCLK2Freq());
  DeviceUsrLog("HAL_RCC_GetPCLK1Freq: %dHz",(int)HAL_RCC_GetPCLK1Freq());
  DeviceUsrLog("SystemCoreClock: %dHz",(int)SystemCoreClock);

  /*** Leds ***/
  hLed.pLedTable = LedList;
  hLed.Records = sizeof(LedList)/sizeof(LedItem_Type);
  LedInit(&hLed);
  LedOff(&hLed, DEVICE_FAIL_LED);

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 100;
  LiveLedInit(&hLiveLed);


  /*** Defaults ***/
  Reset();
  Device.Address = GetAddress();
  DeviceUsrLog("Address: %d", Device.Address);
  Device.Version = DEVICE_FW;
  Device.CanSpeed = &CanSpeeds[GetSpeed()];
  DeviceUsrLog("CAN Baudrate: %lu Baud", CanSpeeds[GetSpeed()].Baud);
  DeviceUsrLog("ECU DFU Rx address: 0x%04X", UDS_RX_ADDR | Device.Address);
  DeviceUsrLog("ECU DFU Tx address: 0x%04X", UDS_TX_ADDR | Device.Address);
  CanInit(Device.CanSpeed);

  Device.Dfu.HardResetFlag = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    LiveLedTask(&hLiveLed);
    LedTask(&hLed);
    Iso15765Task(&Device.Transport);
    ResetTask();
    /* USER CODE END WHILE */

    static uint32_t timestamp = 0;
    static uint8_t counter = DEVICE_WAIT_FOR_CLIENT_SEC;
    if( HAL_GetTick() - timestamp > 1000 && !Device.Dfu.TesterPresentFlag)
    {
      timestamp = HAL_GetTick();
      counter--;

      DeviceUsrLog("Wait for a client... %d",  counter);
      if(!counter)
      {
        if (((*(__IO uint32_t*)APP_FIRST_PAGE_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
          DeviceDbgLog("I found a valid firmware!");

          //IWatchdogDeInit();
          HAL_DeInit();
          HAL_UART_MspDeInit(&huart1);
          HAL_CAN_MspDeInit(&hcan);

          uint32_t appAddress = *(__IO uint32_t*) (APP_FIRST_PAGE_ADDRESS + 4);
          /* Jump to user application */
          pFunction pApp = (pFunction) appAddress;
          /* Initialize user application's Stack Pointer */
          __set_MSP(*(__IO uint32_t*) APP_FIRST_PAGE_ADDRESS);

          pApp();
        }
        else
        {
          DeviceUsrLog("There is nothing");
          counter = DEVICE_WAIT_FOR_CLIENT_SEC;
        }
      }



    }
    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 40;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  HAL_CAN_MspDeInit(&hcan);
  /* FIGYELEM NEM EZT AZ INITET HASZNALOD, a CUBE ezt mindig letrehozza!!! */
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAIL_LED_GPIO_Port, FAIL_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FAIL_LED_Pin */
  GPIO_InitStruct.Pin = FAIL_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAIL_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP1_Pin DIP2_Pin DIP3_Pin DIP4_Pin 
                           DIP5_Pin */
  GPIO_InitStruct.Pin = DIP1_Pin|DIP2_Pin|DIP3_Pin|DIP4_Pin 
                          |DIP5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP6_Pin DIP7_Pin DIP8_Pin */
  GPIO_InitStruct.Pin = DIP6_Pin|DIP7_Pin|DIP8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void CanInit(CanBusSpeedTypeDef *speed)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = speed->Pre;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = speed->SJW;
  hcan.Init.TimeSeg1 = speed->BS1;
  hcan.Init.TimeSeg2 = speed->BS2;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
   Error_Handler();
  }

  /*** Filter Init***/
  CAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
   DeviceErrLog("HAL_CAN_ConfigFilter[%s:%d]",__FILE__,__LINE__);
  }

  /*** Start the CAN peripheral ***/
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
   DeviceErrLog("HAL_CAN_Start[%s:%d]",__FILE__,__LINE__);
  }

  /*** Activate CAN RX notification ***/
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
   DeviceErrLog("HAL_CAN_ActivateNotification[%s:%d]",__FILE__,__LINE__);
  }
}

/* Flahs -------------------------------------------------------------------*/
uint8_t FlashErase(void)
{
  uint32_t NbOfPages = 0;
  uint32_t PageError = 0;
  /* Variable contains Flash operation status */
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef eraseinitstruct;

  /* Get the number of sector to erase from 1st sector*/
  NbOfPages = ((APP_LAST_PAGE_ADDRESS - APP_FIRST_PAGE_ADDRESS) / FLASH_PAGE_SIZE) + 1;
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.PageAddress = APP_FIRST_PAGE_ADDRESS;
  eraseinitstruct.NbPages = NbOfPages;
  status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);
  return status;
}

/*****************************************************************
CRC-16-ANSI
Least significant bit first (little-endian)
x16 + x15 + x2 + 1 => 0x8005
Test:
0x48,0x65,0x6C,0x6C,0x6F,0x20,0x57,0x6F,0x72,0x6C,0x64 -> CRC:0x70C3
uint8_t data[]= "Hello World";
uint16_t crc = CalcCrc16(0, data, sizeof(data) - 1);
if(crc == 0x70C3)
    printf("PASSED.");
*****************************************************************/
uint16_t CalcCrc16Ansi(uint16_t initValue, const void* address, size_t size)
{
  uint16_t remainder = initValue;
  uint16_t polynomial = 0x8005;
  uint8_t *_address = (uint8_t*)address;

  for (size_t i = 0; i < size; ++i)
  {
    remainder ^= (_address[i] << 8);
    for (uint8_t bit = 8; bit > 0; --bit)
    {
      if (remainder & 0x8000)
          remainder = (remainder << 1) ^ polynomial;
      else
          remainder = (remainder << 1);
    }
  }
  return (remainder);
}

/* printf -------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  /*
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_Sen dChar((*ptr++));
  */
  return len;
}

/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

void FeilLedOn(void)
{
  HAL_GPIO_WritePin(FAIL_LED_GPIO_Port, FAIL_LED_Pin, GPIO_PIN_RESET);
}
void FailLedOff(void)
{
  HAL_GPIO_WritePin(FAIL_LED_GPIO_Port, FAIL_LED_Pin, GPIO_PIN_SET);
}

/* Address Switch -----------------------------------------------------------*/
uint8_t GetAddress(void)
{
  uint8_t val = 0;
  (HAL_GPIO_ReadPin(DIP3_GPIO_Port,DIP3_Pin) == GPIO_PIN_SET)? (val&=~0x01): (val|=0x01);
  (HAL_GPIO_ReadPin(DIP4_GPIO_Port,DIP4_Pin) == GPIO_PIN_SET)? (val &=~0x02):(val|=0x02);
  (HAL_GPIO_ReadPin(DIP5_GPIO_Port,DIP5_Pin) == GPIO_PIN_SET)? (val &=~0x04):(val|=0x04);
  (HAL_GPIO_ReadPin(DIP6_GPIO_Port,DIP6_Pin) == GPIO_PIN_SET)? (val &=~0x08):(val|=0x08);
  (HAL_GPIO_ReadPin(DIP7_GPIO_Port,DIP7_Pin) == GPIO_PIN_SET)? (val &=~0x10):(val|=0x10);
  (HAL_GPIO_ReadPin(DIP8_GPIO_Port,DIP8_Pin) == GPIO_PIN_SET)? (val &=~0x20):(val|=0x20);

  return val;
}

uint8_t GetSpeed(void)
{
  uint8_t val = 0;
  (HAL_GPIO_ReadPin(DIP3_GPIO_Port,DIP1_Pin) == GPIO_PIN_SET)? (val&=~0x01): (val|=0x01);
  (HAL_GPIO_ReadPin(DIP4_GPIO_Port,DIP2_Pin) == GPIO_PIN_SET)? (val &=~0x02):(val|=0x02);
  return val;
}

void Reset(void)
{
  Device.Counters.CanRxFrames = 0;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
