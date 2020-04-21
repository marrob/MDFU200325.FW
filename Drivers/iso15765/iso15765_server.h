/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ISO15765__H_
#define _ISO15765__H_
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stringPlus.h"

#define ISO15765_DEBUG_LEVEL  0

/* Config define  -------------------------------------------------------------*/
#define ISO15765_BUFFER_SIZE      0xFF     /* ISO15765_BUFFER_SIZE Standard Size*/
#define ISO15765_TIMEOUT_MS       3000

/* Public define  -------------------------------------------------------------*/
#define ISO15765_OK                 0
#define ISO15765_FAIL               1
#define ISO15765_BUSY               2
#define ISO15765_TIMEOUT            3
#define ISO15765_BUS_LOST           4
#define ISO15765_UNHANDLED_NRC      5
#define ISO15765_BUFFER_TOO_SMALL   6

#define ISO15765_CAN_FRAME_SIZE   8
#define ISO15765_DEFAULT_FILL     0xFF

#define ISO15765_N_PCI_SF                0x00  /* Single Frame */
#define ISO15765_N_PCI_FF                0x10  /* First Frame  */
#define ISO15765_N_PCI_CF                0x20  /*Consecutive Frame*/
#define ISO15765_N_PCI_FC                0x30  /*Flow Control*/
#define ISO15765_NR_CODE                 0x7F  /*Negative Response Code*/
#define ISO15765_RC_POSITION             0x00  /*Response Code Position*/
#define ISO15765_RSID_POSITION           0x01  /*Response SID Position*/

#define ISO15765_NRC_POSITION                     0x02  //Negative Response Code Position
#define ISO15765_NRC_RESP_PENDING                 0x78  //requestCorrectlyReceived_ResponsePending
#define ISO15765_NRC_BUSY_REPEAT_REQUEST          0x21  //busyRepeatRequest
#define ISO15765_NRC_CONDITIONS_NOT_CORRECT       0x22  //conditionsNotCorrect
#define ISO15765_NRC_WRONG_REQ_SEQUENCE_NUMBER    0x24  //requestSequenceError, ezt a VU v�laszolja...
#define ISO15765_NRC_REQUEST_OUT_OF_RANGE         0x31  //requestOutOfRange
#define ISO15765_NRC_WRONG_BLOCK_SEQUENCE_NUMBER  0x73  //wrongBlockSequenceCounter, ezt a VU v�laszolja...
#define ISO15765_NRC_TRANSFER_DATA_SUSPENDED      0x71  //RequestDownloadnál ha nem jó a memorySize...
#define ISO15765_NRC_GENERAL_PROGRAMNING_FAILURE  0x72  //
#define ISO15765_NRC_WRONG_BLOCK_SEQUNCE_COUNTER  0x73


#define ISO15765_SF_DL_SIZE       1        /* Start Frame Data Length Size */
#define ISO15765_FF_DL_SIZE       2        /*First Frame Data Length Size */
#define ISO15765_SN_SIZE          1        /*Sequence Number Size */

/* Exported macro ------------------------------------------------------------*/
#if (ISO15765_DEBUG_LEVEL > 0)
#define  Iso15765UsrLog(...)    {printf("ISO15765:");\
                                 printf(__VA_ARGS__);\
                                 printf("\r\n");}
#else
#define Iso15765UsrLog(...)
#endif

#if (ISO15765_DEBUG_LEVEL > 1)

#define  Iso15765ErrLog(...)    {printf("ERROR.ISO15765:") ;\
                                 printf(__VA_ARGS__);\
                                 printf("\r\n");}
#else
#define Iso15765ErrLog(...)
#endif

#if (ISO15765_DEBUG_LEVEL > 2)
#define  Iso15765DbgLog(...)    {printf("DEBUG.ISO15765:") ;\
                                 printf(__VA_ARGS__);\
                                 printf("\r\n");}
#else
#define Iso15765DbgLog(...)
#endif

typedef struct _Iso15765Var_Type
{
    uint8_t Data[ISO15765_BUFFER_SIZE];
    int32_t  Size;
    int32_t  Offset;
    uint8_t BlockSize;
    uint8_t BlockCnt;
    uint8_t SeparationTime;
    uint8_t FrameCnt;
    int32_t Timestamp;
}Iso15765Var_Type;

typedef enum  _Iso15765State2_Type
{
    ST_ISO15765_START,
    ST_ISO15765_IDLE,

    ST_ISO15765_RX,
    ST_ISO15765_RX_SINGLE,
    ST_ISO15765_RX_FIRST,
    ST_ISO15765_TX_FLOWCTRL,
    ST_ISO15765_RX_CONSECUTIVE,
    ST_ISO15765_RX_DONE,
    ST_ISO15765_RX_ERROR,

    ST_ISO15765_TX,
    ST_ISO15765_TX_SINGLE,
    ST_ISO15765_TX_FIRST,
    ST_ISO15765_RX_FLOWCTRL,
    ST_ISO15765_TX_WAIT,
    ST_ISO15765_TX_CONSECUTIVE,
    ST_ISO15765_TX_ERROR,


    ST_ISO15765_COMPLETE,
    
}Iso15765State_Type;

typedef struct _Iso15765Handle_Type
{
    Iso15765Var_Type    Request;
    Iso15765Var_Type    Response;
    uint8_t             LastNegativeResponseCode;
    uint8_t             CurrentRetry;
    int32_t STminTimestamp;
    struct{
        Iso15765State_Type  Pre;
        Iso15765State_Type  Curr;
        Iso15765State_Type  Next;
    }State;

    struct{
       uint8_t Frame[ISO15765_CAN_FRAME_SIZE];
       int32_t Timestamp;
       volatile  uint8_t Ready;
    }Tx, Rx;
    
    struct{
        uint32_t ResponseSequneceErrorCnt;
        uint32_t RequestBlockSequneceErrorCnt; 
        uint32_t RequestSequneceErrorCnt;
        uint32_t ResponseConsecutiveTimeoutCnt;
        uint32_t TotalRetryCnt;
        uint32_t RequestRepeatCnt;
        uint32_t ResponsePendingCnt;
        uint32_t NotHandledNrcCnt;
    }Diag;
    
}Iso15765Handle_Type;

/* Exported functions --------------------------------------------------------*/
extern uint32_t HAL_GetTick(void);
uint8_t Iso15765ServerInit(Iso15765Handle_Type *hnd, uint8_t blockSize, uint8_t separationTime);

Iso15765State_Type Iso15765GetState(Iso15765Handle_Type *hnd);
uint8_t Iso15765GetStatus(Iso15765Handle_Type *hnd);
uint8_t Iso15765Task(Iso15765Handle_Type *hnd);

__weak uint8_t Iso15765BusReadCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size);
__weak uint8_t Iso15765FlushCallback(Iso15765Handle_Type *hnd);

__weak uint8_t Iso15765ReqRespCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size);

__weak uint8_t Iso15765BusWriteCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size);

uint8_t Iso15765IncomingStream(Iso15765Handle_Type *hnd, uint8_t *data, size_t size);
uint8_t Iso15765Response(Iso15765Handle_Type *hnd, uint8_t *data, size_t size);
uint8_t Iso15765PositiveResponse(Iso15765Handle_Type *hnd, uint8_t sid);
uint8_t Iso15765NegativeResponse(Iso15765Handle_Type *hnd, uint8_t sid, uint8_t nrc);


#endif //_ISO1576__H_
/******************* (C) COPYRIGHT 2011 marrob Design *****END OF FILE******/
