/******************** (C) COPYRIGHT 2020 marrob Design *************************
* File Name          : iso15765_server
* Author             : Margit Robert
* Date First Issued  : 2020.03.24
* Description        : Iso1576 Network Layer
********************************************************************************
*2020.03.24: Javitas: A hibásan jelezett request méretéba nem hal bele.
*2020.03.24: Javitás: 256 bájttól hosszab kérések hossza...
********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "iso15765_server.h"


/* Private variables ---------------------------------------------------------*/
#if ISO15765_DEBUG_LEVEL > 0
static char StringBuffer[80];
#endif

/* Private function prototypes -----------------------------------------------*/
uint8_t Iso15765ServerInit(Iso15765Handle_Type *hnd, uint8_t blockSize, uint8_t separationTime)
{
  if(!hnd)
      return ISO15765_FAIL;

  hnd->Request.BlockSize = blockSize;
  hnd->Request.SeparationTime = separationTime;

  hnd->State.Next = ST_ISO15765_IDLE;

  hnd->Diag.ResponseConsecutiveTimeoutCnt = 0;
  hnd->Diag.RequestSequneceErrorCnt = 0;
  hnd->Diag.RequestBlockSequneceErrorCnt = 0;
  hnd->Diag.ResponseSequneceErrorCnt = 0;
  hnd->Diag.TotalRetryCnt = 0;
  hnd->Diag.RequestRepeatCnt = 0;
  hnd->Diag.ResponsePendingCnt = 0;
  
  hnd->Request.Timestamp = HAL_GetTick();
  hnd->Response.Timestamp = HAL_GetTick();
    
   return ISO15765_OK;
}

/**
  * @brief Incoming Stream
  */
uint8_t Iso15765Task(Iso15765Handle_Type *hnd)
{
  if(!hnd)
      return ISO15765_FAIL;
  
  switch(hnd->State.Curr)
  {
    /*---*/
    case ST_ISO15765_START:
    {
      hnd->State.Next = ST_ISO15765_IDLE;
      Iso15765DbgLog("ST_ISO15765_START -> ST_ISO15765_IDLE");
      break;
    }
    /*---*/
    case ST_ISO15765_IDLE:
    {
      hnd->State.Next = ST_ISO15765_RX;
      Iso15765DbgLog("ST_ISO15765_IDLE -> ST_ISO15765_RX...");
      break;
    }
    /*---*/
    case ST_ISO15765_RX:
    {
      if(hnd->State.Pre != ST_ISO15765_RX)
      {
        /*memory fill default*/
        memset(hnd->Request.Data, ISO15765_DEFAULT_FILL, ISO15765_BUFFER_SIZE);
        memset(hnd->Response.Data,ISO15765_DEFAULT_FILL, ISO15765_BUFFER_SIZE);
        hnd->Request.Offset = 0;
        hnd->Request.Size = 0;
      }

      if(hnd->Rx.Ready)
      {
        hnd->Rx.Ready = 0;
        uint8_t pci = hnd->Rx.Frame[0] & 0xF0;
        if(pci == ISO15765_N_PCI_SF)
        {
          /*SF_DL|DATA*/
          hnd->State.Next = ST_ISO15765_RX_SINGLE;
          Iso15765DbgLog("ST_ISO15765_RX -> ST_ISO15765_RX_SINGLE");
        }
        else if (pci == ISO15765_N_PCI_FF)
        {
          hnd->State.Next = ST_ISO15765_RX_FIRST;
          Iso15765DbgLog("ST_ISO15765_RX -> ST_ISO15765_RX_FIRST");
        }
        else
        {
          hnd->State.Next = ST_ISO15765_RX_ERROR;
          Iso15765DbgLog("ST_ISO15765_RX -> ST_ISO15765_RX_ERROR");
        }
      }
      break;
    }
    /*---*/
    case ST_ISO15765_RX_SINGLE:
    { 
      hnd->Request.Size = hnd->Rx.Frame[0] & 0x0F;
      memcpy( hnd->Request.Data,                              /*dest*/
              hnd->Rx.Frame + ISO15765_SF_DL_SIZE,            /*src*/
              hnd->Request.Size);                             /*size*/
      hnd->State.Next = ST_ISO15765_RX_DONE;
      Iso15765UsrLog("ST_ISO15765_RX_SINGLE -> ST_ISO15765_RX_DONE...");
      break;
    }
    /*---*/
    case ST_ISO15765_RX_FIRST:
    {
      Iso15765DbgLog("Rx;CONS:%s",StringPlusDataToHexaString(hnd->Rx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE));
      hnd->Request.Size = ((hnd->Rx.Frame[0] & 0x0F) << 8 | hnd->Rx.Frame[1]);
      memcpy( hnd->Request.Data,                              /*dest*/
              hnd->Rx.Frame + ISO15765_FF_DL_SIZE,            /*src*/
              ISO15765_CAN_FRAME_SIZE - ISO15765_FF_DL_SIZE); /*size*/
      hnd->Request.Offset += ISO15765_CAN_FRAME_SIZE - ISO15765_FF_DL_SIZE;
      hnd->State.Next = ST_ISO15765_TX_FLOWCTRL;
      Iso15765UsrLog("ST_ISO15765_RX_FIRST -> ST_ISO15765_TX_FLOWCTRL...");
      break;
    }
    /*---*/
    case ST_ISO15765_TX_FLOWCTRL:
    {
      memset(hnd->Tx.Frame,ISO15765_DEFAULT_FILL,ISO15765_CAN_FRAME_SIZE);
      hnd->Tx.Frame[0] = ISO15765_N_PCI_FC;
      hnd->Tx.Frame[1] = hnd->Request.BlockSize;
      hnd->Tx.Frame[2] = hnd->Request.SeparationTime;
      Iso15765BusWriteCallback(hnd, hnd->Tx.Frame, ISO15765_CAN_FRAME_SIZE);
      Iso15765DbgLog("Tx;FLOW:%s ->BS:%d,STmin:%dms",
                      StringPlusDataToHexaString(hnd->Tx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE),
                      hnd->Request.BlockSize,
                      hnd->Request.SeparationTime);
      hnd->State.Next = ST_ISO15765_RX_CONSECUTIVE;
      Iso15765UsrLog("ST_ISO15765_TX_FLOWCTRL -> ST_ISO15765_RX_CONSECUTIVE...");
      hnd->Request.BlockCnt = 0;
      hnd->Request.FrameCnt = 0;
      break;
    }
    /*---*/
    case ST_ISO15765_RX_CONSECUTIVE:
    {
      if(hnd->Rx.Ready)
      {
        hnd->Rx.Ready = 0;
        Iso15765DbgLog("Rx;CONS:%s",StringPlusDataToHexaString(hnd->Rx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE));
        hnd->Request.FrameCnt++;
        /* 21 82 00 00 80 5A BF 35
         * 22 85 52 D3 74 42 0F 09
         * 23 A5 B7 4F B8 55 03 09
         * 24 23 F7 0B D3 4C 56 D8
         * ...
         * 2F 3E 8F C7 F6 81 61 D4
         * 20 64 E3 9F 94 9F 62 A4
         */
        if((hnd->Request.FrameCnt & 0x0F) != (hnd->Rx.Frame[0] & 0x0F))
        {
          /*
           * !A Szekevica hib�t nem lehet jav�tani.
           * A v�gn�n nem lesz annyi b�jt amiennyi kell �s a timeout hiba �rjra pbor�lkozik
           */
          hnd->Diag.RequestSequneceErrorCnt++;
        }
        else
        {
          if(hnd->Request.Size - hnd->Request.Offset >= ISO15765_CAN_FRAME_SIZE - ISO15765_SN_SIZE)
          {/* Receive Full Frame */
            memcpy( hnd->Request.Data + hnd->Request.Offset,       /*dest*/
                    hnd->Rx.Frame + ISO15765_SN_SIZE,               /*src*/
                    ISO15765_CAN_FRAME_SIZE - ISO15765_SN_SIZE);    /*size*/
            hnd->Request.Offset += ISO15765_CAN_FRAME_SIZE - ISO15765_SN_SIZE;
            Iso15765UsrLog("Full Frame new Offset: %ld", hnd->Request.Offset);
          }
          else
          {
            if(hnd->Request.Size - hnd->Request.Offset < 0)
            {
              /*Ha ez itt negativ, akkor azt jelenti hogy többet
               *külödött, mint amennyit jelezett az üzenetben.*/
              hnd->State.Next = ST_ISO15765_RX_ERROR;
              break;
            }
            else
            {
              memcpy( hnd->Request.Data + hnd->Request.Offset,        /*dest*/
                      hnd->Rx.Frame + ISO15765_SN_SIZE,               /*src*/
                      hnd->Request.Size - hnd->Request.Offset);       /*size*/
              hnd->Request.Offset += hnd->Request.Size - hnd->Request.Offset;
              Iso15765UsrLog("New Offset: %ld", hnd->Request.Offset);
            }
          }
          if(hnd->Request.Size - hnd->Request.Offset == 0)
          {
            hnd->State.Next = ST_ISO15765_RX_DONE;
            Iso15765UsrLog("ST_ISO15765_RX_CONSECUTIVE -> ST_ISO15765_RX_DONE...");
          }
          else if((hnd->Request.BlockSize != 0) && (hnd->Request.BlockCnt >= hnd->Request.BlockSize))
          {
            hnd->Request.BlockCnt++;
            hnd->State.Next = ST_ISO15765_TX_FLOWCTRL;
            Iso15765UsrLog("ST_ISO15765_RX_CONSECUTIVE -> ST_ISO15765_TX_FLOWCTRL...");
          }
          else
          {
            Iso15765UsrLog("ST_ISO15765_RX_CONSECUTIVE -> ST_ISO15765_RX_CONSECUTIVE...");
          }
        }
      }
      else
      {
      }
      break;
    }
    /*---*/
    case ST_ISO15765_RX_DONE:
    {
      if(hnd->State.Pre != ST_ISO15765_RX_DONE)
      {
        Iso15765DbgLog("ST_ISO15765_RX_SINGLE -> Wait for response of apliction layer ...");
        Iso15765ReqRespCallback(hnd,hnd->Request.Data, hnd->Request.Size);
      }
      else
      {
      }
      break;
    }
    /*---*/
    case ST_ISO15765_RX_ERROR:
    {
      if(hnd->State.Pre != ST_ISO15765_RX_ERROR)
      {
        Iso15765ErrLog(">!!! ST_ISO15765_RX_ERROR !!!!<");
        hnd->State.Next = ST_ISO15765_IDLE;
        Iso15765ErrLog("ST_ISO15765_COMPLETE -> ST_ISO15765_IDLE");
      }
      break;
    }
    /*---*/
    case ST_ISO15765_TX:
    {
      hnd->Response.Offset = 0;
      if(hnd->Response.Size <= (ISO15765_CAN_FRAME_SIZE - ISO15765_SF_DL_SIZE))
      {
        hnd->State.Next = ST_ISO15765_TX_SINGLE;
        Iso15765DbgLog("ST_ISO15765_TX -> ST_ISO15765_TX_SINGLE");
      }
      else
      {
        hnd->State.Next = ST_ISO15765_TX_FIRST;
        Iso15765DbgLog("ST_ISO15765_TX -> ST_ISO15765_TX_FIRST");
      }
      break;
    }
    /*---*/
    case ST_ISO15765_TX_SINGLE:
    {
      memset(hnd->Tx.Frame, ISO15765_DEFAULT_FILL, ISO15765_CAN_FRAME_SIZE);
      hnd->Tx.Frame[hnd->Response.Offset] = ISO15765_N_PCI_SF | hnd->Response.Size;
      hnd->Response.Offset += ISO15765_SF_DL_SIZE;
      memcpy(hnd->Tx.Frame + hnd->Response.Offset, hnd->Response.Data, hnd->Response.Size);
      Iso15765BusWriteCallback(hnd, hnd->Tx.Frame , ISO15765_CAN_FRAME_SIZE); 
      Iso15765DbgLog("Tx;SING:%s", StringPlusDataToHexaString(hnd->Tx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE));
      hnd->State.Next = ST_ISO15765_COMPLETE;
      Iso15765UsrLog("ST_ISO15765_TX_SINGLE -> ST_ISO15765_COMPLETE...");
      break;
    }
    /*---*/
    case ST_ISO15765_TX_FIRST:
    {
      memset(hnd->Tx.Frame, ISO15765_DEFAULT_FILL, ISO15765_CAN_FRAME_SIZE);
      hnd->Tx.Frame[0] = (ISO15765_N_PCI_FF | (hnd->Response.Size >> 8));        /*msb*/
      hnd->Tx.Frame[1] = hnd->Response.Size;                                     /*lsb*/
      memcpy( hnd->Tx.Frame + ISO15765_FF_DL_SIZE,                               /*dest*/ 
              hnd->Response.Data,                                                /*src*/
              ISO15765_CAN_FRAME_SIZE - ISO15765_FF_DL_SIZE);                    /*size*/
      Iso15765BusWriteCallback(hnd, hnd->Tx.Frame, ISO15765_CAN_FRAME_SIZE);
      hnd->Response.Offset += ISO15765_CAN_FRAME_SIZE - ISO15765_FF_DL_SIZE;
      Iso15765DbgLog("Tx;FIRS:%s", StringPlusDataToHexaString(hnd->Tx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE));
      hnd->State.Next = ST_ISO15765_RX_FLOWCTRL;
      Iso15765UsrLog("ST_ISO15765_TX_FIRST -> ST_ISO15765_RX_FLOWCTRL...");
      break;
    }
    /*---*/
    case ST_ISO15765_RX_FLOWCTRL:
    {
      if(hnd->State.Pre != ST_ISO15765_RX_FLOWCTRL)
      {
        hnd->Rx.Timestamp = HAL_GetTick();
      }
      if(hnd->Rx.Ready)
      {
        hnd->Response.FrameCnt = 0;
        hnd->Response.BlockCnt = 0;
        hnd->Response.BlockSize = hnd->Rx.Frame[1];
        hnd->Response.SeparationTime = hnd->Rx.Frame[2];
        hnd->Rx.Ready = 0;
        hnd->State.Next = ST_ISO15765_TX_WAIT;
        Iso15765UsrLog("ST_ISO15765_RX_FLOWCTRL -> ST_ISO15765_TX_WAIT...");
        Iso15765DbgLog("Rx;FLOW:%s ->BS:%d,STmin:%dms",
                        StringPlusDataToHexaString(hnd->Rx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE),
                        hnd->Response.BlockSize,
                        hnd->Response.SeparationTime);
      }
      else
      {
       if(HAL_GetTick() - hnd->Rx.Timestamp > ISO15765_TIMEOUT_MS)
        {
          hnd->Rx.Ready = 0;
          hnd->State.Next = ST_ISO15765_TX_ERROR;
          Iso15765ErrLog("ST_ISO15765_RX_FLOWCTRL -> ST_ISO15765_TX_ERROR...");
        }
      }
      break;
    }
    /*---*/
    case ST_ISO15765_TX_WAIT:
    {
      if(hnd->State.Pre != ST_ISO15765_TX_WAIT)
      {
        hnd->STminTimestamp = HAL_GetTick(); /*separationTime: 00-7F ->> 0-125ms */
      }
      if(HAL_GetTick() -  hnd->STminTimestamp > hnd->Response.SeparationTime )
      {
        hnd->State.Next = ST_ISO15765_TX_CONSECUTIVE;
        Iso15765UsrLog("ST_ISO15765_TX_WAIT -> ST_ISO15765_TX_CONSECUTIVE...");
      }
      break;
    }
    /*---*/
    case ST_ISO15765_TX_CONSECUTIVE:
    {
      if(hnd->State.Pre != ST_ISO15765_TX_CONSECUTIVE)
      {
        hnd->Response.FrameCnt++;
        memset(hnd->Tx.Frame,ISO15765_DEFAULT_FILL,ISO15765_CAN_FRAME_SIZE);
        hnd->Tx.Frame[0]= ISO15765_N_PCI_CF | (hnd->Response.FrameCnt & 0x0F);

        if((hnd->Response.Size - hnd->Response.Offset) >= (ISO15765_CAN_FRAME_SIZE - ISO15765_SN_SIZE))
        {
          memcpy( hnd->Tx.Frame + ISO15765_SN_SIZE,               /*dest*/ 
                  hnd->Response.Data + hnd->Response.Offset,      /*src*/
                  ISO15765_CAN_FRAME_SIZE - ISO15765_SN_SIZE);    /*size*/

          Iso15765BusWriteCallback(hnd, hnd->Tx.Frame, ISO15765_CAN_FRAME_SIZE);
          hnd->Response.Offset += ISO15765_CAN_FRAME_SIZE - ISO15765_SN_SIZE;
          Iso15765DbgLog("Tx;CONS:%s", StringPlusDataToHexaString(hnd->Tx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE));
        }
        else
        {
          memcpy( hnd->Tx.Frame + ISO15765_SN_SIZE,           /*dest*/
                  hnd->Response.Data + hnd->Response.Offset,    /*src*/
                  hnd->Response.Size - hnd->Response.Offset);   /*size*/
          Iso15765BusWriteCallback(hnd, hnd->Tx.Frame, ISO15765_CAN_FRAME_SIZE);
          {
              hnd->Response.Offset += hnd->Response.Size - hnd->Response.Offset;
              Iso15765DbgLog("Tx;CONS:%s", StringPlusDataToHexaString(hnd->Tx.Frame, StringBuffer, ISO15765_CAN_FRAME_SIZE));
          }
        }
      }
      /*Ha van maxim�lis block m�ret, akkor a blok m�ret el�r�se ut�n �jra kell k�szofg�st olvasni!*/
      if((hnd->Response.BlockSize != 0) && (hnd->Response.BlockCnt >= hnd->Response.BlockSize))
      {
          hnd->Response.BlockCnt++;
          hnd->State.Next = ST_ISO15765_RX_FLOWCTRL;
          Iso15765UsrLog("ST_ISO15765_TX_FIRST -> ST_ISO15765_RX_FLOWCTRL...");
      }                
      if(hnd->Response.Offset >= hnd->Response.Size)
      {
          hnd->State.Next = ST_ISO15765_COMPLETE;
          Iso15765UsrLog("ST_ISO15765_TX_CONSECUTIVE -> ST_ISO15765_COMPLETE...");
      }
      else
      {
          hnd->State.Next = ST_ISO15765_TX_WAIT;
          Iso15765UsrLog("ST_ISO15765_TX_CONSECUTIVE -> ST_ISO15765_TX_WAIT...");
      }
      break;
    }
    /*---*/
    case ST_ISO15765_TX_ERROR:
    {
      if(hnd->State.Pre != ST_ISO15765_TX_ERROR)
      {
        Iso15765ErrLog(">ST_ISO15765_TX_ERROR<");
      }
      break;
    }
    /*---*/
    case ST_ISO15765_COMPLETE:
    {
      if(hnd->State.Pre != ST_ISO15765_COMPLETE)
      {
        hnd->State.Next = ST_ISO15765_IDLE;
        Iso15765UsrLog("ST_ISO15765_COMPLETE -> ST_ISO15765_IDLE");
      }
      break;
    }
  }

  hnd->State.Pre = hnd->State.Curr;
  hnd->State.Curr = hnd->State.Next;
  return ISO15765_OK;
}

Iso15765State_Type Iso15765GetState(Iso15765Handle_Type *hnd)
{
   Iso15765State_Type state = hnd->State.Curr;
   return state;
}

uint8_t Iso15765GetStatus(Iso15765Handle_Type *hnd)
{
    uint8_t status = ISO15765_OK;
    if(hnd->State.Curr == ST_ISO15765_TX_ERROR && hnd->State.Next == ST_ISO15765_TX_ERROR)
    {
        status = ISO15765_BUS_LOST;
    }
    else if(hnd->State.Curr == ST_ISO15765_RX_ERROR && hnd->State.Next == ST_ISO15765_RX_ERROR)
    {
        status = ISO15765_TIMEOUT;
    }
    else if(hnd->State.Curr == ST_ISO15765_COMPLETE || hnd->State.Curr == ST_ISO15765_IDLE)
    {
        status = ISO15765_OK;
    }
    else
    {
        status = ISO15765_BUSY;
    }
    
   return status;
}

/**
  * @brief Response
  */
uint8_t Iso15765Response(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{
  if(size > ISO15765_BUFFER_SIZE)
  {
    return ISO15765_BUFFER_TOO_SMALL;
  }
  memset(hnd->Response.Data,ISO15765_DEFAULT_FILL, sizeof(ISO15765_BUFFER_SIZE)); 
  hnd->State.Next = ST_ISO15765_TX;
  memcpy(hnd->Response.Data , data , size);
  hnd->Response.Size = size;
  return ISO15765_OK;
}


/**
  * @brief Negative Response
  */
uint8_t Iso15765NegativeResponse(Iso15765Handle_Type *hnd, uint8_t sid, uint8_t nrc)
{
  uint8_t resp[] = {ISO15765_NR_CODE, sid, nrc };
  Iso15765Response(hnd, resp, sizeof(resp));
  return ISO15765_OK;
}
/**
  * @brief Request->Response Callback
  */
__weak uint8_t Iso15765ReqRespCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{
    return ISO15765_OK;
}

/**
  * @brief Bus Write Callback
  */
__weak uint8_t Iso15765BusWriteCallback(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{
  return ISO15765_OK;
}

/**
  * @brief Incoming Stream
  */
uint8_t Iso15765IncomingStream(Iso15765Handle_Type *hnd, uint8_t *data, size_t size)
{
  if(hnd->Rx.Ready == 0)
  { 
    memcpy(hnd->Rx.Frame, data, ISO15765_CAN_FRAME_SIZE);
    hnd->Rx.Ready = 1;
    return ISO15765_OK;
  }
  else
  {
    Iso15765NegativeResponse(hnd, 0, ISO15765_NRC_BUSY_REPEAT_REQUEST);
    return ISO15765_BUSY;
  }
}

/**
  * @brief Flush Callback
  */

__weak uint8_t Iso15765FlushCallback(Iso15765Handle_Type *hnd)
{
    return ISO15765_OK;
}
/******************* (C) COPYRIGHT 2015 marrob Design *****END OF FILE******/
