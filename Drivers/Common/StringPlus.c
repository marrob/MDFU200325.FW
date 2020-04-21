/******************** (C) COPYRIGHT 2013 marrob Design *************************
* File Name          : StringPlus.c
* Author             : Margit R�bert
* Date First Issued  : 25/06/2013
* Description        : String...
********************************************************************************/
#include "StringPlus.h"

/**
  * @brief  StringPlusTrimWhitespace
  *
  * http://stackoverflow.com/questions/122616/how-do-i-trim-leading-trailing-whitespace-in-a-standard-way
  *
  */
char *StringPlusTrimWhitespace(char *str)
{
char *end;

    // Trim leading space
    while(isspace(*str)) str++;

    if(*str == 0)  // All spaces?
    return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while(end > str && isspace(*end)) end--;

    // Write new null terminator
    *(end+1) = 0;

    return str;
}

/**
  * @brief  StringPlusDataToHexaString
  *
  * 02 10 60 00 00 00 00 00 06 50 60 00 32 01 F4 00 .
  *
  */
char *StringPlusDataToHexaString(void* data, char *buffer, size_t size)
{
    uint8_t *ptr = data;
    uint8_t i = 0;
    if(size)
    {
        for(; i<size; i++)
            sprintf((buffer +(i*3)),"%02X ", *(ptr+i));
        if(i > 1)
            buffer[(i*3)-1]=0;
    }
    else
    {
        buffer[0]=0;
    }
    return buffer;
}

/**
  * @brief  StringPlusDataToLimitedHexaString
  *
  * 02 10 60 00 00 00 00 00 06 50 60 00 32 01 F4 00 ...
  *
  */
char *StringPlusDataToLimitedHexaString(void* data, int limit, char *buffer, size_t size)
{
  uint8_t *ptr = data;
  uint8_t i;
  if(size)
  {
    for( i = 0; i < size && i < limit; i++)
      sprintf((buffer +(i*3)),"%02X ", *(ptr+i));
    
    if(i >= limit)
    {
      buffer[(i*3)+1]='.';
      buffer[(i*3)+2]='.';
      buffer[(i*3)+3]='.';
      buffer[(i*3) + 4]= 0;
    }
    else
    {
      if(i > 1)
        buffer[(i*3)-1]=0;
    }
  }
  else
  {
    buffer[0]=0;
  }
  return buffer;
}

/**
  * @brief  StringPlusToUpper
  *
  * char str[] = "Hello World";
  * printf("%s",StringPlusToUpper(str));
  */
char *StringPlusToUpper(char* str){
    int i;
    for(i=0; i<strlen(str); i++)
        str[i] = toupper(str[i]);
    return str;
} 


/**
  * @brief  StringPlusCopy
  *
  * int main(void){
  * char value1[] = {"None Hello"};
  * char value2[] = {"World None"};
  * char result[40];
  * StringPlusCopy(result, 0, value1, 5, 5);
  * StringPlusCopy(result,strlen(result),value2,0,6);
  * printf("%s", result);
  */

char* StringPlusCopy(char *dst, int dstOffset, char *src, int srcOffset, int length){
  int i;
//  if((!dst > 0) || (!src > 0))
//    return 0;

  for(i=0; i< length; i++)
    dst[i + dstOffset] = src[i + srcOffset];
 
  dst[i + dstOffset] = '\0';
  
  return dst;
}

/**
  * @brief  IntToUnicode
  */
void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}

/**
  * @brief  IntToAscii
  */
void IntToAscii(uint32_t value , uint8_t *pbuf , uint8_t len)
{
uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[idx] = (value >> 28) + 'A' - 10; 
    }

    value = value << 4;

    pbuf[idx + 1] = 0;
  }
}
/******************* (C) COPYRIGHT 2013 marrob Design *****END OF FILE******/
