/******************** (C) COPYRIGHT 2013 marrob Design *************************
* File Name          : StringPlus.h
* Author             : Margit Róbert
* Date First Issued  : 25/06/2013
* Description        : String...
********************************************************************************/
#ifndef _STRING_PLUS__H_
#define _STRING_PLUS__H_

#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

char *StringPlusTrimWhitespace(char *str);
char *StringPlusDataToHexaString(void* data, char *buffer, size_t size);
char *StringPlusDataToLimitedHexaString(void* data, int limit, char *buffer, size_t size);
char *StringPlusToUpper(char* str);
void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
void IntToAscii(uint32_t value , uint8_t *pbuf , uint8_t len);
char* StringPlusCopy(char *dst, int dstOffset, char *src, int srcOffset, int length);

#endif //_STRING_PLUS__H_
/******************* (C) COPYRIGHT 2013 marrob Design *****END OF FILE******/
