#ifndef __CRC16CCITT_H
#define __CRC16CCITT_H


#include <stdio.h>





uint16_t CRC16_one(uint16_t crcIn, uint8_t data);
uint16_t CRC16_buf(const uint8_t * pBuf, uint16_t len);






#endif