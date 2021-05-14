#ifndef __CRC16CCIT_H
#define __CRC16CCIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



uint16_t CRC16_one(uint16_t crcIn, uint8_t data);
uint16_t CRC16_buf(const uint8_t * pBuf, uint16_t len);



#ifdef __cplusplus
}
#endif

#endif
