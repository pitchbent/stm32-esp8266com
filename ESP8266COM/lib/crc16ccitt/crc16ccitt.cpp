#include <crc16ccitt.h>



// Calculate CRC16 CCITT
// It's a 16 bit CRC with polynomial x^16 + x^12 + x^5 + 1
// input:
//   crcIn - the CRC before (0 for rist step)
//   data - byte for CRC calculation
// return: the CRC16 value
uint16_t CRC16_one(uint16_t crcIn, uint8_t data) {
	crcIn  = (uint8_t)(crcIn >> 8)|(crcIn << 8);
	crcIn ^=  data;
	crcIn ^= (uint8_t)(crcIn & 0xff) >> 4;
	crcIn ^= (crcIn << 8) << 4;
	crcIn ^= ((crcIn & 0xff) << 4) << 1;

	return crcIn;
}



// Calculate CRC16 CCITT value of the buffer
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
// return: the CRC16 value
uint16_t CRC16_buf(const uint8_t * pBuf, uint16_t len) {
	uint16_t crc = 0;

	while (len--) crc = CRC16_one(crc,*pBuf++);

	return crc;
}