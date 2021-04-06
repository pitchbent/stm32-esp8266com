/*Header file for the XTEA implementation*/



#ifndef __XTEA_H
#define __XTEA_H

#ifdef __cplusplus
extern "C" {
#endif



/*Number of times the algorithm is computed*/
#define NUM_CYCLES 32

/*Key hashes*/
#define KEY1 0x69a267ed
#define KEY2 0xc644659f
#define KEY3 0xb40e5f04
#define KEY4 0x93b02db2


/*XTEA functions*/
void Encrypt_XTEA(uint32_t data[2], uint32_t key[4]);
void Decrypt_XTEA(uint32_t data[2], uint32_t key[4]);


#ifdef __cplusplus
}
#endif

#endif
