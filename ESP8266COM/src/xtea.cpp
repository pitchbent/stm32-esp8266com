#include <stdio.h>
#include <xtea.h>

void Encrypt_XTEA(uint32_t v[2],uint32_t k[4])
{
	uint16_t i;
	const uint32_t delta=0x9E3779B9; //won't be changed -> const
	uint32_t v0=v[0], v1=v[1], sum=0;
    for (i=0; i < NUM_CYCLES; i++) {
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
    }
    v[0] = v0; v[1] = v1;

}


void Decrypt_XTEA(uint32_t v[2],uint32_t k[4])
{
	uint16_t i;
	const uint32_t delta=0x9E3779B9; //won't be changed -> const
    uint32_t v0 = v[0], v1 = v[1], sum = delta * NUM_CYCLES;
    for (i=0; i < NUM_CYCLES; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + k[(sum>>11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + k[sum & 3]);
    }
    v[0] = v0; v[1] = v1;

}
