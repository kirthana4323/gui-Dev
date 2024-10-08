#ifndef SPX_THASHX8_H
#define SPX_THASHX8_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>

void thashx8(unsigned char *out0,
             unsigned char *out1,
             unsigned char *out2,
             unsigned char *out3,
             unsigned char *out4,
             unsigned char *out5,
             unsigned char *out6,
             unsigned char *out7,
             const unsigned char *in0,
             const unsigned char *in1,
             const unsigned char *in2,
             const unsigned char *in3,
             const unsigned char *in4,
             const unsigned char *in5,
             const unsigned char *in6,
             const unsigned char *in7, unsigned int inblocks,
             const unsigned char *pub_seed, uint32_t addrx8[8*8]);

#ifdef __cplusplus
}
#endif

#endif
