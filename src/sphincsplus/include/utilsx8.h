#ifndef SPX_UTILSX4_H
#define SPX_UTILSX4_H

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "params.h"

/**
 * For a given leaf index, computes the authentication path and the resulting
 * root node using Merkle's TreeHash algorithm.
 * Expects the layer and tree parts of the tree_addr to be set, as well as the
 * tree type (i.e. SPX_ADDR_TYPE_HASHTREE or SPX_ADDR_TYPE_FORSTREE).
 * Applies the offset idx_offset to indices before building addresses, so that
 * it is possible to continue counting indices across trees.
 *
 * This implementation uses AVX to compute internal nodes 8 at a time (in
 * parallel)
 */
void treehashx8(unsigned char *root, unsigned char *auth_path,
                const unsigned char *sk_seed, const unsigned char *pub_seed,
                uint32_t leaf_idx, uint32_t idx_offset, uint32_t tree_height,
                void (*gen_leafx8)(
                   unsigned char* /* Where to write the leaves */,
                   const unsigned char* /* sk_seed */,
                   const unsigned char* /* pub_seed */,
                   uint32_t addr_idx, void *info),
                uint32_t tree_addrx8[8*8], void *info);

#ifdef __cplusplus
}
#endif

#endif
