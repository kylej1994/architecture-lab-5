/*
 *  * CMSC 22200, Fall 2016
 *   *
 *    * ARM pipeline timing simulator
 *     *
 *      */
 
 
#ifndef _CACHE_H_
#define _CACHE_H_

#include <stdint.h>
#include <string.h>
#include "shell.h"
#include "pipe.h"

#define ICACHE_NUMBER_OF_SETS 64
#define DCACHE_NUMBER_OF_SETS 256

#define ICACHE_SUBBLOCKS_PER_SET 4
#define DCACHE_SUBBLOCKS_PER_SET 8

#define SUBBLOCK_SIZE 8

#define I_CACHE 1
#define D_CACHE 2

#define TRUE 1
#define FALSE -1

typedef struct
{
    uint8_t dirty;
    uint8_t valid;
    uint64_t tag;
    uint64_t position_RU;
    uint64_t startAddr;
    int32_t subblock_data[SUBBLOCK_SIZE];// access this with bits 4 through 2 of the pc 
                        
} subblock_t;

typedef struct
{
    subblock_t block[ICACHE_SUBBLOCKS_PER_SET];

} i_cache_set_t;
 
typedef struct
{
    subblock_t block[DCACHE_SUBBLOCKS_PER_SET];

} d_cache_set_t;

typedef struct
{
    i_cache_set_t i_cache[ICACHE_NUMBER_OF_SETS]; // array of i_cache sets
    d_cache_set_t d_cache[DCACHE_NUMBER_OF_SETS]; // array of d_cache sets
                
} cache_t;

cache_t CACHE;

void cache_new();
void cache_destroy();
int cache_hit(int cache_type, uint64_t addr);
void cache_update(uint64_t addr, int cache_type);
void cache_evict(int cache_type, uint64_t i_cache_index, uint64_t tag);


#endif
