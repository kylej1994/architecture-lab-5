/*
 *  * CMSC 22200, Fall 2016
 *   *
 *    * ARM pipeline timing simulator
 *     *
 *      */
 
#include "cache.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//TODO: IMPLEMENT 50 CYCLE PENALTY

void cache_new()
{
    memset(&CACHE, 0, sizeof(cache_t)); // initialize all cache values to zero
    memset(&CACHE.i_cache, 0, ICACHE_NUMBER_OF_SETS *sizeof(i_cache_set_t));
    memset(&CACHE.d_cache, 0, DCACHE_NUMBER_OF_SETS *sizeof(d_cache_set_t));
    
    /* Set ICache Mem*/
    int i, j, k;
    for(i = 0; i < ICACHE_NUMBER_OF_SETS; i++)
    {
		memset(&CACHE.i_cache[i].block, 0, ICACHE_SUBBLOCKS_PER_SET *sizeof(subblock_t));
    	for(j = 0; j < ICACHE_SUBBLOCKS_PER_SET; j++)
    	{
    		memset(&(CACHE.i_cache[i].block[j].subblock_data), 0, SUBBLOCK_SIZE * sizeof(uint64_t));
    		for(k = 0; k < SUBBLOCK_SIZE; k++)
    		{
    			memset(&(CACHE.i_cache[i].block[j].subblock_data[k]), 0, sizeof(uint64_t));
    		} 
    	} 
    }

    /* Set DCache Mem*/
	for(i = 0; i < DCACHE_NUMBER_OF_SETS; i++){
        memset(&CACHE.d_cache[i].block, 0, DCACHE_SUBBLOCKS_PER_SET *sizeof(subblock_t));
    	for(j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++)
    	{
    		memset(&(CACHE.d_cache[i].block[j].subblock_data), 0, SUBBLOCK_SIZE * sizeof(uint64_t));        		
    		for(k = 0; k < SUBBLOCK_SIZE; k++)
    		{
    			memset(&(CACHE.d_cache[i].block[j].subblock_data[k]), 0, sizeof(uint64_t));
    		}
    	} 	
    }
    return;
}

void cache_destroy()
{
    /* Look for values where dirty bit is set, and write to MEM */
    /* Evict blocks */
    int i, j;
    for(i = 0; i < DCACHE_NUMBER_OF_SETS; i++){
        for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++){
            cache_evict(D_CACHE, i, CACHE.d_cache[i].block[j].tag); 
        }
    }
    return;
    /* No mallocing, so we don't need to unmalloc the cache */
}

/* NOTE THIS IS A BOOLEAN FXN THAT RETURNS -1, NOT 0 WITH A FALSE VALUE.
Otherwise: returns block_index*/
int cache_hit(int cache_type, uint64_t addr)
{ // check the cache for a hit
    uint64_t cache_index;
    uint64_t cache_tag;
    int i = 0;
    if (cache_type == I_CACHE)
    {
        cache_index = (addr & 0x7E0) >> 5; // bits [10:5] of the PC, will be between 0-63
        cache_tag = (addr & 0xFFFFFFFFFFFFF800) >> 11; // find the tag

        for (i = 0; i < ICACHE_SUBBLOCKS_PER_SET; i++)
        {
    		// printf("CACHE_CHECK: the stored tag is %" PRIx64 " when the passed in tag is %" PRIx64 "\n",
    		// CACHE.i_cache[cache_index].block[i].tag, cache_tag);
            
    		if (CACHE.i_cache[cache_index].block[i].tag == cache_tag)
        	{ // if the valid bit for a subblock is 1, and the tag matches
            	return i;                                   
        	}
    	}
    }
    else if (cache_type == D_CACHE)
    {
        printf("CACHE CHECK: DCACHE CASE\n");
        cache_index = (addr & 0x1FE0) >> 5; // bits [12:5] of the PC, will be between 0-255
        cache_tag = (addr & 0xFFFFFFFFFFFFE000) >> 11; // find the tag

        for (i = 0; i < DCACHE_SUBBLOCKS_PER_SET; i++) 
        {
    		// printf("CACHE_CHECK: the stored tag is %" PRIx64 " when the passed in tag is %" PRIx64 "\n",
    		// CACHE.d_cache[cache_index].block[i].tag, cache_tag);
    		if ((CACHE.d_cache[cache_index].block[i].tag == cache_tag) && CACHE.d_cache[cache_index].block[i].valid)
        	{ // if the valid bit for a subblock is 1, and the tag matches
        	    return i;                                   
        	}
    	}
    }
    else
    {
        printf("cache_hit: ERROR, not a recognized cache type");
    }
    return -1; // cache miss
}

void cache_evict(int cache_type, uint64_t cache_index, uint64_t tag)
{
    printf("CACHE EVICT: eviction triggered\n");
	int i, j;
    if (cache_type == I_CACHE){
    	for (i = 0; i < ICACHE_SUBBLOCKS_PER_SET; i++)
        	{
        		if (CACHE.i_cache[cache_index].block[i].tag == tag)
                {
                    CACHE.i_cache[cache_index].block[i].valid = false;
                    CACHE.i_cache[cache_index].block[i].tag = 0;

                	memset(&(CACHE.i_cache[cache_index].block[i].subblock_data),
                		0, SUBBLOCK_SIZE * sizeof(uint64_t));
                	for (j = 0; j < ICACHE_SUBBLOCKS_PER_SET; j++)
        			{
                		memset(&(CACHE.i_cache[cache_index].block[i].subblock_data[j]),
                			0, sizeof(uint64_t));
                	}
                }
            }
    }
    else{
        printf("CACHE EVICT: DCACHE EVICTION TRIGGERED\n");
        for (i = 0; i < DCACHE_SUBBLOCKS_PER_SET; i++)
            {
                // printf("CACHE.d_cache[cache_index].block[i].tag %" PRIx64 " and tag %" PRIx64 "\n", CACHE.d_cache[cache_index].block[i].tag, tag);
                if (CACHE.d_cache[cache_index].block[i].position_RU == 8)
                {
                    // printf("CACHE EVICT: DCACHE TAGS MATCH\n");
                    /* write to memory if dirty bit is set*/ 
                    if (CACHE.d_cache[cache_index].block[i].dirty){
                        uint64_t addr = CACHE.d_cache[cache_index].block[cache_index].startAddr;
                        for (j = 0; j < SUBBLOCK_SIZE; j++) 
                        {
                            int32_t v = CACHE.d_cache[cache_index].block[cache_index].subblock_data[j];
                            // printf("CACHE_EVICT: We are writing %" PRIx64" to mem \n", v);
                            mem_write_32(addr + (j*4), v); 
                        }
                        CACHE.d_cache[cache_index].block[cache_index].dirty = false;
                    }
                    // CACHE.d_cache[cache_index].block[i].dirty = false;
                    // CACHE.d_cache[cache_index].block[i].valid = false;
                    // CACHE.d_cache[cache_index].block[i].tag = 0;

                    // memset(&(CACHE.d_cache[cache_index].block[i].subblock_data),
                    //     0, SUBBLOCK_SIZE * sizeof(uint64_t));
                    // for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++)
                    // {
                    //     memset(&(CACHE.d_cache[cache_index].block[i].subblock_data[j]),
                    //         0, sizeof(uint64_t));
                    // }
                }
            }
    }
}

void dcache_modify(uint64_t addr, int32_t data, int32_t data2, int is_64bit)
{
	// printf("DCACHE MODIFY: running dcahce modify\n");
	
/*You need a store/modification condition when you're modifying cache values*/ 
    uint64_t d_cache_index = (addr & 0x1FE0) >> 5; // bits [12:5] of the PC, will be between 0-255  
    uint64_t d_cache_tag = (addr & 0xFFFFFFFFFFFFF800) >> 11; // bits [63:13] of the PC, will be between 0-255
    uint64_t subblock_mask = (addr & (0x7 << 2)) >> 2;

    int i, j;
    for(i = 0; i < DCACHE_SUBBLOCKS_PER_SET; i++)
    {
    	// printf("DCACHE_MODIFY: the stored tag is %" PRIx64 " when the passed in tag is %" PRIx64 "\n",
    		// CACHE.d_cache[d_cache_index].block[i].tag, d_cache_tag);

        /* on cache miss, find an open block to write data to*/
        /* Find first unsed block to write to*/
    	if (!CACHE.d_cache[d_cache_index].block[i].valid)
    	{// not valid { // change to not use CACHE.d_cache[d_cache_index]

        	CACHE.d_cache[d_cache_index].block[i].valid = true;
            CACHE.d_cache[d_cache_index].block[i].tag = d_cache_tag;
           	CACHE.d_cache[d_cache_index].block[i].startAddr = addr;
            if (!is_64bit)
                CACHE.d_cache[d_cache_index].block[i].subblock_data[subblock_mask] = data;
            else{
                CACHE.d_cache[d_cache_index].block[i].subblock_data[subblock_mask] = data2;
                CACHE.d_cache[d_cache_index].block[i].subblock_data[subblock_mask + 1] = data;
            }
            // printf("DCACHE_MODIFY: reading  data: %" PRIx64 " and data2: %" PRIx64 " into the addr: %d, index: %d cache at %d\n",
				// data, data2, d_cache_index, i, subblock_mask);
         	return;
        }
        /* evict the LRU block bc we need to write the dirty bit blocks to mem*/
        // all valid, write to LRU 
        // printf("DCACHE_MODIFY: The LRU is %" PRIx64"\n", CACHE.d_cache[d_cache_index].block[i].position_RU);

        if ((CACHE.d_cache[d_cache_index].block[i].position_RU == 9) || (CACHE.d_cache[d_cache_index].block[i].tag == d_cache_tag))
        {
            // printf("DCACHE_MODIFY: Have to mem write because we had to evict\n");
            // mem_write_32(addr, data); // before we overwrite the data, we write back to memory
            // printf("DCACHE_MODIFY: We are evicting from block %" PRIx64" % " PRIx64 " in the cache \n", d_cache_index, d_cache_tag);
            if (CACHE.d_cache[d_cache_index].block[i].position_RU == 9)
                cache_evict(D_CACHE, d_cache_index, d_cache_tag);
            if (!is_64bit)
                CACHE.d_cache[d_cache_index].block[i].subblock_data[subblock_mask] = data;
            else{
                CACHE.d_cache[d_cache_index].block[i].subblock_data[subblock_mask] = data2;
                CACHE.d_cache[d_cache_index].block[i].subblock_data[subblock_mask + 1] = data;
            }
          	return;
        }
            //CACHE.d_cache[d_cache_index].block[i].subblock_data[0] = data;
           // return;
    	//}
    }
}

/* NOTE: FOR CACHE_TYPE I_CACHE, addr is PC. For D_cache, addr is mem address*/
void cache_update(uint64_t addr, int cache_type)
{
	int cacheHit = cache_hit(cache_type, addr); 
    int i, j;

	if (cache_type == I_CACHE)
	{
        if (cacheHit != FALSE){
            return;
        }
		/* finding what the cache index will be */                      
		uint64_t i_cache_index = (addr & 0x7E0) >> 5; // bits [10:5] of the PC, will be between 0-63
		/* finding what the cache tag will be */                        
		uint64_t i_cache_tag = (addr & 0xFFFFFFFFFFFFF800) >> 11; // bits [63:11] of the PC, will be between 0-25
		int i, j;
		for(i = 0; i < ICACHE_SUBBLOCKS_PER_SET; i ++)
		{
 			/*if you have a cache value with valid bit not set OR where position is last*/
  			if (!(CACHE.i_cache[i_cache_index].block[i].valid) ||
  			 (CACHE.i_cache[i_cache_index].block[i].position_RU == 4))
  			{ 
  				/*Insert*/
    			CACHE.i_cache[i_cache_index].block[i].valid = true;
    			CACHE.i_cache[i_cache_index].block[i].tag = i_cache_tag;
    			CACHE.i_cache[i_cache_index].block[i].position_RU = 1; 
                CACHE.i_cache[i_cache_index].block[i].startAddr = addr;

    			/*Update all other position_RU values*/
    			for (j = 0; j < ICACHE_SUBBLOCKS_PER_SET; j++)
    			{
    	 		 if (CACHE.i_cache[i_cache_index].block[i].position_RU != 4)
    	    		CACHE.i_cache[i_cache_index].block[i].position_RU += 1;
    			}
                // printf("ICACHE UPDATE: i_cache_index: %d, block: %d\n", i_cache_index, i);
    			/*Load next 8 instructions into cache*/
    			for (j = 0; j < SUBBLOCK_SIZE; j++)
    			{
    				// printf("ICACHE UPDATE: loading data from %" PRIx64 " into the cache \n" ,addr + (j*4));
    				CACHE.i_cache[i_cache_index].block[i].subblock_data[j] = mem_read_32(addr + (j*4)); //addr is PC
    				// printf("ICACHE UPDATE: loaded  %" PRIx64 " into the cache for index %d and block %d \n" , CACHE.i_cache[i_cache_index].block[i].subblock_data[j], i_cache_index, j);
    				// printf("\n\n");
    			}
    			return;
    		}
    	}
	}	
	else // DCACHE case
	{
		uint64_t d_cache_index = (addr & 0x1FE0) >> 5; // bits [12:5] of the PC, will be between 0-255  
		uint64_t d_cache_tag = (addr & 0xFFFFFFFFFFFFF800) >> 11; // bits [63:13] of the PC, will be between 0-255
		//First the corresponding block within the set
        /*This is your LOAD condition because you're reading from some address*/
		for(i = 0; i < DCACHE_SUBBLOCKS_PER_SET; i++)
		{
		  /*if you have a cache value with valid bit not set OR where position is last*/
			if ((!CACHE.d_cache[d_cache_index].block[i].valid))
            {
                // for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++){
                //     printf("DCACHEUPDATE: POSITION_RU %d for block %d\n", CACHE.d_cache[d_cache_index].block[j].position_RU, j);
                //     printf("first block value %" PRIx64 "\n", CACHE.d_cache[d_cache_index].block[j].subblock_data[0]);
                // }
                /*Insert*/
                CACHE.d_cache[d_cache_index].block[i].valid = true;
                CACHE.d_cache[d_cache_index].block[i].tag = d_cache_tag;
                CACHE.d_cache[d_cache_index].block[i].position_RU = 1; 
                CACHE.d_cache[d_cache_index].block[i].startAddr = addr;

                /*Update all other position_RU values*/
                for (j = 0; j < i; j++)
                {
                    CACHE.d_cache[d_cache_index].block[j].position_RU += 1;
                }
                CACHE.d_cache[d_cache_index].block[i].dirty = true;
    
                //Load Information
                for (j = 0; j < SUBBLOCK_SIZE; j++) 
                {
                    // printf("DCACHE UPDATE: loading data from %" PRIx64 " into the cache \n", addr + (j*4));
                    CACHE.d_cache[d_cache_index].block[i].subblock_data[j] =  mem_read_32(addr + (j*4)); //addr is mem_address
                    // printf("DCACHE UPDATE: loaded data %" PRIx64 " into the cache and block %d in subblock %d\n", CACHE.d_cache[d_cache_index].block[i].subblock_data[j], i, j);
                }
                // printf("************************************************BOOM LOADED\n");
                // for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++){
                //     printf("DCACHEUPDATE: POSITION_RU %d for block %d\n", CACHE.d_cache[d_cache_index].block[j].position_RU, j);
                //     printf("first block value %" PRIx64 "\n", CACHE.d_cache[d_cache_index].block[j].subblock_data[0]);
                // }
                return;                
            }

            if (CACHE.d_cache[d_cache_index].block[i].position_RU == 8)
			{ 
                //DEBUG LOOP
                // for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++){
                //     printf("DCACHEUPDATE: POSITION_RU %d for block %d\n", CACHE.d_cache[d_cache_index].block[j].position_RU, j);
                //     printf("first block value %" PRIx64 "\n", CACHE.d_cache[d_cache_index].block[j].subblock_data[0]);
                // }
                //You're evicting whatever your number 8 position_RU tag is
                cache_evict(D_CACHE, d_cache_index, d_cache_tag);

                /*Insert*/
	    		CACHE.d_cache[d_cache_index].block[i].valid = true;
	    		CACHE.d_cache[d_cache_index].block[i].tag = d_cache_tag;
	    		CACHE.d_cache[d_cache_index].block[i].position_RU = 1; 
                CACHE.d_cache[d_cache_index].block[i].startAddr = addr;

	    		/*Update all other position_RU values*/
	    		for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++)
	    		{
                    if (j != i)
   	 	    		    CACHE.d_cache[d_cache_index].block[j].position_RU += 1;
    			}
                CACHE.d_cache[d_cache_index].block[i].dirty = true;
    
    			
                //Load Information
    			for (j = 0; j < SUBBLOCK_SIZE; j++) 
    			{
                    // printf("DCACHE UPDATE: loading data from %" PRIx64 " into the cache \n" ,addr + (j*4));
      				CACHE.d_cache[d_cache_index].block[i].subblock_data[j] =  mem_read_32(addr + (j*4)); //addr is mem_address
                    // printf("DCACHE UPDATE: loaded data %" PRIx64 " into the cache \n", CACHE.d_cache[d_cache_index].block[i].subblock_data[j]);
    			}
                //DEBUG
                // for (j = 0; j < DCACHE_SUBBLOCKS_PER_SET; j++){
                //     printf("DCACHEUPDATE: POSITION_RU %d for block %d\n", CACHE.d_cache[d_cache_index].block[j].position_RU, j);
                //     printf("first block value %" PRIx64 "\n", CACHE.d_cache[d_cache_index].block[j].subblock_data[0]);
                // }
    			return;
  			}
		}
	}	
  return;
}


