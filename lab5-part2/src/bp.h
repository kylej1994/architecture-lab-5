/*
 * ARM pipeline timing simulator
 *
 * CMSC 22200, Fall 2016
 */

#include "pipe.h"
#include <stdint.h>

#ifndef _BP_H_
#define _BP_H_

#define PHT_IDX 256
#define BTB_ENTRY 1024

#define STRONGLY_NOT_TAKEN 0
#define WEAKLY_NOT_TAKEN 1
#define WEAKLY_TAKEN  2
#define STRONGLY_TAKEN 3

typedef struct
{
    /* gshare */
    // used to index into the PHT
     int ghr;
     uint64_t pc;
     int predictor; // xor the ghr with bits 9:2 of the pc
} gshare_t;

typedef struct
{
	//btb entry
	uint64_t address_tag; 
	int valid_bit; //valid(1) unvalid(0)
	int cond_bit; //conditional(1) unconditonal(0)
	uint64_t target;
} btb_entry_t;

typedef struct
{
    /* gshare */
	gshare_t gshare;
	/* pht */
	int pht[PHT_IDX];
	
    /* BTB */
    /*contains:
    	 - the pc (64 bits)
    	 - a valid bit
    	 - conditional or unconditional bit
    	 - target address of the branch
    */
    btb_entry_t btb[BTB_ENTRY];
    int br_hit; //branch hit(1) branch miss(0)
} bp_t;

bp_t BPT;

uint64_t bp_predict(uint64_t pc);
void bp_update(uint64_t pc, int64_t target, int taken, int is_cond);
int adjustPHT(int currPHT, int is_increment);

#endif
