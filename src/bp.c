/*
 * ARM pipeline timing simulator
 *
 * CMSC 22200, Fall 2016
 * Gushu Li and Reza Jokar
 */

#include "bp.h"
#include "pipe.h"
#include <stdlib.h>
#include <stdio.h>

/* Predict next PC */
uint64_t bp_predict(uint64_t pc)
{
    // printf("PREDICT: the address tag value in 03db is%" PRIx64 "\n", BPT.btb[0x3db].address_tag);
    if (VERBOSE_FLAG) printf("PREDICT: predict entered for PC %" PRIx64 "\n", pc);
    /* Index into BPT*/
    uint32_t bptMask = 0x00000FFC;
    uint32_t bptindex = (bptMask & pc) >> 2;
    btb_entry_t entry = BPT.btb[bptindex];

    /*Check for a BPT hit*/
    if (!entry.valid_bit)
    {   
        CURRENT_STATE.PC += 4;
        C_FETCH.p_taken = false;
        if (VERBOSE_FLAG) printf("PREDICT: predict exited for PC %" PRIx64 " valid bit\n", CURRENT_STATE.PC); 
        return;
    }
    //Check if address tags match
    if (entry.address_tag != pc){
        CURRENT_STATE.PC += 4;
        C_FETCH.p_taken = false;
        if (VERBOSE_FLAG) printf("PREDICT: predict exited for PC %" PRIx64 " address tag\n", CURRENT_STATE.PC); 
        return;
    }
    /*Now we know there's a hit.*/


    /* Index into PHT*/
    uint32_t phtMask = 0x000003FC;
    uint32_t phtindex = ((phtMask & pc) >> 2) ^ BPT.gshare.ghr; 
    int currPHT = BPT.pht[phtindex];

    /* Use Gshare to index into PHT*/
    // int gsharePHT = BPT.pht[BPT.gshare.predictor];
    /*Check GShare prediction*/
    int gshareTaken = currPHT & 0b10;

    /*  CHECK IF BTB ENTRY INDICATES BRANCH IS UNCONDITIONAL
        GSHARE PREDICTOR INDICATES BRANCH SHOULD BE TAKEN
        else next pc is PC + 4 */

    /* Check if btb indicates uncond branch OR check if gshare indicates next branch should be taken */
    if ((!entry.cond_bit) || (gshareTaken)){
        CURRENT_STATE.PC = entry.target;
        C_FETCH.p_taken = true;
        if (VERBOSE_FLAG) printf("PREDICT: predict exited for PC %" PRIx64 " jump\n", CURRENT_STATE.PC); 
        return;
    } 
    CURRENT_STATE.PC += 4;
    if (VERBOSE_FLAG) printf("PREDICT: pc as hex is %" PRIx64 "\n", pc);
    C_FETCH.p_taken = false;
    return; //else case

}

/*target is your CURRENT_STATE.PC after a branch, pc is your PC when this particular instruction
was fetched*/
void bp_update(uint64_t pc, int64_t target, int taken, int is_cond)
{
    if (VERBOSE_FLAG) printf("UPDATE: update entered for PC %" PRIx64 "\n", pc);
    /* Index into BPT*/
    uint32_t bptMask = 0x00000FFC;
    uint32_t bptindex = (bptMask & pc) >> 2;
    btb_entry_t entry = BPT.btb[bptindex];

    /* Update GSHARE and PHT for conditional branch*/
    if (is_cond){
        uint32_t phtMask = 0x000003FC;
        /* Index into PHT & update PHT*/
        uint32_t phtindex = ((phtMask & pc) >> 2) ^ BPT.gshare.ghr;
        int currPHT = BPT.pht[phtindex];
        if (taken)
            BPT.pht[phtindex] = adjustPHT(currPHT, true); //?Increment
        else
            BPT.pht[phtindex] = adjustPHT(currPHT, false); //?Decrement

        /* Update gshare directional predictor */
        BPT.gshare.pc = pc;
        // BPT.gshare.predictor = ((phtMask & pc) >> 2) ^ BPT.gshare.ghr;
        BPT.gshare.ghr = ((BPT.gshare.ghr << 1) | taken) & 0xFF;
        // printf("UPDATE: updated GHR  %3x\n", BPT.gshare.ghr);
        // if(BPT.gshare.ghr == 0xFF){
        //     BPT.gshare.ghr = 0x1;
        //     printf("UPDDATE UPDATE: new GHR  %3x\n", BPT.gshare.ghr);
        // } 

    }

    /* Update BTB */
    BPT.btb[bptindex].address_tag = pc;
    BPT.br_hit = taken;
    if (is_cond) 
        BPT.btb[bptindex].cond_bit = true;
    else BPT.btb[bptindex].cond_bit = false;
    BPT.btb[bptindex].target = target; //<-- do not save the target.
    BPT.btb[bptindex].valid_bit = true;
}

int adjustPHT(int currPHT, int is_increment){
    if (is_increment){
        if (currPHT == 3)
            return currPHT;
        else
            return currPHT + 1;
    } else{
        /*Decrement*/
        if (currPHT == 0)
            return currPHT;
        else
            return currPHT - 1;
    }

}
