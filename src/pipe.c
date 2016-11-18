/*
 * CMSC 22200
 *
 * ARM pipeline timing simulator
 *
 * Reza Jokar and Gushu Li, 2016
 */
//Kyle Jablon (jablonk), Wesley Kelley (wesleyk) and Kwaku Ofori-Atta (kwaku)
#include "pipe.h"
#include "shell.h"
#include "bp.h"
#include "cache.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>


    void pipe_init()
    {
        memset(&CURRENT_STATE, 0, sizeof(CPU_State));
        memset(&C_FETCH, 0, sizeof(CYCLE_FETCH));
        memset(&C_DECODE, 0, sizeof(CYCLE_DECODE));
        memset(&C_EXECUTE, 0, sizeof(CYCLE_EXECUTE));
        memset(&C_MEMORY, 0, sizeof(CYCLE_MEMORY));
        memset(&C_WRITE, 0, sizeof(CYCLE_WRITE));
        //initialize BPT
        memset(&BPT, 0, sizeof(bp_t));
        memset(&(BPT.gshare), 0, sizeof(gshare_t));
        int i;
        for (i = 0; i < BTB_ENTRY; i++){
            memset(&(BPT.btb[i]), 0, sizeof(btb_entry_t));
        }
        cache_new();
     
        C_FETCH.instr = -1;
        C_FETCH.pc = 0;
        C_FETCH.stall_bit = false;
        
        C_DECODE.instr = -1;
        C_DECODE.oppCode = OPP_MACRO_UNASS;
        C_DECODE.pc = 0;
        C_DECODE.stall_bit = true;
        C_DECODE.run_bit = 0;  
        C_DECODE.retired = 0;
        
        C_EXECUTE.instr = -1;
        C_EXECUTE.oppCode = OPP_MACRO_UNASS;
        C_EXECUTE.result = 0;
        C_EXECUTE.stall_bit = false;
        C_EXECUTE.pc = 0;
        C_EXECUTE.run_bit = true;
        C_EXECUTE.branch_stall_bit = false;
        
        C_MEMORY.instr = -1;
        C_MEMORY.oppCode = OPP_MACRO_UNASS;
        C_MEMORY.pc = 0;
        C_MEMORY.stall_bit = false;
        C_MEMORY.result = 0;
        C_MEMORY.run_bit = true;

        C_WRITE.instr = OPP_MACRO_UNASS;
        C_WRITE.write_bit = false; 
        C_WRITE.stall_bit = false;
        
        CURRENT_STATE.PC = 0x00400000;
        STALL_FOR_CYCLES = 0;
        VERBOSE_FLAG = true; //<----SET THIS TO FALSE WHEN YOU TURN IN ASSIGNMENTS
        // printf("PC initialized to  %08x\n",  CURRENT_STATE.PC);
    }

    void pipe_cycle()
    {
        if (VERBOSE_FLAG)
            printf("===============================================================\n");
        pipe_stage_wb();
        pipe_stage_mem();
        pipe_stage_execute();
        pipe_stage_decode();
        pipe_stage_fetch();
        if (VERBOSE_FLAG){
            printf("fetch\tdecode\texec\tmem\twrite\n");
            printf("%c\t%d\t%d\t%d\t%d\n", '?', C_DECODE.oppCode, C_EXECUTE.oppCode, C_MEMORY.oppCode, C_WRITE.oppCode);
            printf("===============================================================\n");
        }
        
    }
    
/*******************************************************************
                            WRITE
*******************************************************************/

    //Determine if function writes to a register
    //If it does then access the destination register, and write result to that
    //Increment the PC counter
    void pipe_stage_wb()
    {
        if (C_WRITE.stall_bit)
            return;

        uint32_t currOp = C_MEMORY.oppCode;
        C_WRITE.oppCode = C_MEMORY.oppCode;
        C_WRITE.retired = C_MEMORY.retired;

        int64_t result = -1; //Initialize result to negative 1, so zero flag is not automatically set

        if (is_writeable(currOp)){
            int destReg = C_MEMORY.resultRegister;
            result = C_MEMORY.result;
            CURRENT_STATE.REGS[destReg] = result;
            C_WRITE.write_bit = true;
            C_WRITE.result = C_MEMORY.result;
            C_WRITE.resultRegister = C_MEMORY.resultRegister;
            if (VERBOSE_FLAG)
                printf("*****value: %08x into reg %d\n", (uint32_t)result, destReg);
        }
        if (is_flaggable(currOp)){
            //Set flags equal to flags for execute
            CURRENT_STATE.FLAG_Z = C_MEMORY.FLAG_Z;
            CURRENT_STATE.FLAG_N = C_MEMORY.FLAG_N;
            CURRENT_STATE.FLAG_C = C_MEMORY.FLAG_C;
            CURRENT_STATE.FLAG_V = C_MEMORY.FLAG_V;
        }
        /* global run is set to false and we end simulation
        because HLT is in WB every instruction before it is completed
        */
        result = C_MEMORY.result;
        if (C_WRITE.oppCode != OPP_MACRO_UNK)
            stat_inst_retire += C_WRITE.retired;
        /*Unstall any pipelines if applicable*/
        unset_bits();
        if (C_WRITE.bubble_bit){
            C_WRITE.oppCode = OPP_MACRO_UNK;
            return;
        }
    }

/*******************************************************************
                            MEMORY
                            
                            
*******************************************************************/

    //Set instr, pc and opp_code instr
    //Determine if function accesses memory, if it is then do your opp and set the result register
    //If it does not then do nothing
    void pipe_stage_mem()
    {
        if (C_MEMORY.bubble_bit){
            if (VERBOSE_FLAG)
                printf("MEM: number of dcache stall cycles entering loop %d \n", STALL_FOR_CYCLES_DCACHE);
            if (STALL_FOR_CYCLES_DCACHE  == 0){
                /* execute normal opp*/
                C_MEMORY.oppCode = C_EXECUTE.oppCode;
                C_MEMORY.instr = C_EXECUTE.instr;
                C_MEMORY.pc = C_EXECUTE.pc;
                C_MEMORY.retired = C_EXECUTE.retired;
                C_MEMORY.run_bit = C_EXECUTE.run_bit;
                //Set flags equal to flags for execute
                C_MEMORY.FLAG_Z = C_EXECUTE.FLAG_Z;
                C_MEMORY.FLAG_N = C_EXECUTE.FLAG_N;
                C_MEMORY.FLAG_C = C_EXECUTE.FLAG_C;
                C_MEMORY.FLAG_V = C_EXECUTE.FLAG_V;

                uint32_t currOp = C_EXECUTE.oppCode;
                uint32_t instr = C_MEMORY.instr;
                int cacheHit = cache_hit(D_CACHE, C_EXECUTE.result);

                if (is_memory(currOp)){
                    if (VERBOSE_FLAG)
                        printf("MEMORY: basecase about to be triggered\n");
                    if (cacheHit >= 0){
                        if (VERBOSE_FLAG) printf("DCACHE HIT\n");
                        memoryOperation_hit(currOp);
                    } else {
                        if (VERBOSE_FLAG) printf("DCACHE MISS\n");
                        printf("<=============================DCACHE FILL TRIGGERED\n");
                        memoryOperation_basecase(currOp);
                        // printf("============================> POST FILL\n");
                    }
                    
                    /*Check for store after load stalls*/
                    if (is_stur(C_MEMORY.oppCode)){
                        if (is_load(C_EXECUTE.oppCode)){
                            if (VERBOSE_FLAG) printf("MEMORY: store after a load stall\n");
                            set_stall(PL_STAGE_EXECUTE);
                        }
                    }
                } else {
                    C_MEMORY.result = C_EXECUTE.result;
                    C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                }
                /*Check for false run bit*/
                if (!C_MEMORY.run_bit) {
                    RUN_BIT = false; 
                    return;
                }
                unset_stall(PL_DECODE_INCR_FIFTY);
            }else {
                /* insert bubble*/
                STALL_FOR_CYCLES_DCACHE -= 1;
                if (VERBOSE_FLAG) printf("MEMORY: number of stall cycles after decrement %d \n", STALL_FOR_CYCLES_DCACHE);
            }
            return;
        }
        if (C_MEMORY.stall_bit)
            return;
        if (!RUN_BIT)
            return;
        C_MEMORY.oppCode = C_EXECUTE.oppCode;
        C_MEMORY.instr = C_EXECUTE.instr;
        C_MEMORY.pc = C_EXECUTE.pc;
        C_MEMORY.retired = C_EXECUTE.retired;
        C_MEMORY.run_bit = C_EXECUTE.run_bit;
        //Set flags equal to flags for execute
        C_MEMORY.FLAG_Z = C_EXECUTE.FLAG_Z;
        C_MEMORY.FLAG_N = C_EXECUTE.FLAG_N;
        C_MEMORY.FLAG_C = C_EXECUTE.FLAG_C;
        C_MEMORY.FLAG_V = C_EXECUTE.FLAG_V;

        uint32_t currOp = C_EXECUTE.oppCode;
        uint32_t instr = C_MEMORY.instr;

        if (is_memory(currOp)){

            calculate(currOp);
            int cacheHit = cache_hit(D_CACHE, C_EXECUTE.result);
            if (cacheHit >= 0){
                printf("DCACHE HIT\n");
                memoryOperation_hit(currOp);
            } else {
                set_stall(PL_DECODE_INCR_FIFTY);
            }

            //CHECK IF OPERATING ON SAME REGISTERS, THEN DONE WITH STALL
            /* Check for Stalls. <---- is this ordering correct? should the op go first? */

            /*Check for store after load stalls*/
            if (is_stur(C_MEMORY.oppCode)){
                if (is_load(C_EXECUTE.oppCode)){
                    if (VERBOSE_FLAG) printf("MEMORY: store after a load stall\n");
                    set_stall(PL_STAGE_EXECUTE);
                }
            }
        } else {
            C_MEMORY.result = C_EXECUTE.result;
            C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
        }
        /*Check for false run bit*/
        if (!C_MEMORY.run_bit) {
            RUN_BIT = false; 
            return;
        }

    }
    
    /* Executes memory operations at the end of our miss. Eg: our base case.*/
void memoryOperation_basecase(uint32_t currOp){
        //Code seperated into LOAD and STORE cases for ease of reading
        int cacheHit = cache_hit(D_CACHE, C_EXECUTE.result);

        //LOAD CASE
        if (is_load(currOp)){
            //LDUR and LDURBH will set the C_MEMORY.result, and C_MEMORY.resultRegister flags
          
          	calculate(currOp);
          	int cacheHit = cache_hit(D_CACHE, C_EXECUTE.result);
            uint64_t addr_index = (C_EXECUTE.result & 0x1FE0) >> 5;
            uint64_t cache_tag = (C_EXECUTE.result & 0xFFFFFFFFFFFFE000) >> 11;
          	uint64_t subblock_mask = (C_EXECUTE.result & (0x7 << 2)) >> 2;
            uint32_t data, data_h, data1, data2; 
            switch (currOp){
                case OPP_MACRO_LDUR:
                      	data1 = mem_read_32(C_EXECUTE.result);
                   		data2 = mem_read_32(C_EXECUTE.result + 4);
                    	C_MEMORY.result = ((uint64_t) data2 << 32) | data1;
                    	C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                      	set_stall(PL_DECODE_INCR_FIFTY);
                    	cache_update(C_EXECUTE.result, D_CACHE); // C_EXECUTE.result is the address
                    break;
              
                case OPP_MACRO_LDURB:
                      	data = mem_read_32(C_EXECUTE.result);
                    	C_MEMORY.result = data;
                    	C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                      	set_stall(PL_DECODE_INCR_FIFTY);
                    	cache_update(C_EXECUTE.result, D_CACHE); // C_EXECUTE.result is the address
              		break;
              
                case OPP_MACRO_LDURH:
                      	data_h = mem_read_32(C_EXECUTE.result);
                    	C_MEMORY.result = zeroExtend(data_h);
                    	C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                      	set_stall(PL_DECODE_INCR_FIFTY);
                    	cache_update(C_EXECUTE.result, D_CACHE);
                    break;
                default:
                    if (VERBOSE_FLAG) printf("You triggered is_load for some reason\n");
            }
        } else {
        // STORE CASE
            switch(currOp){
                //NO structs stored
                case OPP_MACRO_STUR:
                    calculate(currOp);
                    cache_update(C_EXECUTE.result, D_CACHE);
                    dcache_modify(C_EXECUTE.result, (CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFFFFFFFF00000000) >> 32, 
                        CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFFFFFFFF, true);
                                 // replaced mem_write_32 with dcache_modify calls
                    break;
                case OPP_MACRO_STURB:
                    calculate(currOp);
                    cache_update(C_EXECUTE.result, D_CACHE);
                    dcache_modify(C_EXECUTE.result, CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFF, 0, false);
                    break;
                case OPP_MACRO_STURH:
                    calculate(currOp);
                    cache_update(C_EXECUTE.result, D_CACHE);
                    dcache_modify(C_EXECUTE.result, CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFFFF, 0, false);
                    break;
                case OPP_MACRO_STURW:
                    calculate(currOp);
                    cache_update(C_EXECUTE.result, D_CACHE);
                    dcache_modify(C_EXECUTE.result, CURRENT_STATE.REGS[C_EXECUTE.resultRegister]);
                    break;
            }
        }
    }

void memoryOperation_hit(uint32_t currOpp){
//Code seperated into LOAD and STORE cases for ease of reading
        //int cacheHit = cache_hit(D_CACHE, C_EXECUTE.resultRegister);
        //LOAD CASE
        if (is_load(currOpp)){
            //LDUR and LDURBH will set the C_MEMORY.result, and C_MEMORY.resultRegister flags


            calculate(currOpp);
            int cacheHit = cache_hit(D_CACHE, C_EXECUTE.result);
            uint64_t addr_index = (C_EXECUTE.result & 0x1FE0) >> 5;
            uint64_t cache_tag = (C_EXECUTE.result & 0xFFFFFFFFFFFFE000) >> 11;
            uint64_t subblock_mask = (C_EXECUTE.result & (0x7 << 2)) >> 2;
            uint32_t data1, data2, data, data_h; 
            switch (currOpp){
                case OPP_MACRO_LDUR:
                        data1 = CACHE.d_cache[addr_index].block[cacheHit].subblock_data[subblock_mask];
                        data2 = CACHE.d_cache[addr_index].block[cacheHit].subblock_data[subblock_mask + 1];
                        C_MEMORY.result = ((uint64_t) data2 << 32) | data1;
                        C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                    break;
              
                case OPP_MACRO_LDURB:
                        data = CACHE.d_cache[addr_index].block[cacheHit].subblock_data[subblock_mask];
                        C_MEMORY.result = data & 0x0000FFFF;
                        C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                    break;
              
                case OPP_MACRO_LDURH:
                        data_h = CACHE.d_cache[addr_index].block[cacheHit].subblock_data[subblock_mask] & 0x0000FFFF;
                        //uint32_t data3 = mem_read_32(C_EXECUTE.result) & 0x0000FFFF;
                        C_MEMORY.result = zeroExtend(data_h);
                        C_MEMORY.resultRegister = C_EXECUTE.resultRegister;
                    break;
                default:
                    if (VERBOSE_FLAG) printf("You triggered is_load for some reason\n");
            }
        } else {
        // STORE CASE
            switch(currOpp){
                //NO structs stored
                case OPP_MACRO_STUR:
                    dcache_modify(C_EXECUTE.result, (CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFFFFFFFF00000000) >> 32, 
                        CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFFFFFFFF, true); //maybe plus 4, this is a 64 bit int
                    break;
                case OPP_MACRO_STURB:
                    dcache_modify(C_EXECUTE.result, CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFF, 0, false);
                    break;
                case OPP_MACRO_STURH:
                    dcache_modify(C_EXECUTE.result, CURRENT_STATE.REGS[C_EXECUTE.resultRegister] & 0xFFFF, 0, false);
                    break;
                case OPP_MACRO_STURW:
                    dcache_modify(C_EXECUTE.result, CURRENT_STATE.REGS[C_EXECUTE.resultRegister]);
                    break;
            }
        }
    }
/*******************************************************************
                        EXECUTE
*******************************************************************/

    void pipe_stage_execute()
    {
        //Set instr, and opp_code instruction
        //1) Determine if function is executeable, if it is then do your opp and set all the registers
        //2) If it does not, then do nothing
        if (C_EXECUTE.stall_bit)
            return;
        if (!RUN_BIT)
            return;
        C_EXECUTE.instr = C_DECODE.instr;
        C_EXECUTE.oppCode = C_DECODE.oppCode;
        C_EXECUTE.pc = C_DECODE.pc;
        C_EXECUTE.predicted_pc = C_DECODE.predicted_pc;
        C_EXECUTE.retired = C_DECODE.retired;
        C_EXECUTE.p_taken = C_DECODE.p_taken;

        uint64_t result;
        uint32_t currOp = C_DECODE.oppCode;

        calculate(C_EXECUTE.oppCode);
        
        if (is_executeable(currOp)){
            //EXECUTE
            if(currOp == OPP_MACRO_HLT) C_EXECUTE.run_bit = 0;

            if(has_exec_result(currOp)){
                //result and resultRegister will be will be set in the helper function
                C_EXECUTE.pc = C_DECODE.pc + 4; 
            }
            uint64_t offset = 0; //dont know what this branching condition is
            C_EXECUTE.pc = C_DECODE.pc + offset;
        }
        
        // if(is_stall_branch(C_EXECUTE.oppCode)){
        //  if(C_DECODE.oppCode == C_EXECUTE.oppCode){   
        //      squash(PL_STAGE_DECODE);
        //  }
        // }
        // if(is_squash_branch(C_EXECUTE.oppCode)){
        //     if(C_EXECUTE.predicted_pc != CURRENT_STATE.PC){
        //         squash(PL_STAGE_DECODE);
        //     }
        // }
    }
    
    /*shortened for 'squash a motherfucker'*/
    void squash(int pl_stage_macro){
        printf("SQUASH TRIGGERED\n");
        switch(pl_stage_macro){
            case PL_STAGE_DECODE:
                C_DECODE.oppCode = OPP_MACRO_NOP;
                C_FETCH.instr = OPP_MACRO_NOP;
                // C_EXECUTE.retired = 0;
                C_FETCH.squash_bit = true;
                break;
            default:
                printf("tried to squash a non squashable macro stage\n");
                break;
        }
    }

    void pseudo_squash(int pl_stage_macro){
        printf("PSEUDO SQUASH TRIGGERED\n");
        switch(pl_stage_macro){
            case PL_STAGE_DECODE:
                C_FETCH.pseudo_stall_bit = true;
                C_FETCH.instr = OPP_MACRO_UNK; //<-- this will clear whats in decode
                break;
            default:
                printf("tried to squash a non squashable macro stage\n");
                break;
        }
    }
    
/*******************************************************************
                            DECODE
*******************************************************************/

    void pipe_stage_decode()
    {
        if (C_DECODE.bubble_bit){
            // C_DECODE.instr = OPP_MACRO_UNK;
            C_DECODE.oppCode = OPP_MACRO_UNK;
            // C_DECODE.p_taken = 0;
            // C_FETCH.bounce_bit = true;
            return;
        }
        if (C_DECODE.stall_bit)
            return;
        if (C_DECODE.oppCode == OPP_MACRO_HLT){
            return;
        }
        if (!RUN_BIT)
            return;
        C_DECODE.run_bit = 1;   
        C_DECODE.predicted_pc = C_FETCH.predicted_pc;
        C_DECODE.pc = C_FETCH.pc;
        C_DECODE.instr = C_FETCH.instr;
        C_DECODE.oppCode = get_opp_code(C_FETCH.instr);
        C_DECODE.p_taken = C_FETCH.p_taken;
        // printf("DECODE: Hexline is %08x\n", C_DECODE.instr);
        /* Do not touch the line below, we don't know why it works but it does*/
        if(is_retirable(C_DECODE.oppCode)){
            C_DECODE.retired = 1;
        } else{
            C_DECODE.retired = 0;
        }
    }
    
/*******************************************************************
                Decode helper to get the oppCode
*******************************************************************/

    int get_opp_code(uint32_t hexLine){
        if (hexLine ==  OPP_MACRO_UNASS) //<-- SPECIAL STALL CASE
                return OPP_MACRO_UNASS;

        //Bitmasks
        int bitMask = 0xFFE00000; //BitMask: 0111 1111 1110 0->
        int bitMaskSix = 0xFC000000;
        int bitMaskEight = 0xFF000000;  
        int bitMaskTen = 0xFFC00000 ;//BitMask: 0111 1111 1100 0->

        // //Actual Bit Counts
        int firstEleven = bitMask & hexLine;
        printf("DECODE: Hexline is %08x\n", hexLine);
        printf("DECODE: The current opp code in hex is %08x\n", firstEleven);



        /* Switch Statement for 11 digit opp codes*/
        switch(firstEleven){
            case 0x8B200000: //extended add
            case 0x8B000000:
                return OPP_MACRO_ADD;
                             //shifted add
            case 0xAB200000: //extended adds
            case 0xAB000000:
                return OPP_MACRO_ADDS;
                             //shifted adds
            case 0X8A000000:
            case 0X8AC00000:
            case 0X8A400000:
            case 0X8A800000:
                return OPP_MACRO_AND; 
            case 0XEA000000:
            case 0XEAC00000:
            case 0XEA400000:
            case 0XEA800000:
                return OPP_MACRO_ANDS;
            case 0XCA000000:
            case 0XCAC00000:
            case 0XCA400000:
            case 0XCA800000:
                return OPP_MACRO_EOR;
            case 0XF8400000: //1_11 1000 0100
                return OPP_MACRO_LDUR;
            case 0X38400000:
                return OPP_MACRO_LDURB;
            case 0x78400000:
                return OPP_MACRO_LDURH;
            case 0X1AC00000:
                if((((hexLine & 0x03) << 15) >> 25) == 0b001000)
                    return OPP_MACRO_SDIV;
            //need an if statement between LSL and LSR around 0x3
                return OPP_MACRO_LSL;
                return OPP_MACRO_LSR;
            case 0xD6000000:
                return OPP_MACRO_LSL;
            case 0XD2000000:
            case 0XD2400000:
            case 0XD2c00000:
            case 0XD2800000: 
                return OPP_MACRO_MOVZ;
            case 0Xaa000000:
            case 0Xaa400000:
            case 0Xaa800000:
            case 0Xaac00000:
                return OPP_MACRO_ORR;
            case 0xF8000000:
                return OPP_MACRO_STUR;
            case 0x38000000:
                return OPP_MACRO_STURB;
            case 0x78000000:
                return OPP_MACRO_STURH;            
            case 0XCB000000:
            case 0XCB800000:
                return OPP_MACRO_SUB;
            case 0XEB000000:
            case 0XEB200000:
                return OPP_MACRO_SUBS;
            case 0X9b000000:
                return OPP_MACRO_MUL;
            case 0Xd4400000:
                return OPP_MACRO_HLT;
                //same as SUBIS
            default:
                /* Ten Digit Opp Codes*/
                switch(hexLine & bitMaskTen){
                    case 0x91000000:
                    case 0x91400000:
                    case 0x91800000:
                    case 0x91c00000:
                        return OPP_MACRO_ADDI;
                    case 0xb1000000:
                    case 0xb1400000:
                    case 0xb1800000:
                    case 0xb1c00000:
                        return OPP_MACRO_ADDIS;
                    case 0xD3400000:
                        return OPP_MACRO_LSLI;
                    case 0xD1000000:
                    case 0xD1400000:
                    case 0xD1800000:
                    case 0xD1C00000:
                        return OPP_MACRO_SUBI;
                    case 0xF1000000:
                    case 0xF1400000:
                    case 0xF1800000:
                    case 0xF1C00000:
                        return OPP_MACRO_SUBIS;
                    default:
                        break;
                }
                switch (hexLine & bitMaskSix){
                    case 0x14000000:
                        return OPP_MACRO_B;
                    case 0x54000000:
                        return OPP_MACRO_BEQ;
                    case 0x94000000:
                        return OPP_MACRO_BL;
                    case 0xD6000000:
                        return OPP_MACRO_BR;
                    default:
                        break;
                }
                switch (hexLine & bitMaskEight){
                    case 0xB5000000:
                        return OPP_MACRO_CBNZ;
                    case 0xB4000000:
                        return OPP_MACRO_CBZ;
                    default:
                        break;
                }
                return OPP_MACRO_UNK;
        }
    }
    
/*******************************************************************
                            FETCH
*******************************************************************/

    void pipe_stage_fetch()
    {
        // printf("FETCH: Currently %d STALL_FOR_CYCLES for start_addr %08x\n", STALL_FOR_CYCLES, STALL_START_ADDR);
        uint32_t currOpp;
        int cacheHit;
        if ((!C_MEMORY.run_bit) || (!C_EXECUTE.run_bit) || (C_MEMORY.bubble_bit)){
            if (STALL_FOR_CYCLES > 0)
                STALL_FOR_CYCLES -= 1;
            return;
         }
        if (C_FETCH.bounce_bit){
            C_FETCH.bounce_bit = false;
            // C_DECODE.bubble_bit = false;
            unset_stall(PL_STAGE_MEMORY);
            return;
        }
        if (C_FETCH.stall_bit){
            /* Finished Stalling. Actually load mem values in */
            if (STALL_FOR_CYCLES == 0){
                unset_stall(PL_INCREMENT_FIFTY);
                cacheHit = cache_hit(I_CACHE, CURRENT_STATE.PC);
                currOpp = mem_read_32(CURRENT_STATE.PC);
                cache_update(STALL_START_ADDR, I_CACHE);
                C_EXECUTE.branch_stall_bit = false;
                C_FETCH.instr = currOpp;
                C_FETCH.pc = CURRENT_STATE.PC;  
                bp_predict(CURRENT_STATE.PC);


                if (C_FETCH.pseudo_stall_bit)
                {
                    C_FETCH.pseudo_stall_bit = false;
                    return;
                }
                C_FETCH.predicted_pc = CURRENT_STATE.PC;
                return;
            } else if (STALL_FOR_CYCLES > 0){
                insert_bubble(PL_STAGE_DECODE);
                STALL_FOR_CYCLES -= 1;
            }
            return;
        }
        if (C_FETCH.squash_bit){
            C_FETCH.squash_bit = false;
            return;
        }

        if (!RUN_BIT)
            return;
        // printf("FETCH: pc as hex is %" PRIx64 "\n", CURRENT_STATE.PC);
        uint64_t current_state_index = (CURRENT_STATE.PC & 0x7E0) >> 5;
        uint64_t current_state_tag = (CURRENT_STATE.PC & 0xFFFFFFFFFFFFF800) >> 11;
        cacheHit = cache_hit(I_CACHE, CURRENT_STATE.PC);
        printf("FETCH: cacheHit is %d\n", cacheHit);
        /*Cache Hit: Load from Cache*/
		if(cacheHit >= 0)
		{
			printf("ICACHE HIT: The index is %d\n", (int) current_state_index);
        	/*setting the curOpp to the instruction that is stored then update cache*/
        	uint64_t subblock_mask = (CURRENT_STATE.PC & (0x7 << 2)) >> 2;
        	currOpp = CACHE.i_cache[current_state_index].block[cacheHit].subblock_data[subblock_mask];
            // printf("FETCH: i_cache_index: %d, block: %d, subblock: %d\n", current_state_index, cacheHit, subblock_mask);
            // printf("FETCH currOpp: %08x\n", currOpp);
        	cache_update(CURRENT_STATE.PC, I_CACHE);
            C_EXECUTE.branch_stall_bit = false;
            C_FETCH.instr = currOpp;
            C_FETCH.pc = CURRENT_STATE.PC;  
            bp_predict(CURRENT_STATE.PC);

            if (C_FETCH.pseudo_stall_bit)
            {
            C_FETCH.pseudo_stall_bit = false;
            return;
            }
            C_FETCH.predicted_pc = CURRENT_STATE.PC;
		}
        /*Cache Miss: Trigger 50 cycle stall*/
		else
        {
        	printf("ICACHE MISS asdfas: The index is %d\n", (int) current_state_index);

            set_stall(PL_INCREMENT_FIFTY);

        	// currOpp = mem_read_32(CURRENT_STATE.PC);
         //    printf("currOpp is %d\n", currOpp);
         //    cache_update(CURRENT_STATE.PC, I_CACHE);
		}
    }
    
/*******************************************************************
    STALLING: Functions to implement stalling
*******************************************************************/

    /*Helper function to set stalls. Input is the stage you "start"
    stalling at. Then you work backwards. EX/ if you "start" at MEM, 
    then stall MEM, EXEC, DECODE AND FETCH*/
    void set_stall(int start_stage){
        
        switch(start_stage){
            case PL_STAGE_FETCH:
                printf("PL_STAGE_FETCH STALL SET\n");
                //set just fetch stall
                C_FETCH.stall_bit = true;
                break;
            case PL_STAGE_DECODE:
                printf("PL_STAGE_DECODE STALL SET\n");
                //set decode stall
                C_EXECUTE.branch_stall_bit = true;
                C_FETCH.stall_bit = true;
                C_DECODE.stall_bit = true;
                break;
            case PL_STAGE_EXECUTE:
                printf("PL_STAGE_EXECUTE STALL SET\n");
                //set execute stall
                C_FETCH.stall_bit = true;
                C_DECODE.stall_bit = true;
                C_EXECUTE.stall_bit = true;

                C_FETCH.bounce_bit = true;
                break;
            case PL_STAGE_MEMORY:
                printf("PL_STAGE_MEMORY STALL SET\n");
                //set memory stall
                C_FETCH.stall_bit = true;
                C_DECODE.stall_bit = true;
                C_EXECUTE.stall_bit = true;
                C_MEMORY.stall_bit = true;
                break;
            case PL_STAGE_WRITE:
                printf("PL_STAGE_WRITE STALL SET\n");
                /*I'm not quite sure when you would execute a stall on write?!?!?!
                Leaving it here for consistency reasons*/
                C_FETCH.stall_bit = true;
                C_DECODE.stall_bit = true;
                C_EXECUTE.stall_bit = true;
                C_MEMORY.stall_bit = true;
                C_WRITE.stall_bit = true;
                printf("Error, attempted stall at WB stage");
                break;
            case PL_INCREMENT_FIFTY:
                printf("PL_INCREMENT_FIFTY STALL SET\n");
                C_FETCH.stall_bit = true;
                // C_DECODE.stall_bit = true;
                // C_EXECUTE.stall_bit = true;
                // C_MEMORY.stall_bit = true;
                // C_WRITE.stall_bit = true;
                STALL_FOR_CYCLES = 49;
                STALL_START_ADDR = CURRENT_STATE.PC;

                break;
            case PL_DECODE_INCR_FIFTY:
                printf("PL_DECODE_INCR_FIFTY STALL SET\n");
                // C_FETCH.stall_bit = true;
                C_DECODE.stall_bit = true;
                C_EXECUTE.stall_bit = true;
                // C_MEMORY.stall_bit = true;
                C_MEMORY.bubble_bit = true;

                C_WRITE.stall_bit = true;
                STALL_FOR_CYCLES_DCACHE = 49;
                break;
            default:
                break;
        }
    }

    /* Inverse of set_stall*/
    void unset_stall(int start_stage){
        switch(start_stage){
            case PL_INCREMENT_FIFTY:
                printf("unincrement fifty\n");
                C_FETCH.stall_bit = false;
                C_DECODE.stall_bit = false;
                C_DECODE.bubble_bit = false;
                C_EXECUTE.stall_bit = false;
                C_MEMORY.stall_bit = false;
                C_WRITE.stall_bit = false;
                STALL_FOR_CYCLES = 0;
                break;

            case PL_STAGE_MEMORY:
                printf("unset pl_stage_memory\n");
                C_FETCH.stall_bit = false;
                C_DECODE.bubble_bit = false;
                C_DECODE.stall_bit = false;
                C_EXECUTE.stall_bit = false;
                C_MEMORY.bubble_bit = false;
                break;
            case PL_DECODE_INCR_FIFTY:
                printf("ending dcache stall\n");
                // C_FETCH.stall_bit = false;
                C_DECODE.stall_bit = false;
                C_EXECUTE.stall_bit = false;
                // C_MEMORY.stall_bit = false;
                C_MEMORY.bubble_bit = false;

                C_WRITE.stall_bit = false;
                STALL_FOR_CYCLES_DCACHE = 0;
                break;
            default:
                printf("You are trying to unstall an invalid stage\n");
                break;
        }
    }
    /*Fxn to unset stall bits */
    void unset_bits(void){
        if (C_WRITE.stall_bit)
            unset_stall(PL_STAGE_WRITE);
        if (C_MEMORY.stall_bit)
            unset_stall(PL_STAGE_MEMORY);
        if (C_EXECUTE.stall_bit)
            unset_stall(PL_STAGE_EXECUTE);
        if (C_DECODE.stall_bit)
            unset_stall(PL_STAGE_DECODE);
        if (C_FETCH.stall_bit)
            unset_stall(PL_STAGE_FETCH);
    }


    void exec_stall(int register_number){
        /*Check for LOAD in MEM, and "write" Execution operations for stalls*/
        //Check if the operation in MEMORY stage is a LOAD, and is possible of triggering stuff
        if (VERBOSE_FLAG) printf("EXEC_STALL: exec stall entered\n");
        if (is_load(C_MEMORY.oppCode)){
            //Check if the registers overlap between MEM, and current
            if (register_number == C_MEMORY.resultRegister){ 
                if (VERBOSE_FLAG) printf("EXEC STALL: EXEC STALL EXECUTED\n");
                set_stall(PL_STAGE_EXECUTE);
                // insert_bubble(PL_STAGE_MEMORY);
                C_EXECUTE.retired = 0;
            }
        }
    }

    void insert_bubble(int at_stage){
        switch(at_stage){
            case PL_STAGE_DECODE:
                printf("bubble inserted\n");

                C_FETCH.stall_bit = true;
                C_DECODE.bubble_bit = true;
                C_DECODE.stall_bit = false;
                C_DECODE.instr = OPP_MACRO_UNK;
                C_DECODE.oppCode = OPP_MACRO_UNK;
                // STALL_FOR_CYCLES = 1;
                break;
            /* Note this option actually inserts a bubble into decode. The code is the same
            as the above, but this is a bubble that is executed from a STALL, and not 
            as a result of a cache miss*/
            case PL_STAGE_MEMORY:
                printf("RAW bubble inserted\n");
                C_FETCH.stall_bit = true;
                C_DECODE.stall_bit = true;
                C_EXECUTE.stall_bit = true;

                C_MEMORY.bubble_bit = true;
                C_MEMORY.instr = OPP_MACRO_UNK;
                C_MEMORY.oppCode = OPP_MACRO_UNK;
                // STALL_FOR_CYCLES = 1;
                break;
            default:
                printf("You are trying to unstall an invalid stage\n");
                break;
            }
    }
/*******************************************************************
    FORWARDING: helper functions for forwarding
*******************************************************************/

    uint64_t forward(int register_number){
        printf("FORWARD: forward on register_number -----> %d\n", register_number);
        int64_t register_value = CURRENT_STATE.REGS[register_number];
        printf("FORWARD: register_value %d for register %d\n", (int) register_value, register_number);
        /*Check for wb forward*/
        int wb_reg = C_WRITE.resultRegister;
        if (wb_reg == register_number){
            printf("FORWARD: WB FORWARD TRIGGERED\n");


            //Check if the value in the WRITE stage is the same register
            register_value = C_WRITE.result; 
        }
        printf("FORWARD: returning WRITE mem forward %d\n", (int) register_value);
        /*Check for mem forward*/
        int mem_reg = C_MEMORY.resultRegister;
        if (mem_reg == register_number){

            //check if the value in the MEMORY stage is the same
            //registers are the same and we SHOULD forward ONE OF THE OPERANDS
            register_value = C_MEMORY.result; 
        }
        printf("FORWARD: returning MEMORY forward %d\n", (int) register_value);

        return register_value;
    }



/*******************************************************************
    OPPCODE HELPERS: helper functions to determine if a certain instruction 
    should pass through following step
*******************************************************************/

    int is_executeable(int opp_code){
        switch(opp_code){
            /* The following codes will not pass through executable */
            case OPP_MACRO_LDUR:
            case OPP_MACRO_LDURB:
            case OPP_MACRO_LDURH:
            case OPP_MACRO_MOVZ:
            case OPP_MACRO_STUR:
            case OPP_MACRO_STURB:
            case OPP_MACRO_STURH:
            case OPP_MACRO_STURW:
            case OPP_MACRO_UNK:
            case OPP_MACRO_UNASS:
                return false;
            default:
                return true;
        }
    }
    int has_exec_result(int opp_code){
        switch(opp_code){
            /* The following opp codes will EXECUTE but will NOT create a result or write to a register */
            case OPP_MACRO_B:
            case OPP_MACRO_BEQ:
            case OPP_MACRO_BL:
            case OPP_MACRO_BR:
            case OPP_MACRO_CBZ:
            case OPP_MACRO_CBNZ:
            case OPP_MACRO_HLT:
            case OPP_MACRO_UNK:
            case OPP_MACRO_UNASS: 
                return false;
            default:
                return true;
        }
    }
    int is_writeable(int opp_code){
        /* The following functions will not write to a register*/
        switch(opp_code){
            case OPP_MACRO_B:
            case OPP_MACRO_BEQ:
            case OPP_MACRO_BL:
            case OPP_MACRO_BR:
            case OPP_MACRO_CBZ:
            case OPP_MACRO_CBNZ:
            case OPP_MACRO_UNK:
            case OPP_MACRO_STUR:
            case OPP_MACRO_STURB:
            case OPP_MACRO_STURH:
            case OPP_MACRO_STURW:
            case OPP_MACRO_HLT:
                return false;
            default:
                return true;
        }
    }
    int is_memory(int opp_code){
        switch (opp_code){
            /* The following codes will access memory*/
            case OPP_MACRO_LDUR:
            case OPP_MACRO_LDURB:
            case OPP_MACRO_LDURH:
            case OPP_MACRO_STUR:
            case OPP_MACRO_STURB:
            case OPP_MACRO_STURH:
            case OPP_MACRO_STURW:
                return true;
            default:
                return false;
        }
    }
    int is_load(int opp_code){
        switch (opp_code){
            /* The following codes are LOADS*/
            case OPP_MACRO_LDUR:
            case OPP_MACRO_LDURB:
            case OPP_MACRO_LDURH:
                return true;
            default:
                return false;
        }
    }
    int is_stur(int opp_code){
    switch (opp_code){
        /* The following codes are LOADS*/
        case OPP_MACRO_STUR:
        case OPP_MACRO_STURB:
        case OPP_MACRO_STURH:
        case OPP_MACRO_STURW:
            return true;
        default:
            return false;
        }
    }
    int is_flaggable(int opp_code){
        switch(opp_code){
            case OPP_MACRO_ADDS:
            case OPP_MACRO_ADDIS:
            case OPP_MACRO_CMP:
            case OPP_MACRO_SUBS:
            case OPP_MACRO_SUBIS:
            case OPP_MACRO_ANDS:
            case OPP_MACRO_ORR:
            case OPP_MACRO_EOR:
                return true;
            default:
                return false;
        }
    }
    int is_retirable(int oppCode){
        //TODO: If have time: fix the HLT thing
        switch(oppCode){
            case OPP_MACRO_UNK:
            case OPP_MACRO_UNASS:
            case OPP_MACRO_HLT:
                return false;
            default:
                return true;
        }
    }
    int is_stall_branch(int oppCode){
        switch(oppCode){
            case OPP_MACRO_CBZ:
            case OPP_MACRO_CBNZ:
            case OPP_MACRO_BEQ:
            case OPP_MACRO_BR:
                return true;
            default:
                return false;
        }
    }
    int is_squash_branch(int oppCode){
        switch(oppCode){
            case OPP_MACRO_CBZ:
            case OPP_MACRO_CBNZ:
            case OPP_MACRO_BEQ:
            case OPP_MACRO_B:
                return true;
            default:
                return false;
        }
    }
    int is_uncond(int opp_code){
        switch(opp_code){
            /* Automatically always take the condition */
            case OPP_MACRO_B:
            case OPP_MACRO_BL:
            case OPP_MACRO_BR:
                return true;
            default:
                return false;
        }
    }

    int same_subblock(uint64_t stall_start_addr, uint64_t test_addr){
        printf("<------------\n");
        int i;
        int flag = false;
        for (i = 0; i < 8; i ++){
            if (test_addr == stall_start_addr + (i * 4)){
                flag = true;
                printf("*********************************SAME SUBBLOCK\n");
            }

        }
        return flag;
    }
/* Helper function to distribute stores and loads */
