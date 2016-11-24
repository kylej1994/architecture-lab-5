#include "pipe.h"
#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>



/**********************************************
    Helper functions for the execute function
    
    TODO: 
    1. Removing Stores and Loads
    
***********************************************/


uint64_t shiftReg(uint64_t m, int shift_type, int shift_amount){
    // printf("We are doing a %d shift of amount %d on", shift_type, shift_amount);
    // printf("%" PRIu64 "\n", m);
    switch (shift_type){ // <-- implement different case types
        case 0: //LSL
            return m << shift_amount;
        case 1: //LSR
            return m >> shift_amount;
        case 2: //ASR arithmetic shift right. 
            return ((int64_t) m) >> shift_amount;
        case 3: //ROR rotate right
            return (m >> shift_amount) | (m << (sizeof(uint64_t) - shift_amount));
        default:
            break;
    }
    return (uint64_t)-1;
}

uint64_t zeroExtend(uint32_t data){

    uint64_t zeros = 0;
    return zeros | data; //<-- will this work?
}

uint64_t signExtend(int32_t data){
    uint64_t result;
    if (0x80000000 & data){
        uint64_t mask = ((((uint64_t)-1) >> 31) << 31);
        result = mask | ((uint64_t) data);
    } else{
        result = zeroExtend(data);
    }
    return result;
}

uint64_t signExtendImm(int32_t data, int extend_bit_at){
    int32_t max_bit = 0x1 << (extend_bit_at - 1);
    if (VERBOSE_FLAG) printf("max_bit is %08x\n", (int32_t)max_bit);
    if (VERBOSE_FLAG) printf("data    is %08x\n", data);
    if (max_bit & data){
        uint64_t mask = (-1 << extend_bit_at); //0xFF...
        int64_t result = mask | data;
        if (VERBOSE_FLAG) printf("mask is %08x\n", (int32_t) mask);
        if (VERBOSE_FLAG) printf("result is %08x\n", (int32_t)result);
        return result;
    }else{
        if (VERBOSE_FLAG) printf("else statement triggered\n");
        return (uint64_t)data;
    }
}

/*
    Function to set the NEXT_STATE registers for ADD
*/
void add(uint32_t hexLine){
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int rM = (0x001F0000 & hexLine) >> 16;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);
    if (VERBOSE_FLAG) printf("rN is %d and rM is %d and rD is %d\n", rN, rM, rD);
    int64_t result  = rN_v + rM_v;
    if (rD == 31){
        C_EXECUTE.resultRegister = STACK_POINTER;
        C_EXECUTE.result = result;
    }
    else{
        C_EXECUTE.resultRegister = rD;
        C_EXECUTE.result = result;
    }
    return;
}

void addi(uint32_t hexLine){
    if (VERBOSE_FLAG) printf("add immediate executed\n");
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int shiftType = (0x00C00000 & hexLine) >> 22;
    int imm = (0x003FFC00 & hexLine) >> 10;
    imm = zeroExtend(imm);
    exec_stall(rN);
    uint64_t rN_v = forward(rN);
    if (VERBOSE_FLAG) printf("rN is %d and rD is %d\n", rN, rD);
    int64_t result  = rN_v + imm;
    if (rD == 31){
        C_EXECUTE.resultRegister = STACK_POINTER;
        C_EXECUTE.result = result;
    }
    else{
        C_EXECUTE.resultRegister = rD;
        C_EXECUTE.result = result;
    }
    return;
}

void adds(uint32_t hexLine){
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int rM = (0x001F0000 & hexLine) >> 16;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);
    if (VERBOSE_FLAG) printf("rN is %d and rM is %d and rD is %d\n", rN, rM, rD);
    int64_t result  = rN_v + rM_v;
    //NEXT_STATE.REGS[rD] = result;
    C_EXECUTE.resultRegister = rD;
    C_EXECUTE.result = result;


    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0 && rM_v < 0)
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    //printf("C_EXECUTE.result:%" PRIu64 "\n", C_EXECUTE.result]);
    return;
}

void addis(uint32_t hexLine){
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int shiftType = (0x00C00000 & hexLine) >> 22;
    int imm = (0x003FFC00 & hexLine) >> 10;
    imm = zeroExtend(imm);

    if (VERBOSE_FLAG) printf("rN is %d and rD is %d\n", rN, rD);
    exec_stall(rN);
    uint64_t rN_v = forward(rN);
    int64_t result  = rN_v + imm;
    C_EXECUTE.resultRegister = rD;
    C_EXECUTE.result = result;

    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0)
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    if (rN_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    return;
}


void and(uint32_t hexLine){
    if (VERBOSE_FLAG) printf("Currently in AND\n");
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int rM = (0x001F0000 & hexLine) >> 16;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);
    int shiftamnt = (0x0000FC00 & hexLine) >> 10;
    int shiftMask = 0x00C00000;
    int shiftType = (shiftMask & hexLine) >> 22;

    if (VERBOSE_FLAG) printf("rN is %d and rM is %d and rD is %d and imm is %d\n", rN, rM, rD, shiftamnt);
    int64_t result = rN_v & shiftReg(rM_v, shiftType, shiftamnt);
    C_EXECUTE.resultRegister = rD;
    C_EXECUTE.result = result;
    return;
}
void ands(uint32_t hexLine){
    if (VERBOSE_FLAG) printf("Currently in ANDS\n");
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int rM = (0x001F0000 & hexLine) >> 16;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);
    int shiftamnt = (0x0000FC00 & hexLine) >> 10;
    int shiftMask = 0x00C00000;
    int shiftType = (shiftMask & hexLine) >> 22;

    int64_t result = rN_v & shiftReg(rM_v, shiftType, shiftamnt);
    C_EXECUTE.resultRegister = rD;
    C_EXECUTE.result = result;

    //SET FLAGS
    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0 && rM_v < 0)
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    return;
}

void eor(uint32_t hexLine){
    int rD = 0x0000001F & hexLine;
    int rN = (0x000003E0 & hexLine) >> 5;
    int rM = (0x001F0000 & hexLine) >> 16;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);
    int shiftamnt = (0x0000FC00 & hexLine) >> 10;
    int shiftMask = 0x00C00000;
    int shiftType = (shiftMask & hexLine) >> 22;

    int64_t result = rN_v ^ shiftReg(rM_v, shiftType, shiftamnt);
    C_EXECUTE.resultRegister = rD;
    C_EXECUTE.result = result;

    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0 && rM_v < 0)
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    return;
}

void branch(uint64_t hexLine){
    int64_t imm26 = (~0xFC000000 & hexLine);
    imm26 = signExtendImm(imm26, 26) * 4;
    int64_t offset = imm26;
    uint64_t FETCH_PC = CURRENT_STATE.PC;
    CURRENT_STATE.PC = C_EXECUTE.pc + offset;
    bp_update(C_EXECUTE.pc, CURRENT_STATE.PC, true, false);
    if (CURRENT_STATE.PC != C_EXECUTE.predicted_pc)
       squash(PL_STAGE_DECODE);
    if (STALL_FOR_CYCLES > 0){
        if (FETCH_PC != CURRENT_STATE.PC)
            unset_stall(PL_INCREMENT_FIFTY);
    }
    return;
}

void branchCond(uint64_t hexLine){
    
    int imm19 = ((0x00FFFFE0 & hexLine) >> 5);
    int64_t signExtended = signExtendImm(imm19, 19) * 4;
    if (VERBOSE_FLAG) {
        printf("signExtended is 0x%" PRIx64 "\n", signExtended);
        printf("Signextended as an integer is %" PRId64 "\n", signExtended);
        printf("Signextended as a 32 bit intger is %d\n", (int32_t)signExtended);
    }
    int cond = (0x0000000F & hexLine);
    int macro;
    switch (cond){
        case 0:
            macro = OPP_MACRO_BEQ;
            break;
        case 1:
            macro = OPP_MACRO_BNE;
            break;
        case 12:
            macro = OPP_MACRO_BGT;
            break;
        case 11:
            macro = OPP_MACRO_BLT;
            break;
        case 10:
            macro = OPP_MACRO_BGE;
            break; 
        case 13:
            macro = OPP_MACRO_BLE;
            break;
        default:
            if (VERBOSE_FLAG) printf("This code does not exist\n");
            break;
    }
    int result = 0;
    if (macro == OPP_MACRO_BEQ){
        if (C_MEMORY.FLAG_Z == 1)
            result = 1;
    }
    if (macro == OPP_MACRO_BNE){
        if (C_MEMORY.FLAG_Z == 0)
            result = 1;
    }
    if (macro == OPP_MACRO_BLT){
        if (C_MEMORY.FLAG_N != C_MEMORY.FLAG_V)
            result = 1;
    }
    if (macro == OPP_MACRO_BGT){
        if ((C_MEMORY.FLAG_Z == 0) && (C_MEMORY.FLAG_N == C_MEMORY.FLAG_V)){
            result = 1;
        }
    }
    if (macro == OPP_MACRO_BLE){
        if (!((C_MEMORY.FLAG_Z == 0) && (C_MEMORY.FLAG_N == C_MEMORY.FLAG_V)))
            result = 1;
    }
    if (macro == OPP_MACRO_BGE){
        if (C_MEMORY.FLAG_N == C_MEMORY.FLAG_V)
            result = 1;
    }
    uint64_t FETCH_PC = CURRENT_STATE.PC;
    if (result){
        if (VERBOSE_FLAG) printf("BRANCHCOND: BRANCH PC                : 0x%" PRIx64 "\n", C_EXECUTE.pc);
        if (VERBOSE_FLAG) printf("BRANCHCOND: PREDICTED PC                : 0x%" PRIx64 "\n", C_EXECUTE.predicted_pc);
        if (VERBOSE_FLAG) printf("BRANCHCOND: PC                : 0x%" PRIx64 "\n", CURRENT_STATE.PC);
        if (!C_EXECUTE.p_taken){
        	if (VERBOSE_FLAG) printf("BRANCHCOND: Branch taken, predicted not taken\n");
        	CURRENT_STATE.PC = C_EXECUTE.pc + signExtended; 
            bp_update(C_EXECUTE.pc, CURRENT_STATE.PC, true, true); 	
        	if (C_EXECUTE.predicted_pc != CURRENT_STATE.PC)
        		squash(PL_STAGE_DECODE);
        } 
        else {
        		if (VERBOSE_FLAG) printf("BRANCHCOND: Branch taken, predicted taken\n");
        		CURRENT_STATE.PC = C_EXECUTE.predicted_pc + 4;
                bp_update(C_EXECUTE.pc, CURRENT_STATE.PC - 4, true, true); 
	   }
        if (VERBOSE_FLAG) printf("BRANCHCOND: PC                : 0x%" PRIx64 "\n", CURRENT_STATE.PC);
        if (VERBOSE_FLAG) printf("BRANCHCOND: BCOND evaluated to TRUE\n");
        C_EXECUTE.pc = CURRENT_STATE.PC;
    } 
    else{
        if (VERBOSE_FLAG) printf("BRANCHCOND: BCOND evaluated to FALSE\n");
        if (C_EXECUTE.p_taken){
            if (VERBOSE_FLAG) printf("BRANCHCOND: Branch not taken, predict branch taken\n");
            CURRENT_STATE.PC = C_EXECUTE.pc + 4;
	    printf("icache hit (0x%" PRIx64") at cycle %d\n", C_FETCH.pc, stat_cycles + 1);	
           if (CURRENT_STATE.PC != C_EXECUTE.predicted_pc)
		      squash(PL_STAGE_DECODE);
	    } else{
            if (VERBOSE_FLAG) printf("BRANCHCOND: Branch not taken, predict branch not taken\n");
        }
        bp_update(C_EXECUTE.pc, CURRENT_STATE.PC, false, true);
        if (VERBOSE_FLAG) printf("BRANCHCOND: CURRENT_STATE.PC                : 0x%" PRIx64 "\n", CURRENT_STATE.PC);
        if (VERBOSE_FLAG) printf("BRANCHCOND: C_DECODE.PC                : 0x%" PRIx64 "\n", C_FETCH.pc);
        /* If you predict you would take the branch, but did not take it then clear pipeline*/

    }
    /*Check if you need to short-circuit your stall*/
    if (STALL_FOR_CYCLES > 0){
        // if (FETCH_PC != CURRENT_STATE.PC)
        //     unset_stall(PL_INCREMENT_FIFTY);
        if (!same_subblock(FETCH_PC, CURRENT_STATE.PC)){
            if (VERBOSE_FLAG) printf("BRANCHCOND: SHORT-CIRCUIT STALL\n");
            if (STALL_FOR_CYCLES > 0) printf("icache bubble (%d)\n", STALL_FOR_CYCLES);
	    unset_stall(PL_INCREMENT_FIFTY);
        } //else C_FETCH.cancel_stall = true;
    }
    return;
}

void cbznz(uint64_t hexLine, int isnz){
    int rt = 0x0000001F & hexLine;
    int imm19 = 0x00FFFFE0 & hexLine;
    bool isZero = (CURRENT_STATE.REGS[rt] == 0);
    if (isnz)
        isZero = !isZero;
    uint64_t FETCH_PC = CURRENT_STATE.PC;
    /* Taken Case */
    if (isZero){
        int64_t offset = signExtendImm(imm19, 19) * 4;
        if (VERBOSE_FLAG) printf("BRANCH PC                : 0x%" PRIx64 "\n", C_EXECUTE.pc);
        if (VERBOSE_FLAG) printf("PREDICTED PC                : 0x%" PRIx64 "\n", C_EXECUTE.predicted_pc);
        if (VERBOSE_FLAG) printf("PC                : 0x%" PRIx64 "\n", CURRENT_STATE.PC);
        if (!C_EXECUTE.p_taken){
            if (VERBOSE_FLAG) printf("Branch taken, predicted not taken\n");
            CURRENT_STATE.PC = C_EXECUTE.pc + offset; 
            bp_update(C_EXECUTE.pc, CURRENT_STATE.PC, true, true);  
            if (C_EXECUTE.predicted_pc != CURRENT_STATE.PC)
                squash(PL_STAGE_DECODE);
        } 
        else {
                if (VERBOSE_FLAG) printf("Branch taken, predicted taken\n");
                CURRENT_STATE.PC = C_EXECUTE.predicted_pc + 4;
                bp_update(C_EXECUTE.pc, CURRENT_STATE.PC - 4, true, true); 
       }
        if (VERBOSE_FLAG) printf("PC                : 0x%" PRIx64 "\n", CURRENT_STATE.PC);
        // bp_update(C_EXECUTE.pc, CURRENT_STATE.PC, true, true); 
        if (VERBOSE_FLAG) printf("The pc is at %" PRIu64 "\n", CURRENT_STATE.PC);
        if (VERBOSE_FLAG) printf("CBZ/CBNZ evaluated to TRUE\n");
        C_EXECUTE.pc = CURRENT_STATE.PC;
    /*Not taken case*/
    } else{
        if (VERBOSE_FLAG) printf("CBZ/CBNZ evaluated to FALSE\n");
        if (C_EXECUTE.p_taken){
            if (VERBOSE_FLAG) printf("Branch not taken, predict branch taken\n");
            CURRENT_STATE.PC = C_EXECUTE.pc + 4;
           if (CURRENT_STATE.PC != C_EXECUTE.predicted_pc)
              squash(PL_STAGE_DECODE);
        } else{
            if (VERBOSE_FLAG) printf("Branch not taken, predict branch not taken\n");
        }
        bp_update(C_EXECUTE.pc, CURRENT_STATE.PC, false, true);
        if (VERBOSE_FLAG) printf("CURRENT_STATE.PC                : 0x%" PRIx64 "\n", CURRENT_STATE.PC);
        if (VERBOSE_FLAG) printf("C_DECODE.PC                : 0x%" PRIx64 "\n", C_FETCH.pc);
        /* If you predict you would take the branch, but did not take it then clear pipeline*/
    }
    if (STALL_FOR_CYCLES > 0){
        // if (FETCH_PC != CURRENT_STATE.PC)
        //     unset_stall(PL_INCREMENT_FIFTY);
        if (!same_subblock(FETCH_PC, CURRENT_STATE.PC))
            unset_stall(PL_INCREMENT_FIFTY);
    }
    return;
}

void ldur(uint64_t hexLine){
    int rt = 0x0000001F & hexLine;
    int rn = (0x000003E0 & hexLine) >> 5;
    // exec_stall(rn);
    uint64_t imm9 = 0 | ((0x001FF000 & hexLine) >> 12);
    int size = (0xC0000000 & hexLine) >> 30;
    int regsize = 32;
    uint64_t address = CURRENT_STATE.REGS[rn];
    if (size == 3)
        regsize = 64;

    if (rn == 31)
        address = CURRENT_STATE.REGS[STACK_POINTER];

    address = address + imm9;
    uint64_t result = address; 
    C_EXECUTE.resultRegister = rt;
    if (VERBOSE_FLAG) printf("result register is %d\n", C_EXECUTE.resultRegister);
    C_EXECUTE.result = result;
    return;
}
//BOTH LOADS ARE NEVER CALLED
void ldurbh(uint64_t hexLine, int isByte){
    int rt = 0x0000001F & hexLine;
    int rn = (0x000003E0 & hexLine) >> 5;
    // exec_stall(rn);
    uint64_t imm9 = 0 | ((0x001FF000 & hexLine) >> 12); //<-- zero extend this
    int size = (0xC0000000 & hexLine) >> 30;
    int regsize = 32;
    uint64_t address = CURRENT_STATE.REGS[rn];
    if (size == 3)
        regsize = 64;

    if (rn == 31)
        address = CURRENT_STATE.REGS[STACK_POINTER];
    else
        address = CURRENT_STATE.REGS[rn];

    address = address + imm9;
    int64_t result = address;
   //  uint32_t data = mem_read_32(address) & 0x0000FFFF;
    // if (isByte){
    //data = data & 0x000000FF;
    //}
    C_EXECUTE.resultRegister = rt;
    C_EXECUTE.result = result;
    // C_EXECUTE.result = zeroExtend(data); 
    return;
}
/**
Execute the LSL Command
shift rn to the right the shift amount
**/
void lsli(uint32_t hexline)
{
    if (VERBOSE_FLAG) printf("Shifting immediate function triggered\n");
    int rD = 0x0000001F & hexline;
    int rN = (0x000003E0 & hexline) >> 5;
    int immr = ((0X3F << 16) & hexline) >> 16;
    exec_stall(rN);
    uint64_t rN_v = forward(rN);
    //right shift
    int shift = (0X1F <<10  & hexline) >> 10;
    if (shift == 0X1F){
        if (VERBOSE_FLAG) printf("right shift triggered\n");
        uint64_t result = ((unsigned) rN_v >> immr);
        C_EXECUTE.resultRegister = rD;
        C_EXECUTE.result = result;
        return;
    }
    //left shift
    if (VERBOSE_FLAG) printf("left shift triggered by amount %d\n", immr);
    uint64_t result = shiftReg(rN_v, 0, (~immr +1));
    //printf(" value %" PRIu64 " placed in dest register\n", result);
    C_EXECUTE.resultRegister = rD;
    C_EXECUTE.result = result;
    return;
}

void lsl(uint64_t hexline)
{
    if (VERBOSE_FLAG) printf("Shifting function triggered\n");
    int rM = (0x001F0000 & hexline) >> 16;
    int rN = (0x000003E0 & hexline) >> 5;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    uint64_t rN_v = forward(rN);
    int shiftamnt = rM % 64; //mod 64 to find number of bits to left shift
    int shiftType = (0X0000FC00  & hexline) >> 10;
    //Right shift case
    if (shiftType == 0x9){
        if (VERBOSE_FLAG) printf("lsr shift\n");
        uint64_t result = shiftReg(rN_v, 4 , shiftamnt); //shiftReg(CURRENT_STATE.REGS[rN], 4, imm21_16);
        C_EXECUTE.result = result;
        C_EXECUTE.resultRegister = rD;
        return;
    }
    //Left shift case
    if (VERBOSE_FLAG) printf("lsl shift by amount %d\n", shiftamnt);
    uint64_t result = shiftReg(rN_v, 0, shiftamnt); //shiftReg(CURRENT_STATE.REGS[rN], 0, imm21_16);    
    C_EXECUTE.result = result;
    C_EXECUTE.resultRegister = rD;
    return;
}
/**
    move a 16 bit value to a register 
    and then Left shift 0, 16, 32, or 48 bits based on the setting of the 22 and 21 bits
**/
void movz(uint64_t hexline)
{
    int imm20_5 = ((0XFFFF) << 5 & hexline) >> 5;
    int rD = 0x0000001F & hexline;
    //int hw = ((0b11 << 21) & hexline) >> 21;
    C_EXECUTE.result = imm20_5;
    C_EXECUTE.resultRegister = rD;
    int hw = ((0b11 << 21) & C_EXECUTE.instr) >> 21;
    switch(hw){
        case 0:
            C_EXECUTE.result =  shiftReg(C_EXECUTE.result, 0, hw*16);
            break;
        case 1:
            C_EXECUTE.result = shiftReg(C_EXECUTE.result, 0, hw*16); 
            break;
        case 2:
            C_EXECUTE.result =  shiftReg(C_EXECUTE.result, 0, hw*16);
            break;
        case 4:
            C_EXECUTE.result =  shiftReg(C_EXECUTE.result, 0, hw*16);
            break;
        default:
            if (VERBOSE_FLAG) printf("MOVZ did not process correctly");
    }
    return;
}

/*
1. shift rM by trhe immediate
2. bitwise OR result with rN
3. store in rD 
*/
void orr(uint64_t hexline)  
{
    int imm15_10 = ((0X3F) << 9 & hexline) >> 9;
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);
    int shift = ((0X3) << 22 & hexline) >> 22; 
    int shiftamnt = (0x0000FC00 & hexline) >> 10;
    int shiftMask = 0x00C00000;
    int shiftType = (shiftMask & hexline) >> 22;

    int64_t result = rN_v | shiftReg(rM_v, shiftType, shiftamnt);
    C_EXECUTE.result = result;
    C_EXECUTE.resultRegister = rD;

    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0 && rM_v < 0)
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    return;
}

void stur(uint32_t hexline)
{
    uint32_t rT = (hexline & 0x1F);
    int rN = (0x000003E0 & hexline) >> 5;
    int imm20_12 = signExtend(((0X1FF << 12)& hexline) >>12);
    //uint32_t d1 = ((CURRENT_STATE.REGS[rT] & 0xFFFFFFFF00000000) >> 32);
    //uint32_t d2 = (CURRENT_STATE.REGS[rT] & 0xFFFFFFFF);
    C_EXECUTE.result = CURRENT_STATE.REGS[rN] + imm20_12;
    //uint32_t addr1 =(CURRENT_STATE.REGS[rN] + 0x4 + imm20_12 );
    //uint32_t addr2 =(CURRENT_STATE.REGS[rN] + imm20_12);
    C_EXECUTE.resultRegister = rT;
    return;
}

void sturb(uint32_t hexline)
{
    uint32_t rT = (hexline & 0x1F);
    int rN = (0x000003E0 & hexline) >> 5;
    int imm20_12 = signExtend(((0X1FF << 12) & hexline) >>12);
    // uint64_t value = CURRENT_STATE.REGS[rN] & 0xFF;
    C_EXECUTE.result = CURRENT_STATE.REGS[rN] + imm20_12; 
    C_EXECUTE.resultRegister = rT;
    /* Need to remove the memory writing from here,
     happens in the write back stage
     */
    // mem_write_32(C_EXECUTE.resultRegister, value);
    //printf("first 32: %" PRIu64 "\n", CURRENT_STATE.REGS[Rt] & 0xFF);
    return;
}

void sturh(uint32_t hexline)
{
    int rT = (0XFF & hexline);
    int rN = (0x000003E0 & hexline) >> 5;
    int imm20_12 = signExtend((0X1FF << 12 & hexline) >> 12);
    uint64_t value = (uint32_t)(CURRENT_STATE.REGS[rN] & 0xFFFF);
    C_EXECUTE.result = CURRENT_STATE.REGS[rN] + imm20_12; 
    C_EXECUTE.resultRegister = value;
    /* Need to remove the memory writing from here,
     happens in the write back stage
     */
    // mem_write_32(C_EXECUTE.resultRegister+ imm20_12 ,value);
    return;
}

void sturw(uint32_t hexline)
{
    int rT = (0XFF & hexline);
    int rN = (0x000003E0 & hexline) >> 5;
    int imm20_12 = signExtend(((0X1FF >> 12) & hexline >>12));
    uint64_t value = (uint32_t)(CURRENT_STATE.REGS[rT] & 0xFFFFFFFF);
    C_EXECUTE.result = CURRENT_STATE.REGS[rN] + imm20_12; 
    C_EXECUTE.resultRegister = value;
    /* Need to remove the memory writing from here,
     happens in the write back stage
     */
    // mem_write_32(C_EXECUTE.resultRegister, value);
    return;
}

void sub(uint32_t hexline)
{
    uint64_t zero = 0x0;
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    exec_stall(rM);
    int64_t rN_v = forward(rN);
    int64_t rM_v = forward(rM);
    int64_t value = rN_v - (rM_v | zero);
    C_EXECUTE.result = value;
    C_EXECUTE.result = rD;
    return;
}

void subi(uint32_t hexline)
{
    int32_t imm21_10 = ((0XFFF << 10) & hexline) >> 10;
    int32_t rN = (0x000003E0 & hexline) >> 5;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    int64_t rN_v = forward(rN);
    int shift = ((0X3) << 21 & hexline) >> 21; 
    int64_t value = 0;

    switch(shift){
        case 0:
            value =  rN_v - imm21_10;
            if (VERBOSE_FLAG) printf("rN_v is %d and imm21_10 is %d\n", (int32_t)rN_v, imm21_10);
            break;
        case 1:
            value =  rN_v - imm21_10;
            break;
        default:
            if (VERBOSE_FLAG) printf("SUBI did not process correctly");
    }
    
    C_EXECUTE.result = value;
    C_EXECUTE.resultRegister = rD;

return;
}

void subs(uint32_t hexline)
{
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    exec_stall(rM);
    int64_t rN_v = forward(rN);
    int64_t rM_v = forward(rM);
    if (VERBOSE_FLAG) {
        printf("CURRENT_STATE.REGS[0]: %d\n", (int) CURRENT_STATE.REGS[0]);
        printf("rN is %d and rM is %d and rD is %d\n", rN, rM, rD);
        printf("SUBS: rN_v %d, rM_v %d\n", (int) rN_v, (int) rM_v);
    }
    int64_t result = 0;
    result =  rN_v - rM_v;
    if (VERBOSE_FLAG) printf("result is %" PRId64 "\n", result);
    C_EXECUTE.result = result;
    C_EXECUTE.resultRegister = rD;
    
    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0 && rM_v > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0 && rM_v < 0){
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    }
    if (rN_v > 0 && rM_v < 0){
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    }
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    //printf("C_EXECUTE.result: %" PRIu64 "\n", C_EXECUTE.result);
    return;
}

void subsi(uint32_t hexline)
{
    uint64_t zero = 0x0;
    int imm21_10 = ((0XFFF << 10) & hexline) >> 10;
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    uint64_t rN_v = forward(rN);
    int shift = ((0X3) << 21 & hexline) >> 21;
    uint64_t result = 0; 
    if (VERBOSE_FLAG) printf("rN is %d and rM is %d and rD is %d\n", rN, rM, rD);

    switch(shift){
        case 0:
            result = rN_v - imm21_10;
            break;
        case 1:
            result = rN_v - (imm21_10 << 12);
            break;
        default :
            if (VERBOSE_FLAG) printf("SUBIS did not process correctly");
    }

    C_EXECUTE.result = result;
    C_EXECUTE.resultRegister = rD;
    
    
    C_EXECUTE.FLAG_N = 0;
    if (result < 0)
        C_EXECUTE.FLAG_N = 1;

    C_EXECUTE.FLAG_C = 0;
    if (rN_v > 0 && imm21_10 > 0)
        if (result < 0)
            C_EXECUTE.FLAG_C = 1;
    //If two negatives results in a positive then thats overflow
    C_EXECUTE.FLAG_V = 0;
    if (rN_v < 0 && imm21_10 < 0){
        if (result > 0)
            C_EXECUTE.FLAG_V = 1;
    }
    if (rN_v > 0 && imm21_10 < 0){
        if (result < 0)
            C_EXECUTE.FLAG_V = 1;
    }
    C_EXECUTE.FLAG_Z = 0;
    if (result == 0)
        C_EXECUTE.FLAG_Z = 1;
    //printf("C_EXECUTE.result: %" PRIu64 "\n", C_EXECUTE.result);
    return;
}

void mul(uint32_t hexline)
{
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);

    C_EXECUTE.result = rN_v * rM_v;
    C_EXECUTE.resultRegister = rD;
}

void sdiv(uint32_t hexline){
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);

    C_EXECUTE.result = rN_v / rM_v;
    C_EXECUTE.resultRegister = rD;
}

void udiv(uint32_t hexline){
    int rN = (0x000003E0 & hexline) >> 5;
    int rM = (0x001F0000 & hexline) >> 16;
    int rD = 0x0000001F & hexline;
    exec_stall(rN);
    exec_stall(rM);
    uint64_t rN_v = forward(rN);
    uint64_t rM_v = forward(rM);

    C_EXECUTE.result = rN_v / rM_v;
    C_EXECUTE.resultRegister = rD;
}

void halt(){
    C_EXECUTE.run_bit = 0;
    return;
}

void bl( uint32_t hexline){
    uint64_t value = CURRENT_STATE.REGS[STACK_POINTER] + 4;
    C_EXECUTE.result = value;
    C_EXECUTE.resultRegister = 30;
    if (CURRENT_STATE.PC != C_FETCH.pc)
        squash(PL_STAGE_DECODE);
    return;
}

void br(uint32_t hexline){
    int rN = (0x000003E0 & hexline) >> 5;
    C_EXECUTE.result = CURRENT_STATE.REGS[rN];
    C_EXECUTE.resultRegister = CURRENT_STATE.REGS[STACK_POINTER];
    if (CURRENT_STATE.PC != C_FETCH.pc)
        squash(PL_STAGE_DECODE);
    return;
}

/*******************************************************************
            Calculate Function to switch on
*******************************************************************/

    void calculate(uint32_t instr){
        switch(instr){
            case OPP_MACRO_ADD:
                add(C_EXECUTE.instr);
                return;
            case OPP_MACRO_ADDI:
                addi(C_EXECUTE.instr);
                return;
            case OPP_MACRO_ADDIS:
                addis(C_EXECUTE.instr);
                return;
            case OPP_MACRO_ADDS:
                adds(C_EXECUTE.instr);
                return;
            case OPP_MACRO_AND:
                and(C_EXECUTE.instr);
                return;
            case OPP_MACRO_ANDS:
                ands(C_EXECUTE.instr);
                return;
            case OPP_MACRO_B:
                branch(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BEQ:
                branchCond(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BNE:
                branchCond(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BGT:
                branchCond(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BLT:
                branchCond(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BGE:
                branchCond(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BLE:
                branchCond(C_EXECUTE.instr);
                return;
            case OPP_MACRO_CBNZ: 
                cbznz(C_EXECUTE.instr,false);
                return;
            case OPP_MACRO_CBZ:
                cbznz(C_EXECUTE.instr, true);
                return;
            case OPP_MACRO_EOR:
                eor(C_EXECUTE.instr);
                return;
            case OPP_MACRO_LDUR:
                ldur(C_EXECUTE.instr);
                return;
            case OPP_MACRO_LDURB:
                ldurbh(C_EXECUTE.instr, true);
                return;
            case OPP_MACRO_LDURH:
                ldurbh(C_EXECUTE.instr,false);
                return;
            case OPP_MACRO_LSL:
                lsl(C_EXECUTE.instr);
                return;
            case OPP_MACRO_LSR:
                lsl(C_EXECUTE.instr);
                return;
            case OPP_MACRO_MOVZ:
                movz(C_EXECUTE.instr);
                return;
            case OPP_MACRO_ORR:
                orr(C_EXECUTE.instr);
                return;
            case OPP_MACRO_STUR:
                stur(C_EXECUTE.instr);
                return;
            case OPP_MACRO_STURB:
                sturb(C_EXECUTE.instr);
                return;
            case OPP_MACRO_STURH:
                sturh(C_EXECUTE.instr);
                return;
            case OPP_MACRO_STURW:
                sturw(C_EXECUTE.instr);
                return;
            case OPP_MACRO_SUB:
                sub(C_EXECUTE.instr);
                return;
            case OPP_MACRO_SUBI:
                subi(C_EXECUTE.instr);
                return;
            case OPP_MACRO_SUBIS:
                subsi(C_EXECUTE.instr);
                return;
            case OPP_MACRO_SUBS:
                subs(C_EXECUTE.instr);
                return;
            case OPP_MACRO_MUL:
                mul(C_EXECUTE.instr);
                return;
            case OPP_MACRO_SDIV:
                sdiv(C_EXECUTE.instr);
                return;
            case OPP_MACRO_UDIV:
                udiv(C_EXECUTE.instr);
                return;
            case OPP_MACRO_HLT:
                halt();
                return;
            case OPP_MACRO_CMP:
                subs(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BL:
                bl(C_EXECUTE.instr);
                return;
            case OPP_MACRO_BR:
                br(C_EXECUTE.instr);
                return;
            case OPP_MACRO_LSLI:
                lsli(C_EXECUTE.instr);
                return;
            case OPP_MACRO_NOP:
            // case OPP_MACRO_HLT:
                if (VERBOSE_FLAG) printf("EXECUTE No op\n");
                return;
            case OPP_MACRO_UNK:
            case OPP_MACRO_UNASS:
                if (VERBOSE_FLAG) printf("EXECUTE ERR: You have reach unassigned or unkown\n" );
                return;
        }
}
