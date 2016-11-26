/*
 * CMSC 22200, Fall 2016
 *
 * ARM pipeline timing simulator
 *
 * Reza Jokar, Gushu Li, 2016
 */

#ifndef _PIPE_H_
#define _PIPE_H_

#include "shell.h"
#include "stdbool.h"
#include "bp.h"
#include <limits.h>
#include <inttypes.h>

#define true 1
#define false 0

#define STACK_POINTER 28

#define OPP_MACRO_ADD 1
#define OPP_MACRO_ADDI 2
#define OPP_MACRO_ADDIS 3
#define OPP_MACRO_ADDS 4
#define OPP_MACRO_AND 5
#define OPP_MACRO_ANDS 6
#define OPP_MACRO_B 7
#define OPP_MACRO_BEQ 8
#define OPP_MACRO_BNE 9
#define OPP_MACRO_BGT 10
#define OPP_MACRO_BLT 11
#define OPP_MACRO_BGE 12
#define OPP_MACRO_BLE 13
#define OPP_MACRO_CBNZ 14
#define OPP_MACRO_CBZ 15
#define OPP_MACRO_EOR 16
#define OPP_MACRO_LDUR 17
#define OPP_MACRO_LDURB 18
#define OPP_MACRO_LDURH 19
#define OPP_MACRO_LSL 20
#define OPP_MACRO_LSR 21
#define OPP_MACRO_MOVZ 22
#define OPP_MACRO_ORR 23
#define OPP_MACRO_STUR 24
#define OPP_MACRO_STURB 25
#define OPP_MACRO_STURH 26
#define OPP_MACRO_STURW 27
#define OPP_MACRO_SUB 28
#define OPP_MACRO_SUBI 29
#define OPP_MACRO_SUBIS 30
#define OPP_MACRO_SUBS 31
#define OPP_MACRO_MUL 32
#define OPP_MACRO_SDIV 33
#define OPP_MACRO_UDIV 34
#define OPP_MACRO_HLT 35
#define OPP_MACRO_CMP 36
#define OPP_MACRO_BL 37
#define OPP_MACRO_BR 38
#define OPP_MACRO_LSLI 45
#define OPP_MACRO_UNK -1
#define OPP_MACRO_UNASS 0 //unassigned macro
#define OPP_MACRO_NOP 99 //No op?!?!? Maybe we won't use it

#define PL_STAGE_FETCH 1
#define PL_STAGE_DECODE 2
#define PL_STAGE_EXECUTE 3
#define PL_STAGE_MEMORY 4
#define PL_STAGE_WRITE 5
#define PL_INCREMENT_FIFTY 6
#define PL_DECODE_INCR_FIFTY 7

typedef struct CPU_State {
	/* register file state */
	int64_t REGS[ARM_REGS];
	int FLAG_N;        /* flag N */
	int FLAG_Z;        /* flag Z */
	int FLAG_V;        /* flag V */
	int FLAG_C;        /* flag C */

	/* program counter in fetch stage */
	uint64_t PC;
	
} CPU_State;

int RUN_BIT;
int STALL_FOR_CYCLES;
int STALL_FOR_CYCLES_DCACHE;
int VERBOSE_FLAG;
uint64_t STALL_START_ADDR;

/* global variable -- pipeline state */
extern CPU_State CURRENT_STATE;

typedef struct CPU_CYCLE_FETCH{
	uint64_t instr;
	uint64_t pc;
	uint64_t predicted_pc;

	int stall_bit;
	int bounce_bit;
	int retired;
	int squash_bit;
	int p_taken; //predict_taken
	int pseudo_stall_bit;
	int cancel_stall;
	int override_exec;
} CYCLE_FETCH;

typedef struct CPU_CYCLE_DECODE{
	uint64_t instr;
	uint64_t pc;
	uint64_t predicted_pc;

	int run_bit;
	int stall_bit;
	int bubble_bit;
	int oppCode;
	int retired;
	int p_taken;
	int is_overrideable_bubble;
} CYCLE_DECODE;

typedef struct CPU_CYCLE_EXECUTE{
	uint64_t instr;
	uint64_t pc;
	uint64_t predicted_pc;
	uint64_t result;
	int resultRegister;
	int run_bit;
	int stall_bit;
	int retired;
	int branch_stall_bit;
	int p_taken;
	
	int oppCode;
	int FLAG_C;
	int FLAG_N;
	int FLAG_Z;
	int FLAG_V;
} CYCLE_EXECUTE;

typedef struct CPU_CYCLE_MEMORY{
	uint64_t instr;
	uint64_t pc;
	uint64_t result;
	int resultRegister;
	int oppCode;
	int run_bit;
	int stall_bit;
	int retired;
	int bubble_bit;
	
	int FLAG_C;
	int FLAG_N;
	int FLAG_Z;
	int FLAG_V;
} CYCLE_MEMORY;

typedef struct CPU_CYCLE_WRITE{
	uint64_t instr;
	uint64_t result;
	int resultRegister;
	int write_bit;
	int stall_bit;
	int oppCode;

	int run_bit;
	int retired;
	int bubble_bit;
} CYCLE_WRITE;

/* global pipeline state */
CPU_State CURRENT_STATE;
CYCLE_FETCH C_FETCH;
CYCLE_DECODE C_DECODE;
CYCLE_EXECUTE C_EXECUTE;
CYCLE_MEMORY C_MEMORY;
CYCLE_WRITE C_WRITE;

/* called during simulator startup */
void pipe_init();

/* this function calls the others */
void pipe_cycle();

/* each of these functions implements one stage of the pipeline */
void pipe_stage_fetch();
void pipe_stage_decode();
void pipe_stage_execute();
void pipe_stage_mem();
void pipe_stage_wb();

void set_stall(int start_stage);
void unset_stall(int start_stage);
void unset_bits(void);
void memoryOperation_basecase(uint32_t currOp);
void memoryOperation_hit(uint32_t currOpp);
uint64_t forward(int register_number);
void squash(int pl_stage_macro);
void insert_bubble(int at_stage);
int same_subblock(uint64_t stall_start_addr, uint64_t test_addr);
void fetch_base(void);

/* helper functions for commands */
void calculate(uint32_t opp_code);

uint64_t shiftReg(uint64_t, int, int);
uint64_t zeroExtend(uint32_t data);
uint64_t signExtend(int32_t data);
uint64_t signExtendImm(int32_t data, int extend_bit_at);
void pseudo_squash(int pl_stage_macro);

int is_retirable(int oppCode);
int is_uncond(int opp_code);
int is_squash_branch(int oppCode);

void add(uint32_t hexLine);

void addi(uint32_t hexLine);

void adds(uint32_t hexLine);

void addis(uint32_t hexLine);

void and(uint32_t hexLine);

void ands(uint32_t hexLine);

void eor(uint32_t hexLine);

void branch(uint64_t hexLine);

void branchCond(uint64_t hexLine);

void cbznz(uint64_t hexLine, int isnz);

void ldur(uint64_t hexLine);

void ldurbh(uint64_t hexLine, int isByte);

void lsli(uint32_t hexline);

void lsl(uint64_t hexline);

void movz(uint64_t hexline);

void orr(uint64_t hexline);

void stur(uint32_t hexline);

void sturh(uint32_t hexline);

void sturw(uint32_t hexline);

void sub(uint32_t hexline);

void subi(uint32_t hexline);

void subs(uint32_t hexline);

void subsi(uint32_t hexline);

void mul(uint32_t hexline);

void sdiv(uint32_t hexline);

void udiv(uint32_t hexline);

void halt();

void bl( uint32_t hexline);

void br(uint32_t hexline);

#endif
