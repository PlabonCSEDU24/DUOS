/*
 * Copyright (c) 2022 
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
 
#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <kstdio.h>
#include <kstring.h>
#include <stdint.h>
#include <usart.h>
#include <timer.h>
#include <types.h>
#include <syscall.h>
uint32_t debug1; uint32_t debug2;

extern int main(void);	 // userland main declaration but not defined
extern void task0(void); // Toggle LED0
extern void task1(void); // Toggle LED1
extern void task2(void); // Toggle LED2
extern void task3(void); // Toggle LED3

uint32_t curr_task;
uint32_t next_task;

uint32_t svc_exc_return;
uint32_t *svc_exc_return_ptr = &svc_exc_return;

uint32_t task0_stack[1024], task1_stack[1024], task2_stack[1024], task3_stack[1024];
TCB_TypeDef TCB[10];	// Task Control Block
uint32_t *PSP_Array[10];

uint32_t *PSP,MSP;

void kmain(void)
{
	__sys_init();

	// Priority set such that SysTick always preempts SVC or PendSV so that the mscount counting doesn't get interrupted
	__NVIC_SetPriorityGrouping(PRI_GROUP_1_BIT);
	__NVIC_SetPriority(SVCall_IRQn, ((PREEMTIVE_PRIORTY_1 << 3) | SUB_PRIORITY_0));		// SVC Higher Priority than SysTick
	__NVIC_SetPriority(SysTick_IRQn, ((PREEMTIVE_PRIORTY_1 << 3) | SUB_PRIORITY_1));	// SysTick Higher Priority than PendSV
	__NVIC_SetPriority(PendSV_IRQn, ((PREEMTIVE_PRIORTY_1 << 3) | SUB_PRIORITY_7));		// PendSV as Lowest Possible Priority 

	kprintf("OS Booting Time : %dms\n",__getTime());
	kprintf("WELCOME..........\r\n");
	kprintf("*************************************\r\n\n");
	
	void(* task_ptr)(void);

	// Create stack frame for task0
	PSP_Array[0] = &task0_stack[1023];
	*(PSP_Array[0]-1) = 0x01000000; // initial xPSR
	*(PSP_Array[0]-2) = task0; // initial Program Counter
	PSP_Array[0] = &task0_stack[1023]-16;	// Top of the stack
	
	// Create stack frame for task1
	PSP_Array[1] = &task1_stack[1023];
	*(PSP_Array[1]-1) = 0x01000000; // initial xPSR
	*(PSP_Array[1]-2) = task1; // initial Program Counter
	PSP_Array[1] = &task1_stack[1023]-16;	// Top of the stack
	
	// Create stack frame for task2
	PSP_Array[2] = &task2_stack[1023];
	*(PSP_Array[2]-1) = 0x01000000; // initial xPSR
	*(PSP_Array[2]-2) = task2; // initial Program Counter
	PSP_Array[2] = &task2_stack[1023]-16;	// Top of the stack
	
	// Create stack frame for task3
	PSP_Array[3] = &task3_stack[1023];
	*(PSP_Array[3]-1) = 0x01000000; // initial xPSR 0x01000000
	*(PSP_Array[3]-2) = task3; // initial Program Counter
	PSP_Array[3] = &task3_stack[1023]-16;	// Top of the stack

	curr_task = 0; 
	__asm volatile("MSR PSP, %0" : : "r" (PSP_Array[curr_task]) : );	// Set PSP to top of task 0 stack
	
	__sysTick_enable();	// Enable SysTick Here
	__sys_setPrivilegeMode(0x3);	// Switch Privilege mode and select PSP

	task0();
	
	// main();	// load userland main program
}

__attribute__((naked)) void SVCall_Handler(void)
{
	/* the handler function evntually call syscall function with a call number */
	__asm volatile("MOV R2, %0" : : "r" (svc_exc_return_ptr) : );		
	__asm volatile("STR LR, [R2]");	// save EXE_RETURN	
	__asm volatile(
		"TST LR, #4\n\t" 		// Test bit 2 of EXC_RETURN (LR)
		"ITE EQ\n\t"
		"MRSEQ R0, MSP\n\t"		// If return stack is MSP
		"MRSNE R0, PSP\n\t"		// If return stack is PSP
		"BL SVC_Handler_C\n\t"
	);
	__asm volatile("MOV R2, %0" : : "r" (svc_exc_return_ptr) : );		
	__asm volatile("LDR LR, [R2]");	// restore EXE_RETURN
	__asm volatile("BX LR");		/* Doesn't exit SVCall Handler for some reason it goes into an infinity loop here*/
}

void SVC_Handler_C(uint32_t *svc_args)
{
	uint32_t service_id = 0;
	uint32_t *svc_inst_ptr = 0;		// svc instruction pointer
	uint32_t stack_frame[15];
	for(int i=0; i<=7; i++){	
		stack_frame[i] = svc_args[i];
		//kprintf("stack_frame[%d] = 0x%x\n",i,stack_frame[i]);
	}
	// Service ID Extration
	svc_inst_ptr = (uint32_t *) (stack_frame[6] - 2);  //kprintf("svc inst %x\n",svc_inst_ptr[0]);
	__asm volatile("LDRB R0, [%0, #0]" : : "r" (svc_inst_ptr) : );	// load the first byte to get service id
	__asm volatile("MOV %0, R0" : "=r" (service_id) :  : "memory");	// assign service id
	//kprintf("service id %d\n",service_id); // kprintf("SP : %x\n",svc_args); kprintf("svc_exc_return : %x\n",svc_exc_return);
	
	// Passing Arguments from utility call to System Call from stack_frame 
	/* MUST NOT USE R3 and R7 */
	__asm volatile("MOV R1, %0" : : "r" (stack_frame[0]) :);	// pass buff* from write utility call to sys_write system call
	__asm volatile("MOV R2, %0" : : "r" (stack_frame[1]) :);	// pass buff_size
	__asm volatile("MOV R4, %0" : : "r" (stack_frame[2]) :);	// pass fd as R4
	syscall(service_id);
	return;
}

__attribute__((naked)) void PendSV_Handler(void){
	// Context switching code
	// Simple version - assume No floating point support
	/************************************************************/
	// Check PSP and MSP
	__asm volatile("MRS %0, PSP" : "=r" (PSP) : : "memory");
	__asm volatile("MRS %0, MSP" : "=r" (MSP) : : "memory");
	// Save current context
	__asm volatile("MRS R0, PSP"); // Get current process stack pointer value
	__asm volatile("STR R0, [%0]" : : "r" (&debug1) :);
	__asm volatile("STMDB R0!, {R4-R11}"); // Save R4 to R11 in task stack (8 regs) Other registers auto pushed by uC
	__asm volatile("STR R0, [%0]" : : "r" (&debug2) :);
	__asm volatile("LDR R2, [%0]" : : "r" (&curr_task) : ); // Get current task ID
	__asm volatile("STR R0, [%0]" : : "r" (&PSP_Array[curr_task]) :); // Save PSP of curr_task into PSP_array
	/************************************************************/
	// Load next context
	__asm volatile("LDR R4, [%0]" : : "r" (&next_task) :); // Get next task ID
	__asm volatile("STR R4, [%0]" : : "r" (&curr_task) :); // Set curr_task = next_task
	__asm volatile("LDR R0, [%0]" : : "r" (&PSP_Array[next_task]) :); // Load PSP value from PSP_array
	__asm volatile("LDMIA R0!, {R4-R11}"); // Load R4 to R11 from task stack (8 regs) Other registers auto poped by uC
	__asm volatile("MSR PSP, R0"); // Set PSP to next task
	__asm volatile("BX LR");
}

void core_registers_debug(void)
{
	// so get the true values of the registers we need to unstack r0-r3,r12,r13,r14,15
	__asm volatile(
		"TST LR, #4\n\t" 		// Test bit 2 of EXC_RETURN (LR)
		"ITE EQ\n\t"
		"MRSEQ R0, MSP\n\t"		// If return stack is MSP
		"MRSNE R0, PSP\n\t"		// If return stack is PSP
	);
	uint32_t *stack_pointer;
	uint32_t R[18];
	uint32_t xPSR;
	uint32_t MSP,PSP;
	uint32_t BASEPRI,PRIMASK,FAULTMASK,CONTROL;
	__asm volatile("MOV %0, R0" : "=r" (stack_pointer) : : "memory");
	stack_pointer +=2;
	R[0] = stack_pointer[0];
	R[1] = stack_pointer[1];
	R[2] = stack_pointer[2];
	R[3] = stack_pointer[3];
	R[12] = stack_pointer[4];
	R[14] = stack_pointer[5];
	R[15] = stack_pointer[6];
	xPSR = stack_pointer[7];
	__asm volatile("MOV %0, R4" : "=r" (R[4]) : : "memory");
	__asm volatile("MOV %0, R5" : "=r" (R[5]) : : "memory");
	__asm volatile("MOV %0, R6" : "=r" (R[6]) : : "memory");
	__asm volatile("MOV %0, R7" : "=r" (R[7]) : : "memory");
	__asm volatile("MOV %0, R8" : "=r" (R[8]) : : "memory");
	__asm volatile("MOV %0, R9" : "=r" (R[9]) : : "memory");
	__asm volatile("MOV %0, R10" : "=r" (R[10]) : : "memory");
	__asm volatile("MOV %0, R11" : "=r" (R[11]) : : "memory");
	__asm volatile("MOV %0, R13" : "=r" (R[13]) : : "memory");
	
	__asm volatile("MRS %0, MSP" : "=r" (MSP) : : "memory");
	__asm volatile("MRS %0, PSP" : "=r" (PSP) : : "memory");
	__asm volatile("MRS %0, PRIMASK" : "=r" (PRIMASK) : : "memory");
	__asm volatile("MRS %0, FAULTMASK" : "=r" (FAULTMASK) : : "memory");
	__asm volatile("MRS %0, BASEPRI" : "=r" (BASEPRI) : : "memory");
	__asm volatile("MRS %0, CONTROL" : "=r" (CONTROL) : : "memory");
	kprintf("\n\n");
	kprintf("Register    Value\n");
	kprintf("CORE\n");
	for(int i=0; i<10; i++){
		kprintf("R[%d]        0x%x\n",i,R[i]);
	}
	for(int i=10; i<16; i++){
		kprintf("R[%d]       0x%x\n",i,R[i]);
	}
	kprintf("xPSR        0x%x\n",xPSR);
	kprintf("BANKED\n");
	kprintf("MSP         0x%x\n",MSP);
	kprintf("PSP         0x%x\n",PSP);
	kprintf("SYSTEM\n");
	kprintf("BASEPRI     0x%x\n",BASEPRI);
	kprintf("PRIMASK     0x%x\n",PRIMASK);
	kprintf("FAULTMASK   0x%x\n",FAULTMASK);
	kprintf("CONTROL     0x%x\n",CONTROL);
	kprintf("INTERNAL\n");
	if(CONTROL & 0x1){
		kprintf("Privilege   UnPrivileged\n");
		kprintf("Stack       PSP\n");
	}
	else{
		kprintf("Privilege   Privileged\n");
		kprintf("Stack       MSP\n");
	}
	kprintf("\n\n");
}
