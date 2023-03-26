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
 
#include <cm4.h>
#include <clock.h>
#include <syscall.h>
#include <kstdio.h>
volatile static uint32_t __mscount;
extern uint32_t curr_task;
extern uint32_t next_task;
/************************************************************************************
* __SysTick_init(uint32_t reload) 
* Function initialize the SysTick clock. The function with a weak attribute enables 
* redefining the function to change its characteristics whenever necessary.
**************************************************************************************/

__attribute__((weak)) void __SysTick_init(uint32_t reload)
{
    SYSTICK->CTRL &= ~(1<<0); //disable systick timer
    SYSTICK->VAL =0; // initialize the counter
    __mscount=0;
    SYSTICK->LOAD = PLL_N*reload;
    SYSTICK->CTRL |= 1<<1 | 1<<2; //enable interrupt and internal clock source
    //SYSTICK->CTRL|=1<<0; //enable systick counter
}

/************************************************************************************
* __sysTick_enable(void) 
* The function enables the SysTick clock if already not enabled. 
* redefining the function to change its characteristics whenever necessary.
**************************************************************************************/
__attribute__((weak)) void __sysTick_enable(void)
{
    if(SYSTICK->CTRL & ~(1<<0)){ 
        SYSTICK->VAL = 0; // initialize the counter
        __mscount=0;
        SYSTICK->CTRL |= 1<<0;
    }
}
__attribute__((weak)) void __sysTick_disable(void)
{
    if(!(SYSTICK->CTRL & ~(1<<0))) SYSTICK->CTRL &= ~(1<<0);
}
__attribute__((weak)) uint32_t __getSysTickCount(void)
{
    return SYSTICK->VAL;
}
/************************************************************************************
* __updateSysTick(uint32_t count) 
* Function reinitialize the SysTick clock. The function with a weak attribute enables 
* redefining the function to change its characteristics whenever necessary.
**************************************************************************************/

__attribute__((weak)) void __updateSysTick(uint32_t count)
{
    SYSTICK->CTRL &= ~(1<<0); //disable systick timer
    SYSTICK->VAL =0; // initialize the counter
    __mscount=0;
    SYSTICK->CTRL |= 1<<1 | 1<<2; //enable interrupt and internal clock source
    SYSTICK->LOAD = PLL_N*count;
    SYSTICK->CTRL|=1<<0; //enable systick counter
}

/************************************************************************************
* __getTime(void) 
* Function return the SysTick elapsed time from the begining or reinitialing. The function with a weak attribute enables 
* redefining the function to change its characteristics whenever necessary.
**************************************************************************************/

__attribute__((weak)) uint32_t __getTime(void)
{
    return (__mscount+(SYSTICK->LOAD-SYSTICK->VAL)/(PLL_N*1000));
}
__attribute__((weak)) void SysTick_Handler()
{
    __mscount+=(SYSTICK->LOAD)/(PLL_N*1000);   
    
    // Simple task round robin scheduler
    switch(curr_task) {
        case(0): 
            next_task=1; 
            break;
        case(1): 
            next_task=2; 
            break;
        case(2): 
            next_task=3; 
            break;
        case(3): 
            next_task=0; 
            break;
        default: 
            next_task=0; 
            break;
    }
    // kprintf("cur_task : %d\n",curr_task);
	// kprintf("next_task : %d\n",next_task);
    if (curr_task!=next_task){ // Context switching needed
       __SetPendSV();  // curr_task will be updated ONLY in PendSV_Handler
    }
    return;
}

void __enable_fpu()
{
    SCB->CPACR |= ((0xF<<20));
}

/*
** NVIC Functions
*/
void __NVIC_SetPriorityGrouping(uint32_t priorityGroup){
    
    // priorityGroup can be 0x0, 0x4, 0x5, 0x6, 0x7
    // Only 3 bits are used and the max can be binary 111 so we AND with 7
    uint32_t priorityGroupTemp = 0;
    priorityGroupTemp = (priorityGroup & 7);        
    
    // Bits 10:8 PRIGROUP: Interrupt priority grouping field
    // First Clear the bits
    SCB->AIRCR &= ~(7 << 8);
    SCB->AIRCR |= (priorityGroupTemp << 8);
}

uint32_t __NVIC_GetPriorityGrouping(void){

    return (uint32_t)((SCB->AIRCR & (7 << 8)) >> 8);
}

/* This function takes two arguments (i) interrupt number and sets the priority to the interrupt. 
Note that priority in the above NVIC register is 8-bit. The priority puts the preference to the ISR 
executing before the lower (higher number) priority interrupts.*/
void __NVIC_SetPriority(IRQn_TypeDef IRQn, uint32_t priority){

    uint32_t irq_number = (uint32_t) IRQn;
    if ((int32_t)(IRQn) >= 0){              // Device Peripheral Interrupt priority setting
        // left shift priority 4 bits as the cortex m4 implements the upper[7:4] bits
        NVIC->IP[irq_number] = (uint8_t)(priority << 4);
    }
    else{                                   // Processor Core Exception Priority setting
        // when assigning signed number to an unsigned variable, reduced modulo UINT_MAX + 1 is applied
        // -1 will convert to UINT_MAX
        // NMI's priority cannot be set
        /* 4  - Memory Management Interrupt                          */
        /* 5  - Bus Fault Interrupt                                  */
        /* 6  - Usage Fault Interrupt                                */
        /* 11 - SV Call Interrupt                                    */
        /* 14 - Pend SV Interrupt                                    */    
        /* 15 - System Tick Interrupt                                */
        uint32_t exception_number = (irq_number & 0xF);
        uint8_t byte_offset = 0;
        if(exception_number == 4 || exception_number == 5 || exception_number == 6){
            byte_offset = exception_number - 4;
            // PRI_4, PRI_5, PRI_6 are in SHPR1
            SCB->SHPR1 |= (priority << 4) << (8*byte_offset);
        }
        else if(exception_number == 11){
            byte_offset = 3;
            // PRI_11 is in SHPR2
            SCB->SHPR2 |= (priority << 4) << (8*byte_offset);
        }
        else if(exception_number == 14 || exception_number == 15){
            byte_offset = (exception_number - 14) + 2;      // + 2 as PRI_14 starting bit is bits 16 in SHPR3 
            // PRI_14, PRI_15 are in SHPR3
            SCB->SHPR3 |= (priority << 4) << (8*byte_offset);
        }
    }
}
// Return the priority set to the interrupt.
uint32_t __NVIC_GetPriority(IRQn_TypeDef IRQn){
    
    if ((int32_t)(IRQn) >= 0){          // peripheral interrupts
        return (uint32_t) (NVIC->IP[(uint32_t)IRQn] >> 4);
    }
    else{                               // system core exceptions
        int exception_number = (IRQn & 0xF);
        uint8_t byte_offset = 0;
        if(exception_number == 4 || exception_number == 5 || exception_number == 6){
            byte_offset = exception_number - 4;
            // PRI_4, PRI_5, PRI_6 are in SHPR1
            return (uint32_t) ((SCB->SHPR1 >> (8*byte_offset)) >> 4);
        }
        else if(exception_number == 11){
            byte_offset = 3;
            // PRI_11 is in SHPR2
            return (uint32_t) ((SCB->SHPR2 >> (8*byte_offset)) >> 4);
        }
        else if(exception_number == 14 || exception_number == 15){
            byte_offset = (exception_number - 14) + 2;      // + 2 as PRI_14 starting bit is bits 16 in SHPR3 
            // PRI_14, PRI_15 are in SHPR3
            return (uint32_t) ((SCB->SHPR3 >> (8*byte_offset)) >> 4);
        }
    }
    return 0;               // if all cases fails
}

// enable interrupt given as argument or interrupt number (IRQn typeDef) – data structure (enumerator) defined earlier.
void __NVIC_EnableIRQn(IRQn_TypeDef IRQn){
    if((uint32_t) IRQn >=0){
        uint8_t bit_offset = IRQn%32;
        NVIC->ISER[((uint32_t)IRQn)/32] |= (uint32_t) (1 << bit_offset);
    }
}

// Disable interrupt.
void __NVIC_DisableIRQn(IRQn_TypeDef IRQn){
    if ((int32_t) (IRQn) >= 0){
        uint8_t bit_offset = IRQn%32;
        NVIC->ICER[((uint32_t)IRQn)/32] |= (uint32_t)(1 << bit_offset);
    }
}

// Masking interrupts __disable_irq() – all interrupts other than HardFault, NMI, and reset.
void __disable_irq(void){           // Set PRIMASK
    __set_PRIMASK(1);
}

// Enable (unmask) all interrupts
void __enable_irq(void){
    __set_PRIMASK(0);
}

/* __set_BASEPRI(uint32_t) function mask interrupt number greater and equal to the given interrupt 
priority as an argument. */
void __set_BASEPRI(uint32_t basePRI){
    basePRI = (basePRI << 4);
    __asm volatile("MSR BASEPRI, %0" : : "r" (basePRI) : "memory");
}

// unmask Interrupts by setting BASEPRI to 0
__attribute__((naked)) void __unset_BASEPRI(void){
    __asm volatile("MOV R0, #0x0");
    __asm volatile("MSR BASEPRI, R0");
}

// Returns basePri masking value
uint32_t __get_BASEPRI(void){
    uint32_t basePRI;
    __asm volatile("MRS %0, BASEPRI" : "=r" (basePRI));
    return basePRI;
}

// Prevent all interrupt without non-maskable interrupt.
void __set_PRIMASK(uint32_t priMask){
    __asm volatile ("MSR PRIMASK, %0" : : "r" (priMask) : "memory");
}

// Return value of the PRIMASK register
uint32_t __get_PRIMASK(void){
    uint32_t priMask;
    __asm volatile("MRS %0, PRIMASK" : "=r" (priMask));;
    return priMask;
}

// Enable all interrupt including FaultMask.
__attribute__((naked)) void __enable_fault_irq(void){
    __asm volatile("MOV R1, #0x0");
    __asm volatile("MSR FAULTMASK, R1");
}
__attribute__((naked)) void __set_FAULTMASK(uint32_t faultMask){
    __asm volatile ("MSR faultmask, %0" : : "r" (faultMask) : "memory");
}

// Disable or prevent all interrupt including FaultMask
void __disable_fault_irq(void){
    __set_FAULTMASK(0);
}
// This function will return the status of the masking value of FaultMask register
uint32_t __get_FAULTMASK(void){
    uint32_t faultMask;
    __asm volatile("MRS %0, FAULTMASK" : "=r" (faultMask));
    return faultMask;
}

// Set interrupt pending bit
void __NVIC_set_pending_IRQn(IRQn_TypeDef IRQn){
    uint8_t bit_offset = IRQn%32;
    NVIC->ISPR[(uint32_t)IRQn/32] |= (uint32_t)(1 << bit_offset);
}
// Clear interrupt pending bit
void __NVIC_clear_pending_IRQn(IRQn_TypeDef IRQn){
    uint8_t bit_offset = IRQn%32;
    NVIC->ICPR[(uint32_t)IRQn/32] |= (uint32_t)(1 << bit_offset);
}

// It returns the pending status of an interrupt.
uint32_t __NVIC_get_pending_IRQn(IRQn_TypeDef IRQn){
    uint8_t bit_offset = IRQn%32;
    if((NVIC->ISPR[(uint32_t)IRQn/32]) & (uint32_t)(1 << bit_offset))
        return 1;
    else return 0;
}

// This function return the active status of the interrupt.
uint32_t __NVIC_GetActive(IRQn_TypeDef IRQn){
    uint32_t bit_offset = IRQn%32;
    uint32_t register_offset = IRQn/32;
    
    if((NVIC->IABR[register_offset]) & (uint32_t)(1 << bit_offset)){
        return 1;
    }
    else return 0;
}

// Set Software trigger interrupt register (NVIC_STIR)
void __NVIC_SetSTIR(IRQn_TypeDef IRQn){
    uint32_t interrupt_id = (uint32_t) IRQn;
    NVIC->STIR = (interrupt_id & 0xFF);
}

// Set PendSV Pending Bit
void __SetPendSV(void){
    SCB->ICSR |= (1 << 28);
}
// Clear PendSV Pending Bit
void __ClearPendSV(void){
    SCB->ICSR |= (1 << 27);
}

// NMI set pending bit.
void __SetNMI(void){
    SCB->ICSR |= (1 << 31);
}
// NMI clear pending bit  
void __ClearNMI(void){
    SCB->ICSR &= ~(1 << 31);
}


