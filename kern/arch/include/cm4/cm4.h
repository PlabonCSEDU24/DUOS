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
 
#ifndef __CM4_H
#define __CM4_H
#include <stdint.h>
/*
* This file defines Cortex-M4 processor internal peripherals
* NVIC, SCB, FPU and so on
*/
#define SYSTICK ((SYSTICK_TypeDef*)0xE000E010)
#define NVIC ((NVIC_TypeDef*)0xE000E100)
#define SCB ((SCB_TypeDef*) 0xE000ED00)
/*
* Data structure for SCB
*/
typedef struct
{
volatile uint32_t CPUID; 	// CPUID Base Register 0x0
volatile uint32_t ICSR;		// Interrupt Control and State Register 0x4
volatile uint32_t VTOR;		// Vector Table Offset Register 0x8
volatile uint32_t AIRCR;	// Application Interrupt and Reset Control Register 0xC
volatile uint32_t SCR;		// System Control Register 0x10
volatile uint32_t CCR;		// Configuration and Control Register 0x14
volatile uint32_t SHPR1;  // System Handler Priority Register 1 0x18
volatile uint32_t SHPR2;  // System Handler Priority Register 2 0x1C 
volatile uint32_t SHPR3;  // System Handler Priority Register 3 0x20
volatile uint32_t SHCSR;	// System Handler Control and State Register 0x24
volatile uint32_t CFSR;		// Configurable Fault Status Register combined of MemManage, Fault Status Register, BusFault Status Register, UsageFault Status Register 0x28
volatile uint32_t HFSR;		// HardFault Status Register 0x2C
volatile uint32_t DFSR;		// Hint information for causes of debug events
volatile uint32_t MMFAR;	// MemManage Fault Address Register 0x34
volatile uint32_t BFAR;		// BusFault Address Register 0x38
volatile uint32_t AFSR;		// Auxiliary Fault Status Register 0x3C: Information for device-specific fault status
volatile uint32_t PFR[2]; 	// Read only information on available processor features
volatile uint32_t DFR;		// Debug Feature: Read only information on available debug features
volatile uint32_t AFR;		// Auxiliary Feature: Read only information on available auxiliary features
volatile uint32_t MMFR[4]; 	// Memory Model	Feature Registers: Read only information
volatile uint32_t ISAR[5]; 	// Instruction Set Attributes Register: Register Read only information
uint32_t RESERVED1[5];		// Now it is reserved: Unknown 	
volatile uint32_t CPACR; 	// Coprocessor access control register 0x88
} SCB_TypeDef;
/*
* SysTick Data Structure
*/
typedef struct 
{
    //define systick register compenenets -- use volatile data type
    volatile uint32_t CTRL; //systick controller register
    volatile uint32_t LOAD; //systick reload value
    volatile uint32_t VAL; // systick down counter
    volatile uint32_t CALIB; //systick calibration register
}SYSTICK_TypeDef;

// enum IRQn_TypeDef {
//   NonMaskableInt_IRQn = -14,
//   HardFault_IRQn = -13,
//   MemoryManagement_IRQn = -12,
//   BusFault_IRQn = -11,
//   UsageFault_IRQn = -10,
//   SecureFault_IRQn = -9,
//   SVCall_IRQn = -5,
//   DebugMonitor_IRQn = -4,
//   PendSV_IRQn = -2,
//   SysTick_IRQn = -1,
//   WWDG_STM_IRQn = 0,
//   PVD_STM_IRQn = 1
// };

// IRQ Interrupt Number Definition
typedef enum{
// Cortex-M4 Processor Core Exceptions Numbers
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
//  Peripheral Interrupt Numbers
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
  SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                         */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */  
} IRQn_TypeDef;

typedef struct
{
    //define NVIC register compenenets -- use volatile data type
  volatile uint32_t ISER[8];     /*!< Offset: 0x000  Interrupt Set Enable Register           */
  uint32_t RESERVED0[24];                               
  volatile uint32_t ICER[8];     /*!< Offset: 0x180  Interrupt Clear Enable Register         */
  uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];     /*!< Offset: 0x200  Interrupt Set Pending Register          */
  uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];     /*!< Offset: 0x280  Interrupt Clear Pending Register        */
  uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];     /*!< Offset: 0x300  Interrupt Active bit Register           */
  uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];     /*!< Offset: 0x400  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;   	 /*!< Offset: 0xF00  Software Trigger Interrupt Register     */
}NVIC_TypeDef;

typedef struct
{
    //define FPU register compenenets -- use volatile data type
    

}FPU_TypeDef;
/**
* Function related to SysTick
*/
__attribute__((weak)) void __SysTick_init(uint32_t);
__attribute__((weak)) void __sysTick_enable(void);
__attribute__((weak)) void __sysTick_disable(void);
__attribute__((weak)) uint32_t __getSysTickCount(void);
__attribute__((weak)) void __updateSysTick(uint32_t) ;
__attribute__((weak)) uint32_t __getTime(void);
/**
* Functions on FPU
**/
void __enable_fpu(void);



/*
**  Functions on NVIC
*/
#define PRI_GROUP_0_BIT 7   // 0 bits for grouping
#define PRI_GROUP_1_BIT 6   // 1 bits for grouping
#define PRI_GROUP_2_BIT 5   // 2 bits for grouping
#define PRI_GROUP_3_BIT 4   // 3 bits for grouping
#define PRI_GROUP_4_BIT 0   // 4 bits for grouping
// PRI_N Preemptive Bit Definitions
#define PREEMTIVE_PRIORTY_0 0
#define PREEMTIVE_PRIORTY_1 1
#define PREEMTIVE_PRIORTY_2 2
#define PREEMTIVE_PRIORTY_3 3
#define PREEMTIVE_PRIORTY_4 4
#define PREEMTIVE_PRIORTY_5 5
#define PREEMTIVE_PRIORTY_6 6
#define PREEMTIVE_PRIORTY_7 7
#define PREEMTIVE_PRIORTY_8 8
#define PREEMTIVE_PRIORTY_9 9
#define PREEMTIVE_PRIORTY_10 10
#define PREEMTIVE_PRIORTY_11 11
#define PREEMTIVE_PRIORTY_12 12
#define PREEMTIVE_PRIORTY_13 13
#define PREEMTIVE_PRIORTY_14 14
#define PREEMTIVE_PRIORTY_15 15
// PRI_N Sub-priority Bit Definitions
#define SUB_PRIORITY_0 0
#define SUB_PRIORITY_1 1
#define SUB_PRIORITY_2 2
#define SUB_PRIORITY_3 3
#define SUB_PRIORITY_4 4
#define SUB_PRIORITY_5 5
#define SUB_PRIORITY_6 6
#define SUB_PRIORITY_7 7
#define SUB_PRIORITY_8 8
#define SUB_PRIORITY_9 9
#define SUB_PRIORITY_10 10
#define SUB_PRIORITY_11 11
#define SUB_PRIORITY_12 12
#define SUB_PRIORITY_13 13
#define SUB_PRIORITY_14 14
#define SUB_PRIORITY_15 15

void __NVIC_SetPriorityGrouping(uint32_t);  // Set Interrupt Priority Grouping in SCB
uint32_t __NVIC_GetPriorityGrouping(void);  // Get Interrupt Priority Grouping

/* This function takes two arguments (i) interrupt number and sets the priority to the interrupt. 
Note that priority in the above NVIC register is 8-bit. The priority puts the preference to the ISR 
executing before the lower (higher number) priority interrupts.*/
void __NVIC_SetPriority(IRQn_TypeDef, uint32_t);
uint32_t __NVIC_GetPriority(IRQn_TypeDef);  // Return the priority set to the interrupt.

void __NVIC_EnableIRQn(IRQn_TypeDef); // enable interrupt given as argument or interrupt number (IRQn typeDef) – data structure (enumerator) defined earlier.
void __NVIC_DisableIRQn(IRQn_TypeDef);  // Disable interrupt.

void __disable_irq(void);  // Masking interrupts __disable_irq() – all interrupts other than HardFault, NMI, and reset.
void __enable_irq(void); // Enable (unmask) all interrupts

void __set_BASEPRI(uint32_t); // __set_BASEPRI(uint32_t) function mask interrupt number greater and equal to the given interrupt priority as an argument.
__attribute__((naked)) void __unset_BASEPRI(void);  // __unset_BASEPRI(uint32_t) function unmask interrupts greater or equal to the given argument/priority number.
uint32_t __get_BASEPRI(void); // Returns basePri masking value

void __set_PRIMASK(uint32_t); // Prevent all interrupt without non-maskable interrupt.
void __unset_PRIMASK(uint32_t); // unmask all interrupts
uint32_t __get_PRIMASK(void);  // Return value of the PRIMASK register

__attribute__((naked)) void __enable_fault_irq(void); // Enable all interrupt including FaultMask.
__attribute__((naked)) void __set_FAULTMASK(uint32_t); // set or reset Faultmask register
void __disable_fault_irq(void); // Disable or prevent all interrupt including FaultMask
uint32_t __get_FAULTMASK(void); // This function will return the status of the masking value of FaultMask register

void __NVIC_set_pending_IRQn(IRQn_TypeDef); // Set interrupt pending bit
void __NVIC_clear_pending_IRQn(IRQn_TypeDef); // Clear interrupt pending bit
uint32_t __NVIC_get_pending_IRQn(IRQn_TypeDef); // It returns the pending status of an interrupt.
uint32_t __NVIC_GetActive(IRQn_TypeDef);  // This function return the active status of the interrupt.

void __NVIC_SetSTIR(IRQn_TypeDef);  // Set Software trigger interrupt register (NVIC_STIR)

void __SetPendSV(void);   // Set PendSV Pending Bit
void __ClearPendSV(void); // Clear PendSV Pending Bit

void __SetNMI(void);    // NMI set pending bit.
void __ClearNMI(void);  // NMI clear pending bit  

#endif
