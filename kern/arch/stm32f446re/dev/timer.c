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
#include <timer.h>
void DRV_TIMER_INIT(TIM_TypeDef* tim)
{
    RCC->APB1ENR |= (1<<4); // Enable timer clock
    tim->ARR = 0xffff;      // Set ARR
    tim->PSC = (90 - 1);    // Set Prescaler
    tim->CR1 |= (1<<0);     // Enable Timer 
    while(!(tim->SR & (1<<0))); // Wait for it to be ready
}
void DELAY_US(TIM_TypeDef* tim, uint16_t delayAmount)
{
    tim->CNT = 0;
    while(tim->CNT < delayAmount);
}
void DELAY_MS(TIM_TypeDef* tim, uint16_t delayAmount)
{
    for(int i=0; i<delayAmount; i++)
        DELAY_US(tim,1000);
}
void DELAY_S(TIM_TypeDef* tim, uint16_t delayAmount)
{
    for(int i=0; i<delayAmount; i++)
        DELAY_MS(tim,1000);
}

