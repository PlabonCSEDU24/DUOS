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
#include <gpio.h>
void DRV_GPIO_INIT(GPIO_TypeDef* gpio)
{
    if(gpio == GPIOA)
			RCC->AHB1ENR |= (1<<0);
	else if(gpio == GPIOB)
			RCC->AHB1ENR |= (1<<1);
	else if(gpio == GPIOC)
			RCC->AHB1ENR |= (1<<2);
	else if(gpio == GPIOD)
			RCC->AHB1ENR |= (1<<3);
	else if(gpio == GPIOE)
			RCC->AHB1ENR |= (1<<4);
    else if(gpio == GPIOF)
			RCC->AHB1ENR |= (1<<5);
    else if(gpio == GPIOG)
			RCC->AHB1ENR |= (1<<6);
    else if(gpio == GPIOH)
			RCC->AHB1ENR |= (1<<7);
}

void DRV_GPIO_OUPUT_MODE(GPIO_TypeDef *gpio, int pinNum){
		
	gpio->MODER &= ~(3 << pinNum*2);
	gpio->MODER |= (1 << pinNum*2);
}

void DRV_GPIO_INPUT_MODE(GPIO_TypeDef *gpio, int pinNum){
	
	gpio->MODER &= ~(3 << pinNum*2);
}

void GPIO_SET(GPIO_TypeDef *gpio, uint32_t pinNum){
	gpio->BSRR |= (1<<pinNum);
}

void GPIO_RESET(GPIO_TypeDef *gpio, uint32_t pinNum){
	gpio->BSRR |= (1<<(pinNum+16));
}

