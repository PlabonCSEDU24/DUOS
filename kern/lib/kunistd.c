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
#include <kunistd.h>
#include <cm4.h>
#include <usart.h>
#include <gpio.h>
#include <timer.h>
#include <kstdio.h>

/* Add your functions here */
void __SYS_read(uint32_t fd, uint8_t *buff, uint32_t size){
	
    switch (fd)
    {
    case STDIN_FILENO:
        _USART_READ_STR(USART2, buff, size);
        break;
    default:
        break;
    }
}

void __SYS_write(uint32_t fd, uint8_t *buff, uint32_t size)
{
    switch (fd)
    {
    case STDOUT_FILENO:
        _USART_WRITE(USART2, (uint8_t *)buff);
        break;
    case STDGPIO_FILENO:    // buff[0] = gpio_port, buff[1] = pinNum, buff[3] = state;
        if(buff[3] == 1){   // HIGH
            if(buff[0] == 'A' || buff[0] == 'a'){
                GPIO_SET(GPIOA, buff[1]);
            }
            else if(buff[0] == 'B' || buff[0] == 'b'){
                GPIO_SET(GPIOB, buff[1]);
            }
            else if(buff[0] == 'C' || buff[0] == 'c'){
                GPIO_SET(GPIOC, buff[1]);
            }
            else if(buff[0] == 'D' || buff[0] == 'd'){
                GPIO_SET(GPIOD, buff[1]);
            }
            else if(buff[0] == 'E' || buff[0] == 'e'){
                GPIO_SET(GPIOE, buff[1]);
            }
            else if(buff[0] == 'F' || buff[0] == 'f'){
                GPIO_SET(GPIOF, buff[1]);
            }
            else if(buff[0] == 'G' || buff[0] == 'g'){
                GPIO_SET(GPIOG, buff[1]);
            }
            else if(buff[0] == 'H' || buff[0] == 'h'){
                GPIO_SET(GPIOH, buff[1]);
            }
        }
        else{           // LOW
            if(buff[0] == 'A' || buff[0] == 'a'){
                GPIO_RESET(GPIOA, buff[1]);
            }
            else if(buff[0] == 'B' || buff[0] == 'b'){
                GPIO_RESET(GPIOB, buff[1]);
            }
            else if(buff[0] == 'C' || buff[0] == 'c'){
                GPIO_RESET(GPIOC, buff[1]);
            }
            else if(buff[0] == 'D' || buff[0] == 'd'){
                GPIO_RESET(GPIOD, buff[1]);
            }
            else if(buff[0] == 'E' || buff[0] == 'e'){
                GPIO_RESET(GPIOE, buff[1]);
            }
            else if(buff[0] == 'F' || buff[0] == 'f'){
                GPIO_RESET(GPIOF, buff[1]);
            }
            else if(buff[0] == 'G' || buff[0] == 'g'){
                GPIO_RESET(GPIOG, buff[1]);
            }
            else if(buff[0] == 'H' || buff[0] == 'h'){
                GPIO_RESET(GPIOH, buff[1]);
            }
        }
        break;
    default:
        break;
    }
}

void __SYS_time(uint32_t *buff){            
	*buff = __getTime();
}

void __SYS_nanosleep(uint32_t *buff){
	uint32_t ms_delay = *buff;
	DELAY_MS(TIM6, ms_delay);
}

void __SYS_reboot(void);
void __SYS__exit(void);                      
//void __SYS_getpid(uint32_t *);                   
void __SYS_yield(void);
