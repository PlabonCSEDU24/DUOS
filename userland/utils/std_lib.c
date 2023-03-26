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

#include <std_lib.h>
#include <unistd.h>
#include <kstring.h>

/* Write your highlevel I/O details */
/* User Functions */
void printf(char *format,...)
{
    uint8_t buff[1024];    // 1kb Buffer
    uint32_t buff_size = 0;
	uint8_t *temp_buff;
	int len;
	char *tr;
	uint32_t i;
	uint8_t *str;
	va_list list;
	double dval;
	//uint32_t *intval;
	va_start(list,format);
	for(tr = format;*tr != '\0';tr++)
	{
		while(*tr != '%' && *tr!='\0')
		{
			buff[buff_size++] = *tr;
			tr++;
		}
		if(*tr == '\0') break;
		tr++;
		switch (*tr)
		{
			case 'c': i = va_arg(list,int);
				buff[buff_size++] = i;
				break;
			case 'd': i = va_arg(list,int);
				if(i<0)
				{
					buff[buff_size++] = '-';
					i=-i;				
				}
				temp_buff = convert(i,10);
				len = __strlen(temp_buff);
				for(int u = 0; u<len; u++)
					buff[buff_size++] = temp_buff[u];
				break;
			case 'o': i = va_arg(list,int);
				if(i<0)
				{
					buff[buff_size++] = '-';
					i=-i;				
				}
				temp_buff = convert(i,8);
				len = __strlen(temp_buff);
				for(int u = 0; u<len; u++)
					buff[buff_size++] = temp_buff[u];
				break;
			case 'x': i = va_arg(list,int);
				if(i<0)
				{
					buff[buff_size++] = '-';
					i=-i;				
				}
				temp_buff = convert(i,16);
				len = __strlen(temp_buff);
				for(int u = 0; u<len; u++)
					buff[buff_size++] = temp_buff[u];
				break;
			case 's': str = va_arg(list,uint8_t*);
				temp_buff = str;
				len = __strlen(temp_buff);
				for(int u = 0; u<len; u++)
					buff[buff_size++] = temp_buff[u];
				break;
			case 'f': 
				dval = va_arg(list,double);
				temp_buff = float2str(dval);
				len = __strlen(temp_buff);
				for(int u = 0; u<len; u++)
					buff[buff_size++] = temp_buff[u];
				break;
			default:
				break;
		}
	}
	va_end(list);
	buff[buff_size++] = '\0';

    write(1,buff,buff_size);	// Utility Function Call which will do a write system call
}

void scanf(char *format,...)
{
	va_list list;
	char *ptr;
	uint8_t buff[1024];
	uint8_t *str;
	int len;
	ptr=format;
	va_start(list,format);
	while (*ptr)
	{
		if(*ptr == '%') //looking for format of an input
		{
			ptr++;
			switch (*ptr)
			{
			case 'c': //charater
				read(0, buff, 1);	// read a single character
				*(uint8_t*)va_arg(list,uint8_t*)=*buff;	
				break;
			case 'd': //integer number 
				read(0, buff, 1024);
				*(uint32_t*)va_arg(list,uint32_t*)=__str_to_num(buff,10);	
				break;
			case 's': //string without spaces
				read(0, buff, 1024);
				str = va_arg(list,uint8_t*);
				len = __strlen(buff);
				for(int u = 0; u<=len; u++)	// copy from buff to user defined char pointer (i.e string)
					str[u] = buff[u];
				break;
			case 'x': //hexadecimal number
				read(0, buff, 1024);
				*(uint32_t*)va_arg(list,uint32_t*)=__str_to_num(buff,16);	
				break;	
			case 'o': //octal number
				read(0, buff, 1024);
				*(uint32_t*)va_arg(list,uint32_t*)=__str_to_num(buff,8);	
				break;	
			case 'f': //floating point number
				read(0, buff, 1024);
				//*(uint32_t*)va_arg(list,double*) = __str_to_num(buff,10);
				*(float*)va_arg(list,float*) = str2float(buff);	// Works for float but not for double !!!
				break;	
			default: //rest not recognized
				break;
			}
		}
		ptr++;
	}
	va_end(list);
}

void reboot(void){
	sys_reboot();
}

uint16_t getpid(void){
	uint32_t buff;
	sys_getpid(&buff);
	return buff;
}

void exit(void){
	sys_exit();
}   

void yield(void){
	sys_yield();
}

/*
** Ex of usage : digitalWrite("PA5", 1);	// Sets PA5
*/
void digitalWrite(char *pin, uint8_t state){
	uint8_t gpio_port = pin[1]; // GPIOA/B/C/D/E/F/G/H
	uint8_t pinNum;	
	uint8_t buff[20];
	int i = 2;	// Skipping first two characters as they are P and A/B/C/D/E/F/G/H
	if(pin[3] == '\0')
		pinNum = pin[2] - '0';
	else
		pinNum = (pin[2]-'0')*10 + (pin[3]-'0');
	/*	buff contains the pin information as the following format :
		gpio_port,pinNum,state
	*/
	buff[0] = gpio_port;
	buff[1] = pinNum;
	buff[3] = state;
	buff[4] = '\0';
	write(4,buff,4);	// Utility Function Call which will do a write system call
}