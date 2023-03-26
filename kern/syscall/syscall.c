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
#include <syscall.h>
#include <syscall_def.h>
#include <errno.h>
#include <errmsg.h>
#include <kmain.h>
#include <kstdio.h>

void syscall(uint16_t callno)	/* The SVC_Handler calls this function to evaluate and execute the actual function */
{
	uint8_t *buff;
	uint32_t buff_size;
	uint32_t fd;
	
	/* Extracting Arguments R1 : buff* and R2 : buff_size R4 : fd */
	/* Must Copy R1-R3 to other registers so that they don't get changed while using inline assembly */
	/* Problem with using R3 directy as R3 gets modified in the previous inline assembly instruction */
	/* MUST NOT USE R3 and R7 */
	__asm volatile("MOV R5, R1");	// buff*
	__asm volatile("MOV R6, R2");	// buff_size
	__asm volatile("MOV R8, R4");	// fd
	__asm volatile("MOV %0, R5" : "=r" (buff) : : "memory");		
	__asm volatile("MOV %0, R6" : "=r" (buff_size) : : "memory");
	__asm volatile("MOV %0, R8" : "=r" (fd) : : "memory");	
	
	switch(callno)
	{
	/* Write your code to call actual function (kunistd.h/c or times.h/c and handle the return value(s) */
	case SYS_read:
		__SYS_read(fd,buff,buff_size);
		break;
	case SYS_write:
		__SYS_write(fd,buff,buff_size);
		break;
	case SYS_reboot:
		break;	
	case SYS__exit:
		break;
	case SYS_getpid:
		break;
	case SYS___time:
		__SYS_time((uint32_t *)buff);
		break;
	case SYS_nanosleep:
		__SYS_nanosleep((uint32_t *)buff);
		break;
	case SYS_yield:
		break;				
	/* return error code see error.h and errmsg.h ENOSYS sys_errlist[ENOSYS]*/	
	default: ;
	}
	/* Handle SVC return here */
	return;
}

