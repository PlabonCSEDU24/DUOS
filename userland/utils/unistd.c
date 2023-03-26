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

#include <unistd.h>

/* Utility Functions */
void write(uint32_t fd, uint8_t *buff, uint32_t size){
	uint32_t buff_ptr = (uint32_t) buff;
	// Passing Arguments buff and buff_size to SVC in stack
	__asm volatile("MOV R0, %0" : : "r" (buff_ptr) :);	// Send buff* as R0
	__asm volatile("MOV R1, %0" : : "r" (size) :);		// Send buff Size as R1
	__asm volatile("MOV R2, %0" : : "r" (fd) :);		// Send file descriptor as R2
    __asm volatile("SVC 55");	// System Call for SYS_write Service
}

void read(uint32_t fd, uint8_t *buff, uint32_t size){
	uint32_t buff_ptr = (uint32_t) buff;
	// Passing Arguments buff and buff_size to SVC in stack
	__asm volatile("MOV R0, %0" : : "r" (buff_ptr) :);	// Send buff* as R0
	__asm volatile("MOV R1, %0" : : "r" (size) :);		// Send buff Size as R1
	__asm volatile("MOV R2, %0" : : "r" (fd) :);		// Send file descriptor as R2
    __asm volatile("SVC 50");	// System Call for SYS_read Service
}

void sys_getTime(uint32_t *buff){
	uint32_t buff_ptr = (uint32_t) buff;
	// Passing Arguments buff to SVC in stack
	__asm volatile("MOV R0, %0" : : "r" (buff_ptr) :);	// Send buff* as R0
	__asm volatile("SVC 113");	// System Call for SYS___time Service
}

void sys_sleep(uint32_t *buff){
	uint32_t buff_ptr = (uint32_t) buff;
	// Passing Arguments buff to SVC in stack
	__asm volatile("MOV R0, %0" : : "r" (buff_ptr) :);	// Send buff* as R0
	__asm volatile("SVC 115");	// System Call for SYS_nanosleep Service
}

void sys_reboot(void){
	__asm volatile("SVC 119");	// System Call for SYS_reboot Service
}

void sys_exit(void){
	__asm volatile("SVC 3");	// System Call for SYS__exit        
}

void sys_getpid(uint32_t *buff){
	uint32_t buff_ptr = (uint32_t) buff;
	// Passing Arguments buff to SVC in stack
	__asm volatile("MOV R0, %0" : : "r" (buff_ptr) :);	// Send buff* as R0
	__asm volatile("SVC 5");	// System Call for SYS_getpid        
}

void sys_yield(void){
	__asm volatile("SVC 120");	// System Call for SYS_yield        
}

