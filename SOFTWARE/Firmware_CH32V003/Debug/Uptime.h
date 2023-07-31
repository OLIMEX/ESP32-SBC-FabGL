#ifndef	_UPTIME_H_
#define	_UPTIME_H_

#include <ch32v00x.h>


void Uptime_Init();

uint32_t Uptime_S();

uint32_t Uptime_Us();

uint32_t Uptime_Ms();

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void);

#endif
