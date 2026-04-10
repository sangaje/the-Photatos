#include "device_driver.h"

#define SYSTICK_RELOAD_MAX    0x00FFFFFFU

static volatile unsigned int SysTick_TargetMs = 0;
static volatile unsigned int SysTick_ElapsedMs = 0;

void SysTick_Run(unsigned int msec)
{
	unsigned int ticks_per_ms = (unsigned int)((HCLK / 8.0f) / 1000.0f + 0.5f);

	if (ticks_per_ms == 0)
		ticks_per_ms = 1;
	if (ticks_per_ms > SYSTICK_RELOAD_MAX)
		ticks_per_ms = SYSTICK_RELOAD_MAX;

	SysTick->CTRL = 0; // Disable SysTick while configuring
	SysTick->LOAD = ticks_per_ms - 1;
	SysTick->VAL = 0;

	SysTick_TargetMs = msec;
	SysTick_ElapsedMs = 0;

	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

int SysTick_Check_Timeout(void)
{
	return (SysTick_ElapsedMs >= SysTick_TargetMs) ? 1 : 0;
}

unsigned int SysTick_Get_Time(void)
{
	return SysTick->VAL;
}

unsigned int SysTick_Get_Load_Time(void)
{
	return SysTick->LOAD;
}

void SysTick_Stop(void)
{
	SysTick->CTRL = 0;
	SysTick_TargetMs = 0;
	SysTick_ElapsedMs = 0;
}

void SysTick_Handler(void)
{
	if (SysTick_ElapsedMs < SysTick_TargetMs)
	{
		SysTick_ElapsedMs++;
	}
}
