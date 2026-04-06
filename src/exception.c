#include "device_driver.h"

static void Uart2_Send_String(const char *s)
{
	while (*s)
	{
		Uart2_Send_Byte(*s++);
	}
}

void _Invalid_ISR(void)
{
	unsigned int r = Macro_Extract_Area(SCB->ICSR, 0x1ff, 0);
	(void)r;
	Uart2_Send_String("\nFault in ISR\n");
	for(;;);
}
