#include "device_driver.h"

static void Uart2_Send_String(const char *s)
{
	while (*s)
	{
		Uart2_Send_Byte(*s++);
	}
}

static void Uart2_Send_Hex(uint32_t value)
{
    char hex_digits[] = "0123456789ABCDEF";
    char buffer[9]; // 8자리 + null
    buffer[8] = '\0';
    for (int i = 7; i >= 0; i--) {
        buffer[i] = hex_digits[value & 0xF];
        value >>= 4;
    }
    Uart2_Send_String(buffer);
}

void _Invalid_ISR(void)
{
	unsigned int r = Macro_Extract_Area(SCB->ICSR, 0x1ff, 0);
	(void)r;
	Uart2_Send_String("\nFault in ISR\n");
	for(;;);
}

void HardFault_Handler(void)
{
    Uart2_Send_String("\n=== HARD FAULT OCCURRED ===\n");

    // Hard Fault Status Register
    uint32_t hfsr = SCB->HFSR;
    Uart2_Send_String("HFSR: 0x");
    Uart2_Send_Hex(hfsr);
    Uart2_Send_String("\n");

    // CFSR: Configurable Fault Status Register
    uint32_t cfsr = SCB->CFSR;
    Uart2_Send_String("CFSR: 0x");
    Uart2_Send_Hex(cfsr);
    Uart2_Send_String("\n");

    // 기타 레지스터
    uint32_t mmfar = SCB->MMFAR;
    Uart2_Send_String("MMFAR: 0x");
    Uart2_Send_Hex(mmfar);
    Uart2_Send_String("\n");

    uint32_t bfar = SCB->BFAR;
    Uart2_Send_String("BFAR: 0x");
    Uart2_Send_Hex(bfar);
    Uart2_Send_String("\n");

    // 무한 루프
    for(;;);
}
