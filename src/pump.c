#include "pump.h"

void Pump_Init(void) 
{
    Macro_Set_Bit(RCC->AHB1ENR, PUMP_RCC_BIT);                      // Enable Clock for the specific Port

    // Configure Pin as Output Mode (01)
    // MODER uses 2 bits per pin, so position is (Pin * 2)
    Macro_Write_Block(PUMP_PORT->MODER, 0x3, 0x1, (PUMP_PIN * 2));  // Set to Output Mode

    Pump_Off();                                                     // Initial State: OFF
}

inline void Pump_On(void) 
{
    Macro_Set_Bit(PUMP_PORT->ODR, PUMP_PIN);                        // Set Target Pin to High (ON)
}

inline void Pump_Off(void) 
{
    Macro_Clear_Bit(PUMP_PORT->ODR, PUMP_PIN);                      // Set Target Pin to Low (OFF)
}

inline void Pump_Delay(volatile int count) 
{
    for (int i = 0; i < count * 3000; i++)                          // Loop for delay based on CPU clock
    {
        __asm__("nop");                                             // No Operation (Assembly instruction)
    } 
}