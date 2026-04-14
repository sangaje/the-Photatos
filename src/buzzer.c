#include "buzzer.h"

void Buzzer_Init(void) 
{
    Macro_Set_Bit(RCC->AHB1ENR, BUZZER_RCC_BIT);                       // Enable Clock for the specific Port
    
    // MODER uses 2 bits per pin, so position is (Pin * 2)
    Macro_Write_Block(BUZZER_PORT->MODER, 0x3, 0x1, (BUZZER_PIN * 2)); // Set to Output Mode (01)

    Buzzer_Off();                                                      // Initial State: OFF
}

void Buzzer_On(void) 
{
    Macro_Clear_Bit(BUZZER_PORT->ODR, BUZZER_PIN);                     // Set Target Pin to Low
    // printf("Buzzer_On() called\n");
}

void Buzzer_Off(void) 
{
    Macro_Set_Bit(BUZZER_PORT->ODR, BUZZER_PIN);                       // Set Target Pin to High
    // printf("Buzzer_Off() called\n");
}