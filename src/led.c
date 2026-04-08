#include "led.h"

void LED_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, LED_RCC_BIT);
    Macro_Write_Block(LED_PORT->MODER, 0x3, 0x1, (LED_GREEN_PIN * 2));
    Macro_Write_Block(LED_PORT->MODER, 0x3, 0x1, (LED_RED_PIN * 2));
    LED_All_Off();
}

void LED_Green_On(void) {
    Macro_Set_Bit(LED_PORT->ODR, LED_GREEN_PIN);
}

void LED_Green_Off(void) {
    Macro_Clear_Bit(LED_PORT->ODR, LED_GREEN_PIN);
}

void LED_Red_On(void) {
    Macro_Set_Bit(LED_PORT->ODR, LED_RED_PIN);
}

void LED_Red_Off(void) {
    Macro_Clear_Bit(LED_PORT->ODR, LED_RED_PIN);
}

void LED_All_Off(void) {
    LED_Green_Off();
    LED_Red_Off();
}