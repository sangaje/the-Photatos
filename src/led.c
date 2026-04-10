#include "led.h"
#include <stdio.h>
void LED_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, LED_RCC_BIT);
    Macro_Write_Block(LED_PORT->MODER, 0x3, 0x1, (LED_GREEN_PIN * 2));
    Macro_Write_Block(LED_PORT->MODER, 0x3, 0x1, (LED_RED_PIN * 2));
    LED_Green_On(); // 테스트: 초록 LED 켜기
}

void LED_Green_On(void) {
    // Active Low: 0을 써야 켜짐
    printf("LED_Green_On() called\n");   
    Macro_Clear_Bit(LED_PORT->ODR, LED_GREEN_PIN); 
    LED_Red_Off();
}

void LED_Green_Off(void) {
    // Active Low: 1을 써야 꺼짐
    Macro_Set_Bit(LED_PORT->ODR, LED_GREEN_PIN);
}

void LED_Red_On(void) {
    printf("LED_Red_On() called\n");
    Macro_Clear_Bit(LED_PORT->ODR, LED_RED_PIN);
    LED_Green_Off();
}

void LED_Red_Off(void) {
    Macro_Set_Bit(LED_PORT->ODR, LED_RED_PIN);
}
