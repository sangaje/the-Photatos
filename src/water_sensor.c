#include "water_sensor.h"

void WaterSensor_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, WATER_RCC_BIT);                         // Enable Clock for the specific Port

    // Configure Pin as Analog Mode (11)
    // MODER uses 2 bits per pin, so position is (Pin * 2)
    Macro_Write_Block(WATER_PORT->MODER, 0x3, 0x3, (WATER_PIN * 2));    // Set to Analog Mode

    // Configure ADC1 Peripheral
    Macro_Set_Bit(RCC->APB2ENR, 8);                                     // Enable ADC1 clock
    Macro_Set_Bit(ADC1->CR2, 0);                                        // Turn on ADC1 (ADON)
    
    // Select ADC Channel (WATER_ADC_CH)
    Macro_Write_Block(ADC1->SQR3, 0x1F, WATER_ADC_CH, 0);               // Set Channel in Sequence
    
    // Set Sample Time (SMPR2 for CH0~9, SMPR1 for CH10~18)
    // Position for SMPR2 is (Channel * 3) bits
    Macro_Write_Block(ADC1->SMPR1, 0x7, 0x7, ((WATER_ADC_CH - 10 )* 3));       // Set Sample Time
}

int WaterSensor_Read(void) {
    Macro_Set_Bit(ADC1->CR2, 30); // SWSTART

    // 변환이 완료될 때까지 대기 (EOC 비트가 1이 될 때까지)
    while(!Macro_Check_Bit_Set(ADC1->SR, 1)); 

    return ADC1->DR & 0xFFF; // 12비트 데이터 리턴
}