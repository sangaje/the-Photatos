#include "water_sensor.h"

extern volatile uint16_t *others_raw;

void _DMA_Init_Water(void)
{
    // DMA2 Clock 활성화
    Macro_Set_Bit(RCC->AHB1ENR, 22);        // DMA2 Clock ON

    // 설정 전 Stream 비활성화 (안전장치)
    DMA2_Stream4->CR &= ~(1 << 0);
    while (DMA2_Stream4->CR & (1 << 0))
        ;                                   // 꺼질 때까지 대기

    // DMA2_Stream4 설정
    DMA2_Stream4->PAR = (uint32_t)&(ADC1->DR);
    DMA2_Stream4->M0AR = (uint32_t)&water_sensor_raw;
    DMA2_Stream4->NDTR = 1;

    // CR 설정 + 맨 마지막에 (1 << 0)을 더해서 EN(Enable) 시킴
    DMA2_Stream4->CR = (0 << 25) | (2 << 16) | (1 << 13) | (1 << 11) |
                       (1 << 10) | (1 << 8) | (1 << 0);  // 맨 끝에 1(EN) 추가!
}

void WaterSensor_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, WATER_RCC_BIT);                         // Enable Clock for the specific Port

    // Configure Pin as Analog Mode (11)
    // MODER uses 2 bits per pin, so position is (Pin * 2)
    Macro_Write_Block(WATER_PORT->MODER, 0x3, 0x3, (WATER_PIN * 2));    // Set to Analog Mode

    // Configure ADC1 Peripheral
    Macro_Set_Bit(RCC->APB2ENR, 8);                                     // Enable ADC1 clock
    
    // Select ADC Channel (WATER_ADC_CH)
    Macro_Write_Block(ADC1->SQR3, 0x1F, WATER_ADC_CH, 0);               // Set Channel in Sequence
    
    // Set Sample Time (SMPR2 for CH0~9, SMPR1 for CH10~18)
    // Position for SMPR2 is (Channel * 3) bits
    Macro_Write_Block(ADC1->SMPR1, 0x7, 0x7, ((WATER_ADC_CH - 10 )* 3));       // Set Sample Time

    // ADC DMA 설정
    Macro_Set_Bit(ADC1->CR2, 8);                                        // DMA: DMA 모드 활성화
    Macro_Set_Bit(ADC1->CR2, 9);                                        // DDS: DMA 계속 요청 (Circular 대응)

    // DMA 초기화
    _DMA_Init_Water();

    // ADC 시작
    Macro_Set_Bit(ADC1->CR2, 0);                                        // Turn on ADC1 (ADON)
    Macro_Set_Bit(ADC1->CR2, 30);                                       // SWSTART: 변환 시작
}


int WaterSensor_Read(void) {
    return *others_raw & 0xFFF; // 12비트 데이터 리턴
}