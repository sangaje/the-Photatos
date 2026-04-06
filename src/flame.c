#include "flame.h"
#include "macro.h"
#include "malloc.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>

volatile uint16_t flame_sensors_raw[SENSOR_NUM] = {
    0x0,
};

void _ADC_Init(void)
{
    // 2. ADC 기본 설정
    Macro_Set_Bit(RCC->APB2ENR, 8);            // ADC1 Clock ON
    Macro_Write_Block(ADC->CCR, 0x3, 0x2, 16); // ADC Clock = PCLK2/6

    // ADC 제어 비트들 (CONT, DMA, DDS)
    Macro_Set_Bit(ADC1->CR2, 1); // CONT: 연속 변환 모드
    Macro_Set_Bit(ADC1->CR2, 8); // DMA: DMA 모드 활성화
    Macro_Set_Bit(ADC1->CR2, 9); // DDS: DMA 계속 요청 (Circular 대응)

    // 3. 시퀀스
    Macro_Write_Block(ADC1->SQR1, 0xF, SENSOR_NUM - 1, 20); // 3개 채널 (L=2)
}

void _ADC_Config_By_Channel(int ch)
{
    static int _i = 0;
    if (ch < 10)
        Macro_Write_Block(ADC1->SMPR2, 0x7, SAMPLING_TIME, ch * 3); // CH0 480 Cycles
    else
        Macro_Write_Block(ADC1->SMPR1, 0x7, SAMPLING_TIME, (ch % 10) * 3); // CH0 480 Cycles

    switch (_i / 6)
    {
    case 0:
        Macro_Write_Block(ADC1->SQR3, 0x1F, ch, (_i % 6) * 5);
        break;
    case 1:
        Macro_Write_Block(ADC1->SQR2, 0x1F, ch, (_i % 6) * 5);
        break;
    case 2:
        Macro_Write_Block(ADC1->SQR1, 0x1F, ch, (_i % 6) * 5);
        break;
    }
    _i++;
}

GPIO_TypeDef *Get_GPIO_Port_by_Channel(int ch)
{
    GPIO_TypeDef *GPIOx;
    if (ch < 8)
    {
        GPIOx = GPIOA; // PA0-PA7
        // 1. GPIO & Clock 설정 (PA0, PA1, PA6)
        Macro_Set_Bit(RCC->AHB1ENR, 0); // GPIOA Clock ON
    }
    else if (ch < 16)
    {
        GPIOx = GPIOB; // PB0-PB7
        // 1. GPIO & Clock 설정 (PB0, PB1, PB6)
        Macro_Set_Bit(RCC->AHB1ENR, 1); // GPIOB Clock ON
    }
    else if (ch < 24)
    {
        GPIOx = GPIOC; // PC0-PC7
        // 1. GPIO & Clock 설정 (PC0, PC1, PC6)
        Macro_Set_Bit(RCC->AHB1ENR, 2); // GPIOC Clock ON
    }
    else
    {
        printf("Invalid channel number: %d\n", ch);
        return NULL;
    }
    return GPIOx;
}

void _GPIO_Init_by_Channel(int ch)
{
    GPIO_TypeDef *GPIOx = Get_GPIO_Port_by_Channel(ch);
    if (GPIOx == NULL)
        return;

    Macro_Write_Block(GPIOx->MODER, 0x3, 0x3, (ch % 8) * 2); // Alternate Function Mode
    Macro_Write_Block(GPIOx->PUPDR, 0x3, 0x0, (ch % 8) * 2); // No Pull-up, Pull-down

    _ADC_Config_By_Channel(ch);
}

void _DMA_Init(void)
{
    // 4. DMA2_Stream0 설정 (여기가 핵심!)
    Macro_Set_Bit(RCC->AHB1ENR, 22); // DMA2 Clock ON

    // 설정 전 Stream 비활성화 (안전장치)
    DMA2_Stream0->CR &= ~(1 << 0);
    while (DMA2_Stream0->CR & (1 << 0))
        ; // 꺼질 때까지 대기

    DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);
    DMA2_Stream0->M0AR = (uint32_t)flame_sensors_raw;
    DMA2_Stream0->NDTR = SENSOR_NUM;

    // CR 설정 + 맨 마지막에 (1 << 0)을 더해서 EN(Enable) 시킴
    DMA2_Stream0->CR = (0 << 25) | (2 << 16) | (1 << 13) | (1 << 11) |
                       (1 << 10) | (1 << 8) | (1 << 0); // 맨 끝에 1(EN) 추가!
}

void _ADC_Start(void)
{
    Macro_Set_Bit(ADC1->CR1, 8); // SCAN Mode ON

    // 5. 최종 가동
    Macro_Set_Bit(ADC1->CR2, 0);  // ADC ON (ADON)
    Macro_Set_Bit(ADC1->CR2, 30); // SWSTART: 변환 시작!
}

void Flame_Init(int *chs)
{
    // #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
    __DSB();                                           // Data Synchronization Barrier (설정 적용 대기)
    __ISB();                                           // Instruction Synchronization Barrier
                                                       // #endif
    _ADC_Init();

    for (int i = 0; i < SENSOR_NUM; i++)
        _GPIO_Init_by_Channel(chs[i]);

    _DMA_Init();
    _ADC_Start();
}

void _init_linearize_sensor_data(float *values)
{
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        values[i] = 125.f;
    }
}

void get_linearize_sensor_data(float *values)
{
    static int initialized = 0;
    static float linearized_values[SENSOR_NUM] = {
        0,
    };
    if (!initialized)
    {
        _init_linearize_sensor_data(linearized_values);
        initialized = 1;
    }
    float temp_values[SENSOR_NUM] = {
        0,
    };
    for (int _i = 0; _i < NUMBER_OF_SAMPLES; _i++)
    {
        for (int i = 0; i < SENSOR_NUM; i++)
        {
            volatile float v = (float)flame_sensors_raw[i];
            v = v < 1 ? 1.f : v;
            v = 0xffff / v - 1; // Normalize to [0, 1]
            v = sqrtf(v);
            temp_values[i] += v;
        }
    }
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        temp_values[i] = temp_values[i] / NUMBER_OF_SAMPLES;
        linearized_values[i] = linearized_values[i] * (1 - FILTER_COEFFICIENT) + temp_values[i] * FILTER_COEFFICIENT; // Simple low-pass filter
        values[i] = linearized_values[i];
    }
}