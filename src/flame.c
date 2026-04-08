#include "flame.h"
#include "macro.h"
#include "malloc.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>

volatile uint16_t flame_sensors_raw[SENSOR_NUM + 1] = {
    0x0,
};

volatile uint16_t *others_raw = flame_sensors_raw + SENSOR_NUM;

volatile float directional_component_vector[SENSOR_NUM][2] = {{0, 1}};

/**
 * @brief Initialize ADC peripheral global configuration for multi-channel sampling.
 */
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

/**
 * @brief Configure one ADC channel's sampling time and sequence order.
 * @param ch ADC channel number.
 */
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

/**
 * @brief Resolve GPIO port from ADC channel index and enable the matching GPIO clock.
 * @param ch ADC channel number.
 * @return Pointer to the GPIO peripheral, or NULL if the channel is invalid.
 */
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

/**
 * @brief Initialize GPIO pin for the given ADC channel and bind it to ADC sequencing.
 * @param ch ADC channel number.
 */
void _GPIO_Init_by_Channel(int ch)
{
    GPIO_TypeDef *GPIOx = Get_GPIO_Port_by_Channel(ch);
    if (GPIOx == NULL)
        return;

    Macro_Write_Block(GPIOx->MODER, 0x3, 0x3, (ch % 8) * 2); // Alternate Function Mode
    Macro_Write_Block(GPIOx->PUPDR, 0x3, 0x0, (ch % 8) * 2); // No Pull-up, Pull-down

    _ADC_Config_By_Channel(ch);
}

/**
 * @brief Initialize DMA2 Stream0 for circular ADC data transfer into sensor buffer.
 */
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
    DMA2_Stream0->NDTR = SENSOR_NUM + 1; // 센서 수 + 1 (others_raw 공간)

    // CR 설정 + 맨 마지막에 (1 << 0)을 더해서 EN(Enable) 시킴
    DMA2_Stream0->CR = (0 << 25) | (2 << 16) | (1 << 13) | (1 << 11) |
                       (1 << 10) | (1 << 8) | (1 << 0); // 맨 끝에 1(EN) 추가!
}

/**
 * @brief Start ADC scan conversion using software trigger.
 */
void _ADC_Start(void)
{
    Macro_Set_Bit(ADC1->CR1, 8); // SCAN Mode ON

    // 5. 최종 가동
    Macro_Set_Bit(ADC1->CR2, 0);  // ADC ON (ADON)
    Macro_Set_Bit(ADC1->CR2, 30); // SWSTART: 변환 시작!
}

/**
 * @brief Initialize per-sensor directional unit vectors on a circular layout.
 * @example
 * @code
 * _Init_Directional_Component_Vector();
 * @endcode
 */
void _Init_Directional_Component_Vector(void)
{
    volatile float angle_increment = 2 * M_PI / SENSOR_NUM;
    volatile float cos_val = cosf(angle_increment);
    volatile float sin_val = sinf(angle_increment);
    for (int i = 1; i < SENSOR_NUM; i++)
    {
        volatile float x = directional_component_vector[i - 1][0];
        volatile float y = directional_component_vector[i - 1][1];
        directional_component_vector[i][0] = cos_val * x - sin_val * y;
        directional_component_vector[i][1] = sin_val * x + cos_val * y;
    }
    for (size_t i = 0; i < SENSOR_NUM; i++)
    {
        printf("\nSensor %d Direction: [%.4f, %.4f]\n", i, directional_component_vector[i][0], directional_component_vector[i][1]);
        /* code */
    }
}

/**
 * @brief Initialize flame sensor acquisition pipeline (GPIO, ADC, DMA, and start conversion).
 * @param chs Pointer to ADC channel list for each sensor.
 */
void Flame_Init(int *chs)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
    __DSB();                                           // Data Synchronization Barrier (설정 적용 대기)
    __ISB();                                           // Instruction Synchronization Barrier
#endif
    _ADC_Init();

    for (int i = 0; i < SENSOR_NUM; i++)
        _GPIO_Init_by_Channel(chs[i]);

    _DMA_Init();
    _ADC_Start();
    _Init_Directional_Component_Vector();
}

/**
 * @brief Initialize linearized sensor output array with boundary baseline values.
 * @param values Output array to initialize.
 */
void _init_linearize_sensor_data(volatile float *values)
{
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        values[i] = FRAMES_BASIS_BOUNDARY;
    }
}

/**
 * @brief Accumulate normalized sensor magnitudes across multiple samples.
 * @param values Output accumulation array.
 * @param count Number of samples to accumulate.
 */
void _sum_sensor_data(volatile float *values, int count)
{
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        values[i] = 0;
    }
    for (int _i = 0; _i < count; _i++)
    {
        for (int i = 0; i < SENSOR_NUM; i++)
        {
            volatile float v = (float)flame_sensors_raw[i];
            v = v < 1 ? 1.f : v;
            v = 0xffff / v - 1; // Normalize to [0, 1]
            v = sqrtf(v);
            values[i] += v;
        }
    }
}

/**
 * @brief Compute the exponential moving average for a single sample.
 * @param current_value Current measurement value.
 * @param previous_ema Previous EMA result.
 * @param alpha EMA smoothing factor in the range [0, 1].
 * @return Updated EMA value.
 * @example
 * @code
 * float ema = EMA_Filter(12.0f, 10.0f, 0.2f);
 * @endcode
 */
volatile float EMA_Filter(volatile float current_value, volatile float previous_ema, volatile float alpha)
{
    return alpha * current_value + (1 - alpha) * previous_ema;
}

/**
 * @brief Compute filtered, linearized flame sensor values with baseline compensation.
 * @param values Output array that receives processed sensor values.
 */
void get_linearize_sensor_data(volatile float *values)
{
    static int initialized = 0;
    static volatile float linearized_values[SENSOR_NUM] = {
        0,
    };
    static volatile float linearized_values_basis[SENSOR_NUM] = {
        0,
    };
    volatile float temp_values[SENSOR_NUM];
    if (!initialized)
    {
        _init_linearize_sensor_data((volatile float *)linearized_values);
        initialized = 1;
    }
    _sum_sensor_data((volatile float *)temp_values, NUMBER_OF_SAMPLES);
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        temp_values[i] = temp_values[i] / NUMBER_OF_SAMPLES;
        if (temp_values[i] > FRAMES_BASIS_BOUNDARY)
        {
            linearized_values_basis[i] = EMA_Filter(temp_values[i] - FRAMES_BASIS_BOUNDARY, linearized_values_basis[i], FILTER_COEFFICIENT);
        }
        linearized_values[i] = EMA_Filter(temp_values[i], linearized_values[i], FILTER_COEFFICIENT); // Simple low-pass filter
        values[i] = linearized_values[i] - linearized_values_basis[i];
        // printf("RAW[%d]=%010.u LIN=%.4f BAS=%.4f OUT=%.4f\n", i, flame_sensors_raw[i], linearized_values[i], linearized_values_basis[i], values[i]);
    }
}

/**
 * @brief Estimate the fire direction vector and average intensity from sensor values.
 * @param values Processed sensor values used for vector accumulation.
 * @return FireVector_t containing x/y direction and averaged intensity.
 */
FireVector_t fire_vector_estimation(volatile float *values)
{
    FireVector_t retv = {
        .x = 0,
        .y = 0,
        .intensity = 0,
    };

    volatile float v[SENSOR_NUM] = {
        0,
    };
    volatile float sum = 0;
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        v[i] = values[i];
        sum += v[i] * v[i];
    }
    sum = sqrtf(sum);

    for (int i = 0; i < SENSOR_NUM; i++)
    {
        v[i] = v[i] / sum; // 정규화
    }

    for (int i = 0; i < SENSOR_NUM; i++)
    {
        retv.x += directional_component_vector[i][0] * v[i];
        retv.y += directional_component_vector[i][1] * v[i];
        retv.intensity += values[i];
    }

    retv.y = -retv.y / SENSOR_NUM;                // 평균 방향 벡터 y
    retv.x = -retv.x / SENSOR_NUM;                // 평균 방향 벡터 x
    retv.intensity = retv.intensity / SENSOR_NUM; // 평균 강도

    return retv;
}