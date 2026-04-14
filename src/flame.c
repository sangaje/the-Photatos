#include "flame.h"
#include "macro.h"
#include "malloc.h"
#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// --- 1단계: 실제 글자를 붙이는 역할 (치환 안 함) ---
#define _INNER_CAT(a, b) a##b
#define _INNER_CAT3(a, b, c) a##b##c

// --- 2단계: 인수를 값(2)으로 먼저 확장시키는 역할 (중요!) ---
#define CAT(a, b) _INNER_CAT(a, b)
#define CAT3(a, b, c) _INNER_CAT3(a, b, c)

// --- 3단계: 실제 레지스터 별명 정의 ---
#define TIMx CAT(TIM, TimerNumber)
#define TIMx_IRQn CAT3(TIM, TimerNumber, _IRQn)
#define TIMx_IRQHandler CAT3(TIM, TimerNumber, _IRQHandler)

// 에러 났던 부분: 반드시 CAT3를 써야 함!
#define RCC_TIMx_EN CAT3(RCC_APB1ENR_TIM, TimerNumber, EN)

volatile int flame_print_ready = 0;

volatile uint16_t flame_sensors_raw[SENSOR_NUM] = {
    0x0,
};

volatile float flame_sensors_linearized[SENSOR_NUM] = {
    0,
};

volatile float directional_component_vector[SENSOR_NUM][2] = {{0, 1}};

extern int x;
extern volatile float y;
extern volatile float servo_angle;

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
        // printf("Invalid channel number: %d\n", ch);
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
    DMA2_Stream0->NDTR = SENSOR_NUM; // 센서 수

    // CR 설정 + 맨 마지막에 (1 << 0)을 더해서 EN(Enable) 시킴
    DMA2_Stream0->CR = (0 << 25) | (2 << 16) | (1 << 13) | (1 << 11) |
                       (1 << 10) | (1 << 8) | (1 << 0); // 맨 끝에 1(EN) 추가!
}

void _Timer_Init(uint16_t psc, uint16_t arr)
{
    // 1. 해당 타이머 클럭 활성화 (APB1 기준)
    RCC->APB1ENR |= RCC_TIMx_EN;

    // 2. 초기화 및 설정
    TIMx->CR1 &= ~TIM_CR1_CEN; // 일단 정지
    TIMx->CNT = 0;             // 카운터 초기화

    TIMx->PSC = psc - 1; // 분주기 설정
    TIMx->ARR = arr - 1; // 자동 재로드 값 설정

    // 3. 인터럽트 설정
    TIMx->DIER |= TIM_DIER_UIE; // Update Interrupt Enable

    // 4. NVIC 우선순위 설정 (최우선순위 0)
    NVIC_SetPriority(TIMx_IRQn, 0);
    NVIC_EnableIRQ(TIMx_IRQn);

    // 5. 타이머 시작
    TIMx->CR1 |= TIM_CR1_CEN;
}

volatile FireVector_t latest_fire_vector = {0, 0, 0};

#define PRINT_DECIMATION 10 /* printf는 PRINT_DECIMATION 회마다 1번 */

void TIMx_IRQHandler(void)
{
    if (TIMx->SR & 0x1) // Update interrupt
    {
        TIMx->SR &= ~0x1;
        static int print_cnt = 0;

        volatile float raw_linearized[SENSOR_NUM];
        _get_linearize_sensor_data(raw_linearized);
        volatile float u[CONTROL_DIM] = {(float)x, y};
        Kalman_Filter(raw_linearized, (volatile float *)flame_sensors_linearized, u);
        latest_fire_vector = fire_vector_estimation();

        if (++print_cnt >= PRINT_DECIMATION)
        {
            print_cnt = 0;
            flame_print_ready = 1;
        }
    }
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
    // for (size_t i = 0; i < SENSOR_NUM; i++)
    // {
    //     printf("\nSensor %d Direction: [%.4f, %.4f]\n", i, directional_component_vector[i][0], directional_component_vector[i][1]);
    // }
    // for (size_t i = 0; i < SENSOR_NUM; i++)
    // {
    //     printf("\nSensor %d Direction: [%.4f, %.4f]\n", i, directional_component_vector[i][0], directional_component_vector[i][1]);
    // }
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
    _Timer_Init(960, 10); // 10ms마다 인터럽트 발생 (TIMXCLK 96MHz / 960 = 100kHz, ARR=10 → 10ms)
}

// /**
//  * @brief Initialize linearized sensor output array with boundary baseline values.
//  * @param values Output array to initialize.
//  */
// void _init_linearize_sensor_data(volatile float *values)
// {
//     for (int i = 0; i < SENSOR_NUM; i++)
//     {
//         values[i] = FRAMES_BASIS_BOUNDARY;
//     }
// }

// int compare_floats(const void *a, const void *b)
// {
//     float fa = *(const float *)a;
//     float fb = *(const float *)b;
//     return (fa > fb) - (fa < fb); // 양수, 음수, 또는 0 반환
// }

// /**
//  * @brief Accumulate normalized sensor magnitudes across multiple samples.
//  * @param values Output accumulation array.
//  * @param count Number of samples to accumulate.
//  */
// void _sum_sensor_data(volatile float *values, int count)
// {
//     volatile float temp_values[SENSOR_NUM][NUMBER_OF_SAMPLES + 100] = {
//         0,
//     }; // 여유 공간 확보
//     for (int i = 0; i < SENSOR_NUM; i++)
//     {
//         values[i] = 0;
//     }
//     for (int i = 0; i < count; i++)
//     {
//         for (int j = 0; j < SENSOR_NUM; j++)
//         {
//             volatile float v = (float)flame_sensors_raw[j];
//             v = v < 1 ? 1.f : v;
//             v = 0xffff / v - 1; // Normalize to [0, 1]
//             v = sqrtf(v);
//             temp_values[j][i] = v;
//         }
//     }
//     for (int i = 0; i < SENSOR_NUM; i++)
//     {
//         qsort((void *)temp_values[i], count + 100, sizeof(float), compare_floats); // Sort each sensor's samples
//     }
//
//     for (int i = 0; i < SENSOR_NUM; i++)
//     {
//         for (int j = 50; j < NUMBER_OF_SAMPLES + 50; j++)
//         {
//             values[i] += temp_values[i][j];
//         }
//     }
// }

// /**
//  * @brief Compute the exponential moving average for a single sample.
//  * @param current_value Current measurement value.
//  * @param previous_ema Previous EMA result.
//  * @param alpha EMA smoothing factor in the range [0, 1].
//  * @return Updated EMA value.
//  * @example
//  * @code
//  * float ema = EMA_Filter(12.0f, 10.0f, 0.2f);
//  * @endcode
//  */
// volatile float EMA_Filter(volatile float current_value, volatile float previous_ema, volatile float alpha)
// {
//     return alpha * current_value + (1 - alpha) * previous_ema;
// }

/**
 * @brief Compute filtered, linearized flame sensor values with baseline compensation.
 * @param values Output array that receives processed sensor values.
 */
void _get_linearize_sensor_data(volatile float *values)
{
    for (int i = 0; i < SENSOR_NUM; i++)
    {
        volatile float v = (float)flame_sensors_raw[i];
        v = v < 1 ? 1.f : v;
        v = 0xffff / v - 1;
        v = sqrtf(v);
        values[i] = v;
    }
}

/* ================================================================
 *  4x4 행렬 연산 (칼만 필터 전용)
 * ================================================================ */
#define N SENSOR_NUM

// C = A + B
static void mat_add(float C[N][N], const float A[N][N], const float B[N][N])
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            C[i][j] = A[i][j] + B[i][j];
}

// C = A - B
static void mat_sub(float C[N][N], const float A[N][N], const float B[N][N])
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            C[i][j] = A[i][j] - B[i][j];
}

// C = A * B
static void mat_mul(float C[N][N], const float A[N][N], const float B[N][N])
{
    float tmp[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
        {
            tmp[i][j] = 0;
            for (int k = 0; k < N; k++)
                tmp[i][j] += A[i][k] * B[k][j];
        }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            C[i][j] = tmp[i][j];
}

// dst = I (단위 행렬)
static void mat_identity(float dst[N][N])
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            dst[i][j] = (i == j) ? 1.0f : 0.0f;
}

// Gauss-Jordan 방식 4x4 역행렬: inv = src^(-1), 성공 시 1 반환
static int mat_inv(float inv[N][N], const float src[N][N])
{
    float aug[N][2 * N];
    // [src | I] 확장 행렬 구성
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            aug[i][j] = src[i][j];
            aug[i][j + N] = (i == j) ? 1.0f : 0.0f;
        }
    }
    // 전방 소거 + 피벗
    for (int col = 0; col < N; col++)
    {
        // 최대 피벗 탐색
        int max_row = col;
        float max_val = fabsf(aug[col][col]);
        for (int row = col + 1; row < N; row++)
        {
            float v = fabsf(aug[row][col]);
            if (v > max_val)
            {
                max_val = v;
                max_row = row;
            }
        }
        if (max_val < 1e-12f)
            return 0; // 특이 행렬
        // 행 교환
        if (max_row != col)
        {
            for (int j = 0; j < 2 * N; j++)
            {
                float tmp = aug[col][j];
                aug[col][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }
        // 피벗 행 정규화
        float pivot = aug[col][col];
        for (int j = 0; j < 2 * N; j++)
            aug[col][j] /= pivot;
        // 소거
        for (int row = 0; row < N; row++)
        {
            if (row == col)
                continue;
            float factor = aug[row][col];
            for (int j = 0; j < 2 * N; j++)
                aug[row][j] -= factor * aug[col][j];
        }
    }
    // 결과 추출
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            inv[i][j] = aug[i][j + N];
    return 1;
}

/* ================================================================
 *  칼만 필터 (4-state, 측정 = 상태)
 *
 *  상태: x[4] = F 센서 값
 *  예측: x_pred = x_prev  (등속 모델, B*u = 0)
 *  측정: z[4]  = raw 변환 센서 값
 *
 *  P = P + Q        (불확실성 증가)
 *  K = P (P + R)^-1 (칼만 이득)
 *  x = x + K(z - x) (상태 업데이트)
 *  P = (I - K) P    (불확실성 감소)
 * ================================================================ */
static float kf_x[N];              // 상태 추정치
static float kf_P[N][N];           // 오차 공분산
static float kf_R_base[N][N];      // 측정 노이즈 공분산 (기준값)
static float kf_Q[N][N];           // 프로세스 노이즈 공분산
static float kf_B[N][CONTROL_DIM]; // 제어 입력 행렬 (매 스텝 동적 계산)
static int kf_initialized = 0;

/**
 * @brief Dominant 센서 기반으로 B 행렬을 동적 계산.
 *
 *  Pitch(servo, u[1]): S0(상) vs S2(하)
 *    pitch_dir = (S0 > S2) ? +1 : -1
 *    B[0][1] =  Kp * pitch_dir * max(S0, S2)
 *    B[2][1] = -Kp * pitch_dir * max(S0, S2)
 *
 *  Yaw(stepper, u[0]): S1 vs S3
 *    yaw_dir = (S1 > S3) ? +1 : -1
 *    B[1][0] =  Kp * yaw_dir * max(S1, S3)
 *    B[3][0] = -Kp * yaw_dir * max(S1, S3)
 */
static void Update_B_Matrix(const volatile float *z)
{
    float s0 = z[0], s1 = z[1], s2 = z[2], s3 = z[3];

    // B 행렬 초기화 (교차 항은 0)
    for (int i = 0; i < N; i++)
        for (int j = 0; j < CONTROL_DIM; j++)
            kf_B[i][j] = 0.0f;

    // Pitch (u[1]): S0 vs S2
    if (s0 > s2) {
        kf_B[0][1] = KALMAN_B_Kp;
        kf_B[2][1] = 1.1f * KALMAN_B_Kp;
    } else {
        kf_B[0][1] = 1.1f * KALMAN_B_Kp;
        kf_B[2][1] = KALMAN_B_Kp;
    }

    // Yaw (u[0]): S1 vs S3
    if (s1 > s3) {
        kf_B[1][0] = KALMAN_B_Kp;
        kf_B[3][0] = 1.1f * KALMAN_B_Kp;
    } else {
        kf_B[1][0] = 1.1f * KALMAN_B_Kp;
        kf_B[3][0] = KALMAN_B_Kp;
    }
}

static void Kalman_Init(const volatile float *z_init)
{
    // R 초기화
    float R_init[N][N] = KALMAN_R;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            kf_R_base[i][j] = R_init[i][j];

    // Q 초기화 (대각)
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            kf_Q[i][j] = (i == j) ? KALMAN_Q_DIAG : 0.0f;

    // P 초기화 (대각, 큰 값 = 초기 불확실성 높음)
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            kf_P[i][j] = (i == j) ? KALMAN_P_INIT : 0.0f;

    // 상태를 첫 측정치로 초기화
    for (int i = 0; i < N; i++)
        kf_x[i] = z_init[i];

    // B는 매 스텝 동적 계산 → 초기값은 첫 측정치 기반
    Update_B_Matrix(z_init);

    kf_initialized = 1;
}

void Kalman_Filter(volatile float *z, volatile float *x_out, volatile float *u)
{
    if (!kf_initialized)
    {
        Kalman_Init(z);
        for (int i = 0; i < N; i++)
            x_out[i] = kf_x[i];
        return;
    }

    /* 0. 측정치 기반 B 행렬 동적 갱신 */
    Update_B_Matrix(z);

    /* 1. 예측 단계: x_pred = x + B*u */
    for (int i = 0; i < N; i++)
    {
        float bu = 0;
        for (int j = 0; j < CONTROL_DIM; j++)
            bu += kf_B[i][j] * u[j];
        kf_x[i] += bu;
    }

    /* 2. 불확실성 예측: P = P + Q */
    float P_pred[N][N];
    mat_add(P_pred, kf_P, kf_Q);

    /* 2.5 Adaptive R: innovation 기반으로 R 스케일링 */
    float innovation[N];
    float R_adapted[N][N];
    for (int i = 0; i < N; i++)
        innovation[i] = z[i] - kf_x[i];

    // R_adapted = R_base 복사 후 대각만 스케일링
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            R_adapted[i][j] = kf_R_base[i][j];

    for (int i = 0; i < N; i++)
    {
        float abs_innov = fabsf(innovation[i]);
        float adaptive_term = ADAPTIVE_R_ALPHA * expf(abs_innov / ADAPTIVE_R_BETA);
        if (adaptive_term > ADAPTIVE_R_THRESHOLD)
            adaptive_term = ADAPTIVE_R_THRESHOLD;
        R_adapted[i][i] = kf_R_base[i][i] + adaptive_term;
    }

    /* 3. 칼만 이득: K = P_pred * (P_pred + R_adapted)^(-1) */
    float S[N][N]; // S = P_pred + R_adapted
    mat_add(S, P_pred, R_adapted);

    float S_inv[N][N];
    if (!mat_inv(S_inv, S))
    {
        // 역행렬 실패 시 측정치 그대로 사용
        for (int i = 0; i < N; i++)
            x_out[i] = z[i];
        return;
    }

    float K[N][N]; // K = P_pred * S_inv
    mat_mul(K, P_pred, S_inv);

    /* 4. 상태 업데이트: x = x + K * innovation */
    for (int i = 0; i < N; i++)
    {
        float correction = 0;
        for (int j = 0; j < N; j++)
            correction += K[i][j] * innovation[j];
        kf_x[i] += correction;
    }

    /* 5. 오차 공분산 업데이트: P = (I - K) * P_pred */
    float I_mat[N][N];
    mat_identity(I_mat);

    float IK[N][N]; // I - K
    mat_sub(IK, I_mat, K);

    mat_mul(kf_P, IK, P_pred);

    // 출력
    for (int i = 0; i < N; i++)
        x_out[i] = kf_x[i];
}

// /* 원본 _get_linearize_sensor_data (중앙값 + EMA 버전) */
// void _get_linearize_sensor_data(volatile float *values)
// {
//     static int initialized = 0;
//     static volatile float linearized_values[SENSOR_NUM] = {
//         0,
//     };
//     static volatile float linearized_values_basis[SENSOR_NUM] = {
//         0,
//     };
//     volatile float temp_values[SENSOR_NUM];
//     if (!initialized)
//     {
//         _init_linearize_sensor_data((volatile float *)linearized_values);
//         initialized = 1;
//     }
//     _sum_sensor_data((volatile float *)temp_values, NUMBER_OF_SAMPLES);
//     for (int i = 0; i < SENSOR_NUM; i++)
//     {
//         temp_values[i] = temp_values[i] / NUMBER_OF_SAMPLES;
//         if (temp_values[i] > FRAMES_BASIS_BOUNDARY)
//         {
//             linearized_values_basis[i] = EMA_Filter(temp_values[i] - FRAMES_BASIS_BOUNDARY, linearized_values_basis[i], FILTER_COEFFICIENT);
//         }
//         linearized_values[i] = EMA_Filter(temp_values[i], linearized_values[i], FILTER_COEFFICIENT);
//         values[i] = linearized_values[i] - linearized_values_basis[i];
//     }
// }

/**
 * @brief Estimate the fire direction vector and average intensity from sensor values.
 * @param values Processed sensor values used for vector accumulation.
 * @return FireVector_t containing x/y direction and averaged intensity.
 */
FireVector_t fire_vector_estimation()
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
        v[i] = flame_sensors_linearized[i];
        sum += v[i] * v[i];
    }
    sum = sqrtf(sum);
    if (sum == 0.0f)
    {
        return retv;
    }

    for (int i = 0; i < SENSOR_NUM; i++)
    {
        v[i] = v[i] / sum; // 정규화
    }

    for (int i = 0; i < SENSOR_NUM; i++)
    {
        retv.x += directional_component_vector[i][0] * v[i];
        retv.y += directional_component_vector[i][1] * v[i];
        retv.intensity += flame_sensors_linearized[i];
    }

    retv.y = -retv.y / SENSOR_NUM;                // 평균 방향 벡터 y
    retv.x = -retv.x / SENSOR_NUM;                // 평균 방향 벡터 x
    retv.intensity = retv.intensity / SENSOR_NUM; // 평균 강도

    return retv;
}