#include "stm32f4xx.h"

#define SENSOR_NUM 4
#define SAMPLING_TIME 0x7
#define FILTER_COEFFICIENT 0.1f
#define FRAMES_BASIS_BOUNDARY 45.f
#define NUMBER_OF_SAMPLES 100

#define TimerNumber 3

typedef struct
{
    volatile float x;
    volatile float y;
    volatile float intensity;
} FireVector_t;

/* ── 칼만 필터 튜닝 파라미터 ── */

// 측정 노이즈 공분산 R (4x4) — data.txt에서 계산된 값
#define KALMAN_R { \
    {585.1073f,   2.5076f, -15.1616f,   2.2921f}, \
    {  2.5076f, 200.7127f,  -8.3256f,   8.8815f}, \
    {-15.1616f,  -8.3256f, 632.0889f,  -2.7160f}, \
    {  2.2921f,   8.8815f,  -2.7160f, 365.1587f}  \
}

// 프로세스 노이즈 공분산 Q — 작을수록 예측 신뢰, 클수록 측정 신뢰
#define KALMAN_Q_DIAG 10.0f

// 초기 오차 공분산 P 대각 값
#define KALMAN_P_INIT 1000.0f

extern volatile FireVector_t latest_fire_vector;

void Flame_Init(int *chs);
void _get_linearize_sensor_data(volatile float *values);
void Kalman_Filter(volatile float *z, volatile float *x_out);
FireVector_t fire_vector_estimation();