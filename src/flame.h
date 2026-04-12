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
#define KALMAN_R                             \
    {                                        \
        {429.077, -0.1982, -2.2893, 1.742},  \
        {-0.1982, 106.2749, 2.351, -0.2305}, \
        {-2.2893, 2.351, 283.2446, -0.021},  \
        {1.742, -0.2305, -0.021, 70.1943}}

// 프로세스 노이즈 공분산 Q — 작을수록 예측 신뢰, 클수록 측정 신뢰
#define KALMAN_Q_DIAG 10.0f

// 초기 오차 공분산 P 대각 값
#define KALMAN_P_INIT 1000.0f

// Adaptive R 파라미터: R_eff[i][i] = R_base[i][i] + min(THRESHOLD, ALPHA * e^(|innov|/BETA))
// ALPHA: 스케일 강도
// BETA:  민감도 (작을수록 작은 innovation에도 반응)
// THRESHOLD: R 증가 상한선 (R_base 대비 최대 더해지는 값)
#define ADAPTIVE_R_ALPHA 1.0f
#define ADAPTIVE_R_BETA 10.0f
#define ADAPTIVE_R_THRESHOLD 3000.0f

// 제어 입력 차원: u = [stepper_x, servo_y]
#define CONTROL_DIM 2

// 제어 입력 행렬 B (4x2): 모터 움직임이 센서 값에 미치는 영향
// B[i][0] = stepper(pan)가 센서 i에 미치는 계수
// B[i][1] = servo(tilt)가 센서 i에 미치는 계수
#define KALMAN_B {  \
    {0.1f, 0.1f},   \
    {0.1f, -0.1f},  \
    {-0.1f, -0.1f}, \
    {-0.1f, 0.1f}}

extern volatile FireVector_t latest_fire_vector;

void Flame_Init(int *chs);
void _get_linearize_sensor_data(volatile float *values);
void Kalman_Filter(volatile float *z, volatile float *x_out, volatile float *u);
FireVector_t fire_vector_estimation();