#include "buzzer.h"
#include "device_driver.h"
#include "flame.h"
#include "led.h"
#include "pump.h"
#include "servo.h"
#include "stepper.h"
#include "water.h"
#include <stdarg.h>
#include <stdio.h>

extern volatile int flame_print_ready;
extern volatile uint16_t flame_sensors_raw[];
extern volatile float flame_sensors_linearized[];

#define STEP_DELAY_MS 2
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_INIT_ANGLE 55
#define AIM_SERVO_ACT_DEADBAND 1
#define AIM_STEPPER_ACT_DEADBAND 2
#define STEPPER_X_DEADBAND 8

volatile float servo_angle = SERVO_INIT_ANGLE;
static int current = 0;
static int last_servo_cmd = SERVO_INIT_ANGLE;
static int mode = 0; // 0: Idle, 1: Active
volatile float y = 0.0f;
int x = 0;
static volatile int stepper_pending = 0;

FireState fire_state = STATE_SAFE;

#define MOTOR_TIMER_PSC 16000 // 16MHz / 16000 = 1kHz
#define MOTOR_TIMER_ARR 50    // 1kHz / 50 = 20Hz = 50ms

#define EMA_ALPHA 0.1f /* 0에 가까울수록 둔감, 1이면 EMA 없음 */

/* TIM5 ISR: 50ms 주기로 서보/벡터 계산 (블로킹 없음) */
void TIM5_IRQHandler(void)
{
    if (!(TIM5->SR & TIM_SR_UIF))
        return;
    TIM5->SR &= ~TIM_SR_UIF;

    static float vx_ema = 0.0f;
    static float vy_ema = 0.0f;

    volatile float vx_raw = latest_fire_vector.x;
    volatile float vy_raw = latest_fire_vector.y;
    volatile float intensity = latest_fire_vector.intensity;

    vx_ema = EMA_ALPHA * vx_raw + (1.0f - EMA_ALPHA) * vx_ema;
    vy_ema = EMA_ALPHA * vy_raw + (1.0f - EMA_ALPHA) * vy_ema;

    Pump_Control_Update(vx_ema, vy_ema, latest_fire_vector.intensity, flame_sensors_linearized, SENSOR_NUM);
    fire_state = Update_Fire_State(fire_state, latest_fire_vector.intensity);
    if (intensity < 90.f)
    {
        /* --- stepper (pan) --- 계산만, 실제 구동은 main loop에서 */
        x = -(int)(vx_ema * 100.f);
        if (x > AIM_STEPPER_ACT_DEADBAND || x < -AIM_STEPPER_ACT_DEADBAND)
        {
            if (x > STEPPER_X_DEADBAND || x < -STEPPER_X_DEADBAND)
                mode = 1;
            stepper_pending = x + 5;
        }
        else
        {
            x = 0;
            stepper_pending = 0;
        }

        /* --- servo (tilt) --- */
        y = vy_ema * 2.f;
        y = y > 0.5f ? 1.0f : (y < -0.5f ? -1.0f : y);
        servo_angle -= y;
        Servo_Set_Angle(servo_angle);
    }
    else
    {
        Servo_Set_Angle(SERVO_INIT_ANGLE);
        servo_angle = SERVO_INIT_ANGLE;
        stepper_pending = 0;
    }
}

static void Motor_Timer_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->CNT = 0;
    TIM5->PSC = MOTOR_TIMER_PSC - 1;
    TIM5->ARR = MOTOR_TIMER_ARR - 1;
    TIM5->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM5_IRQn, 2); // 센서(TIM3)=0 보다 낮은 우선순위
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 |= TIM_CR1_CEN;
}

static void Uart2_Send_Hex(uint32_t value)
{
    char hex_digits[] = "0123456789ABCDEF";
    char buffer[9]; // 8자리 + null
    buffer[8] = '\0';
    for (int i = 7; i >= 0; i--)
    {
        buffer[i] = hex_digits[value & 0xF];
        value >>= 4;
    }
    printf("%s", buffer);
}

static void Check_Reset_Reason(void)
{
    uint32_t csr = RCC->CSR;
    printf("Reset Reason CSR: 0x");
    Uart2_Send_Hex(csr);
    printf("\n");

    if (csr & (1 << 31))
        printf(" - Low-power reset\n");
    if (csr & (1 << 30))
        printf(" - Window watchdog reset\n");
    if (csr & (1 << 29))
        printf(" - Independent watchdog reset\n");
    if (csr & (1 << 28))
        printf(" - Software reset\n");
    if (csr & (1 << 27))
        printf(" - POR/PDR reset\n");
    if (csr & (1 << 26))
        printf(" - PIN reset\n");
    if (csr & (1 << 25))
        printf(" - BOR reset\n");

    // 플래그 클리어
    RCC->CSR |= RCC_CSR_RMVF;
}

static void Sys_Init(int baud)
{
    int chs[SENSOR_NUM] = {5, 7, 8, 9};

    SCB->CPACR |= (0x3 << (10 * 2)) | (0x3 << (11 * 2));
    Clock_Init();
    Uart2_Init(baud);
    LED_Init();

    // Pump_Init(); // PC4
    // Buzzer_Init(); // PC0

    Stepper_Init();
    Servo_Init();
    Servo_Set_Angle(SERVO_INIT_ANGLE);

    /* flame 센서 초기화 */
    Flame_Init(chs);

    /* 모터 제어 타이머 (TIM5, 50ms) */
    Motor_Timer_Init();

    // 리셋 원인 확인
    Check_Reset_Reason();
}
void Main(void)
{
    volatile float flames[SENSOR_NUM] = {0.0f};

    Sys_Init(230400);

    printf("\n=== FLAME MONITOR START ===\n");
    Servo_Set_Angle(SERVO_INIT_ANGLE);
    TIM2_Delay(800);

    while (1)
    {
        // stepper 구동 (ISR에서 계산된 값, main loop에서 블로킹 실행)
        int steps = stepper_pending;
        if (steps != 0)
        {
            stepper_pending = 0;
            Stepper_Move_Relative(steps);
        }

        // 센서/칼만/벡터 상태 출력
        if (flame_print_ready)
        {
            flame_print_ready = 0;
            printf("RAW=[%4d %4d %4d %4d] F=[%4.4f %4.4f %4.4f %4.4f] V=[%4.4f %4.4f] SUM=%4.4f x=%4d y=%4.4f ANG=%4.4f\n",
                   flame_sensors_raw[0], flame_sensors_raw[1], flame_sensors_raw[2], flame_sensors_raw[3],
                   flame_sensors_linearized[0], flame_sensors_linearized[1], flame_sensors_linearized[2], flame_sensors_linearized[3],
                   latest_fire_vector.x, latest_fire_vector.y, latest_fire_vector.intensity, x, y, servo_angle);
        }

        // TIM2_Delay(30); // 속도 조절
    }
}
