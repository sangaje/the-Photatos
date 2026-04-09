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

#define STEP_DELAY_MS 2
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_INIT_ANGLE 55
#define AIM_SERVO_ACT_DEADBAND 1
#define AIM_STEPPER_ACT_DEADBAND 2
#define STEPPER_X_DEADBAND 8


static volatile float servo_angle = SERVO_INIT_ANGLE;
static int current = 0;
static int last_servo_cmd = SERVO_INIT_ANGLE;
static int mode = 0; // 0: Idle, 1: Active

FireState fire_state = STATE_SAFE; // 초기 상태는 안전

static void Uart2_Send_Hex(uint32_t value)
{
    char hex_digits[] = "0123456789ABCDEF";
    char buffer[9]; // 8자리 + null
    buffer[8] = '\0';
    for (int i = 7; i >= 0; i--) {
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

    if (csr & (1 << 31)) printf(" - Low-power reset\n");
    if (csr & (1 << 30)) printf(" - Window watchdog reset\n");
    if (csr & (1 << 29)) printf(" - Independent watchdog reset\n");
    if (csr & (1 << 28)) printf(" - Software reset\n");
    if (csr & (1 << 27)) printf(" - POR/PDR reset\n");
    if (csr & (1 << 26)) printf(" - PIN reset\n");
    if (csr & (1 << 25)) printf(" - BOR reset\n");

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

    Pump_Init(); // PC4
    Buzzer_Init(); // PC0

    Stepper_Init();
    Servo_Init();
    Servo_Set_Angle(SERVO_INIT_ANGLE);

    /* flame 3채널 초기화 */
    Flame_Init(chs);

    // 리셋 원인 확인
    Check_Reset_Reason();
}
void Main(void)
{
    volatile float flames[SENSOR_NUM] = {
        0.0f,
    };
    volatile float v[2] = {
        0.0f,
    };

    Sys_Init(115200);

    unsigned char pump_status = 0;      // 0: OFF, 1: ON
    unsigned char prev_btn_state = 0;   // 버튼 엣지 검출용
    unsigned char water_error_flag = 0; // 물 부족 경고 중복 실행 방지용

    printf("\n=== BASIC + FLAME MONITOR START ===\n");

    printf("SELF TEST: STEPPER + SERVO\n");
    Servo_Set_Angle(55);
    TIM2_Delay(800);

    printf("SELF TEST DONE\n");

    while (1)
    {
        FireVector_t fire_vector = fire_vector_estimation();

        v[0] = fire_vector.x;
        v[1] = fire_vector.y;
        extern uint16_t flame_sensors_raw[SENSOR_NUM];

        // Servo_Set_Angle(90);
        
        if (fire_vector.intensity < 40.f)
        {
            /* --- stepper (pan) --- */
            int x = -(int)(fire_vector.x * 200.f);
            
            if (x > AIM_STEPPER_ACT_DEADBAND || x < -AIM_STEPPER_ACT_DEADBAND)
            {
                Stepper_Move_Relative(x + 5); // calibrating overshooting by adding a small bias
                if (x > STEPPER_X_DEADBAND || x < -STEPPER_X_DEADBAND)
                {
                    mode = 1;
                }
            }
            else
            {
                x = 0;  /* for the printf */
            }
            
            /* --- servo (tilt) --- */
            volatile float y = -(fire_vector.y * 5.f);
            y = y > 0.5 ? 1 : (y < -0.5 ? -1 : y);
            
                servo_angle -= y;
                servo_angle = (servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE :
                ((servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : servo_angle);
                Servo_Set_Angle(servo_angle+10);
            
            Pump_Control_Update(v[0], v[1], fire_vector.intensity, flames, SENSOR_NUM);
        }
        else {
            Servo_Set_Angle(55);
        }

        fire_state = Update_Fire_State(fire_state, fire_vector.intensity);
        TIM2_Delay(10);
    }
}
