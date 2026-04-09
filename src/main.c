#include "buzzer.h"
#include "device_driver.h"
#include "flame.h"
#include "led.h"
#include "pump.h"
#include "servo.h"
#include "stepper.h"
#include "water_sensor.h"
#include <stdarg.h>
#include <stdio.h>

#define STEP_DELAY_MS 2
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_INIT_ANGLE 55
#define AIM_SERVO_ACT_DEADBAND 1

#define STEPPER_X_DEADBAND 8
#define AIM_STEPPER_ACT_DEADBAND 2

#define BTN_PORT GPIOC                                                    // PC
#define BTN_PIN 13                                                        // Pin 13
#define IS_BTN_PRESSED (Macro_Check_Bit_Set(BTN_PORT->IDR, BTN_PIN) == 0) // Press = 0

static volatile float servo_angle = SERVO_INIT_ANGLE;
static int current = 0;
static int last_servo_cmd = SERVO_INIT_ANGLE;
static int mode = 0; // 0: Idle, 1: Active
static void Sys_Init(int baud)
{
    int chs[SENSOR_NUM] = {5, 7, 8, 9};

    SCB->CPACR |= (0x3 << (10 * 2)) | (0x3 << (11 * 2));
    Clock_Init();
    Uart2_Init(baud);
    LED_Init();

    Pump_Init(); // PC4
    // WaterSensor_Init(); // PC3 (ADC CH13)
    Buzzer_Init(); // PC0
    LED_Init();    // PC1, PC2

    Stepper_Init();
    Servo_Init();
    Servo_Set_Angle(SERVO_INIT_ANGLE);

    /* flame 3채널 초기화 */
    Flame_Init(chs);
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
        get_linearize_sensor_data(flames);

        FireVector_t fire_vector = fire_vector_estimation(flames);

        v[0] = fire_vector.x;
        v[1] = fire_vector.y;
        extern uint16_t flame_sensors_raw[SENSOR_NUM];

        // Servo_Set_Angle(90);

        if (fire_vector.intensity < 100.f)
        {
                        /* --- stepper (pan) --- */
            int x = -(int)(fire_vector.x * 200.f);
            x = x > 50 ? 50 : (x < -50 ? -50 : x);

            if (x > AIM_STEPPER_ACT_DEADBAND || x < -AIM_STEPPER_ACT_DEADBAND)
            {
                Stepper_Move_Relative(x); // calibrating overshooting by adding a small bias
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
            volatile float y = -(fire_vector.y * 30.f);
            // y = y > 3 ? 3 : (y < -3 ? -3 : y);

            if (y > AIM_SERVO_ACT_DEADBAND || y < -AIM_SERVO_ACT_DEADBAND)
            {
                servo_angle -= y;
                servo_angle = (servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE :
                            ((servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : servo_angle);
                Servo_Set_Angle(servo_angle+10);
            }
            // int new_servo_angle = servo_angle - y;
            // new_servo_angle = (new_servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE :
            //                 ((new_servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : new_servo_angle);

            // /* only command servo if target actually moved enough */
            // int delta = new_servo_angle - last_servo_cmd;
            // if (delta >= AIM_SERVO_ACT_DEADBAND || delta <= -AIM_SERVO_ACT_DEADBAND)
            // {
            //     Servo_Set_Angle(new_servo_angle);
            //     last_servo_cmd = new_servo_angle;
            // }
            // servo_angle = new_servo_angle;
        }

        // --- [로직 3] 워터펌프 버튼 토글 제어 ---
        if (IS_BTN_PRESSED && !prev_btn_state)
        {
            printf("\n[BUTTON PRESSED] Toggling Pump State...\n");
            pump_status = !pump_status; // 상태 반전

            if (pump_status)
            {
                Pump_On();
            }
            else
            {
                Pump_Off();
            }

            // 버튼 디바운싱
            Pump_Delay(200);
        }

        // 현재 버튼 상태 저장
        prev_btn_state = IS_BTN_PRESSED;

        TIM2_Delay(50);
    }
}
