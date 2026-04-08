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
#define SERVO_MIN_ANGLE 30
#define SERVO_MAX_ANGLE 150
#define SERVO_INIT_ANGLE 90
#define BTN_PORT GPIOC                                                    // PC
#define BTN_PIN 13                                                        // Pin 13
#define IS_BTN_PRESSED (Macro_Check_Bit_Set(BTN_PORT->IDR, BTN_PIN) == 0) // Press = 0

static int servo_angle = SERVO_INIT_ANGLE;
static int current = 0;
static void Sys_Init(int baud)
{
    int chs[SENSOR_NUM] = {5, 7, 8, 9};

    SCB->CPACR |= (0x3 << (10 * 2)) | (0x3 << (11 * 2));
    Clock_Init();
    Uart2_Init(baud);
    LED_Init();

    Pump_Init();        // PC4
    WaterSensor_Init(); // PC3 (ADC CH13)
    Buzzer_Init();      // PC0
    LED_Init();         // PC1, PC2

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
    float v[2] = {
        0.0f,
    };

    Sys_Init(115200);

    unsigned char pump_status = 0;      // 0: OFF, 1: ON
    unsigned char prev_btn_state = 0;   // 버튼 엣지 검출용
    unsigned char water_error_flag = 0; // 물 부족 경고 중복 실행 방지용

    printf("\n=== BASIC + FLAME MONITOR START ===\n");

    printf("SELF TEST: STEPPER + SERVO\n");
    Servo_Set_Angle(90);
    TIM2_Delay(800);

    printf("SELF TEST DONE\n");

    while (1)
    {
        get_linearize_sensor_data(flames);

        FireVector_t fire_vector = fire_vector_estimation(flames);

        v[0] = fire_vector.x;
        v[1] = fire_vector.y;

        printf("F=[%.4f %.4f %.4f] V=[%.4f %.4f] SUM=%.4f ANG=%d\r",
               flames[0], flames[1], flames[2], v[0], v[1], fire_vector.intensity, servo_angle);

        // Servo_Set_Angle(90);

        if (fire_vector.intensity < 200.f)
        {
            if (current++ % 2 == 0)
            {
                Stepper_Move_Relative((int)(fire_vector.x * 100.f));
                TIM2_Delay(100);
            }
            else
            {
                servo_angle -= (int)(fire_vector.y * 25.f);
                servo_angle = (servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE : ((servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : servo_angle);
                Servo_Set_Angle(servo_angle);
                TIM2_Delay(100);
            }
        }

        // --- [로직 1 & 2] 수위 센서 및 LED/부저 제어 ---
        int water_level = WaterSensor_Read();

        if (water_level >= WATER_DETECTED)
        {
            // [조건 1] 물이 감지된 경우
            LED_Green_On();
            LED_Red_Off();
            water_error_flag = 0; // 물이 다시 찼으므로 플래그 리셋
        }
        else if (water_level < WATER_EMPTY)
        {
            // [조건 2] 물이 감지되지 않는 경우
            LED_Green_Off();
            LED_Red_On();

            // 처음 물 부족이 감지된 순간에만 부저를 3초간 울림
            if (water_error_flag == 0)
            {
                Buzzer_On();
                Pump_Delay(3000); // 3초 대기 (소프트웨어 지연)
                Buzzer_Off();
                water_error_flag = 1; // 다시 물이 차기 전까지는 부저를 울리지 않음
            }
        }

        // --- [로직 3] 워터펌프 버튼 토글 제어 ---
        if (IS_BTN_PRESSED && prev_btn_state == 0)
        {
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

        TIM2_Delay(10);
    }

}
