#include "buzzer.h"
#include "device_driver.h"
#include "flame.h"
#include "led.h"
#include "pump.h"
#include "servo.h"
#include "stepper.h"
#include <stdarg.h>
#include <stdio.h>

#define STEP_DELAY_MS 2
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_INIT_ANGLE 55
#define STEPPER_X_DEADBAND 8


static int servo_angle = SERVO_INIT_ANGLE;
static int current = 0;
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
    float v[2] = {
        0.0f,
    };

    Sys_Init(115200);

    unsigned char pump_status = 0;      // 0: OFF, 1: ON
    unsigned char prev_btn_state = 0;   // 버튼 엣지 검출용
    unsigned char water_error_flag = 0; // 물 부족 경고 중복 실행 방지용
    int mode = 0;

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
            int x = -(int)(fire_vector.x * 200.f);
            x = x > 50 ? 50 : (x < -50 ? -50 : x);
            // if (mode == 0)
            Stepper_Move_Relative(x);
            // Stepper_Move_Relative(-(int)(fire_vector.x * 150.f));
            int y = -(int)(fire_vector.y * 100.f);
            // printf("Raw Y: %.4f, Mapped Y: %d\n", (fire_vector.y * 50.f), y);
            y = y > 3 ? 3 : (y < -3 ? -3 : y);
            servo_angle -= y;
            servo_angle = (servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE : ((servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : servo_angle);
            Servo_Set_Angle(servo_angle);
            // TIM2_Delay(30);
            if (x > STEPPER_X_DEADBAND || x < -STEPPER_X_DEADBAND)
            {
                mode = 1;
            }
            // printf("Stepper Move: %d steps\n", x);
            printf("RAW=[%4d %4d %4d %4d] F=[%4.4f %4.4f %4.4f %4.4f] V=[%4.4f %4.4f] SUM=%4.4f ANG=%4d x=%d y=%d\n",
                   flame_sensors_raw[0], flame_sensors_raw[1], flame_sensors_raw[2], flame_sensors_raw[3],
                   flames[0], flames[1], flames[2], flames[3], v[0], v[1], fire_vector.intensity, servo_angle, x, -(int)(fire_vector.y * 100.f));
        }
    }
}
