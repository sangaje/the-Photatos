#include "device_driver.h"
#include "flame.h"
#include "servo.h"
#include "stepper.h"
#include <stdarg.h>
#include <stdio.h>

#define STEP_DELAY_MS 2
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_INIT_ANGLE 90

#define SQRT3_CONST 1.7320508f

static const unsigned char step_table[8][4] = {
    {1, 0, 0, 0},
    {1, 0, 0, 1},
    {0, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 0},
    {1, 1, 0, 0}};

static int step_seq = 0;
static int servo_angle = SERVO_INIT_ANGLE;
static int current = 0;
static void Sys_Init(int baud)
{
    int chs[SENSOR_NUM] = {5, 7, 8, 9};

    SCB->CPACR |= (0x3 << (10 * 2)) | (0x3 << (11 * 2));
    Clock_Init();
    Uart2_Init(baud);
    LED_Init();

    /* flame 3채널 초기화 */
    Flame_Init(chs);
}
void Main(void);
int main(void)
{
    Main();
    return 0;
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

    Stepper_Init();
    Servo_Init();
    Servo_Set_Angle(SERVO_INIT_ANGLE);

    printf("\n=== BASIC + FLAME MONITOR START ===\n");

    printf("SELF TEST: STEPPER + SERVO\n");
    // Stepper_Move_Relative(50);
    // TIM2_Delay(500);
    // Stepper_Move_Relative(-50);
    // TIM2_Delay(500);

    Servo_Set_Angle(90);
    // UART2_Printf("SERVO 60\n");
    TIM2_Delay(800);

    // Servo_Set_Angle(120);
    // UART2_Printf("SERVO 120\n");
    TIM2_Delay(800);

    // Servo_Set_Angle(SERVO_INIT_ANGLE);
    // UART2_Printf("SERVO 90\n");
    // TIM2_Delay(800);

    printf("SELF TEST DONE\n");

    while (1)
    {
        get_linearize_sensor_data(flames);

        FireVector_t fire_vector = fire_vector_estimation(flames);

        v[0] = fire_vector.x;
        v[1] = fire_vector.y;

        printf("F=[%.4f %.4f %.4f] V=[%.4f %.4f] SUM=%.4f ANG=%d           \r",
               flames[0], flames[1], flames[2], v[0], v[1], fire_vector.intensity, servo_angle);

        // Servo_Set_Angle(90);

        if (fire_vector.intensity < 200.f)
        {
            if (current++ % 2 == 0)
            {
                Stepper_Move_Relative(-(int)(fire_vector.x * 150.f));
                // TIM2_Delay(100);
            }
            else
            {
                servo_angle -= (int)(fire_vector.y * 50.f);
                servo_angle = (servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE : ((servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : servo_angle);
                Servo_Set_Angle(servo_angle);
                // TIM2_Delay(100);
            }
        }

        TIM2_Delay(10);
    }
}
