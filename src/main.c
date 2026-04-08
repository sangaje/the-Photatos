#include "device_driver.h"
#include "flame.h"
#include <stdarg.h>
#include <stdio.h>

#define STEP_DELAY_MS 2
#define SERVO_MIN_ANGLE 30
#define SERVO_MAX_ANGLE 150
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

void Main(void);
int main(void)
{
    Main();
    return 0;
}

static void UART2_Printf(char *fmt, ...)
{
    va_list ap;
    char str[256];
    char *p;

    va_start(ap, fmt);
    vsprintf(str, fmt, ap);
    va_end(ap);

    for (p = str; *p != 0; p++)
        Uart2_Send_Byte(*p);
}

static void Sys_Init(int baud)
{
    int chs[SENSOR_NUM] = {7, 8, 5};

    SCB->CPACR |= (0x3 << (10 * 2)) | (0x3 << (11 * 2));
    Clock_Init();
    Uart2_Init(baud);
    LED_Init();

    /* flame 3채널 초기화 */
    Flame_Init(chs);
}

static void Stepper_GPIO_Init(void)
{
    RCC->AHB1ENR |= (1 << 0);

    GPIOA->MODER &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 8) | (0x3 << 12));
    GPIOA->MODER |= ((0x1 << 0) | (0x1 << 2) | (0x1 << 8) | (0x1 << 12));

    GPIOA->OTYPER &= ~((1 << 0) | (1 << 1) | (1 << 4) | (1 << 6));
    GPIOA->PUPDR &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 8) | (0x3 << 12));

    GPIOA->ODR &= ~((1 << 0) | (1 << 1) | (1 << 4) | (1 << 6));
}

static void Stepper_Output(int a, int b, int c, int d)
{
    if (a)
        GPIOA->ODR |= (1 << 0);
    else
        GPIOA->ODR &= ~(1 << 0);
    if (b)
        GPIOA->ODR |= (1 << 1);
    else
        GPIOA->ODR &= ~(1 << 1);
    if (c)
        GPIOA->ODR |= (1 << 4);
    else
        GPIOA->ODR &= ~(1 << 4);
    if (d)
        GPIOA->ODR |= (1 << 6);
    else
        GPIOA->ODR &= ~(1 << 6);
}

static void Stepper_Release(void)
{
    Stepper_Output(0, 0, 0, 0);
}

static void Stepper_Step(int dir)
{
    step_seq += dir;

    if (step_seq > 7)
        step_seq = 0;
    if (step_seq < 0)
        step_seq = 7;

    Stepper_Output(
        step_table[step_seq][0],
        step_table[step_seq][1],
        step_table[step_seq][2],
        step_table[step_seq][3]);

    TIM2_Delay(STEP_DELAY_MS);
}

static void Stepper_Move_Relative(int steps)
{
    int i;

    if (steps > 0)
    {
        for (i = 0; i < steps; i++)
            Stepper_Step(+1);
    }
    else if (steps < 0)
    {
        for (i = 0; i < -steps; i++)
            Stepper_Step(-1);
    }

    Stepper_Release();
}

static void Servo_Init_PB6(void)
{
    unsigned int psc;

    RCC->AHB1ENR |= (1 << 1);
    RCC->APB1ENR |= (1 << 2);

    GPIOB->MODER &= ~(0x3 << 12);
    GPIOB->MODER |= (0x2 << 12);

    GPIOB->AFR[0] &= ~(0xF << 24);
    GPIOB->AFR[0] |= (0x2 << 24);

    GPIOB->PUPDR &= ~(0x3 << 12);

    psc = (unsigned int)(TIMXCLK / 1000000.0 + 0.5) - 1;

    TIM4->PSC = psc;
    TIM4->ARR = 20000 - 1;
    TIM4->CCR1 = 1500;

    TIM4->CCMR1 &= ~(0xFF << 0);
    TIM4->CCMR1 |= (6 << 4);
    TIM4->CCMR1 |= (1 << 3);

    TIM4->CCER |= (1 << 0);
    TIM4->CR1 |= (1 << 7);
    TIM4->EGR |= (1 << 0);
    TIM4->CR1 |= (1 << 0);
}

static void Servo_Set_Angle(int angle)
{
    unsigned int pulse;

    if (angle < SERVO_MIN_ANGLE)
        angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    servo_angle = angle;

    pulse = 500 + (unsigned int)((2000.0f * angle) / 180.0f);
    TIM4->CCR1 = pulse;
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

    Stepper_GPIO_Init();
    Servo_Init_PB6();
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

        printf("F=[%.4f %.4f %.4f] V=[%.4f %.4f] SUM=%.4f ANG=%d\r",
               flames[0], flames[1], flames[2], v[0], v[1], fire_vector.intensity, servo_angle);

        if (fire_vector.intensity < 200.f)
        {
            if (current++ % 2 == 0)
            {
                Stepper_Move_Relative(-(int)(fire_vector.x * 10.f));
                // TIM2_Delay(400);
            }
            else
            {
                servo_angle -= (int)(fire_vector.y * 5.f);
                servo_angle = (servo_angle < SERVO_MIN_ANGLE) ? SERVO_MIN_ANGLE : ((servo_angle > SERVO_MAX_ANGLE) ? SERVO_MAX_ANGLE : servo_angle);
                Servo_Set_Angle(servo_angle);
                // TIM2_Delay(400);
            }
        }

        TIM2_Delay(10);
    }
}
