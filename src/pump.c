#include "pump.h"
#include <stdio.h>

volatile unsigned char pump_auto_state = 0;
static int stable_time_count = 0;
static int fire_off_count = 0;

void Pump_Init(void)
{
    Macro_Set_Bit(RCC->AHB1ENR, PUMP_RCC_BIT);

    Macro_Write_Block(PUMP_PORT->MODER, 0x3, 0x1, (PUMP_PIN * 2));

    Pump_Off();
    pump_auto_state = 0;
    stable_time_count = 0;
    fire_off_count = 0;
}

inline void Pump_On(void)
{
    Macro_Set_Bit(PUMP_PORT->ODR, PUMP_PIN);
}

inline void Pump_Off(void)
{
    Macro_Clear_Bit(PUMP_PORT->ODR, PUMP_PIN);
}

inline void Pump_Delay(volatile int count)
{
    for (int i = 0; i < count * 3000; i++)
    {
        __asm__("nop");
    }
}

unsigned char Pump_Is_On(void)
{
    return pump_auto_state;
}

void Pump_Control_Update(int step_x, int servo_y, float intensity, const volatile float *flames, int flame_count)
{
    // if (flames == 0 || flame_count <= 0)
    //     return;

    /* -----------------------------
       pump OFF 상태일 때만 ON 조건 검사
       움직임이 작은 상태가 5초 이상 유지되면 ON
       ----------------------------- */
    if (pump_auto_state == 0)
    {
        /* 불이 어느 정도 감지되고,
           stepper/servo 움직임이 매우 작은 상태 */
        if ((intensity < 45.0f) &&
            (step_x >= -0.04f && step_x <= 0.04f) &&
            (servo_y >= -0.04f && servo_y <= 0.04f))
        {
            if (stable_time_count < 200)
                stable_time_count++;
        }
        else
        {
            stable_time_count = 0;
        }

        /* 50ms * 100 = 약 5초 */
        if (stable_time_count >= 100)
        {
            Pump_On();
            pump_auto_state = 1;
            stable_time_count = 0;
            fire_off_count = 0;
            printf("[PUMP] ON\n");
        }
    }
    /* -----------------------------
       pump ON 상태일 때는 OFF 조건만 검사
       flame 값 기준으로만 OFF
       ----------------------------- */
    else
    {
        printf("[PUMP] ON - Intensity=%.4f\n", intensity);
        /* 불이 꺼지면 intensity가 다시 커짐 */
        if (intensity > 30.0f)
        {
            if (fire_off_count < 20)
                fire_off_count++;
        }
        else
        {
            fire_off_count = 0;
        }

        if (fire_off_count >= 8)
        {
            Pump_Off();
            pump_auto_state = 0;
            stable_time_count = 0;
            fire_off_count = 0;
            printf("[PUMP] OFF\n");
        }
    }
}