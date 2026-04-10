#include "pump.h"
#include <stdio.h>

volatile unsigned char pump_auto_state = 0;
static unsigned char stable_timer_running = 0;
static unsigned char pump_off_timer_running = 0;

void Pump_Init(void)
{
    Macro_Set_Bit(RCC->AHB1ENR, PUMP_RCC_BIT);
    Macro_Write_Block(PUMP_PORT->MODER, 0x3, 0x1, (PUMP_PIN * 2));

    Pump_Off();
    stable_timer_running = 0;
    pump_off_timer_running = 0;

    SysTick_Stop();
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

void Pump_Control_Update(volatile float step_x, volatile float servo_y, volatile float intensity, const volatile float *flames, int flame_count)
{
    unsigned char stable_now = 0;

    if (flames == 0 || flame_count <= 0)
        return;

    /* -----------------------------
       pump OFF 상태일 때:
       "움직임이 엄청 적어진 시점"부터 정확히 5초 후 ON
       ----------------------------- */
    if (pump_auto_state == 0)
    {
        if ((intensity < 40.0f) &&
            (step_x >= -0.04f && step_x <= 0.04f) &&
            (servo_y >= -0.04f && servo_y <= 0.04f))
        {
            stable_now = 1;
        }

        if (stable_now)
        {
            /* 안정 상태에 처음 진입한 순간 5초 타이머 시작 */
            if (!stable_timer_running)
            {
                SysTick_Run(5000);
                stable_timer_running = 1;
                printf("[PUMP] Stable detected -> 5s timer start\n");
            }
            else
            {
                /* 5초가 끝났으면 pump ON */
                if (SysTick_Check_Timeout())
                {
                    Pump_On();
                    pump_auto_state = 1;
                    stable_timer_running = 0;
                    SysTick_Stop();
                    printf("[PUMP] ON\n");
                }
                
            }
        }
        else
        {
            /* 안정 상태가 깨지면 타이머 취소 */
            if (stable_timer_running)
            {
                SysTick_Stop();
                stable_timer_running = 0;
                printf("[PUMP] Stable broken -> timer reset\n");
            }
        }
    }

    /* -----------------------------
       pump ON 상태일 때:
       flame 값 기준으로만 OFF
       ----------------------------- */
    else
    {
        if (intensity > 30.0f)
        {
            if (!pump_off_timer_running)
            {
                SysTick_Run(2000);
                pump_off_timer_running = 1;
                printf("[PUMP] High intensity detected -> off timer start\n");
            }
            else if (SysTick_Check_Timeout())
            {
                Pump_Off();
                pump_auto_state = 0;
                stable_timer_running = 0;
                pump_off_timer_running = 0;
                SysTick_Stop();
                printf("[PUMP] OFF\n");
            }
        }
        else
        {
            if (pump_off_timer_running)
            {
                SysTick_Stop();
                pump_off_timer_running = 0;
                printf("[PUMP] High intensity cleared -> off timer reset\n");
            }
        }
    }
}