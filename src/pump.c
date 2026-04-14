#include "pump.h"
#include "flame.h"
#include <math.h>
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

inline void Pump_On(void) { Macro_Set_Bit(PUMP_PORT->ODR, PUMP_PIN); }
inline void Pump_Off(void) { Macro_Clear_Bit(PUMP_PORT->ODR, PUMP_PIN); }

unsigned char Pump_Is_On(void) { return pump_auto_state; }

void Pump_Control_Update(volatile float step_x, volatile float servo_y, volatile float intensity, const volatile float *flames, int flame_count)
{
    if (flames == 0 || flame_count <= 0)
        return;

    float vx = latest_fire_vector.x;
    float vy = latest_fire_vector.y;
    float distance = sqrtf(vx * vx + vy * vy);

    /* --- pump OFF → 안정 감지 후 ON --- */
    if (pump_auto_state == 0)
    {
        unsigned char stable_now =
            (intensity < PUMP_INTENSITY_LOW) &&
            (distance < PUMP_VECTOR_DEADBAND);

        if (stable_now)
        {
            if (!stable_timer_running)
            {
                SysTick_Run(PUMP_STABLE_DELAY_MS);
                stable_timer_running = 1;
                // printf("[PUMP] Stable -> timer start\n");
            }
            else if (SysTick_Check_Timeout())
            {
                // printf("[PUMP] Stable for %d ms -> ON\n", PUMP_STABLE_DELAY_MS);
                Pump_On();
                pump_auto_state = 1;
                stable_timer_running = 0;
                SysTick_Stop();
                // printf("[PUMP] ON\n");
            }
        }
        else if (stable_timer_running)
        {
            SysTick_Stop();
            stable_timer_running = 0;
            // printf("[PUMP] Stable broken -> reset\n");
        }
    }
    /* --- pump ON → 고강도 감지 후 OFF --- */
    else
    {
        if (intensity > PUMP_INTENSITY_HIGH)
        {
            if (!pump_off_timer_running)
            {
                SysTick_Run(PUMP_OFF_DELAY_MS);
                pump_off_timer_running = 1;
                // printf("[PUMP] High intensity -> off timer\n");
            }
            else if (SysTick_Check_Timeout())
            {
                Pump_Off();
                pump_auto_state = 0;
                stable_timer_running = 0;
                pump_off_timer_running = 0;
                SysTick_Stop();
                // printf("[PUMP] OFF\n");
            }
        }
        else if (pump_off_timer_running)
        {
            SysTick_Stop();
            pump_off_timer_running = 0;
            // printf("[PUMP] Intensity cleared -> timer reset\n");
            // printf("[PUMP] Intensity cleared -> reset\n");
        }
    }
}