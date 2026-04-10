#include "device_driver.h"
#include "stepper.h"

#define STEP_DELAY_MS 2
extern volatile unsigned char pump_auto_state;
static const unsigned char step_table[8][4] = {
    {1,0,0,0},
    {1,0,0,1},
    {0,0,0,1},
    {0,0,1,1},
    {0,0,1,0},
    {0,1,1,0},
    {0,1,0,0},
    {1,1,0,0}
};

static int step_seq = 0;

static void Stepper_Output(int a, int b, int c, int d)
{
    if(a) GPIOA->ODR |=  (1 << 0); else GPIOA->ODR &= ~(1 << 0);
    if(b) GPIOA->ODR |=  (1 << 1); else GPIOA->ODR &= ~(1 << 1);
    if(c) GPIOA->ODR |=  (1 << 4); else GPIOA->ODR &= ~(1 << 4);
    if(d) GPIOA->ODR |=  (1 << 6); else GPIOA->ODR &= ~(1 << 6);
}

void Stepper_Init(void)
{
    RCC->AHB1ENR |= (1 << 0);

    GPIOA->MODER &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 8) | (0x3 << 12));
    GPIOA->MODER |=  ((0x1 << 0) | (0x1 << 2) | (0x1 << 8) | (0x1 << 12));

    GPIOA->OTYPER &= ~((1 << 0) | (1 << 1) | (1 << 4) | (1 << 6));
    GPIOA->PUPDR  &= ~((0x3 << 0) | (0x3 << 2) | (0x3 << 8) | (0x3 << 12));

    GPIOA->ODR &= ~((1 << 0) | (1 << 1) | (1 << 4) | (1 << 6));
}

void Stepper_Release(void)
{
    Stepper_Output(0,0,0,0);
}

static void Stepper_Step(int dir)
{
    step_seq += dir;

    if(step_seq > 7) step_seq = 0;
    if(step_seq < 0) step_seq = 7;

    Stepper_Output(
        step_table[step_seq][0],
        step_table[step_seq][1],
        step_table[step_seq][2],
        step_table[step_seq][3]
    );

    TIM2_Delay(STEP_DELAY_MS);
}

void Stepper_Move_Relative(int steps)
{
    int i;
    if (pump_auto_state)
    {
        steps /= 10; // 펌프가 켜져 있을 때는 스텝 속도를 1/10로 줄임
    }

    if(steps > 0)
    {
        for(i = 0; i < steps; i++) Stepper_Step(+1);
    }
    else if(steps < 0)
    {
        for(i = 0; i < -steps; i++) Stepper_Step(-1);
    }

    Stepper_Release();
}