#include "device_driver.h"
#include "servo.h"

static int servo_angle = SERVO_INIT_ANGLE;

void Servo_Init(void)
{
    unsigned int psc;

    RCC->AHB1ENR |= (1 << 1);
    RCC->APB1ENR |= (1 << 2);

    GPIOB->MODER &= ~(0x3 << 12);
    GPIOB->MODER |=  (0x2 << 12);

    GPIOB->AFR[0] &= ~(0xF << 24);
    GPIOB->AFR[0] |=  (0x2 << 24);

    GPIOB->PUPDR &= ~(0x3 << 12);

    psc = (unsigned int)(TIMXCLK / 1000000.0 + 0.5) - 1;

    TIM4->PSC = psc;
    TIM4->ARR = 20000 - 1;
    TIM4->CCR1 = 1500;

    TIM4->CCMR1 &= ~(0xFF << 0);
    TIM4->CCMR1 |=  (6 << 4);
    TIM4->CCMR1 |=  (1 << 3);

    TIM4->CCER |= (1 << 0);
    TIM4->CR1  |= (1 << 7);
    TIM4->EGR  |= (1 << 0);
    TIM4->CR1  |= (1 << 0);

    Servo_Set_Angle(SERVO_INIT_ANGLE);
}

void Servo_Set_Angle(int angle)
{
    unsigned int pulse;

    if(angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if(angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    servo_angle = angle;

    pulse = 500 + (unsigned int)((2000.0f * angle) / 180.0f);
    TIM4->CCR1 = pulse;
}

int Servo_Get_Angle(void)
{
    return servo_angle;
}