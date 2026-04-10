#ifndef PUMP_H
#define PUMP_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define PUMP_PORT           GPIOC           // Target Port (e.g., GPIOA, GPIOB)
#define PUMP_RCC_BIT        2               // RCC AHB1ENR bit (0:A, 1:B)
#define PUMP_PIN            4               // Target Pin Number (PC4)

/* --- Function Prototypes --- */
void Pump_Init(void);                       // Initialize Pump
void Pump_On(void);                         // Turn Pump ON
void Pump_Off(void);                        // Turn Pump OFF
void Pump_Delay(volatile int count);
void Pump_Control_Update(volatile float step_x, volatile float servo_y, volatile float intensity, const volatile float *flames, int flame_count);
unsigned char Pump_Is_On(void);        // Software Delay

#endif