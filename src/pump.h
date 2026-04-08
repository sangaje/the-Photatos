#ifndef PUMP_H
#define PUMP_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define PUMP_PORT           GPIOA           // Target Port (e.g., GPIOA, GPIOB)
#define PUMP_RCC_BIT        0               // RCC AHB1ENR bit (0:A, 1:B)
#define PUMP_PIN            0               // Target Pin Number (PA0)

/* --- Function Prototypes --- */
void Pump_Init(void);                       // Initialize Pump
void Pump_On(void);                         // Turn Pump ON
void Pump_Off(void);                        // Turn Pump OFF
void Pump_Delay(volatile int count);        // Software Delay

#endif