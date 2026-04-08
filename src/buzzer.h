#ifndef BUZZER_H
#define BUZZER_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define BUZZER_PORT         GPIOB           // Target Port (e.g., GPIOA, GPIOB)
#define BUZZER_RCC_BIT      1               // RCC AHB1ENR bit (0:A, 1:B)
#define BUZZER_PIN          4               // Target Pin Number (PB4)

/* --- Function Prototypes --- */
void Buzzer_Init(void);                     // Initialize Buzzer
void Buzzer_On(void);                       // Turn Buzzer ON
void Buzzer_Off(void);                      // Turn Buzzer OFF

#endif