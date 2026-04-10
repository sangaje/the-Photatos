#ifndef BUZZER_H
#define BUZZER_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define BUZZER_PORT         GPIOC           // Target Port (e.g., GPIOA, GPIOB)
#define BUZZER_RCC_BIT      2               // RCC AHB1ENR bit (0:A, 1:B)
#define BUZZER_PIN          2               // Target Pin Number (PC2)

/* --- Function Prototypes --- */
void Buzzer_Init(void);                     // Initialize Buzzer
void Buzzer_On(void);                       // Turn Buzzer ON
void Buzzer_Off(void);                      // Turn Buzzer OFF

#endif