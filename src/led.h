#ifndef LED_H
#define LED_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define LED_PORT            GPIOC           // Target Port (e.g., GPIOA, GPIOB)
#define LED_RCC_BIT         2               // RCC AHB1ENR bit (0:A, 1:B)

#define LED_GREEN_PIN       0               // Green LED Pin Number (PC0)
#define LED_RED_PIN         1               // Red LED Pin Number (PC1)

/* --- Function Prototypes --- */
void LED_Init(void);                        // Initialize LEDs
void LED_Green_On(void);                    // Turn Green LED ON
void LED_Green_Off(void);
void LED_Red_On(void);                      // Turn Red LED ON
void LED_Red_Off(void);
void LED_All_Off(void);                     // Turn All LEDs OFF

#endif