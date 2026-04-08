#ifndef WATER_SENSOR_H
#define WATER_SENSOR_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define WATER_PORT          GPIOA           // Target Port (e.g., GPIOA)
#define WATER_RCC_BIT       0               // RCC AHB1ENR bit (0:A)
#define WATER_PIN           6               // Target Pin Number (PA6)
#define WATER_ADC_CH        6               // Matching ADC Channel (Channel 6)

/* --- Water Level Thresholds --- */
#define WATER_DETECTED      1000            // Threshold for sufficient water
#define WATER_EMPTY         300             // Threshold for low water

/* --- Function Prototypes --- */
void WaterSensor_Init(void);                // Initialize Water Sensor
int WaterSensor_Read(void);                 // Read Analog Value

#endif