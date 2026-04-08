#ifndef WATER_SENSOR_H
#define WATER_SENSOR_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define WATER_PORT          GPIOC           // Target Port (e.g., GPIOC)
#define WATER_RCC_BIT       2               // RCC AHB1ENR bit (0:A)
#define WATER_PIN           3               // Target Pin Number (PC3)
#define WATER_ADC_CH        13               // Matching ADC Channel (Channel 13)

/* --- Water Level Thresholds --- */
#define WATER_DETECTED      1000            // Threshold for sufficient water
#define WATER_EMPTY         300             // Threshold for low water

/* --- Function Prototypes --- */
void WaterSensor_Init(void);                // Initialize Water Sensor
int WaterSensor_Read(void);                 // Read Analog Value

#endif