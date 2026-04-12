#ifndef PUMP_H
#define PUMP_H

#include "device_driver.h"

/* --- Hardware Configuration --- */
#define PUMP_PORT           GPIOC
#define PUMP_RCC_BIT        2
#define PUMP_PIN            4

/* --- Pump Control Tuning --- */
#define PUMP_INTENSITY_LOW      100.0f  // intensity 이하일 때 안정 판정
#define PUMP_VECTOR_DEADBAND    0.06f   // V.x, V.y 안정 판정 범위
#define PUMP_STABLE_DELAY_MS    1000    // 안정 후 펌프 ON까지 대기 (ms)
#define PUMP_INTENSITY_HIGH     60.0f   // intensity 이상이면 OFF 판정
#define PUMP_OFF_DELAY_MS       500     // 고강도 지속 후 OFF까지 대기 (ms)

/* --- Function Prototypes --- */
void Pump_Init(void);
void Pump_On(void);
void Pump_Off(void);
void Pump_Control_Update(volatile float step_x, volatile float servo_y, volatile float intensity, const volatile float *flames, int flame_count);
unsigned char Pump_Is_On(void);

#endif