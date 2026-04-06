#include "stm32f4xx.h"


#define SENSOR_NUM 3
#define SAMPLING_TIME 0x7

void Flame_Init(int *chs);
void get_linearize_sensor_data(float *values);