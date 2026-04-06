#include "stm32f4xx.h"


#define SENSOR_NUM 4
#define SAMPLING_TIME 0x7
#define FILTER_COEFFICIENT 0.1f
#define FRAMES_BASIS_BOUNDARY 165.f
#define NUMBER_OF_SAMPLES 100

void Flame_Init(int *chs);
void get_linearize_sensor_data(float *values);