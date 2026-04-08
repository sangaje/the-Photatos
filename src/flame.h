#include "stm32f4xx.h"

#define SENSOR_NUM 4
#define SAMPLING_TIME 0x7
#define FILTER_COEFFICIENT 0.5f
#define FRAMES_BASIS_BOUNDARY 240.f
#define NUMBER_OF_SAMPLES 1

typedef struct
{
    volatile float x;
    volatile float y;
    volatile float intensity;
} FireVector_t;

void Flame_Init(int *chs);
void get_linearize_sensor_data(volatile float *values);
FireVector_t fire_vector_estimation(volatile float *values);