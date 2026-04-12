#include "stm32f4xx.h"

#define SENSOR_NUM 4
#define SAMPLING_TIME 0x7
#define FILTER_COEFFICIENT 0.1f
#define FRAMES_BASIS_BOUNDARY 45.f
#define NUMBER_OF_SAMPLES 100

#define TimerNumber 3

typedef struct
{
    volatile float x;
    volatile float y;
    volatile float intensity;
} FireVector_t;

void Flame_Init(int *chs);
void _get_linearize_sensor_data(volatile float *values);
FireVector_t fire_vector_estimation();