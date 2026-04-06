#include "device_driver.h"
#include "flame.h"
#include <math.h>
#include <stdio.h>

static void Sys_Init(int baud)
{
	SCB->CPACR |= (0x3 << 10 * 2) | (0x3 << 11 * 2);
	Clock_Init();
	Uart2_Init(baud);
	setvbuf(stdout, NULL, _IONBF, 0);
	LED_Init();
	int chs[SENSOR_NUM] = {0, 1, 6};
	Flame_Init(chs);
}

void Main(void)
{
	Sys_Init(115200);
	printf("ADC Test\n\n");

	for (;;)
	{

		float flames[SENSOR_NUM] = {
			0,
		};
		volatile float v[2] = {
			0,
		};
		get_linearize_sensor_data(flames);

		v[0] = -1.f * flames[0] + (flames[1] + flames[2]) / 2.f;
		v[1] = -1.f * sqrtf(3) * (flames[2] - flames[1]) * 0.5f;

		// printf("Flame Sensor Values: CH0 = %.4f, CH1 = %.4f, CH6 = %.4f\n", v[0], v[1], v[2]);
		// printf("CH0 = %.4f, CH1 = %.4f, CH6 = %.4f\n", v[0], v[1], v[2]);
		// printf("CH0 = %.4f, CH1 = %.4f, CH6 = %.4f\r", flames[0], flames[1], flames[2]);
		printf("CH1 = %.4f, CH0 = %.4f, CH6 = %.4f [%.4f, %.4f]\r", flames[0], flames[1], flames[2], v[0], v[1]);

		for (i = 0; i < 0x70000; i++)
			;
	}
}
