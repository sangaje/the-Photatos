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
	
	volatile int i;
	
	for (;;)
	{
		
		float flames[SENSOR_NUM] = {
			0,
		};
		volatile float v[3] = {
			0.f,
			0.f,
			0.f,
		};
		for (i = 0; i < 4000; i++)
		{
			get_linearize_sensor_data(flames);
			v[0] += flames[0];
			v[1] += flames[1];
			v[2] += flames[2];
		}
		v[0] = v[0] / 4000.f;
		v[1] = v[1] / 4000.f;
		v[2] = v[2] / 4000.f;

		// printf("Flame Sensor Values: CH0 = %.4f, CH1 = %.4f, CH6 = %.4f\n", v[0], v[1], v[2]);
		// printf("CH0 = %.4f, CH1 = %.4f, CH6 = %.4f\n", v[0], v[1], v[2]);
		printf("CH0 = %.4f\n", v[0]);

		for (i = 0; i < 0x400000; i++)
			;
	}
}
