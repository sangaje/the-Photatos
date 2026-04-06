#include "device_driver.h"

// volatile uint16_t adc_results[3] = {0, };

// void ADC1_Multi_Init(void)
// {
//     // 1. GPIO & Clock 설정 (PA0, PA1, PA6)
//     Macro_Set_Bit(RCC->AHB1ENR, 0);          // GPIOA Clock ON
//     Macro_Write_Block(GPIOA->MODER, 0x3, 0x3, 0);   // PA0 Analog
//     Macro_Write_Block(GPIOA->MODER, 0x3, 0x3, 2);   // PA1 Analog
//     Macro_Write_Block(GPIOA->MODER, 0x3, 0x3, 12);  // PA6 Analog

//     // 2. ADC 기본 설정
//     Macro_Set_Bit(RCC->APB2ENR, 8);          // ADC1 Clock ON
//     Macro_Write_Block(ADC->CCR, 0x3, 0x2, 16); // ADC Clock = PCLK2/6
    
//     // ADC 제어 비트들 (CONT, DMA, DDS)
//     Macro_Set_Bit(ADC1->CR2, 1);  // CONT: 연속 변환 모드
//     Macro_Set_Bit(ADC1->CR2, 8);  // DMA: DMA 모드 활성화
//     Macro_Set_Bit(ADC1->CR2, 9);  // DDS: DMA 계속 요청 (Circular 대응)

//     // 3. 샘플링 타임 & 시퀀스
//     Macro_Write_Block(ADC1->SMPR2, 0x7, 0x7, 0);   // CH0 480 Cycles
//     Macro_Write_Block(ADC1->SMPR2, 0x7, 0x7, 3);   // CH1 480 Cycles
//     Macro_Write_Block(ADC1->SMPR2, 0x7, 0x7, 18);  // CH6 480 Cycles

//     Macro_Write_Block(ADC1->SQR1, 0xF, 2, 20);     // 3개 채널 (L=2)
//     Macro_Write_Block(ADC1->SQR3, 0x1F, 0, 0);     // 1st: CH0
//     Macro_Write_Block(ADC1->SQR3, 0x1F, 1, 5);     // 2nd: CH1
//     Macro_Write_Block(ADC1->SQR3, 0x1F, 6, 10);    // 3rd: CH6

//     Macro_Set_Bit(ADC1->CR1, 8);  // SCAN Mode ON

//     // 4. DMA2_Stream0 설정 (여기가 핵심!)
//     Macro_Set_Bit(RCC->AHB1ENR, 22); // DMA2 Clock ON

//     // 설정 전 Stream 비활성화 (안전장치)
//     DMA2_Stream0->CR &= ~(1 << 0); 
//     while(DMA2_Stream0->CR & (1 << 0)); // 꺼질 때까지 대기

//     DMA2_Stream0->PAR  = (uint32_t)&(ADC1->DR);
//     DMA2_Stream0->M0AR = (uint32_t)adc_results;
//     DMA2_Stream0->NDTR = 3;

//     // CR 설정 + 맨 마지막에 (1 << 0)을 더해서 EN(Enable) 시킴
//     DMA2_Stream0->CR = (0 << 25) | (2 << 16) | (1 << 13) | (1 << 11) | 
//                        (1 << 10) | (1 << 8)  | (1 << 0); // 맨 끝에 1(EN) 추가!

//     // 5. 최종 가동
//     Macro_Set_Bit(ADC1->CR2, 0);  // ADC ON (ADON)
//     Macro_Set_Bit(ADC1->CR2, 30); // SWSTART: 변환 시작!
// }

// // void ADC1_IN6_Init(void)
// // {
// // 	Macro_Set_Bit(RCC->AHB1ENR, 0); 				// PA POWER ON
// // 	Macro_Write_Block(GPIOA->MODER, 0x3, 0x3, 12);	// PA6(ADC-IN6) = Analog Mode

// // 	Macro_Set_Bit(RCC->APB2ENR, 8); 				// ADC1 POWER ON
// // 	Macro_Write_Block(ADC1->SMPR2, 0x7, 0x7, 18); 	// Clock Configuration of CH6 = 480 Cycles
// // 	Macro_Write_Block(ADC1->SQR1, 0xF, 0x0, 20); 	// Conversion Sequence No = 1
// // 	Macro_Write_Block(ADC1->SQR3, 0x1F, 6, 0); 		// Sequence Channel of No 1 = CH6

// // 	Macro_Write_Block(ADC->CCR, 0x3, 0x2, 16); 		// ADC CLOCK = 16MHz(PCLK2/6)
// // 	Macro_Set_Bit(ADC1->CR2, 0); 					// ADC ON
// // }

// void ADC1_Start(void)
// {
// 	Macro_Set_Bit(ADC1->CR2, 30); // ADC SW Start
// }

// void ADC1_Stop(void)
// {
// 	Macro_Clear_Bit(ADC1->CR2, 30); // ADC Stop
// 	Macro_Clear_Bit(ADC1->CR2, 0);	// ADC OFF
// }

// int ADC1_Get_Status(void)
// {
// 	int r = Macro_Check_Bit_Set(ADC1->SR, 1);

// 	if (r)
// 	{
// 		Macro_Clear_Bit(ADC1->SR, 1);
// 		Macro_Clear_Bit(ADC1->SR, 4);
// 	}

// 	return r;
// }

// int ADC1_Get_Data(void)
// {
// 	return Macro_Extract_Area(ADC1->DR, 0xFFF, 0);
// }
