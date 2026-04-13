#include "device_driver.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* ════════════════════════════════════════════════════════
 *  USART2 DMA TX — DMA1_Stream6, Channel 4
 *  (Flame ADC는 DMA2_Stream0 사용 → 충돌 없음)
 * ════════════════════════════════════════════════════════ */

#define TX_BUF_SIZE 2048

static volatile char  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;   /* 쓰기 포인터 (producer) */
static volatile uint16_t tx_tail = 0;   /* DMA 읽기 포인터 (consumer) */
static volatile int dma_tx_busy = 0;

/* DMA1_Stream6 HIFCR 클리어 마스크:
 *   CTCIF6(21) | CHTIF6(20) | CTEIF6(19) | CDMEIF6(18) | CFEIF6(16) */
#define DMA1_S6_CLEAR_FLAGS  ((1u<<21)|(1u<<20)|(1u<<19)|(1u<<18)|(1u<<16))

static void Uart2_DMA_Kick(void)
{
  uint16_t h = tx_head;
  uint16_t t = tx_tail;

  if (h == t) { dma_tx_busy = 0; return; }
  dma_tx_busy = 1;

  /* 연속 전송 가능 길이 (wrap 경계까지) */
  uint16_t len = (h > t) ? (h - t) : (TX_BUF_SIZE - t);

  DMA1_Stream6->CR &= ~(uint32_t)1;       /* Stream 비활성화 */
  while (DMA1_Stream6->CR & 1);
  DMA1->HIFCR = DMA1_S6_CLEAR_FLAGS;      /* 이전 플래그 클리어 */

  DMA1_Stream6->M0AR = (uint32_t)&tx_buf[t];
  DMA1_Stream6->NDTR = len;
  DMA1_Stream6->CR |= 1;                  /* Enable */

  tx_tail = (t + len) % TX_BUF_SIZE;
}

static void Uart2_DMA_HW_Init(void)
{
  /* DMA1 Clock ON (AHB1ENR bit 21) */
  Macro_Set_Bit(RCC->AHB1ENR, 21);

  DMA1_Stream6->CR &= ~(uint32_t)1;
  while (DMA1_Stream6->CR & 1);

  DMA1_Stream6->PAR  = (uint32_t)&(USART2->DR);
  DMA1_Stream6->M0AR = 0;
  DMA1_Stream6->NDTR = 0;

  /* CR: Channel 4 [27:25]=100, MINC [10]=1, DIR=Mem→Periph [7:6]=01, TCIE [4]=1 */
  DMA1_Stream6->CR = (4u << 25) | (1u << 10) | (1u << 6) | (1u << 4);

  /* USART2 DMAT enable (CR3 bit 7) */
  Macro_Set_Bit(USART2->CR3, 7);

  /* NVIC: 센서 TIM3(prio 0), 모터 TIM5(prio 2) 보다 낮게 */
  NVIC_SetPriority(DMA1_Stream6_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

/* DMA1_Stream6 Transfer-Complete ISR */
void DMA1_Stream6_IRQHandler(void)
{
  if (DMA1->HISR & (1u << 21))       /* TCIF6 */
  {
    DMA1->HIFCR = (1u << 21);
    Uart2_DMA_Kick();
  }
}

/* printf → _write → Uart2_DMA_Write: 링 버퍼에 넣고 DMA kick */
void Uart2_DMA_Write(const char *data, int len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (int i = 0; i < len; i++)
  {
    /* \n → \r\n 변환 */
    if (data[i] == '\n')
    {
      uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
      if (next == tx_tail) break;       /* 버퍼 풀 → drop */
      tx_buf[tx_head] = '\r';
      tx_head = next;
    }
    uint16_t next = (tx_head + 1) % TX_BUF_SIZE;
    if (next == tx_tail) break;
    tx_buf[tx_head] = data[i];
    tx_head = next;
  }

  if (!dma_tx_busy)
    Uart2_DMA_Kick();

  __set_PRIMASK(primask);
}

void Uart2_Init(int baud)
{
  double div;
  unsigned int mant;
  unsigned int frac;

  Macro_Set_Bit(RCC->AHB1ENR, 0);                   // PA2,3
  Macro_Set_Bit(RCC->APB1ENR, 17);                   // USART2 ON
  Macro_Write_Block(GPIOA->MODER, 0xf, 0xa, 4);     // PA2,3 => ALT
  Macro_Write_Block(GPIOA->AFR[0], 0xff, 0x77, 8);  // PA2,3 => AF07
  Macro_Write_Block(GPIOA->PUPDR, 0xf, 0x5, 4);     // PA2,3 => Pull-Up  

  volatile unsigned int t = GPIOA->LCKR & 0x7FFF;
  GPIOA->LCKR = (0x1<<16)|t|(0x3<<2);                // Lock PA2, 3 Configuration
  GPIOA->LCKR = (0x0<<16)|t|(0x3<<2);
  GPIOA->LCKR = (0x1<<16)|t|(0x3<<2);
  t = GPIOA->LCKR;

  div = PCLK1/(16. * baud);
  mant = (int)div;
  frac = (int)((div - mant) * 16. + 0.5);
  mant += frac >> 4;
  frac &= 0xf;

  USART2->BRR = (mant<<4)|(frac<<0);
  USART2->CR1 = (1<<13)|(0<<12)|(0<<10)|(1<<3)|(1<<2);
  USART2->CR2 = 0<<12;
  USART2->CR3 = 0;

  /* DMA TX 초기화 (반드시 USART2 CR 설정 뒤에) */
  Uart2_DMA_HW_Init();
}

/* 블로킹 전송 — HardFault 등 예외 핸들러 전용 (DMA 못 쓸 때) */
void Uart2_Send_Byte(char data)
{
  if(data == '\n')
  {
    while(!Macro_Check_Bit_Set(USART2->SR, 7));
    USART2->DR = 0x0d;
  }

  while(!Macro_Check_Bit_Set(USART2->SR, 7));
  USART2->DR = data;
}

void Uart2_RX_Interrupt_Enable(int en)
{
  if(en)
  {
    Macro_Set_Bit(USART2->CR1, 5);
    NVIC_ClearPendingIRQ(38);
    NVIC_EnableIRQ(38);
  }
  else
  {
    Macro_Clear_Bit(USART2->CR1, 5);
    NVIC_DisableIRQ(38);
  }
}

void Uart1_Init(int baud)
{
  double div;
  unsigned int mant;
  unsigned int frac;

  Macro_Set_Bit(RCC->AHB1ENR, 0);                   // PA9,10
  Macro_Set_Bit(RCC->APB2ENR, 4);                   // USART1 ON
  Macro_Write_Block(GPIOA->MODER, 0xf, 0xa, 18);    // PA9,10 => ALT
  Macro_Write_Block(GPIOA->AFR[1], 0xff, 0x77, 4);  // PA9,10 => AF07
  Macro_Write_Block(GPIOA->PUPDR, 0xf, 0x5, 18);    // PA9,10 => Pull-Up
  
  volatile unsigned int t = GPIOA->LCKR & 0x7FFF;
  GPIOA->LCKR = (0x1<<16)|t|(0x3<<9);               // Lock PA9, 10 Configuration
  GPIOA->LCKR = (0x0<<16)|t|(0x3<<9);
  GPIOA->LCKR = (0x1<<16)|t|(0x3<<9);
  t = GPIOA->LCKR;

  div = PCLK2 / (16. * baud);
  mant = (int)div;
  frac = (int)((div - mant) * 16 + 0.5);
  mant += frac >> 4;
  frac &= 0xf;
  USART1->BRR = (mant<<4)|(frac<<0);

  USART1->CR1 = (1<<13)|(0<<12)|(0<<10)|(1<<3)|(1<<2);
  USART1->CR2 = 0 << 12;
  USART1->CR3 = 0;
}

void Uart1_Send_Byte(char data)
{
  if(data == '\n')
  {
    while(!Macro_Check_Bit_Set(USART1->SR, 7));
    USART1->DR = 0x0d;
  }

  while(!Macro_Check_Bit_Set(USART1->SR, 7));
  USART1->DR = data;
}

void Uart1_Send_String(char *pt)
{
  while(*pt != 0)
  {
    Uart1_Send_Byte(*pt++);
  }
}

void Uart1_Printf(char *fmt,...)
{
	va_list ap;
	char string[256];

	va_start(ap,fmt);
	vsprintf(string,fmt,ap);
	Uart1_Send_String(string);
	va_end(ap);
}

char Uart1_Get_Pressed(void)
{
	if(Macro_Check_Bit_Set(USART1->SR, 5))
	{
		return (char)USART1->DR;
	}

	else
	{
		return (char)0;
	}
}

char Uart1_Get_Char(void)
{
	while(!Macro_Check_Bit_Set(USART1->SR, 5));
	return (char)USART1->DR;
}
