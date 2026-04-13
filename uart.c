#include "device_driver.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

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
}

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

char Uart2_Get_Char(void)
{
	while(!Macro_Check_Bit_Set(USART2->SR, 5));
	return USART2->DR;
}

char Uart2_Get_Pressed(void)
{
	// 글자가 입력이 되었으면 입력된 글자를 리턴
	// 글자 입력이 없으면 0 리턴
  unsigned char a;
  if(Macro_Check_Bit_Set(USART2->SR, 5))
  {
    a = (char)USART2->DR;
    return a;
  }
  else
  {
    a = (char)0;
    return a;
  }

}

// 블루투스 (HC-06) 초기화: PA9(TX), PA10(RX), 9600bps, RX 인터럽트 켜기
void Bluetooth_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, 0);  // GPIOA 켜기
    Macro_Set_Bit(RCC->APB2ENR, 4);  //  USART1 켜기 (APB2 버스, 96MHz)

    // 1. PA9, PA10 핀을 Alternate Function(AF) 모드로 변경
    Macro_Write_Block(GPIOA->MODER, 0xF, 0xA, 18); 
    
    // 2. PA9, PA10을 AF07(USART1)로 연결
    Macro_Write_Block(GPIOA->AFR[1], 0xFF, 0x77, 4);

    // 3. 통신 속도 설정 (96MHz / 9600bps = 10000)
    USART1->BRR = 10000;

    // 4.  수신(RX) 인터럽트 활성화 (데이터가 들어오면 CPU 찔러라!)
    USART1->CR1 |= (1 << 5); // RXNEIE 비트 켜기

    // 5. 송신(TE), 수신(RE), USART 활성화(UE)
    // USART1 켜기 + RX 인터럽트(수신 알람) 켜기
    USART1->CR1 |= (1 << 3) | (1 << 2) | (1 << 13);

    // 6. NVIC (인터럽트 컨트롤러)에서 USART1 인터럽트 스위치 켜기
    NVIC_EnableIRQ(USART1_IRQn); 
}

// 초고속 인터럽트 함수 
// 앱에서 블록을 클릭할 때마다 하드웨어가 1마이크로초 만에 이 함수를 실행하고 사라짐
void USART1_IRQHandler(void)
{
    if (USART1->SR & (1 << 5)) // 우체통에 데이터가 들어왔다면
    {
        // 아까처럼 문자를 모을 필요 없이, 그냥 값을 바로 변수에 집어넣으면 끝
        bt_rx_data = USART1->DR & 0xFF; 
        bt_rx_flag = 1; // 메인 루프에 택배 왔음 깃발 올리기
    }
}