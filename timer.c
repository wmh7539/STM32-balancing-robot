#include "device_driver.h"

#define TIM2_TICK         	(20) 				// usec
#define TIM2_FREQ 	  		(1000000/TIM2_TICK)	// Hz
#define TIME2_PLS_OF_1ms  	(1000/TIM2_TICK)
#define TIM2_MAX	  		(0xffffu)

#define TIM4_TICK	  		(20) 				// usec
#define TIM4_FREQ 	  		(1000000/TIM4_TICK) // Hz
#define TIME4_PLS_OF_1ms  	(1000/TIM4_TICK)
#define TIM4_MAX	  		(0xffffu)

void TIM2_Stopwatch_Start(void)
{
	Macro_Set_Bit(RCC->APB1ENR, 0);

	TIM2->CR1 = (1<<4)|(1<<3);
	TIM2->PSC = (unsigned int)(TIMXCLK/50000.0 + 0.5)-1;
	TIM2->ARR = TIM2_MAX;

	Macro_Set_Bit(TIM2->EGR,0);
	Macro_Set_Bit(TIM2->CR1, 0);
}

unsigned int TIM2_Stopwatch_Stop(void)
{
	unsigned int time;

	Macro_Clear_Bit(TIM2->CR1, 0);
	time = (TIM2_MAX - TIM2->CNT) * TIM2_TICK;
	return time;
}

/* Delay Time Max = 65536 * 20use = 1.3sec */

#if 0

void TIM2_Delay(int time)
{
	Macro_Set_Bit(RCC->APB1ENR, 0);

	TIM2->CR1 = (1<<4)|(1<<3);
	TIM2->PSC = (unsigned int)(TIMXCLK/(double)TIM2_FREQ + 0.5)-1;
	TIM2->ARR = TIME2_PLS_OF_1ms * time;

	Macro_Set_Bit(TIM2->EGR,0);
	Macro_Clear_Bit(TIM2->SR, 0);
	Macro_Set_Bit(TIM2->CR1, 0);

	while(Macro_Check_Bit_Clear(TIM2->SR, 0));

	Macro_Clear_Bit(TIM2->CR1, 0);
}

#else

/* Delay Time Extended */

void TIM2_Delay(int time)
{
	int i;
	unsigned int t = TIME2_PLS_OF_1ms * time;

	Macro_Set_Bit(RCC->APB1ENR, 0);

	TIM2->PSC = (unsigned int)(TIMXCLK/(double)TIM2_FREQ + 0.5)-1;
	TIM2->CR1 = (1<<4)|(1<<3);
	TIM2->ARR = 0xffff;
	Macro_Set_Bit(TIM2->EGR,0);

	for(i=0; i<(t/0xffffu); i++)
	{
		Macro_Set_Bit(TIM2->EGR,0);
		Macro_Clear_Bit(TIM2->SR, 0);
		Macro_Set_Bit(TIM2->CR1, 0);
		while(Macro_Check_Bit_Clear(TIM2->SR, 0));
	}

	TIM2->ARR = t % 0xffffu;
	Macro_Set_Bit(TIM2->EGR,0);
	Macro_Clear_Bit(TIM2->SR, 0);
	Macro_Set_Bit(TIM2->CR1, 0);
	while (Macro_Check_Bit_Clear(TIM2->SR, 0));

	Macro_Clear_Bit(TIM2->CR1, 0);
}

#endif

void TIM4_Repeat(int time)
{
	Macro_Set_Bit(RCC->APB1ENR, 2);

	TIM4->CR1 = (1<<4)|(0<<3);
	TIM4->PSC = (unsigned int)(TIMXCLK/(double)TIM4_FREQ + 0.5)-1;
	TIM4->ARR = TIME4_PLS_OF_1ms * time - 1;

	Macro_Set_Bit(TIM4->EGR,0);
	Macro_Clear_Bit(TIM4->SR, 0);
	Macro_Set_Bit(TIM4->CR1, 0);
}

int TIM4_Check_Timeout(void)
{
	if(Macro_Check_Bit_Set(TIM4->SR, 0))
	{
		Macro_Clear_Bit(TIM4->SR, 0);
		return 1;
	}
	else
	{
		return 0;
	}
}

void TIM4_Stop(void)
{
	Macro_Clear_Bit(TIM4->CR1, 0);
}

void TIM4_Change_Value(int time)
{
	TIM4->ARR = TIME4_PLS_OF_1ms * time;
}

void TIM4_Repeat_Interrupt_Enable(int en, int time)
{
	if(en)
	{
		Macro_Set_Bit(RCC->APB1ENR, 2);

		TIM4->CR1 = (1<<4)|(0<<3);
		TIM4->PSC = (unsigned int)(TIMXCLK/(double)TIM4_FREQ + 0.5)-1;
		TIM4->ARR = TIME4_PLS_OF_1ms * time;
		Macro_Set_Bit(TIM4->EGR,0);

		Macro_Clear_Bit(TIM4->SR, 0);
		NVIC_ClearPendingIRQ(30);

		Macro_Set_Bit(TIM4->DIER, 0);
		NVIC_EnableIRQ(30);

		Macro_Set_Bit(TIM4->CR1, 0);
	}

	else
	{
		NVIC_DisableIRQ(30);
		Macro_Clear_Bit(TIM4->CR1, 0);
		Macro_Clear_Bit(TIM4->DIER, 0);
	}
}

#define TIM3_FREQ 	  			(8000000) 	      	// Hz
#define TIM3_TICK	  			(1000000/TIM3_FREQ)	// usec
#define TIME3_PLS_OF_1ms  		(1000/TIM3_TICK)

void TIM3_Out_Init(void)
{
	Macro_Set_Bit(RCC->AHB1ENR, 1);
	Macro_Set_Bit(RCC->APB1ENR, 1);

	Macro_Write_Block(GPIOB->MODER, 0x3, 0x2, 0);  	// PB0 => ALT
	Macro_Write_Block(GPIOB->AFR[0], 0xf, 0x2, 0); 	// PB0 => AF02

	Macro_Write_Block(TIM3->CCMR2,0xff, 0x60, 0);
	TIM3->CCER = (0<<9)|(1<<8);
}

void TIM3_Out_Freq_Generation(unsigned short freq)
{
	TIM3->PSC = (unsigned int)(TIMXCLK/(double)TIM3_FREQ + 0.5)-1;
	TIM3->ARR = (double)TIM3_FREQ/freq-1;
	TIM3->CCR3 = TIM3->ARR/2;

	Macro_Set_Bit(TIM3->EGR,0);
	TIM3->CR1 = (1<<4)|(0<<3)|(0<<1)|(1<<0);
}

void TIM3_Out_Stop(void)
{
	Macro_Clear_Bit(TIM3->CR1, 0);
}


void TIM5_Out_Init(void)
{
	Macro_Set_Bit(RCC->AHB1ENR, 0);  // GPIOA 클럭 활성화
	Macro_Set_Bit(RCC->APB1ENR, 3);  //TIM5 클럭 활성화

	Macro_Write_Block(GPIOA->MODER, 0x3, 0x2, 0);  	// PA0 => ALT
	Macro_Write_Block(GPIOA->AFR[0], 0xf, 0x2, 0); 	// PA0 => AF02 // PBA 일반 GPIO가 아닌 타이머 출력으로

	Macro_Write_Block(TIM5->CCMR1,0xff, 0x60, 0);  //타이머 채널1의 PWM 모드 1 선택
	TIM5->CCER |= (0<<1)|(1<<0);  // 채널1 출력 활성화, 출력을 Active High로 

	Macro_Write_Block(GPIOA->MODER, 0x3, 0x2, 2);  	// PA1 => ALT
	Macro_Write_Block(GPIOA->AFR[0], 0xf, 0x2, 4); 	// PA1 => AF02 // PBA 일반 GPIO가 아닌 타이머 출력으로

	Macro_Write_Block(TIM5->CCMR1,0xff, 0x60, 8);  //타이머 채널2의 PWM 모드 1 선택
	TIM5->CCER |= (0<<5)|(1<<4);  // 채널2 출력 활성화, 출력을 Active High로 

}


void TIM5_Out_Freq_Generation(void)
{
	// Timer 주파수가 TIM3_FREQ가 되도록 PSC 설정
	TIM5->PSC = 96-1;  //1MHz 가 되도록 PSC 설정


	// 요청한 주파수가 되도록 ARR 설정
	TIM5->ARR = 200 - 1;  // 주기 5Hz 완성
	// Duty Rate 90%가 되도록 CCR1, 2 설정
	TIM5->CCR1 = 180; //90퍼센트
	TIM5->CCR2 = 180; //90퍼센트

	// Manual Update(UG 발생)
	TIM5->EGR |= (1<<0);
	// up Counter, Repeat Mode, Timer Start
	TIM5->CR1 |= (1<<0);

}

void TIM5_duty(unsigned short duty)  //100
{
	if (duty > 100) duty = 100;
	TIM5->CCR1 = duty*2; // 속도 조절
	TIM5->CCR2 = duty*2; // 속도 조절
}

void TIM5_CH1_duty(unsigned short duty)  //100
{
	if (duty > 100) duty = 100;
	TIM5->CCR1 = duty*2; // 속도 조절
}

void TIM5_CH2_duty(unsigned short duty)  //100
{
	if (duty > 100) duty = 100;
	TIM5->CCR2 = duty*2; // 속도 조절
}


// PB13, PB14를 TIM1 PWM 핀으로 만드는 함수
void TIM1_PWM_Init(void) {
    Macro_Set_Bit(RCC->AHB1ENR, 1);  // GPIOB 켜기
    Macro_Set_Bit(RCC->APB2ENR, 0);  // @ TIM1 켜기 (APB2 버스)

    // 1. PB13, PB14 핀을 Alternate Function(AF) 모드로 변경
    Macro_Write_Block(GPIOB->MODER, 0xF, 0xA, 26); // PB[14:13] = 10 (AF)
    
    // 2. PB13, PB14를 AF01(TIM1)로 연결
    Macro_Write_Block(GPIOB->AFR[1], 0xFF, 0x11, 20); // AFRH의 13, 14번 핀에 0001 연결
    
    // 3. 타이머 속도 설정 (TIM4와 동일하게 맞춰주세요. 아래는 ARR 1000 기준 예시)
    TIM1->PSC = 4 - 1;    // 96MHz / 4 = 24MHz 
    TIM1->ARR = 1200 - 1; // 24MHz / 1200 = 20kHz 주기
    
    // 4. PWM 모드 1 설정 (CH1, CH2)
    TIM1->CCMR1 |= (6 << 4) | (1 << 3);  // CH1 PWM 모드 1, Preload 활성화
    TIM1->CCMR1 |= (6 << 12) | (1 << 11); // CH2 PWM 모드 1, Preload 활성화
    
    // 5. @ 역상 채널(CH1N, CH2N) 출력 활성화 (TIM1 전용 안전장치 해제)
    TIM1->CCER |= (1 << 2) | (1 << 6); // CC1NE, CC2NE 비트 켜기
    TIM1->BDTR |= (1 << 15);           // MOE (Main Output Enable) 켜기!
    
    // 6. 타이머 시작!
    TIM1->CR1 |= (1 << 0);
}

// 왼쪽 모터용 (PB6, PB7) TIM4 PWM 초기화 함수
void TIM4_PWM_Init(void) {
    // 1. 클럭 켜기
    Macro_Set_Bit(RCC->AHB1ENR, 1);  // GPIOB 켜기
    Macro_Set_Bit(RCC->APB1ENR, 2);  //  TIM4 켜기 (비트 2번)

    // 2. PB6, PB7 핀을 Alternate Function(AF) 모드로 변경
    Macro_Write_Block(GPIOB->MODER, 0xF, 0xA, 12); // PB[7:6] = 10 (AF)
    
    // 3. PB6, PB7을 AF02(TIM4)로 연결
    Macro_Write_Block(GPIOB->AFR[0], 0xFF, 0x22, 24); // AFRL의 6, 7번 핀에 0010(AF02) 연결
    
    // 4.  96MHz 클럭에 맞춘 타이머 속도 설정 (1kHz 주기)
    TIM4->PSC = 4 - 1;    // 96MHz / 4 = 24MHz 
    TIM4->ARR = 1200 - 1; // 24MHz / 1200 = 20kHz 주기
    
    // 5. PWM 모드 1 설정 (CH1, CH2)
    TIM4->CCMR1 |= (6 << 4) | (1 << 3);  // CH1 PWM 모드 1, Preload 활성화
    TIM4->CCMR1 |= (6 << 12) | (1 << 11); // CH2 PWM 모드 1, Preload 활성화
    
    // 6. 출력 활성화
    TIM4->CCER |= (1 << 0) | (1 << 4); // CC1E, CC2E 비트 켜기
    
    // 7. 타이머 시작!
    TIM4->CR1 |= (1 << 0);
}


// 엔코더 핀 및 타이머 초기화 함수
void Encoder_Init(void) {
    // ---------------------------------------------------------
    // [왼쪽 모터] PA0, PA1 (TIM5 엔코더 모드 사용 - TIM2 충돌 회피)
    // ---------------------------------------------------------
    Macro_Set_Bit(RCC->AHB1ENR, 0);  // GPIOA 클럭 ON
    Macro_Set_Bit(RCC->APB1ENR, 3);  // TIM5 클럭 ON

    // PA0, PA1을 Alternate Function(AF) 모드(10)로 설정
    Macro_Write_Block(GPIOA->MODER, 0xF, 0xA, 0); 
    // PA0, PA1을 AF02(TIM5)로 연결
    Macro_Write_Block(GPIOA->AFR[0], 0xFF, 0x22, 0);
    // 엔코더 신호 안정화를 위해 내부 Pull-up(01) 설정
    Macro_Write_Block(GPIOA->PUPDR, 0xF, 0x5, 0);

    // TIM5 엔코더 모드 3 설정 (CH1, CH2 양쪽 에지 모두 카운트 -> x4 체배)
    TIM5->CCMR1 |= (1 << 0) | (1 << 8); // CC1S=01, CC2S=01 (입력 매핑)
    TIM5->CCER &= ~((1 << 1) | (1 << 5)); // 극성: Rising Edge (기본값)
    TIM5->SMCR |= (3 << 0); // SMS = 011 (Encoder Mode 3)
    TIM5->ARR = 0xFFFFFFFF; // 32비트 타이머의 최대값
    TIM5->CR1 |= (1 << 0);  // 카운터 시작

    // ---------------------------------------------------------
    // [오른쪽 모터] PA6, PA7 (TIM3 엔코더 모드 사용)
    // ---------------------------------------------------------
    Macro_Set_Bit(RCC->APB1ENR, 1);  // TIM3 클럭 ON
    
    // PA6, PA7을 Alternate Function(AF) 모드(10)로 설정 (비트 12부터 4비트)
    Macro_Write_Block(GPIOA->MODER, 0xF, 0xA, 12); 
    // PA6, PA7을 AF02(TIM3)로 연결 (비트 24부터 8비트)
    Macro_Write_Block(GPIOA->AFR[0], 0xFF, 0x22, 24);
    // 엔코더 신호 안정화를 위해 내부 Pull-up(01) 설정
    Macro_Write_Block(GPIOA->PUPDR, 0xF, 0x5, 12);

    // TIM3 엔코더 모드 3 설정 (CH1, CH2 양쪽 에지 모두 카운트 -> x4 체배)
    TIM3->CCMR1 |= (1 << 0) | (1 << 8);
    TIM3->CCER &= ~((1 << 1) | (1 << 5)); 
    TIM3->SMCR |= (3 << 0); // SMS = 011 (Encoder Mode 3)
    TIM3->ARR = 0xFFFF;     // 16비트 타이머의 최대값
    TIM3->CR1 |= (1 << 0);  // 카운터 시작
}