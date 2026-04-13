#include "device_driver.h"

void Key_Poll_Init(void)
{
	Macro_Set_Bit(RCC->AHB1ENR, 2); 
	Macro_Write_Block(GPIOC->MODER, 0x3, 0x0, 26);

	// PC7 (외부 스위치) 입력 모드 설정 (14번 비트 위치: 7 * 2)
	Macro_Write_Block(GPIOC->MODER, 0x3, 0x0, 14);  //입력으로 사용
	Macro_Write_Block(GPIOC->PUPDR, 0x3, 0x1, 14);  // 풀업저항 설정
}

int Key_Get_Pressed(void)
{
	return Macro_Check_Bit_Clear(GPIOC->IDR, 13);	
}

void Key_Wait_Key_Pressed(void)
{
	while(!Macro_Check_Bit_Clear(GPIOC->IDR, 13));
}

void Key_Wait_Key_Released(void)
{
	while(!Macro_Check_Bit_Set(GPIOC->IDR, 13));
}

void Key_ISR_Enable(int en)
{
	if(en)
	{
		Macro_Set_Bit(RCC->AHB1ENR, 2); 
		Macro_Write_Block(GPIOC->MODER, 0x3, 0x0, 26);

		Macro_Set_Bit(RCC->APB2ENR, 14); 
		Macro_Write_Block(SYSCFG->EXTICR[3], 0xf, 0x2, 4);

		Macro_Set_Bit(EXTI->FTSR, 13);
		EXTI->PR = 0x1 << 13;
		
		NVIC_ClearPendingIRQ((IRQn_Type)40);
		Macro_Set_Bit(EXTI->IMR, 13);
		NVIC_EnableIRQ((IRQn_Type)40);
	}

	else
	{
		NVIC_DisableIRQ((IRQn_Type)40);
	}
}

// 추가: PC7 버튼 읽기 (Active Low 가정)
// int Ext_Key_Get_Pressed(void)
// {
// 	return Macro_Check_Bit_Clear(GPIOC->IDR, 7);	
// }
#define N 		50000


static int Key_Check_Input(void)
{
	return Macro_Check_Bit_Clear(GPIOC->IDR, 7);	
}


/// @@@ 중요 @@@ ///
// 채터링을 방지하는 키 입력 로직 //
int Ext_Key_Get_Pressed(void)  //스위치 입력 오류를 방지하기 위한 채터링 키 pressed
{
	unsigned int i, k;

	for(;;)
	{
		k = Key_Check_Input();

		for(i = 0; i < N; i++)
		{
			if(k != Key_Check_Input())
			{
				break;
			}
		}

		if(i == N) break;
	}

	return k;
}