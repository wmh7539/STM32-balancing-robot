#include "device_driver.h"

/* freg = 5000 ~ 100000 */

#define SC16IS752_I2CADDR									0x9A
#define SC16IS752_I2CADDR_WR								(SC16IS752_I2CADDR|0x0)
#define SC16IS752_I2CADDR_RD								(SC16IS752_I2CADDR|0x1)

/* I2C1_SCL => PB6, I2C1_SDA => PB7 */

#if 0
void I2C1_SC16IS752_Init(unsigned int freq)
{
	unsigned int r;
	volatile int i;
	
	Macro_Set_Bit(RCC->AHB1ENR, 1); 					// Port-B Clock On
	Macro_Set_Bit(RCC->APB1ENR, 21); 					// I2C1 Clock On

	Macro_Clear_Bit(RCC->APB1RSTR, 21); 				// I2C1 Reset
	Macro_Set_Bit(RCC->APB1RSTR, 21);
	for(i = 0; i < 1000; i++);
	Macro_Clear_Bit(RCC->APB1RSTR, 21);

	Macro_Write_Block(GPIOB->MODER, 0xf, 0xa, 12);  	// PB[7:6] => ALT
	Macro_Write_Block(GPIOB->AFR[0], 0xff, 0x44, 24); 	// PB[7:6] => AF04
	Macro_Write_Block(GPIOB->OTYPER, 0x3, 0x3, 6); 		// PB[7:6] => Open Drain
	Macro_Write_Block(GPIOB->OSPEEDR, 0xf, 0xa, 12); 	// PB[7:6] => Fast Speed
	Macro_Write_Block(GPIOB->PUPDR, 0xf, 0x5, 12); 		// PB[7:6] => Internal Pull-up

	Macro_Write_Block(I2C1->CR2, 0x3f, PCLK1 / 1000000, 0);
	Macro_Clear_Bit(I2C1->CR1, 0);
	I2C1->TRISE = (PCLK1 / 1000000) + 1;
	r = PCLK1 / (freq * 2);
	I2C1->CCR = ((r < 4) ? 4 : r);

	Macro_Clear_Bit(I2C1->CR1, 1);
	Macro_Set_Bit(I2C1->CR1, 0);
	Macro_Set_Bit(I2C1->CR1, 10);
}

void I2C1_SC16IS752_Write_Reg(unsigned int addr, unsigned int data)
{
	while(Macro_Check_Bit_Set(I2C1->SR2, 1)); 					// Idle OK

	Macro_Set_Bit(I2C1->CR1, 8); 								// Start
	while(Macro_Check_Bit_Clear(I2C1->SR1, 0));					// Check Start

	I2C1->DR = SC16IS752_I2CADDR_WR ;							// Send WR Address
	while(Macro_Check_Bit_Clear(I2C1->SR1, 1));					// Check Address
	(void)I2C1->SR2;											// Clear ADDR flag by reading SR2

	I2C1->DR = addr << 3;										// Send Register Address
	while(Macro_Check_Bit_Clear(I2C1->SR1, 2));					// Check Byte Transfer Finished

	I2C1->DR = data;											// Send Data
	while(Macro_Check_Bit_Clear(I2C1->SR1, 2));					// Check Byte Transfer Finished

	Macro_Set_Bit(I2C1->CR1, 9); 								// Stop
	while(Macro_Check_Bit_Set(I2C1->CR1, 9));					// Check Stop(Auto Cleared)
}

void I2C1_SC16IS752_Config_GPIO(unsigned int config)
{
	I2C1_SC16IS752_Write_Reg(SC16IS752_IODIR, config);
}

void I2C1_SC16IS752_Write_GPIO(unsigned int data)
{
	I2C1_SC16IS752_Write_Reg(SC16IS752_IOSTATE, data);
}
#endif

void I2C1_Init(void)
{

	unsigned int r;
    volatile int i;

    // 1. 클록 활성화 (GPIOB, I2C1)
    Macro_Set_Bit(RCC->AHB1ENR, 1);  // GPIOB 활성화
    Macro_Set_Bit(RCC->APB1ENR, 21); // I2C1 활성화

	// 2. 강력한 하드웨어 리셋 (실습 코드 방식 적용)
    Macro_Clear_Bit(RCC->APB1RSTR, 21);                 // I2C1 Reset 준비
    Macro_Set_Bit(RCC->APB1RSTR, 21);                   // 리셋 시작
    for(i = 0; i < 1000; i++);                          // 노이즈가 날아갈 때까지 대기
    Macro_Clear_Bit(RCC->APB1RSTR, 21);                 // 리셋 해제


    // 3. PB8, PB9 핀 설정
    // MODER: Alternate function (10)

	Macro_Write_Block(GPIOB->MODER, 0xf, 0xa, 16);      // PB[9:8] => ALT (10)
    Macro_Write_Block(GPIOB->AFR[1], 0xff, 0x44, 0);    // PB[9:8] => AF04 연결
    Macro_Write_Block(GPIOB->OTYPER, 0x3, 0x3, 8);      // PB[9:8] => Open Drain
    Macro_Write_Block(GPIOB->OSPEEDR, 0xf, 0xa, 16);    // PB[9:8] => Fast Speed
    Macro_Write_Block(GPIOB->PUPDR, 0xf, 0x5, 16);      // PB[9:8] => Internal Pull-up

    // 4. I2C 하드웨어 설정 (PCLK1 기반 400kHz 고속 세팅)
	// I2C 하드웨어 타이밍 설정
    Macro_Write_Block(I2C1->CR2, 0x3f, PCLK1 / 1000000, 0);
    Macro_Clear_Bit(I2C1->CR1, 0);                      // 설정 중 PE=0

    // 400kHz 세팅 (Fast Mode)
    I2C1->TRISE = ((PCLK1 / 1000000) * 300 / 1000) + 1; // FM 최대 상승 시간 300ns
    I2C1->CCR = 0x8000 | (PCLK1 / 1200000);             // Bit 15(Fast Mode) 활성화 및 주기 계산

    // 4. I2C 활성화
	Macro_Clear_Bit(I2C1->CR1, 1);                      // I2C 모드 선택
    Macro_Set_Bit(I2C1->CR1, 0); // PE(Peripheral Enable) 비트 ON
	Macro_Set_Bit(I2C1->CR1, 10);                       // ACK=1 (응답 비트 켜기)
}

// I2C 시작 신호
void I2C1_Start(void) {
    I2C1->CR1 |= (1 << 8); // START 비트 세트
    while (!(I2C1->SR1 & (1 << 0))); // SB(Start Bit) 플래그 대기
}

// I2C 주소 전송
void I2C1_WriteAddress(uint8_t address) {
    I2C1->DR = address;
    while (!(I2C1->SR1 & (1 << 1))); // ADDR 플래그 대기
    volatile int temp = I2C1->SR1;   // ADDR 플래그 클리어 (읽기 동작)
    temp = I2C1->SR2;
}

// I2C 데이터 전송
void I2C1_WriteData(uint8_t data) {
    while (!(I2C1->SR1 & (1 << 7))); // TXE(Tx Empty) 대기
    I2C1->DR = data;
    while (!(I2C1->SR1 & (1 << 2))); // BTF(Byte Transfer Finished) 대기
}

// 특정 레지스터에서 1바이트 읽어오기
uint8_t MPU6050_ReadReg(uint8_t reg) {
    uint8_t data;

    I2C1_Start();
    I2C1_WriteAddress(0xD0); // MPU6050 쓰기 주소 (0x68 << 1 | 0)
    I2C1_WriteData(reg);     // 읽고자 하는 레지스터 주소 전송

    I2C1_Start();            // Restart 신호
    I2C1_WriteAddress(0xD1); // MPU6050 읽기 주소 (0x68 << 1 | 1)

    // 1바이트만 읽을 때는 ACK를 끄고 STOP을 예약해야 함
    I2C1->CR1 &= ~(1 << 10); // ACK 비활성화
    I2C1->CR1 |= (1 << 9);   // STOP 신호 예약

    while (!(I2C1->SR1 & (1 << 6))); // RXNE(Rx Not Empty) 대기
    data = I2C1->DR;         // 데이터 수신

	Macro_Set_Bit(I2C1->CR1, 10); // [추가] 다음 통신을 위해 ACK 다시 켜기

    return data;
}

// I2C 종료 신호
void I2C1_Stop(void) {
    I2C1->CR1 |= (1 << 9); // STOP 비트 세트
}

// 특정 레지스터에 1바이트 쓰기
void MPU6050_WriteReg(uint8_t reg, uint8_t data) {
    I2C1_Start();            // 1. 시작 신호
    I2C1_WriteAddress(0xD0); // 2. MPU6050 쓰기 주소 전송
    I2C1_WriteData(reg);     // 3. 기록할 레지스터 주소 전송
    I2C1_WriteData(data);    // 4. 기록할 데이터(설정값) 전송
    I2C1_Stop();             // 5. 통신 종료
}

// MPU6050 센서 초기화 (잠 깨우기 및 필터 설정)
void MPU6050_Init(void) {
    // 1. 전원 관리 레지스터(0x6B)에 0x00을 써서 수면 모드 해제
    MPU6050_WriteReg(0x6B, 0x00); 
    
    // 2. DLPF (디지털 로우 패스 필터) 설정 (0x1A 레지스터)
    // 진동이 많은 로봇을 위해 노이즈를 살짝 깎아줍니다. (0x03 입력 시 약 40Hz 필터링)
    MPU6050_WriteReg(0x1A, 0x03);
}




// MPU6050 관련 설정하기 
// MPU6050 6축 원시 데이터(Raw Data) 읽어오기
void MPU6050_Get_RawData(MPU6050_Data *sensor) {
    // 가속도 데이터 읽기 (레지스터 0x3B 부터 순차적으로 저장되어 있음)
    sensor->Accel_X = (MPU6050_ReadReg(0x3B) << 8) | MPU6050_ReadReg(0x3C);
    sensor->Accel_Y = (MPU6050_ReadReg(0x3D) << 8) | MPU6050_ReadReg(0x3E);
    sensor->Accel_Z = (MPU6050_ReadReg(0x3F) << 8) | MPU6050_ReadReg(0x40);

    // 자이로 데이터 읽기 (레지스터 0x43 부터 순차적으로 저장되어 있음)
    sensor->Gyro_X = (MPU6050_ReadReg(0x43) << 8) | MPU6050_ReadReg(0x44);
    sensor->Gyro_Y = (MPU6050_ReadReg(0x45) << 8) | MPU6050_ReadReg(0x46);
    sensor->Gyro_Z = (MPU6050_ReadReg(0x47) << 8) | MPU6050_ReadReg(0x48);
}


// 1. 다중 바이트 연속 쓰기 함수
void I2C1_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    I2C1_Start();
    I2C1_WriteAddress(devAddr << 1); 
    I2C1_WriteData(regAddr);         
    for (uint8_t i = 0; i < length; i++) {
        I2C1_WriteData(data[i]);
    }
    I2C1_Stop();
}

// 2. 다중 바이트 연속 읽기 함수
void I2C1_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    I2C1_Start();
    I2C1_WriteAddress(devAddr << 1); 
    I2C1_WriteData(regAddr);         
    I2C1_Start(); 
    I2C1_WriteAddress((devAddr << 1) | 1); 
    for (uint8_t i = 0; i < length; i++) {
        if (i == length - 1) {
            I2C1->CR1 &= ~(1 << 10); // NACK
            I2C1->CR1 |= (1 << 9);   // STOP 예약
        } else {
            I2C1->CR1 |= (1 << 10);  // ACK
        }
        while (!(I2C1->SR1 & (1 << 6))); 
        data[i] = I2C1->DR; 
    }
    Macro_Set_Bit(I2C1->CR1, 10); 
}

// 3. 라이브러리 연결용 래퍼 함수 (0 반환)
int my_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    I2C1_WriteBytes(slave_addr, reg_addr, length, (uint8_t*)data);
    return 0; 
}
int my_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    I2C1_ReadBytes(slave_addr, reg_addr, length, data);
    return 0; 
}

// 라이브러리용 시간 중계 함수
void my_get_ms(unsigned long *count) {
    *count = SysTick_Get_Time(); // 내 시스템 함수로 뱉어낸 시간을 포인터에 담아줌
}

// MPU6050 인터럽트 핀(PB12) 초기화 함수
void MPU6050_INT_Init(void) {
    // 1. 클록 활성화: GPIOB와 SYSCFG(외부 인터럽트 설정용) 켜기
    RCC->AHB1ENR |= (1 << 1);  // GPIOB 클록 ON
    RCC->APB2ENR |= (1 << 14); // SYSCFG 클록 ON 

    // 2. PB12를 입력(Input) 모드로 설정
    GPIOB->MODER &= ~(0x3 << 24); // PB12 [25:24] 비트를 00으로 클리어 (Input 모드)
    
    // 3. PB12 풀다운(Pull-down) 저항 설정 (노이즈로 인한 오작동 방지)
    GPIOB->PUPDR &= ~(0x3 << 24);
    GPIOB->PUPDR |= (0x2 << 24);  // 10: Pull-down

    // 4. SYSCFG 설정: EXTI12 라인을 Port B(PB12)로 연결
    // EXTICR[3] 레지스터의 하위 4비트[3:0]를 0001(Port B)로 설정
    SYSCFG->EXTICR[3] &= ~(0xF << 0);
    SYSCFG->EXTICR[3] |= (0x1 << 0);

    // 5. EXTI(외부 인터럽트) 설정: 상승 에지(Rising Edge) 트리거
    EXTI->IMR  |= (1 << 12);      // EXTI12 마스크 해제 (인터럽트 허용)
    EXTI->RTSR |= (1 << 12);      // 상승 에지(0V -> 3.3V)에서 트리거 활성화
    EXTI->FTSR &= ~(1 << 12);     // 하강 에지 트리거는 비활성화

    // 6. NVIC(인터럽트 컨트롤러) 설정: EXTI15_10_IRQ 켜기 (PB12는 15~10 그룹에 속함)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}