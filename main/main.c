#include "device_driver.h"
#include <stdio.h>
#include <math.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

volatile uint8_t bt_rx_data = 0;
volatile uint8_t bt_rx_flag = 0;

volatile uint8_t mpu_data_ready = 0; // 전역 변수: 데이터 준비 깃발

// IMU값 읽어오기 (테스트0406 오후 4시 )
void Set_Motor_Power(float pid_output);
// DMP
void MPU6050_DMP_Init(void);
// MPU int핀 인터럽트
void EXTI15_10_IRQHandler(void);

void Find_Deadzone_Task(void);

static void Sys_Init(int baud) 
{
	SCB->CPACR |= (0x3 << 10*2)|(0x3 << 11*2); 
	Clock_Init();
	Uart2_Init(baud);
	setvbuf(stdout, NULL, _IONBF, 0);
	LED_Init();


}

// [해결책] 소수점(float) 변수들이 있는 메인 루프를 별도 함수로 격리! (해결)
// Balancing_Task 함수 수정
// Balancing_Task 함수 수정
void Balancing_Task(void)
{
    short gyro[3], accel[3]; 
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float q0, q1, q2, q3; 
    float current_angle = 0.0;
    
    float target_angle = 0.0;
    float error = 0.0, prev_error = 0.0;
    float Kp = 24.0, Ki = 350.0, Kd = 1.2;
    float P_term = 0.0, I_term = 0.0, D_term = 0.0;
    float PID_output = 0.0;

    int fifo_status; // 추가됨

    printf("\n--- Start Balancing with DMP ---\n");

    // ==========================================================
    // [핵심 추가] 부팅하는 동안 꽉 차버린 쓰레기 데이터를 
    // 루프 시작 직전에 깔끔하게 한 번 비워줍니다! (이게 비기입니다)
    // ==========================================================
    mpu_reset_fifo(); 
    mpu_data_ready = 0; // 깃발도 초기화

    while(1) 
    {   
        // 1. 인터럽트 신호가 들어왔을 때만 진입
        if (mpu_data_ready == 1) 
        {
            mpu_data_ready = 0; // 깃발 내리기
            
            // 2. FIFO 읽기
            fifo_status = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
            
            if (fifo_status == 0) {
                // [정상 작동]
                q0 = quat[0] / 1073741824.0f;
                q1 = quat[1] / 1073741824.0f;
                q2 = quat[2] / 1073741824.0f;
                q3 = quat[3] / 1073741824.0f;

                // 추가한 부분 (수정 필요한지 고려)
                float gravity = -2.0f * (q1 * q3 - q0 * q2);
                
                // 오차 때문에 1.0을 넘어가서 NaN(수학 에러)이 뜨는 것을 원천 차단!
                if (gravity > 1.0f) gravity = 1.0f;
                else if (gravity < -1.0f) gravity = -1.0f;

                // 여기까지 추가한 부분


                // current_angle = asin(-2.0f * (q1 * q3 - q0 * q2)) * 180.0f / 3.14159265f;
                current_angle = asin(gravity) * 180.0f / 3.14159265f; //gravity로 대체


                error = current_angle - target_angle;
                P_term = Kp * error;
                I_term += Ki * error * 0.01;
                D_term = Kd * (error - prev_error) / 0.01;
                PID_output = P_term + I_term + D_term;
                
                Set_Motor_Power(PID_output);
                prev_error = error;
                
                printf("Angle: %.2f | Motor: %.2f\n", current_angle, PID_output);
            }
            else {
                //  [이전 해결책] 작동 중 버퍼가 꼬였을 때 멈추지 않고 강제로 비워줌
                mpu_reset_fifo();
            }
        }// mpu_data_ready
    }       
}

// float 변수가 단 1개도 없는 껍데기 Main 함수
void Main(void)
{
    // 1. 여기서 FPU부터 안전하게 켭니다. (이제 뻗지 않습니다!)
    Sys_Init(115200);

    Uart2_Send_Byte('O');
    Uart2_Send_Byte('K');
    Uart2_Send_Byte('\n');

    printf("\n--- System Booting ---\n");
    
    I2C1_Init();
    printf(">> I2C Init OK!\n");

    TIM1_PWM_Init();
    TIM4_PWM_Init();
    Bluetooth_Init();
    // MPU6050_Init(); 

    uint8_t who_am_i = MPU6050_ReadReg(0x75);
    if (who_am_i == 0x68) {
        printf(">> [SUCCESS] MPU6050 Connection OK! (ID: 0x68)\n");
    } else {
        printf(">> [ERROR] MPU6050 Not Found. (ID: 0x%X)\n", who_am_i);
    }


    MPU6050_DMP_Init(); // <--- 기존 MPU6050_Init() 대신
    MPU6050_INT_Init();  //DMP 인터럽트 활성화
    printf(">> MPU6050 Wake-up OK!\n");
    

    // 2. FPU가 완벽하게 켜지고 센서도 깨어났으니, 소수점 함수 진입!
    Balancing_Task();

    // 모터 최저 PWM 테스트 함수
    // Find_Deadzone_Task();
}




void Set_Motor_Power(float pid_output) {
    int pwm_speed = (int)fabs(pid_output);
    
    // ==========================================
    // [보정 적용] 양쪽 모터 데드존을 따로 분리!
    // ==========================================
    int MIN_PWM_L = 60; 
    int MIN_PWM_R = 80; // 우측이 느리므로 20 더 강하게 줍니다.

    int pwm_L = pwm_speed;
    int pwm_R = pwm_speed;

    // PID가 돌라고 명령했을 때 데드존 더해주기
    if (pwm_speed > 0) {
        pwm_L = pwm_L + MIN_PWM_L;
        pwm_R = pwm_R + MIN_PWM_R;
    }

    // 모터 최대 스피드 락
    if (pwm_L > 1199) pwm_L = 1199; 
    if (pwm_R > 1199) pwm_R = 1199; 

    if (pid_output > 0) {
        // [전진]
        TIM1->CCR1 = 0; 
        TIM1->CCR2 = pwm_L; // 모터 L (PB14)

        TIM4->CCR1 = pwm_R; // 모터 R (PB6)
        TIM4->CCR2 = 0;         
    } 
    else if (pid_output < 0) {
        // [후진] 꼬여있던 L, R 방향을 바로잡았습니다!
        TIM1->CCR1 = pwm_L; // 모터 L (PB13)
        TIM1->CCR2 = 0; 

        TIM4->CCR1 = 0; 
        TIM4->CCR2 = pwm_R; // 모터 R (PB7)
    } 
    else {
        // [정지]
        TIM4->CCR1 = 0; TIM4->CCR2 = 0;
        TIM1->CCR1 = 0; TIM1->CCR2 = 0;
    } 
}


void MPU6050_DMP_Init(void) {
    printf(">> MPU6050 Init Start...\n");
    if (mpu_init(NULL) != 0) { printf(">> [ERROR] MPU Init Failed!\n"); return; }
    
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    
    if (dmp_load_motion_driver_firmware() != 0) { printf(">> [ERROR] DMP FW Failed!\n"); return; }
    
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(100); // 100Hz
    mpu_set_dmp_state(1);   // DMP 켜기
    
    printf(">> [SUCCESS] DMP Init OK!\n");
}

// PB10 ~ PB15 핀에 인터럽트가 걸리면 무조건 이 함수가 실행
void EXTI15_10_IRQHandler(void) {
    // 1. 정말 PB12(EXTI12)에서 발생한 인터럽트인지 확인
    if (EXTI->PR & (1 << 12)) {
        
        // 2. 인터럽트 발생 기록(Pending Register) 지우기 
        // (이걸 안 지우면 무한 루프에 빠집니다)
        EXTI->PR = (1 << 12); 
        
        // 3. 깃발 들기: "메인 루프야, 데이터 가져가라!"
        mpu_data_ready = 1;
    }
}


// 우측 모터 20 더 빠르게 해야 함
// 모터가 돌기 시작하는 최소 PWM(데드존) 값을 찾는 테스트 함수
void Find_Deadzone_Task(void)
{
    printf("\n==================================\n");
    printf("--- Motor Dead-zone Test Start ---\n");
    printf("로봇을 들어서 바퀴가 땅에 닿지 않게 하세요!\n");
    printf("3초 뒤 테스트를 시작합니다...\n");
    printf("==================================\n");
    
    TIM2_Delay(3000); // 3초 대기 (안전)

    // PWM을 0부터 200까지 2씩 천천히 올려봅니다.
    for (int test_pwm = 60; test_pwm <= 200; test_pwm += 2) {
        
        // 양쪽 모터에 직접 전진 방향으로 서서히 강해지는 PWM을 넣습니다.
        TIM1->CCR1 = 0; // 왼쪽 전진
        TIM1->CCR2 = test_pwm; 

        TIM4->CCR1 = test_pwm+20; // 오른쪽 전진
        TIM4->CCR2 = 0;
    
        
        // 현재 넣고 있는 파워를 터미널에 출력
        printf("Current Test PWM: %d\n", test_pwm);
        
        // 0.1초 대기 (천천히 파워가 올라가도록)
        TIM2_Delay(100); 
    }

    // 테스트가 끝나면 모터를 끕니다.
    TIM4->CCR1 = 0; TIM4->CCR2 = 0;
    TIM1->CCR1 = 0; TIM1->CCR2 = 0;

    printf("\n--- Test Finished! ---\n");
    printf("바퀴가 부드럽게 돌기 시작했던 때의 PWM 값을 기억하시나요?\n");
    printf("그 값을 Set_Motor_Power() 함수의 MIN_PWM에 적어주세요!\n");
    
    // 무한 루프에 빠뜨려서 로봇이 더 이상 안 움직이게 함
    while(1); 
}
