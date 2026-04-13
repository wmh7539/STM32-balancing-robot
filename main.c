// 프로젝트 main.c 파일 주석 정리된 코드

#include "device_driver.h"
#include <stdio.h>
#include <math.h>
#include "max7219_8x32.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

// --- 통신 및 상태 플래그 변수 ---
volatile uint8_t bt_rx_data = 0;       // 블루투스 수신 데이터 보관
volatile uint8_t bt_rx_flag = 0;       // 블루투스 수신 완료 깃발
volatile uint8_t mpu_data_ready = 0;   // MPU6050 데이터 준비 완료 깃발

// --- 주행 제어 변수 ---
volatile float target_velocity = 0.0f; // 전/후진 목표 속도 (+: 전진)
volatile float turn_speed = 0.0f;      // 좌/우 회전 목표 속도 (+: 우회전)

volatile float target_velocity_control = 0.0f; // 블루투스 명령에 따른 전후진 속도 증감분
volatile float turn_speed_control = 0.0f;      // 블루투스 명령에 따른 회전 속도 증감분

// --- 상태 및 애니메이션 변수 ---
volatile int robot_start = 0;  // 로봇 구동 상태 (1: ON, 0: OFF)
volatile int stop_face = 0;    // 진입 금지 표정 강제 출력 플래그
volatile int face_turn = 0;    // 회전 시 애니메이션 프레임 카운터

static float prev_target_velocity = 0.0f; // 이전 루프의 목표 속도 (상태 변화 감지용)

// --- 함수 선언 ---
void Set_Motor_Power(float pid_output);
void Set_Motor_Power_LR(float pid_L, float pid_R);
void MPU6050_DMP_Init(void);
void EXTI15_10_IRQHandler(void);
void Find_Deadzone_Task(void);
void Encoder_Test_Task(void);

// 시스템 기본 설정 (클럭, UART, LED 초기화)
static void Sys_Init(int baud) 
{
	SCB->CPACR |= (0x3 << 10*2)|(0x3 << 11*2); 
	Clock_Init();
	Uart2_Init(baud);
	setvbuf(stdout, NULL, _IONBF, 0);
	LED_Init();

}

// 메인 밸런싱 및 주행 제어 루프
// 이중 루프(Cascade PID) 구조: 외부 루프(속도 제어, 50ms) + 내부 루프(자세 제어, 10ms)
void Balancing_Task(void)
{
    short gyro[3], accel[3]; 
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float q0, q1, q2, q3; 

    // --- 내부 루프 (자세 제어) 변수 ---
    float current_angle = 0.0;    // 현재 기울기 각도
    float target_angle = 0.5;     // 유지해야 할 목표 기울기 (외부 루프에서 계산됨)
    
    float error = 0.0, prev_error = 0.0;
    float Ki = 180.0, Kd = 0.5;
    float Kp = 35.0f;             // 기본 P 게인
    float Kp_boost = 15.0f;       // 비선형 제어용 추가 P 게인 (오차가 클 때 파워 뻥튀기용)

    float P_term = 0.0, I_term = 0.0, D_term = 0.0;
    float PID_output = 0.0;       // 최종 모터 출력값

    // --- 외부 루프 (속도 제어) 변수 ---
    int loop_counter = 0;         // 50ms 주기를 맞추기 위한 카운터
    int prev_enc_L = 0, prev_enc_R = 0;
    float current_velocity = 0.0;
    float filtered_velocity = 0.0; // 센서 노이즈 제거용 로우패스 필터 적용 속도
    
    float vel_Kp = 0.02f;         // 속도 제어 P 게인
    float vel_Ki = 0.001f;        // 속도 제어 I 게인
    float vel_error = 0.0f, vel_prev_error = 0.0f;
    float vel_P = 0.0, vel_I = 0.0, vel_D = 0.0;
    float vel_PID_output = 0.0;   // 속도 보상을 위한 추가 기울기 값

    float default_angle = 1.0f;   // 로봇의 물리적 무게중심 영점 각도
    float goal_angle = 1.0f;      // 외부 루프가 내부 루프에게 지시할 최종 목표 각도
    float active_velocity = 0.0f; // 급가속 방지용 스무딩 처리된 현재 목표 속도

    int fifo_status;

    static int current_face = -1; // 현재 매트릭스에 출력된 표정 (-1: 초기화)
    int target_face = 0;          // 이번 루프에 출력할 표정 명령

    printf("\n--- Start Balancing with DMP ---\n");

    // 초기 쓰레기 데이터 정리 및 플래그 초기화
    mpu_reset_fifo(); 
    mpu_data_ready = 0; 
    
    // 엔코더 초기값 기록 (시작점 기준)
    prev_enc_L = (int)TIM5->CNT;
    prev_enc_R = -((short)TIM3->CNT);

    while(1) 
    {   
        // 1. 블루투스 명령 처리 (가장 최우선 처리)
        if (bt_rx_flag == 1) 
        {
            bt_rx_flag = 0; // 명령 수신 확인 완료

            // 프로토콜에 따른 명령 분기
            switch (bt_rx_data) {
                case 1:  robot_start = 1; break; // 구동 시작
                case 2:  robot_start = 0; break; // 구동 정지
                case 3:  // 전/후진 속도 증가 (최대 80 제한)
                    target_velocity_control += 5.0f;
                    if(target_velocity_control >= 80.0f) target_velocity_control = 80.0f;
                    break;
                case 4:  // 전/후진 속도 감소 (최저 -65 제한)
                    target_velocity_control -= 5.0f; 
                    if(target_velocity_control <= -65.0f) target_velocity_control = -65.0f;
                    break;
                case 5:  // 우회전 속도 증가
                    turn_speed_control += 5.0f;
                    if(turn_speed_control >= 40.0f) turn_speed_control = 40.0f;
                    break;
                case 6:  // 좌회전 속도 증가
                    turn_speed_control -= 5.0f;
                    if(turn_speed_control <= -25.0f) turn_speed_control = -25.0f;
                    break;
                case 7:  // 모든 보정 속도 초기화
                    target_velocity_control = 0.0f;
                    turn_speed_control = 0.0f;
                    break;
                case 8:  stop_face = 1; break; // 진입 금지 표정 강제 ON
                
                // 주행 매크로 명령
                case 10: // 직진
                    target_velocity = 80.0f + target_velocity_control; 
                    turn_speed = 0.0f; stop_face = 0; break;
                case 11: // 후진
                    target_velocity = -80.0f - target_velocity_control; 
                    turn_speed = 0.0f; stop_face = 0; break;
                case 12: // 제자리 좌회전
                    target_velocity = 0.0f; 
                    turn_speed = -40.0f - turn_speed_control; stop_face = 0; break;
                case 13: // 제자리 우회전
                    target_velocity = 0.0f; 
                    turn_speed = 40.0f + turn_speed_control; stop_face = 0; break;
                case 14: // 전진 + 우회전 (우커브)
                    target_velocity = 80.0f; turn_speed = 20.0f; stop_face = 0; break;
                case 15: // 전진 + 좌회전 (좌커브)
                    target_velocity = 80.0f; turn_speed = -20.0f; stop_face = 0; break;
                case 16: // 제동명령 대기
                    target_velocity = 0.0f; turn_speed = 0.0f; break;
                case 0:  // 완전 정지 (속도 적분항 초기화하여 관성 주행 방지)
                    target_velocity = 0.0f; turn_speed = 0.0f; vel_I = 0.0f; break;
            }
        }

        // 2. 센서 데이터 수신 및 제어 루프
        if (mpu_data_ready == 1) 
        {
            mpu_data_ready = 0; 
            fifo_status = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
            
            if (fifo_status == 0) {
                // 밀린 데이터 처리: 리셋 대신 최신 데이터가 나올 때까지 전부 읽어 덮어씀 (DMP 적분 연속성 유지)
                while (more > 0) {
                    dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
                }

                // 쿼터니언을 오일러 각(기울기)으로 변환
                q0 = quat[0] / 1073741824.0f;
                q1 = quat[1] / 1073741824.0f;
                q2 = quat[2] / 1073741824.0f;
                q3 = quat[3] / 1073741824.0f;
                
                float gravity = -2.0f * (q1 * q3 - q0 * q2); // 중력 방향 벡터 추출
                
                // 수학 함수(asin) 오류 방지를 위한 클램핑
                if (gravity > 1.0f) gravity = 1.0f;
                else if (gravity < -1.0f) gravity = -1.0f;

                current_angle = asin(gravity) * 180.0f / 3.14159265f; 

                // =========================================================
                // [외부 루프] 속도 제어 (50ms 주기 / 20Hz)
                // 이유: 속도는 관성 때문에 각도보다 천천히 변하므로 주기를 길게 설정
                // =========================================================
                loop_counter++;
                if (loop_counter >= 5) 
                {
                    loop_counter = 0;
                    
                    // 1. 엔코더 변화량으로 현재 속도 계산
                    int curr_enc_L = (int)TIM5->CNT;
                    int curr_enc_R = -((short)TIM3->CNT); 
                    int delta_L = curr_enc_L - prev_enc_L;
                    int delta_R = curr_enc_R - prev_enc_R;
                    prev_enc_L = curr_enc_L;
                    prev_enc_R = curr_enc_R;
                    
                    float delta_avg = (delta_L + delta_R) / 2.0f;
                    current_velocity = delta_avg * 2.686f; // 엔코더 틱을 물리 속도로 변환
                    
                    // 로우패스 필터: 엔코더 튀는 값(노이즈) 부드럽게 상쇄
                    filtered_velocity = (filtered_velocity * 0.8f) + (current_velocity * 0.2f);
                    
                    // 2. 가속도 스무딩 (급출발/모터 헛도는 현상 방지)
                    if (target_velocity == 0.0f) {
                        active_velocity = 0.0f; // 정지 명령 시 엑셀 즉각 오프
                    } else {
                        // 목표 속도까지 10%씩 서서히 도달시킴
                        active_velocity = (active_velocity * 0.9f) + (target_velocity * 0.1f);
                    }

                    // 3. 속도 오차 계산
                    vel_error = active_velocity - filtered_velocity; 
                    
                    // 4. 스냅백(밀림 현상) 방지 로직
                    if (target_velocity == 0.0f) {
                        // 주행 -> 정지로 바뀌는 순간, 누적된 I항(엑셀)을 비워버림
                        if (prev_target_velocity != 0.0f) vel_I = 0.0f; 
                        
                        // 미세 속도는 무시하여 제자리 진동 방지
                        if (fabs(vel_error) < 1.5f) vel_error = 0.0f; 
                        
                        // 브레이크(P항)는 데드존 없이 즉각 개입하도록 게인 1.5배 증폭
                        vel_P = (vel_Kp * 1.5f) * vel_error; 
                    }
                    else 
                    {   // 주행 모드
                        if (fabs(vel_error) < 5.0f) vel_error = 0.0f; // 주행 중 잔진동 무시
                        vel_P = vel_Kp * vel_error;
                    }

                    vel_I += vel_Ki * vel_error;
                    
                    // I항(적분) 누적 제한: 과도하게 각도가 틀어지는 것(Windup) 방지
                    if(vel_I > 3.0f) vel_I = 3.0f;
                    if(vel_I < -3.0f) vel_I = -3.0f;
                    if(fabs(current_angle) > 20.0f) vel_I = 0.0f; // 크게 넘어졌을 땐 엑셀 떼기

                    vel_PID_output = vel_P + vel_I; 
                    
                    // 5. 최종 목표 기울기 설정
                    // 속도를 내려면 로봇을 앞으로 기울여야 함 (영점 각도 + 속도 제어기 출력)
                    float raw_target_angle = default_angle + vel_PID_output;
                    
                    // 너무 눕지 않도록 안전 각도 제한 (±7도)
                    if (raw_target_angle > default_angle + 7.0f) raw_target_angle = default_angle + 7.0f;
                    if (raw_target_angle < default_angle - 7.0f) raw_target_angle = default_angle - 7.0f;

                    goal_angle = raw_target_angle;
                }
                
                // =========================================================
                // [내부 루프] 자세 제어 (10ms 주기 / 100Hz)
                // 이유: 넘어지지 않으려면 중력 변화에 즉각적으로 반응해야 함
                // =========================================================
                
                // 목표 각도 스무딩: 외부 루프가 목표 각도를 홱홱 바꿔도 서서히 따라가게 만듦
                target_angle = (target_angle * 0.6f) + (goal_angle * 0.4f);

                error = current_angle - target_angle; 

                // 비선형 P 제어: 기본 제어(선형) + 오차 제곱에 비례하는 제어 추가
                // 로봇이 많이 기울어질수록 기하급수적으로 강하게 모터를 돌려 잡아챔
                P_term = (Kp * error) + (Kp_boost * error * fabs(error));
                
                // P항 한계치 제한
                if (P_term > 900.0f) P_term = 900.0f;  
                if (P_term < -900.0f) P_term = -900.0f;  

                D_term = Kd * (error - prev_error) / 0.01; // D 제어: 각도 변화 속도(각속도) 제어

                // 모터가 구동 상태일 때만 I 제어 활성화
                if (robot_start == 1) {
                    I_term += Ki * error * 0.01;
                    
                    // I항 한계치 제한 (Anti-windup)
                    if (I_term > 300.0f) I_term = 300.0f;
                    else if (I_term < -300.0f) I_term = -300.0f;
                    
                    // 오차 방향이 반대로 꺾이면(중심을 지나치면) I항 초기화하여 진동 감소
                    if (error * prev_error < 0.0f) I_term = 0.0f; 

                    PID_output = P_term + I_term + D_term;
                    
                    // 좌/우 회전 제어를 위해 기본 출력값에 turn_speed를 가감
                    float final_pid_L = PID_output + turn_speed;
                    float final_pid_R = PID_output - turn_speed;

                    Set_Motor_Power_LR(final_pid_L, final_pid_R); 
                }
                else {
                    // 강제 정지 상태: 오차가 쌓여 폭주하지 않도록 모든 적분항과 모터 파워 초기화
                    I_term = 0.0f;
                    vel_I = 0.0f; 
                    PID_output = 0.0f;
                    Set_Motor_Power_LR(0.0f, 0.0f); 
                }

                prev_error = error;
                prev_target_velocity = target_velocity; // 상태 감지용

                // =========================================================
                // 표정(LED 매트릭스) 애니메이션 상태 판별 로직
                // (우선순위가 높은 조건일수록 if문 상단에 배치)
                // =========================================================
                if (fabs(current_angle) > 40.0f) target_face = 3;      // [0순위] 자빠짐 (Dead)
                else if (stop_face == 1)         target_face = 4;      // [1순위] 진입 금지 강제 출력
                else if (target_velocity < -10.0f) target_face = 5;    // [2순위] 후진 중
                else if (turn_speed > 10.0f)     target_face = 6;      // [3순위] 우회전 중
                else if (turn_speed < -10.0f)    target_face = 7;      // [4순위] 좌회전 중
                else if (target_velocity > 10.0f) target_face = 0;     // [5순위] 전진 중 (Happy)         
                else
                {   // [6순위] 대기/정지 상태 (오차값에 따라 흔들림 감지)
                    if (fabs(error) > 8.0f)      target_face = 2;      // 크게 흔들림 (Angry)
                    else if (fabs(error) > 3.0f) target_face = 1;      // 미세 조정 중 (Smile)
                    else                         target_face = 0;      // 완벽한 밸런스 (Happy)
                }

                if (current_face != target_face) face_turn = 0; // 표정 변경 시 프레임 리셋

                // 회전 애니메이션 프레임(스텝) 계산
                int current_step = 0; 
                if(target_face == 6 || target_face == 7) {
                    face_turn++;  // 10ms마다 카운트업
                    if(face_turn >= 60) face_turn = 0; // 0~59 프레임 반복
                    
                    // 구간별로 1, 2, 3 프레임 결정
                    if(face_turn < 20) current_step = 1;
                    else if(face_turn < 40) current_step = 2;
                    else current_step = 3;
                } else {
                    face_turn = 0; 
                }

                // 표정 하드웨어 출력부
                static int prev_step = 0; 

                // 표정 종류가 바뀌었거나, 애니메이션 프레임이 넘어갔을 때만 I2C 통신 실행 (루프 부하 감소)
                if (current_face != target_face || (current_face == target_face && prev_step != current_step)) 
                {
                    current_face = target_face; 
                    prev_step = current_step;   

                    if (target_face == 0) MAX7219_16x32_Show_Happy();  
                    else if (target_face == 1) MAX7219_16x32_Show_Smile();  
                    else if (target_face == 2) MAX7219_16x32_Show_Angry();  
                    else if (target_face == 3) MAX7219_16x32_Show_Dead();  
                    else if (target_face == 4) MAX7219_16x32_Show_Dead();  
                    else if (target_face == 5) MAX7219_16x32_Show_back();  
                    else if (target_face == 6) { 
                        if (current_step == 1) MAX7219_16x32_Show_right1();
                        else if (current_step == 2) MAX7219_16x32_Show_right2();
                        else if (current_step == 3) MAX7219_16x32_Show_right3();
                    }
                    else if (target_face == 7) { 
                        if (current_step == 1) MAX7219_16x32_Show_left1();
                        else if (current_step == 2) MAX7219_16x32_Show_left2();
                        else if (current_step == 3) MAX7219_16x32_Show_left3();
                    }
                }
                
                // 모니터링 출력 (10번에 1번, 10Hz 주기)
                static int print_count = 0;
                print_count++;
                if (print_count >= 10) { 
                    printf("A: %.1f | M: %.0f\n", current_angle, PID_output);
                    printf("T_Vel: %.1f | Cur_Vel: %.1f | Target_Ang: %.2f | Cur_Ang: %.2f\n", 
                            target_velocity, filtered_velocity, target_angle, current_angle);
                    print_count = 0; 
                }
            }
        }
    }       
}
// float 변수가 단 1개도 없는 껍데기 Main 함수
void Main(void)
{
    // FPU(부동소수점 처리기)부터 안전하게 켬 (안 켜면 float 계산 시 뻗어버림)
    Sys_Init(115200);

    Uart2_Send_Byte('O'); Uart2_Send_Byte('K'); Uart2_Send_Byte('\n');
    printf("\n--- System Booting ---\n");
    
    I2C1_Init(); printf(">> I2C Init OK!\n");

    TIM1_PWM_Init();
    TIM4_PWM_Init();
    Bluetooth_Init();

    // 센서 연결 테스트
    uint8_t who_am_i = MPU6050_ReadReg(0x75);
    if (who_am_i == 0x68) printf(">> [SUCCESS] MPU6050 Connection OK! (ID: 0x68)\n");
    else printf(">> [ERROR] MPU6050 Not Found. (ID: 0x%X)\n", who_am_i);

    MPU6050_DMP_Init();  // 하드웨어 자이로 적분기(DMP) 초기화
    MPU6050_INT_Init();  // DMP 인터럽트 핀 활성화
    printf(">> MPU6050 Wake-up OK!\n");

    Encoder_Init();      // 모터 틱 카운터 활성화
    
    MAX7219_16x32_Init();
    MAX7219_16x32_Set_Brightness(0x08); // LED 전력 소모를 줄여 모터로 가는 전류 안정성 확보
    MAX7219_16x32_Show_Happy();

    // 셋업 완료, 밸런싱 루프 진입
    Balancing_Task();
}

// --- 양쪽 모터 공통 제어 함수 ---
void Set_Motor_Power(float pid_output) {
    float abs_pid = fabs(pid_output); 
    
    // 모터 데드존(Deadzone) 보정값 설정
    // 이유: 모터에 전압을 너무 적게 주면 소리만 나고 돌지 않는 마찰 구간이 존재함.
    // PID 결과값이 아주 작아도 이 마찰력 값을 기본으로 깔아주어 즉각 반응하게 만듦.
    int MIN_PWM_L = 25; 
    int MIN_PWM_R = 34; // 우측 모터 효율이 떨어져 마찰 보정값을 더 줌

    int pwm_L = 0, pwm_R = 0;

    // 파워가 0.1 이상일 때만 보정값 적용 (0점 근처 노이즈 구간 제어)
    if (abs_pid > 0.1f) {
        // 소수점 파워에 마찰력을 더한 뒤, 레지스터에 넣기 직전에 int 캐스팅
        pwm_L = (int)(abs_pid + MIN_PWM_L);
        pwm_R = (int)(abs_pid + MIN_PWM_R);
    }

    // 모터 레지스터 최대값 락 (과부하 보호)
    if (pwm_L > 1199) pwm_L = 1199; 
    if (pwm_R > 1199) pwm_R = 1199; 

    // 방향 제어
    if (pid_output > 0.5f) {
        TIM1->CCR1 = 0; TIM1->CCR2 = pwm_L; // L 전진
        TIM4->CCR1 = pwm_R; TIM4->CCR2 = 0; // R 전진
    } 
    else if (pid_output < -0.5f) { 
        TIM1->CCR1 = pwm_L; TIM1->CCR2 = 0; // L 후진
        TIM4->CCR1 = 0; TIM4->CCR2 = pwm_R; // R 후진
    } 
    else {
        // 정지 구간 (H-Bridge 제동)
        TIM4->CCR1 = 0; TIM4->CCR2 = 0;
        TIM1->CCR1 = 0; TIM1->CCR2 = 0;
    } 
}

// --- 좌/우 모터 독립 제어 함수 (회전 기능 구현) ---
void Set_Motor_Power_LR(float pid_L, float pid_R) {
    float abs_L = fabs(pid_L); 
    float abs_R = fabs(pid_R); 

    int MIN_PWM_L = 33; 
    int MIN_PWM_R = 30; 

    int pwm_L = 0, pwm_R = 0;

    // 점진적 데드존 보정 로직
    // 이유: 오차가 작을 때 데드존 파워를 한 번에 주면 덜덜거리는 진동(채터링)이 발생.
    // 0~0.5 사이에서는 마찰력 보정값도 비율에 맞춰 서서히 올려 부드럽게 기동하게 함.
    if (abs_L > 0.5f) {
        pwm_L = (int)(abs_L + MIN_PWM_L); 
    } else {
        pwm_L = (int)(abs_L + (MIN_PWM_L * (abs_L / 0.5f))); 
    }
    
    if (abs_R > 0.5f) {
        pwm_R = (int)(abs_R + MIN_PWM_R);
    } else {
        pwm_R = (int)(abs_R + (MIN_PWM_R * (abs_R / 0.5f))); 
    }

    if (pwm_L > 1199) pwm_L = 1199; 
    if (pwm_R > 1199) pwm_R = 1199; 

    // 왼쪽 모터 구동
    if (pid_L > 0.1f) { 
        TIM1->CCR1 = 0; TIM1->CCR2 = pwm_L; 
    } else if (pid_L < -0.1f) { 
        TIM1->CCR1 = pwm_L; TIM1->CCR2 = 0; 
    } else {
        TIM1->CCR1 = 0; TIM1->CCR2 = 0;     
    }

    // 오른쪽 모터 구동
    if (pid_R > 0.1f) { 
        TIM4->CCR1 = pwm_R; TIM4->CCR2 = 0; 
    } else if (pid_R < -0.1f) { 
        TIM4->CCR1 = 0; TIM4->CCR2 = pwm_R; 
    } else {
        TIM4->CCR1 = 0; TIM4->CCR2 = 0;     
    }
}

// --- 센서 초기화 함수 ---
void MPU6050_DMP_Init(void) {
    printf(">> MPU6050 Init Start...\n");
    if (mpu_init(NULL) != 0) { printf(">> [ERROR] MPU Init Failed!\n"); return; }
    
    // DMP(센서 내부 하드웨어) 계산 주기를 200Hz로 설정
    mpu_set_sample_rate(200); 

    // 하드웨어 로우패스 필터 (LPF) 20Hz 설정
    // 이유: 모터 진동이나 차체 흔들림이 자이로 센서에 그대로 들어오는 것을 막아줌.
    mpu_set_lpf(20); 
    
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    
    if (dmp_load_motion_driver_firmware() != 0) { printf(">> [ERROR] DMP FW Failed!\n"); return; }

    dmp_set_orientation(136);  // 센서 부착 방향 보정 매트릭스 값

    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    
    // MCU로 데이터를 넘겨주는 FIFO 버퍼 갱신 주기를 100Hz(10ms)로 유지 (메인 제어 루프 주기와 동일)
    dmp_set_fifo_rate(100); 
    mpu_set_dmp_state(1);   
    
    printf(">> [SUCCESS] DMP Init OK!\n");
}

// MPU6050 인터럽트 처리기 (데이터가 준비되면 호출됨)
void EXTI15_10_IRQHandler(void) {
    // 1. PB12(EXTI12) 핀의 인터럽트 발생 여부 확인
    if (EXTI->PR & (1 << 12)) {
        
        // 2. 인터럽트 발생 기록 삭제 (무한 루프 방지)
        EXTI->PR = (1 << 12); 
        
        // 3. 메인 루프에 데이터 수신 완료 신호 전송
        mpu_data_ready = 1;
    }
}


// 모터 최소 구동 PWM(데드존) 탐색용 테스트 함수
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

// 엔코더 방향 및 틱 카운트 점검용 테스트 함수
void Encoder_Test_Task(void) {
    printf("\n==================================\n");
    printf("--- Encoder Test Mode Start ---\n");
    printf("바퀴를 앞뒤로 손으로 돌려보세요!\n");
    printf("==================================\n");
    
    // 1. 엔코더 하드웨어 초기화
    Encoder_Init();
    
    // 2. 카운터 0으로 초기화
    TIM5->CNT = 0;
    TIM3->CNT = 0;

    while(1) {
        // [중요 포인트] 
        // TIM5는 32비트라 int에 바로 넣어도 되지만, 
        // TIM3는 16비트 타이머이므로 뒤로 돌릴 때 65535 같은 쓰레기 값이 
        // 나오지 않도록 반드시 short(16비트 부호형)로 캐스팅 후 출력해야 함
        int left_cnt = (int)TIM5->CNT;
        short right_cnt = -(short)TIM3->CNT;
        
        // 값 출력
        printf("Left(TIM5): %6d  |  Right(TIM3): %6d\n", left_cnt, right_cnt);
        
        // 너무 빨리 출력되지 않게 0.1초 딜레이
        TIM2_Delay(100); 
    }
}