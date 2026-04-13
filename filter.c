#include "device_driver.h"
#include <math.h>
#define PI 3.14159265358979323846

// 이전 각도를 기억해야 하므로 전역 변수로 선언합니다.
float pitch_angle = 0.0;

// 상보 필터를 적용하여 최종 기울기 각도를 계산하는 함수
float MPU6050_Get_Angle(MPU6050_Data *sensor, float dt) {
    // 1. 가속도 센서로 '절대 각도' 계산
    // 로봇이 앞뒤로 넘어지는 축이 X축이라고 가정 (조립 방향에 따라 Y로 바뀔 수 있음)
    float accel_pitch = atan2(sensor->Accel_X, sensor->Accel_Z) * 180.0 / PI;

    // 2. 자이로 센서로 '회전 속도(dps)' 계산
    // MPU6050 자이로의 기본 감도는 131 LSB/(deg/s) 입니다.
    float gyro_rate = sensor->Gyro_Y / 131.0;

    // 3. 상보 필터 수식 적용
    // (자이로가 계산한 현재 각도 * 0.96) + (가속도가 측정한 절대 각도 * 0.04)
    pitch_angle = 0.96 * (pitch_angle + gyro_rate * dt) + 0.04 * accel_pitch;

    return pitch_angle;
}