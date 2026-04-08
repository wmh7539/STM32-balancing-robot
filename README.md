# STM32-balancing-robot
A self-balancing robot developed with STM32 and embedded control

## 📌프로젝트 개요

이 프로젝트는 STM32 NUCLEO-F411RE를 기반으로 제작한 2륜 밸런싱 로봇입니다.  
MPU6050 센서를 이용해 로봇의 기울기와 움직임을 측정하고, PID 제어를 통해 모터를 제어하여 균형을 유지합니다.  
또한 MAX7219 도트매트릭스를 사용해 로봇의 표정이나 상태를 표시하고, HC-06 블루투스 모듈을 통해 무선 통신 기능을 확장할 수 있도록 설계하였습니다.


## 🔧하드웨어 구성 요소

| 부품 | 모델명 | 역할 |
|---|---|---|
| 마이크로컨트롤러 보드 | STM32 NUCLEO-F411RE | 밸런싱 로봇의 전체 동작을 제어하는 메인 보드 |
| 모터 | JGA25-371 | 로봇 바퀴를 구동하고 균형 제어를 수행 |
| 모터 드라이버 | MDD3A | 좌우 DC 모터 2개의 속도와 방향을 제어 |
| IMU 센서 | MPU6050 | 기울기, 가속도, 각속도 측정 |
| DC-DC 컨버터 | LM2596 | 배터리 전압을 안정적으로 변환 |
| 도트매트릭스 | MAX7219 × 2 | 표정 및 상태 표시 |
| 블루투스 모듈 | HC-06 | 외부 기기와 무선 통신 |
| 배터리 | Battery Pack | 시스템 전체 전원 공급 |

## ✨주요 기능

- MPU6050을 이용한 기울기 및 자세 측정
- PID 제어를 통한 실시간 밸런싱
- MDD3A를 이용한 좌우 모터 속도 및 방향 제어
- MAX7219 도트매트릭스를 이용한 표정 출력
- HC-06을 이용한 블루투스 통신 확장 가능


## 👥팀원 소개

<table>
  <tr>
    <td align="center" width="25%">
      <a href="https://github.com/KKH2007">
        <img src="https://github.com/KKH2007.png" width="120px;" alt="KKH2007"/>
        <br />
        <sub><b>KKH2007</b></sub>
      </a>
      <br />
      김광현
      <br />
    </td>
    <td align="center" width="25%">
      <a href="https://github.com/JH010918">
        <img src="https://github.com/JH010918.png" width="120px;" alt="JH010918"/>
        <br />
        <sub><b>JH010918</b></sub>
      </a>
      <br />
      명지훈
      <br />
    </td>
    <td align="center" width="25%">
      <a href="https://github.com/wmh7539">
        <img src="https://github.com/wmh7539.png" width="120px;" alt="wmh7539"/>
        <br />
        <sub><b>wmh7539</b></sub>
      </a>
      <br />
      위오현
      <br />
    </td>
    <td align="center" width="25%">
      <a href="https://github.com/chiya0123">
        <img src="https://github.com/chiya0123.png" width="120px;" alt="chiya0123"/>
        <br />
        <sub><b>chiya0123</b></sub>
      </a>
      <br />
      최민지
      <br />
    </td>
  </tr>
</table>
