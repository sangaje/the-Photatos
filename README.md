# the-Photatos

## 1. 프로젝트 개요

**the-Photatos**는 
STM32 보드와 IR 화염 센서를 이용해 **화염의 위치를 탐지하고 자동으로 소화하는 자율 소화 시스템**입니다.  
센서 데이터를 실시간으로 수집하고, 스테퍼 모터/서보 모터를 이용해 화염 방향으로 분사 장치를 조준한 뒤, 워터 펌프를 작동시켜 초기 화재를 자동으로 진압하는 것을 목표로 합니다.

---

## 2. 프로젝트 목표

- 다중 IR 센서를 이용한 화염 방향 추정
- STM32 기반 실시간 임베디드 제어 시스템 구현
- 팬/틸트(Pan-Tilt) 구조를 통한 자동 조준
- 워터 펌프 자동 제어를 통한 자율 소화
- Python 기반 시리얼 모니터링 도구를 이용한 상태 시각화

---

## 3. 시스템 구성

```text
[ Flame ]
   ↓
[ IR Flame Sensors ]
   ↓
[ ADC + DMA Sampling ]
   ↓
[ Signal Processing / Estimation ]
   ↓
[ Direction Calculation ]
   ↓
[ Stepper + Servo Motor Control ]
   ↓
[ Water Pump Activation ]
```

---

## 4. 주요 기능

### 4.1 화염 탐지
- 4채널 IR 화염 센서 사용
- ADC + DMA 기반 연속 샘플링
- 센서 값 필터링 및 신호 보정

### 4.2 화염 위치 추정
- 다중 센서 입력을 기반으로 화염 방향 계산
- 선형화 및 필터링 적용
- EMA / Kalman 기반 추정 로직 포함

### 4.3 자동 조준
- **Stepper Motor**: 좌우(Pan) 회전
- **Servo Motor**: 상하(Tilt) 각도 제어
- 추정된 화염 방향으로 분사 장치 자동 정렬

### 4.4 자동 소화
- 워터 펌프 ON/OFF 제어
- 화염 감지 상태에 따라 분사 수행
- 소화 완료 후 시스템 복귀 가능

### 4.5 실시간 모니터링
- UART를 통해 센서 데이터 전송
- Python 스크립트로 실시간 그래프 확인
- 실험 데이터 분석 및 튜닝 지원

---

## 5. 사용 기술

### Firmware / Embedded
- **STM32F411xE**
- **C language**
- ADC / DMA
- Timer Interrupt
- UART
- PWM
- GPIO Control

### Analysis / Visualization
- **Python**
- **matplotlib**
- **pyserial**
- **Jupyter Notebook**

---

## 6. 디렉토리 구조

```text
src/         STM32 펌웨어 코드
analysis/    Python 기반 실시간 모니터링 및 데이터 분석
README.md    프로젝트 설명 문서
location.py  초기 실험용 위치 계산 코드
```

---

## 7. 빌드 및 실행

### 7.1 펌웨어 빌드
```bash
cd src
make
```

### 7.2 실시간 모니터 실행
```bash
cd analysis
python live_monitor.py
```

---

## 8. 기대 효과

- 초기 화재 대응 자동화 시스템 프로토타입 구현
- 임베디드 제어, 센서 신호 처리, 모터 제어를 통합한 시스템 설계 경험 확보
- 실시간 데이터 기반 제어 알고리즘 검증 가능

---

## 9. 향후 개선 방향

- 화염 위치 추정 정확도 향상
- 서보/스테퍼 제어 정밀도 개선
- 소화 성공 여부 판단 알고리즘 추가
- 카메라/열화상 센서와 융합한 고도화
- 소형 자율 소화 로봇 플랫폼으로 확장
