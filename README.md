# the-Photatos

> **STM32 기반 자율 소화 로봇 프로젝트**  
> 4채널 IR 화염 센서로 화염 방향을 추정하고, 스테퍼/서보 모터로 조준한 뒤 워터 펌프를 자동 제어합니다.

---

## 한눈에 보기

- **프로젝트 유형:** Embedded Systems / Robotics / Control
- **핵심 보드:** STM32F411xE
- **주요 언어:** C, Python, Jupyter Notebook
- **핵심 기능:** 화염 감지, 방향 벡터 추정, 자동 조준, 자동 소화, 실시간 모니터링

---

## 주요 기능

- **4채널 IR 화염 감지**  
  ADC + DMA 기반으로 센서 값을 연속 수집합니다.

- **화염 방향 추정**  
  선형화된 센서 데이터와 칼만 필터를 이용해 화염 방향 벡터를 계산합니다.

- **자동 조준 제어**  
  스테퍼 모터(Pan)와 서보 모터(Tilt)를 사용해 화염 방향으로 헤드를 이동합니다.

- **자동 워터 펌프 제어**  
  화염이 안정적으로 감지되면 펌프를 켜고, 소화가 감지되면 자동으로 종료���니다.

- **실시간 PC 모니터링**  
  Python 기반 시리얼 모니터링 도구로 센서 값과 방향 벡터를 시각화합니다.

---

## 사용 기술

### Firmware
- Bare-metal C
- STM32F411xE
- ADC / DMA
- Timer Interrupt
- PWM Servo Control
- Stepper Motor Control
- UART DMA Telemetry

### Analysis & Monitoring
- Python
- matplotlib
- pyserial
- NumPy
- Jupyter Notebook

---

## 시스템 구성

```text
IR Flame Sensors
   ↓
ADC + DMA Sampling
   ↓
Signal Processing / Kalman Filter
   ↓
Fire Direction Vector Estimation
   ↓
Pan/Tilt Motor Control
   ↓
Pump Auto Control
   ↓
UART Telemetry → Python Live Monitor
```

---

## 프로젝트 구조

```text
src/         STM32 펌웨어 소스
analysis/    Python 모니터링 및 실험 데이터 분석
README.md    프로젝트 소개
location.py  초기 실험용 코드
```

---

## 실행 / 빌드

### Firmware 빌드
```bash
cd src
make
```

### 펌웨어 플래시
```bash
cd src
make flash
```

### 실시간 모니터 실행
```bash
cd analysis
python live_monitor.py
```

> `src/Makefile`은 ARM GNU Toolchain과 STM32CubeProgrammer CLI 환경을 기준으로 작성되어 있습니다.

---

## 포트폴리오 포인트

이 프��젝트에서는 아래 역량을 보여줍니다.

- **임베디드 펌웨어 설계**
- **실시간 센서 데이터 처리**
- **제어 로직 및 상태 머신 구현**
- **STM32 주변장치 직접 제어 (ADC, DMA, TIM, UART, PWM)**
- **Python 기반 시각화 및 실험 분석**

---

## 대표 구현 파일

- `src/main.c` — 메인 루프, 모터 제어 ISR, 텔레메트리 출력
- `src/flame.c` — 화염 센서 처리, 선형화, 칼만 필터, 방향 벡터 계산
- `src/stepper.c` — 스테퍼 모터 제어
- `src/servo.c` — 서보 PWM 제어
- `src/pump.c` — 자동 펌프 제어 로직
- `analysis/live_monitor.py` — 실시간 시각화 도구

---

## 한 줄 소개

**화염을 감지하고 방향을 추정해 자동으로 조준·소화하는 STM32 기반 자율 소화 로봇입니다.**
