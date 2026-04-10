#include "water.h"
#include "led.h"
#include "buzzer.h"
#include "pump.h"
#include <stdio.h>

// ─────────────────────────────────────────
//  안전 상태 설정 함수
//  intensity >= 80 일 때 호출
//  - 초록LED ON  / 빨간LED OFF  (독립 동작)
//  - 부저 OFF                   (독립 동작)
//  - 워터펌프 OFF               (독립 동작)
// ─────────────────────────────────────────
void Set_Safe_State(void)
{
    LED_Green_On();
    Buzzer_Off();
    // Pump_Off();
    printf("[SAFE] 초록LED ON / 빨간LED OFF / 부저 OFF / 펌프 OFF\n");
}

// ─────────────────────────────────────────
//  화재 상태 설정 함수
//  intensity <= 40 일 때 호출
//  - 초록LED OFF / 빨간LED ON   (독립 동작)
//  - 부저 ON                    (독립 동작)
//  - 워터펌프 ON                (독립 동작)
// ─────────────────────────────────────────
void Set_Fire_State(void)
{
    LED_Red_On();
    Buzzer_On();
    // Pump_On();
    printf("[FIRE] 초록LED OFF / 빨간LED ON / 부저 ON / 펌프 ON\n");
}

// ─────────────────────────────────────────
//  화재 상태 업데이트 (히스테리시스 적용)
//  
//  안전 상태에서 intensity가 FIRE_THRESHOLD 이하로 내려가면 화재로 전환
//  화재 상태에서 intensity가 SAFE_THRESHOLD 이상으로 올라가면 안전으로 전환
//  FIRE_THRESHOLD < intensity < SAFE_THRESHOLD : 히스테리시스 구간 (이전 상태 유지)
// ─────────────────────────────────────────
FireState Update_Fire_State(FireState current_state, float intensity)
{
    FireState new_state = current_state;

    // 안전 상태에서 intensity가 FIRE_THRESHOLD 이하로 내려가면 화재로 전환
    if (current_state == STATE_SAFE && intensity <= FIRE_THRESHOLD)
    {
        printf("[STATE CHANGE] SAFE -> FIRE (Intensity=%.4f)\n", intensity);
        new_state = STATE_FIRE;
        Set_Fire_State();
    }
    // 화재 상태에서 intensity가 SAFE_THRESHOLD 이상으로 올라가면 안전으로 전환
    // (진압 완료 또는 화재 소멸)
    else if (current_state == STATE_FIRE && intensity >= SAFE_THRESHOLD)
    {
        new_state = STATE_SAFE;
        Set_Safe_State();
    }
    // FIRE_THRESHOLD < intensity < SAFE_THRESHOLD : 히스테리시스 구간 → 이전 상태 유지 (아무것도 안함)

    return new_state;
}