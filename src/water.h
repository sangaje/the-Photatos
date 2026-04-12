#ifndef WATER_H
#define WATER_H

// ─────────────────────────────────────────
//  화재 판단 임계값 (fire_vector.intensity 기준)
//  intensity <= FIRE_THRESHOLD : 화재 감지
//  intensity >= SAFE_THRESHOLD : 안전 (화재 진압 완료)
//  FIRE_THRESHOLD < intensity < SAFE_THRESHOLD : 히스테리시스 구간 (이전 상태 유지)
// ─────────────────────────────────────────
#define FIRE_THRESHOLD      100.0f
#define SAFE_THRESHOLD      80.0f

// ─────────────────────────────────────────
//  화재 상태 열거형
//  STATE_SAFE: 정상 상태
//  STATE_FIRE: 화재 감지 상태
// ─────────────────────────────────────────
typedef enum {
    STATE_SAFE = 0,
    STATE_FIRE = 1
} FireState;

// ─────────────────────────────────────────
//  함수 선언
// ─────────────────────────────────────────

/**
 * @brief 안전 상태 설정 함수
 * - 초록LED ON / 빨간LED OFF
 * - 부저 OFF
 * - 워터펌프 OFF
 */
void Set_Safe_State(void);

/**
 * @brief 화재 상태 설정 함수
 * - 초록LED OFF / 빨간LED ON
 * - 부저 ON
 * - 워터펌프 ON
 */
void Set_Fire_State(void);

/**
 * @brief 화재 상태 업데이트 (히스테리시스 적용)
 * @param current_state 현재 화재 상태 (STATE_SAFE 또는 STATE_FIRE)
 * @param intensity 플레임 센서의 intensity 값
 * @return 업데이트된 화재 상태 (STATE_SAFE 또는 STATE_FIRE)
 */
FireState Update_Fire_State(FireState current_state, float intensity);

#endif // WATER_H