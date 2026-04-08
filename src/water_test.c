#include "device_driver.h"
#include "water_sensor.h"
#include "buzzer.h"
#include "pump.h"
#include "led.h"

/* --- Button Configuration --- */
#define BTN_PORT             GPIOC           // PC
#define BTN_PIN              13              // Pin 13
#define IS_BTN_PRESSED       (Macro_Check_Bit_Set(BTN_PORT->IDR, BTN_PIN) == 0) // Press = 0

// int Main(void) {
//     // 1. 모든 하드웨어 드라이버 초기화
//     Pump_Init();        // PC4
//     WaterSensor_Init(); // PC3 (ADC CH13)
//     Buzzer_Init();      // PC0
//     LED_Init();         // PC1, PC2

//     // 2. 사용자 버튼 초기화 (PC13 입력 모드)
//     Macro_Set_Bit(RCC->AHB1ENR, 2); // Port C 클록 활성화 (이미 되어있겠지만 명시적 처리)
//     Macro_Write_Block(BTN_PORT->MODER, 0x3, 0x0, (BTN_PIN * 2)); 

//     unsigned char pump_status = 0;      // 0: OFF, 1: ON
//     unsigned char prev_btn_state = 0;   // 버튼 엣지 검출용
//     unsigned char water_error_flag = 0; // 물 부족 경고 중복 실행 방지용

//     while (1) {
//         // --- [로직 1 & 2] 수위 센서 및 LED/부저 제어 ---
//         int water_level = WaterSensor_Read();

//         if (water_level >= WATER_DETECTED) {
//             // [조건 1] 물이 감지된 경우
//             LED_Green_On();
//             LED_Red_Off();
//             water_error_flag = 0; // 물이 다시 찼으므로 플래그 리셋
//         } 
//         else if (water_level < WATER_EMPTY) {
//             // [조건 2] 물이 감지되지 않는 경우
//             LED_Green_Off();
//             LED_Red_On();

//             // 처음 물 부족이 감지된 순간에만 부저를 3초간 울림
//             if (water_error_flag == 0) {
//                 Buzzer_On();
//                 Pump_Delay(3000); // 3초 대기 (소프트웨어 지연)
//                 Buzzer_Off();
//                 water_error_flag = 1; // 다시 물이 차기 전까지는 부저를 울리지 않음
//             }
//         }

//         // --- [로직 3] 워터펌프 버튼 토글 제어 ---
//         if (IS_BTN_PRESSED && prev_btn_state == 0) {
//             pump_status = !pump_status; // 상태 반전
            
//             if (pump_status) {
//                 Pump_On();
//             } else {
//                 Pump_Off();
//             }
            
//             // 버튼 디바운싱
//             Pump_Delay(200); 
//         }

//         // 현재 버튼 상태 저장
//         prev_btn_state = IS_BTN_PRESSED;

//         // 시스템 루프 지연 (반응성 유지)
//         Pump_Delay(10); 
//     }
// }