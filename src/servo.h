#ifndef __SERVO_H__
#define __SERVO_H__

#define SERVO_MIN_ANGLE  30
#define SERVO_MAX_ANGLE  150
#define SERVO_INIT_ANGLE 90

void Servo_Init(void);
void Servo_Set_Angle(int angle);
int  Servo_Get_Angle(void);

#endif