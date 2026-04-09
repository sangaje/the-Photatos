#ifndef __STEPPER_H__
#define __STEPPER_H__

void Stepper_Init(void);
void Stepper_Move_Relative(int steps);
void Stepper_Release(void);

#endif