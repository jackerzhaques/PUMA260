#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

//Standard includes
#include <stdbool.h>

//Project includes
#include "Globals.h"

//Tivaware includes

void MD_Initialize(void);
void MD_EnableMotor(JOINT_POSITION Joint, bool bEnabled);
void MD_SetMotorDutyCycle(JOINT_POSITION Joint, float DutyCycle);
void MD_SetMotorDirection(JOINT_POSITION Joint, bool Clockwise);
void MD_EnableBrake(bool bEnabled);
float MD_GetMotorCurrent(JOINT_POSITION Joint);

#endif
