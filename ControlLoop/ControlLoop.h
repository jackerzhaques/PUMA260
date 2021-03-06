#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "Globals.h"
#include "ControlGlobals.h"

#define ARM_CONTROL_LOOP_FREQUENCY  200
#define CONTROL_SWITCH_THRESHOLD    20      //The error in degrees for when to switch from Speed controller to Position controller
#define CONTROL_SPEED               1000

void InitializeControlLoop(void);

//Speed control functions
void SetJointSpeed(JOINT_POSITION Joint, float Speed);

//Angle control functions
void SetJointAngle(JOINT_POSITION Joint, float Angle);
float GetJointAngle(JOINT_POSITION Joint);

//Position control functions
void SetArmPosition(PositionVector Target);
PositionVector* GetArmPosition(void);

#endif
