#include "ControlLoop.h"

//Standard includes
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//Project includes
#include "MotorDriver/MotorDriver.h"
#include "EIB/Encoders.h"

//Tivaware includes
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
#include <utils/uartstdio.h>

/*
 * Inverse Kinematics Defines
 */

#define     L1          13.0    //Length of joint1
#define     L2         -2.5     //Offset from j1 to j2
#define     L3          8.0     //Length of joint2
#define     L4         -2.5     //Offset from j2 to j3
#define     L5          8.0     //Length of joint3
#define     L6          3.0     //Length of joint5

//Forward declarations
void ControlLoopISR(void);
void InitializeTimer1(void);
void FindJointLimits(void);

//PID structs
sPID *SpeedPIDs = NULL;
sPID *PositionPIDs = NULL;

//File global variables
PositionVector RobotArm = {0,0,0,0,0,0};

//Initializes the motors, encoders, and control loop ISR
void InitializeControlLoop(void){
    uint8_t i = 0;

    //Get a reference to the PIDs
    SpeedPIDs = GetSpeedPIDs();
    PositionPIDs = GetPositionPIDs();

    //Initialize motor drivers
    MD_Initialize();

    //Initialize encoders
    Enc_Initialize();

    //Disable the brake for joints 1-3
    MD_EnableBrake(false);

    //Enable each joint's motor driver
    for(i = 0; i < JOINT_COUNT; i++){
        MD_EnableMotor((JOINT_POSITION)(i), true);
    }

    //Initialize control loop timer
    InitializeTimer1();
}

void InitializeTimer1(void){
    //Initialize Timer1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //Wait for the clock to stabilize
    SysCtlDelay(10);

    //Configure the timer to be periodic
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    //Configure the timer to have a 10ms period
    TimerLoadSet(TIMER1_BASE, TIMER_A, CLOCK_FREQ / ARM_CONTROL_LOOP_FREQUENCY);

    //Register the timer ISR
    TimerIntRegister(TIMER1_BASE, TIMER_BOTH, ControlLoopISR);

    //ENable timer interrupts
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //Enable the timer
    TimerEnable(TIMER1_BASE, TIMER_A);

    //Enable master interrupts
    IntMasterEnable();
}

//The order of joints to home
int JointIndexes[] = {1, 2, 0, 4, 3, 5};// Joint 2 -> 3 -> 1 -> 5 -> 4 -> 6

void FindJointLimits(void){
    int i = 0;
    for(i = 0; i < JOINT_COUNT; i++){
        int JointIndex = JointIndexes[i];
        //Skip joints that are not being used
        if(JointIndex == 3  || JointIndex == 5){
            continue;
        }

        JOINT_POSITION Joint = (JOINT_POSITION)(JointIndex);
        PositionPIDs[Joint].Target = -600;
        PositionPIDs[Joint].iState = 0;
        SpeedPIDs[Joint].Target = -CONTROL_SPEED;
        SpeedPIDs[Joint].iState = 0;
        while(MD_GetMotorCurrent(Joint) < 1.5){

        }
        //Reset joint to 0 and wait 1 second
        Enc_ResetEncoder(Joint);
        SetJointAngle(Joint, 0);
        SysCtlDelay(120000000 / 3);

        //Move joint to origin
        SetJointAngle(Joint, -PositionPIDs[Joint].TargetMin);

        //Wait 6 seconds for joint to reach its target
        SysCtlDelay(120000000 * 2);

        //Reset joint degrees to match origin
        Enc_ResetEncoder(Joint);
        SetJointAngle(Joint, 0);
    }
}

void SetJointSpeed(JOINT_POSITION Joint, float Speed){
    if(Speed > SpeedPIDs[Joint].TargetMax){
        Speed = SpeedPIDs[Joint].TargetMax;
    }
    else if(Speed < SpeedPIDs[Joint].TargetMin){
        Speed = SpeedPIDs[Joint].TargetMin;
    }

    if(Speed < 0){
        //SpeedPIDs[Joint].OutputMin = -0.75;
        //SpeedPIDs[Joint].OutputMax = 0;
    }
    else{
        //SpeedPIDs[Joint].OutputMin = 0;
        //SpeedPIDs[Joint].OutputMax = 0.75;
    }

    SpeedPIDs[Joint].Target = Speed;
}

void SetJointAngle(JOINT_POSITION Joint, float Angle){
    if(Angle > PositionPIDs[Joint].TargetMax){
        Angle = PositionPIDs[Joint].TargetMax;
    }
    else if(Angle < PositionPIDs[Joint].TargetMin){
        Angle = PositionPIDs[Joint].TargetMin;
    }

    PositionPIDs[Joint].Target = Angle;
    PositionPIDs[Joint].iState = 0;

    //Set the SpeedPID to travel in the correct direction at 500 ticks per second
    float PositionError = Angle - Enc_GetJointEncoder(Joint)->Degrees;
    if(PositionError < 0){
        SpeedPIDs[Joint].Target = -CONTROL_SPEED;
    }
    else{
        SpeedPIDs[Joint].Target = CONTROL_SPEED;
    }
    SpeedPIDs[Joint].iState = 0;
}

void SetArmPosition(PositionVector Target){
    //See matlab script InverseKinematics for more info
    float L_to_target = sqrt((Target.x * Target.x) + (Target.y * Target.y));
    float L_Offsets = L2 + L4;
    float Theta1 = atan2(Target.y,Target.x) - acosf(L_Offsets/L_to_target);

    float OriginX = (L2 + L4) * cos(Theta1);
    float OriginY = (L2 + L4) * sin(Theta1);
    float Radius = sqrt(pow(Target.x - OriginX,2) + pow(Target.y - OriginY, 2));

    float dx = cos(Target.theta * M_PI/180) * L6;
    float dz = sin(Target.theta * M_PI/180) * L6;

    float x = Radius - dx;
    float y = (Target.z - dz) - L1;

    float c2 = (pow(x,2) + pow(y,2) - pow(L3,2) - pow(L5,2)) / (2*L3*L5);
    float s2 = -sqrt(1 - pow(c2,2));    //Changing this sign to negative will make Joint 2 angle upwards. Positive is downwards
    float Theta3 = atan2(s2,c2);

    float k1 = L3 + L5*c2;
    float k2 = L5 * s2;
    float Theta2 = atan2(y,x) - atan2(k2,k1);
    float Theta3_ToX = Theta2 + Theta3;
    float Theta4 = Target.theta - Theta3_ToX;

    //Convert all thetas to degrees
    Theta1 *= 180/M_PI;
    Theta2 *= 180/M_PI;
    Theta3 *= 180/M_PI;
    Theta4 *= 180/M_PI;

    //Assign all angles
    SetJointAngle(JOINT1, Theta1);
    SetJointAngle(JOINT2, Theta2);
    SetJointAngle(JOINT3, -Theta3); //These two joints are reversed;
    SetJointAngle(JOINT5, -Theta4);
}

PositionVector* GetArmPosition(void){
    return 0;
}

void ControlLoopISR(void){

    uint8_t i;

    //Update speed PID
    for(i = 0; i < JOINT_COUNT; i++){
        JOINT_POSITION Joint = (JOINT_POSITION)(i);
        sPID* SpeedPID = &SpeedPIDs[i];
        sPID* PositionPID = &PositionPIDs[i];
        sEncoder *Enc = Enc_GetJointEncoder(Joint);

        float Error = SpeedPID->Target - Enc->Degrees;
        float P = SpeedPID->Kp * Error;
        float I = SpeedPID->iState + SpeedPID->Ki * Error;
        float D = SpeedPID->Kd * (Error - SpeedPID->dState);

        if(I < SpeedPID->iMin){
            I = SpeedPID->iMin;
        }
        else if(I > SpeedPID->iMax){
            I = SpeedPID->iMax;
        }
        SpeedPID->iState = I;
        SpeedPID->dState = Error;

        float Output = P + I + D;

        if(fabs(Output) < SpeedPID->Threshold){
            Output = 0;
        }

        //Add the DC offset in the direction we are travelling in
        if(Output < 0){
            Output -= SpeedPID->DcBias;
        }
        else if(Output > 0){
            Output += SpeedPID->DcBias;
        }
        else{
            //0 when Output is 0
        }

        if(Output > SpeedPID->OutputMax){
            Output = SpeedPID->OutputMax;
        }
        else if(Output < SpeedPID->OutputMin){
            Output = SpeedPID->OutputMin;
        }

        SpeedPID->Output = Output;

        MD_SetMotorDutyCycle(Joint, fabs(Output));

        if(Output < 0){
            MD_SetMotorDirection(Joint, false);
        }
        else if(Output > 0){
            MD_SetMotorDirection(Joint, true);
        }
    }

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}
