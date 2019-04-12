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

//Forward declarations
void ControlLoopISR(void);
void InitializeTimer1(void);

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

    //Initialize Timer1
    InitializeTimer1();

    //Disable the brake for joints 1-3
    MD_EnableBrake(false);

    //Enable each joint's motor driver
    for(i = 0; i < JOINT_COUNT; i++){
        MD_EnableMotor((JOINT_POSITION)(i), true);
    }
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
}

void SetArmPosition(PositionVector Vector){

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

        /*
         * Position
         */
        float PositionError = PositionPID->Target - Enc->Degrees;
        float PositionProportional = PositionPID->Kp * PositionError;
        PositionPID->iState += PositionError * PositionPID->Ki;
        float PositionDerivative = (PositionError - PositionPID->dState) * PositionPID->Kd;
        PositionPID->dState = PositionError;

        PositionPID->Output = PositionProportional + PositionDerivative + PositionPID->iState;

        if(PositionPID->Output > PositionPID->OutputMax){
            PositionPID->Output = PositionPID->OutputMax;
        }
        else if(PositionPID->Output < PositionPID->OutputMin){
            PositionPID->Output = PositionPID->OutputMin;
        }

        SpeedPID->Target = PositionPID->Output;

        /*
         * Speed
         */

        //Calculate Error
        float Error = SpeedPID->Target - Enc->Speed;

        //Calculate P and I terms
        float P = Error * SpeedPID->Kp;

        SpeedPID->iState += Error * SpeedPID->Ki;
        if(SpeedPID->iState > SpeedPID->iMax){
            SpeedPID->iState = SpeedPID->iMax;
        }
        else if(SpeedPID->iState < SpeedPID->iMin){
            SpeedPID->iState = SpeedPID->iMin;
        }

        float I = SpeedPID->iState;

        float D = (Error - SpeedPID->dState) * SpeedPID->Kd;
        SpeedPID->dState = Error;

        float Output = P + I + D;
        if(Output > SpeedPID->OutputMax){
            Output = SpeedPID->OutputMax;
        }
        else if(Output < SpeedPID->OutputMin){
            Output = SpeedPID->OutputMin;
        }

        SpeedPID->Output = Output;

        //Set the motor speed
        if(SpeedPID->Output < 0){
            MD_SetMotorDirection(Joint, false);
        }
        else{
            MD_SetMotorDirection(Joint, true);
        }

        MD_SetMotorDutyCycle(Joint, fabs(SpeedPID->Output));
    }

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}
