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

//File global variables
PositionVector RobotArm = {0,0,0,0,0,0};

//Initializes the motors, encoders, and control loop ISR
void InitializeControlLoop(void){
    uint8_t i = 0;

    //Get a reference to the PIDs
    SpeedPIDs = GetSpeedPIDs();

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
    //TODO: Implement Angles
    //AnglePIDs[Joint].Target = Angle;
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
        sPID* PID = &SpeedPIDs[i];
        sEncoder *Enc = Enc_GetJointEncoder(Joint);

        //Bound the target if necessary
        if(PID->Target > PID->TargetMax){
            PID->Target = PID->TargetMax;
        }
        else if(PID->Target < PID->TargetMin){
            PID->Target = PID->TargetMin;
        }

        //Calculate Error
        float Error = PID->Target - Enc->Speed;

        //Calculate P and I terms
        float P = Error * PID->Kp;

        PID->iState += Error * PID->Ki;
        if(PID->iState > PID->iMax){
            PID->iState = PID->iMax;
        }
        else if(PID->iState < PID->iMin){
            PID->iState = PID->iMin;
        }

        float I = PID->iState;

        float D = (Error - PID->dState) * PID->Kd;
        PID->dState = Error;

        float Output = P + I + D;
        if(Output > PID->OutputMax){
            Output = PID->OutputMax;
        }
        else if(Output < PID->OutputMin){
            Output = PID->OutputMin;
        }

        PID->Output = Output;

        //Set the motor speed
        if(PID->Output < 0){
            MD_SetMotorDirection(Joint, false);
        }
        else{
            MD_SetMotorDirection(Joint, true);
        }

        MD_SetMotorDutyCycle(Joint, fabs(PID->Output));
    }

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}
