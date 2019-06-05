//Project includes
#include "Encoders.h"
#include "EncoderInterface.h"

//Tivaware includes
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
#include <utils/uartstdio.h>
#include <driverlib/uart.h>
#include <stdio.h>
#include <math.h>

//Forward declarations
void InitializeTimer0(void);

//File global variables
static sEncoder Encoders[JOINT_COUNT] = {
     {0,0,0,false,0,0,0,0},
     {0,0,0,false,0,0,0,0},
     {0,0,0,false,0,0,0,0},
     {0,0,0,false,0,0,0,0},
     {0,0,0,false,0,0,0,0},
     {0,0,0,false,0,0,0,0}
};

static float TicksToDegrees[JOINT_COUNT] = {
     0.0071005,     //40842 ticks over 290 degrees
     0.0050458,     //62428 ticks over 315 degrees
     0.0082716,     //35664 ticks over 295 degrees
     0,             //Unknown at 4-12-19
     0.0094661,     //21128 ticks over 200 degrees
     0              //Unknown at 4-12-19
};

void printfloat(char *Buffer, float val, int nDecimals){
    int UpperVal = (int)(val);
    int LowerVal = (uint32_t)(val * pow(10,nDecimals));
    sprintf(Buffer, "%i.%u", UpperVal, LowerVal);
}

void UpdateEncoders(void){
    uint8_t i = 0;
    int32_t JointValues[JOINT_COUNT] = {0};

    //Read all joint values
    //We want to read the joint values as fast as possible,
    // so this must run before any math is done
    for(i = 0; i < JOINT_COUNT; i++){
        EncoderDeviceSelect Encoder = (EncoderDeviceSelect)(i);
        JointValues[i] = EI_ReadEncoderValue(Encoder);

        //Calculate the speed in ticks per second
        float EncoderSpeed = SAMPLE_RATE * (JointValues[i] - Encoders[i].EncoderCount);

        //Use a simple IIR filter on the speed
        float FilteredSpeed = (FILTER_WEIGHT * Encoders[i].Speed)
                + ((1 - FILTER_WEIGHT) * EncoderSpeed);

        float Degrees = JointValues[i] * TicksToDegrees[i];

        Encoders[i].EncoderCount = JointValues[i];
        Encoders[i].Speed = FilteredSpeed;
        Encoders[i].Degrees = Degrees;
    }

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void Enc_Initialize(void){
    //Initialize the Encoder Interface
    EI_Initialize();

    //Initialize Timer0 to be used to update the encoder count and measure speed
    InitializeTimer0();
}

void InitializeTimer0(void){
    //Initialize Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //Wait for the clock to stabilize
    SysCtlDelay(10);

    //Configure the timer to be periodic
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //Configure the timer to have a 1ms period
    TimerLoadSet(TIMER0_BASE, TIMER_A, CLOCK_FREQ / SAMPLE_RATE);

    //Register the timer ISR
    TimerIntRegister(TIMER0_BASE, TIMER_BOTH, UpdateEncoders);

    //ENable timer interrupts
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //Enable the timer
    TimerEnable(TIMER0_BASE, TIMER_A);

    //Enable master interrupts
    IntMasterEnable();
}

void Enc_ResetEncoder(JOINT_POSITION Joint){
    Encoders[Joint].EncoderCount = 0;
    Encoders[Joint].Degrees = 0;
    Encoders[Joint].Speed = 0;
    Encoders[Joint].NotMoving = false;
    EI_ClearEncoder(Joint);
}

sEncoder* Enc_GetJointEncoder(JOINT_POSITION Joint){
    return &Encoders[Joint];
}
