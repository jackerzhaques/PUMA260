//Standard includes
#include <stdbool.h>
#include <stdint.h>

//Project includes
#include "pinout.h"
#include "EIB/Encoders.h"
#include "MotorDriver/MotorDriver.h"
#include "ControlLoop/ControlLoop.h"

//Tivaware includes
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"

//Testing
#include <math.h>
float angle = 0;

void EnableClock(void);
void EnablePeripherals(void);
void InitConsole(void);

void printfloat(char *Buffer, float val, int nDecimals){
    int UpperVal = (int)(val);
    int LowerVal = (uint32_t)(val * pow(10,nDecimals));
    sprintf(Buffer, "%i.%u", UpperVal, LowerVal);
}

void Wait(float seconds, float Target){
    static uint32_t SampleInterval = 120000000 / 10 / 3;
    uint32_t DelayAmount = 120000000.0 * seconds / 3;
    uint32_t nDelays = DelayAmount / SampleInterval;
    uint32_t i = 0;
    for(i = 0; i < nDelays; i++){
        float EncVal = Enc_GetJointEncoder(JOINT3)->Degrees;
        UARTCharPut(UART0_BASE, 0x02);
        char Buffer[30];
        printfloat(Buffer, EncVal, 2);
        UARTprintf("1,");
        UARTprintf(Buffer);
        UARTCharPut(UART0_BASE, 0x0A);

        UARTCharPut(UART0_BASE, 0x02);
        printfloat(Buffer, Target, 2);
        UARTprintf("2,");
        UARTprintf(Buffer);
        UARTCharPut(UART0_BASE, 0x0A);

        SysCtlDelay(SampleInterval);
    }
}

int main(void)
{
    uint8_t i = 0;
    EnableClock();
    EnablePeripherals();

    float Targets[] = {-10, -80};
    float Delay[]   = {6, 6};
    int nTargets = sizeof(Targets)/sizeof(float);

    UARTCharPut(UART0_BASE, 0x03);
    UARTprintf("AA,FF,33");
    UARTCharPut(UART0_BASE, 0x0A);

    UARTCharPut(UART0_BASE, 0x01);
    UARTprintf("1,Position");
    UARTCharPut(UART0_BASE, 0x0A);
    UARTCharPut(UART0_BASE, 0x01);
    UARTprintf("2,Target");
    UARTCharPut(UART0_BASE, 0x0A);

    float XVals[] = {-10, -10, -10, -10, -10, -10};
    float YVals[] = {0,    2,  4,  6,  8,  10};
    float ZVals[] = {5,    5,  5,  5,  5,  5};
    float TVals[] = {0,    0,  0,  0,  0,  0};
    int nVals = sizeof(XVals)/sizeof(float);



    while(1){
        for(i = 0; i < nVals; i++){
            PositionVector Target;
            Target.x = XVals[i];
            Target.y = YVals[i];
            Target.z = ZVals[i];
            Target.theta = TVals[i];
            SetArmPosition(Target);
            if(i == 0){
                SysCtlDelay(120000000 / 2);
            }
            else{
                SysCtlDelay(120000000 / 6);
            }
        }
    }

    /*
    uint32_t i;
    for(i = 0; i < 1000000; i++){
        angle = (double)i / 200;
        out = sin(angle) * 1000;
        SetJointSpeed(JOINT1, out);
        SysCtlDelay(100000);
    }
    MD_EnableMotor(JOINT1, false);
*/
    /*
    //Enable the motor
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);
    //Enable PWM for PF1 (M0)
    //Set the PWM clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Enable the PWM peripheral and wait
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //Wait for the clock to stabilize
    SysCtlDelay(1);

    //Conifgure PWM0 to count up/down without synchronization
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    //PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    //PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the PWM frequency to 250Hz
    //N = (1 / f) * SysClk, where N is the function parameter and f is the frequency
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 64000);
    //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 64000);
    //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 64000);
    //Set the duty cycle to 25%
    //Duty cycle is a function of the period, use PWMGenPeriodGet()
    //For a frequency of 25%, Calculation is N * 25%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0.0*PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0.25*PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2));
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) / 4);
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) / 4);
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 4);

    //Enable the PWM Bit3(PF3) signal
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);

    //Enable the PWM generator block
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
*/

    while(1)
    {
        //MD_EnableMotor(JOINT1, true);
        //MD_SetMotorDirection(JOINT1, true);
        //MD_SetMotorDutyCycle(JOINT1, 0.1);
    }
#if 0
        uint32_t ui32ReceivedEncoderTicks;

#ifdef TEST_MOTORS
        //Disengage the brake
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_PIN_3);
#endif

#ifdef TEST_GPIO_OUTPUT
        /*
         * Encoder Interface Board Low
         */
        //EIB Slave Select Low
        GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, 0);   //SS2
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0);   //SS0
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0);   //SS1
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);   //SS Clk

        //EIB DFLAG Low
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);   //DFLAG CLK

        //EIB LED
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, 0);

        /*
         * Encoder Interface Board High
         */

        //EIB Slave Select High
        GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, GPIO_PIN_0);
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);

        //EIB DFLAG High
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);   //DFLAG CLK

        //EIB LED
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_PIN_0);
#endif
    }
#endif
}

void EnableInvert(void);

void EnableClock(void){
    //Run the PLL at 120MHz
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                      SYSCTL_OSC_MAIN |
                      SYSCTL_USE_PLL |
                      SYSCTL_CFG_VCO_480), 120000000);
}

/*
    Enables all peripherals needed for this motor driver test
*/
void EnablePeripherals(void){
    //Use the pinout code generated by TI PinMux.

    //Enable the SPI Peripheral and wait for the clock to settle
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    InitConsole();
    PinoutSet();
    //MD_Initialize();
    InitializeControlLoop();
}

//Initializes UART0 to be used as a console.
void InitConsole(void){
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}
