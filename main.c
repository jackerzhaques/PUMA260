//Standard includes
#include <stdbool.h>
#include <stdint.h>

//Project includes
#include "pinout.h"
#include "EIB/Encoders.h"
#include "MotorDriver/MotorDriver.h"
#include "ControlLoop/ControlLoop.h"
#include "EIB/Encoders.h"
#include "Mailbox/MessageSubscriber.h"
#include "Tasks/DeviceTasks.h"
#include "EIB/SPI.h"

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

#define SYS_CLK         120000000
#define SYS_DELAY_1_S   SYS_CLK / 3

//Testing
#include <math.h>
float angle = 0;

void EnableClock(void);
void EnablePeripherals(void);

//static PositionVector vec;

int main(void)
{
    EnableClock();
    EnablePeripherals();

    //Disable broken joints
    MD_EnableMotor(JOINT4, false);
    MD_EnableMotor(JOINT6, false);

    while(1){
        ProcessMessages();
    }
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
    //Enable the SPI Peripheral and wait for the clock to settle
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    PinoutSet();
    InitializeTasks(SYS_CLK);
    InitializeControlLoop();
    //InitializeSPI(SYS_CLK);
}
