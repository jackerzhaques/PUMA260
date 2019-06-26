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
    //MD_EnableMotor(JOINT5, false); //Pending issue where encoder ticks are being missed when joint is rotating too fast.
    MD_EnableMotor(JOINT6, false);

//      vec.x = 10;
//      vec.y = 0;
//      vec.z = 5;
//      vec.theta = 0;

//      float centerx = 10;
//      float centery = 0;
//      float radius = 1.5;

    while(1){
        ProcessMessages();
//          float x = radius*cos(angle) + centerx;
//          float y = radius*sin(angle) + centery;
//          vec.x = x;
//          vec.y = y;
//          //vec.theta = -90;
//          SetArmPosition(vec);
//          angle += M_PI * .001;
//          SysCtlDelay(120000000 / 300);
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
