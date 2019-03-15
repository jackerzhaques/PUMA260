#include "pinout.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"

#define InvertPort  GPIO_PORTK_BASE
#define InvertPin   GPIO_PIN_7

//#define TEST_GPIO_OUTPUT
//#define TEST_GPIO_INPUT
//#define TEST_SPI
//#define TEST_UART
#define TEST_MOTORS
//#define TEST_ADC

#define ADC_BASE    ADC0_BASE

void EnableClock(void);
void EnablePeripherals(void);
void EnableSPI(void);

int main(void)
{
    int i;
    EnableClock();
    EnablePeripherals();

    //Enable PWM for PF1 (M0)
    //Set the PWM clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Enable the PWM peripheral and wait
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //Enable GPIO Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Wait for the clock to stabilize
    SysCtlDelay(1);

    //Conifgure PWM0 to count up/down without synchronization
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    //PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    //PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the PWM frequency to 250Hz
    //N = (1 / f) * SysClk, where N is the function parameter and f is the frequency
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 64000);
    //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 64000);
    //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 64000);

    //Set the duty cycle to 25%
    //Duty cycle is a function of the period, use PWMGenPeriodGet()
    //For a frequency of 25%, Calculation is N * 25%
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) / 4);
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) / 4);
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 4);

    //Enable the PWM Bit3(PF3) signal
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    //Enable the PWM generator block
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    while(1)
    {
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

#ifdef TEST_SPI
       /*
        * SPI Transfer
        *
        * Transfer 60 bytes
        */

        //Encoder slave select
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);

       //Write the control word to read the encoder count
       SSIDataPut(SSI3_BASE, 0x90);

       //Wait until the transfer is complete
       while(SSIBusy(SSI3_BASE)){

       }

       SSIDataPut(SSI3_BASE, 0xAA);

       while(SSIBusy(SSI3_BASE))
       {

       }
       GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);

       SysCtlDelay(SysCtlClockGet() / 1000);

        //Enable encoder select
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);

        //Write the control word to read the encoder count
        SSIDataPut(SSI3_BASE, 0x48);

        //Wait until the transfer is complete
        while(SSIBusy(SSI3_BASE)){

        }


        //Read the data in
        for(i = 0; i < 4; i++){
            uint32_t ui32ReceivedData = 0;

            //Grab the received data
            while(SSIDataGetNonBlocking(SSI3_BASE, &ui32ReceivedData) > 0){

            }

            //Shift the data and add it to all received data (MSB First)
            ui32ReceivedEncoderTicks += (ui32ReceivedData) << (3 - i);
        }

        //Deselect all devices
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
#endif
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
    //Use the pinout code generated by TI PinMux.

    //Enable the SPI Peripheral and wait for the clock to settle
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    PinoutSet();

    EnableSPI();
}

void EnableSPI(void){
    uint32_t ui32SysClock;

    SysCtlDelay(1);

    //Configure the QSSI Interface
    /*
     * Mode:        SPI Master
     * Frequency:   1MHz
     * Data Size:   8bits
     */
    ui32SysClock = SysCtlClockGet();
    SSIConfigSetExpClk(SSI3_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 100000, 8);

    //Enable the module
    SSIEnable(SSI3_BASE);
}
