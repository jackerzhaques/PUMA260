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
#define TEST_GPIO_INPUT
#define TEST_SPI
#define TEST_UART
#define TEST_PWM
#define TEST_ADC

#define ADC_BASE    ADC0_BASE

void EnableClock(void);
void EnablePeripherals(void);
void EnableSPI(void);

int main(void)
{
    int i;
    EnableClock();
    EnablePeripherals();

    while(1)
    {
        uint32_t ui32ReceivedEncoderTicks;
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

        //Enable encoder select
               GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);

               //Select any Encoder device
               //001, device 1
               GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1);
               GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0);
               GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, 0);

               //Write the control word to read the encoder count
               SSIDataPut(SSI3_BASE, 0x90);

               //Wait until the transfer is complete
               while(SSIBusy(SSI3_BASE)){

               }

               SSIDataPut(SSI3_BASE, 0xAA);

               while(SSIBusy(SSI3_BASE))
               {

               }

               //Deselect all devices
               GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0);
               GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 7);

        //Enable encoder select
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);

        //Select any Encoder device
        //001, device 1
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0, 0);

        //Write the control word to read the encoder count
        SSIDataPut(SSI3_BASE, 0x48);

        //Wait until the transfer is complete
        while(SSIBusy(SSI3_BASE)){

        }


        //Read the data in
        for(i = 0; i < 1; i++){
            uint32_t ui32ReceivedData = 0;

            //Grab the received data
            while(SSIDataGetNonBlocking(SSI3_BASE, &ui32ReceivedData) > 0){

            }

            //Shift the data and add it to all received data (MSB First)
            ui32ReceivedEncoderTicks += (ui32ReceivedData) << (3 - i);
        }

        //Deselect all devices
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 7);
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
