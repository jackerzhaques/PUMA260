//*****************************************************************************
//
// Configure the device pins for different signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 3/13/2019 at 8:20:02 PM
// by TI PinMux version 4.0.1530 
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "pinout.h"

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the customer specific usage.
//!
//! \return None.
//
//*****************************************************************************
void
PinoutSet(void)
{
    //
    // Enable Peripheral Clocks 
    //
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

	//
	// Unlock the Port Pin and Set the Commit Bit
	//
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;
	
    //
    // Configure the GPIO Pin Mux for PD7
	// for AIN4_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PD5
	// for AIN6_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PE1
	// for AIN2_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PE2
	// for AIN1_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PE0
	// for AIN3_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PD6
	// for AIN5_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PP5
	// for GPIO_PP5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PK5
	// for GPIO_PK5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PA7
	// for GPIO_PA7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PM0
	// for GPIO_PM0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PM1
	// for GPIO_PM1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PM2
	// for GPIO_PM2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PH0
	// for GPIO_PH0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PH2
	// for GPIO_PH2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PD0
	// for GPIO_PD0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PH3
	// for GPIO_PH3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PL4
	// for GPIO_PL4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PL5
	// for GPIO_PL5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PD1
	// for GPIO_PD1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PM3
	// for GPIO_PM3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PL0
	// for GPIO_PL0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PL1
	// for GPIO_PL1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PL2
	// for GPIO_PL2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PL3
	// for GPIO_PL3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PN2
	// for GPIO_PN2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PM4
	// for GPIO_PM4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PK4
	// for M0PWM6
    //
	MAP_GPIOPinConfigure(GPIO_PK4_M0PWM6);
	MAP_GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PG0
	// for M0PWM4
    //
	MAP_GPIOPinConfigure(GPIO_PG0_M0PWM4);
	MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PG1
	// for M0PWM5
    //
	MAP_GPIOPinConfigure(GPIO_PG1_M0PWM5);
	MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PF2
	// for M0PWM2
    //
	MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PF1
	// for M0PWM1
    //
	MAP_GPIOPinConfigure(GPIO_PF1_M0PWM1);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PF3
	// for M0PWM3
    //
	MAP_GPIOPinConfigure(GPIO_PF3_M0PWM3);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PQ2
	// for SSI3XDAT0
    //
	MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
	MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PQ3
	// for SSI3XDAT1
    //
	MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
	MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PQ1
	// for SSI3FSS
    //
	MAP_GPIOPinConfigure(GPIO_PQ1_SSI3FSS);
	MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PQ0
	// for SSI3CLK
    //
	MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
	MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA0
	// for U0RX
    //
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA1
	// for U0TX
    //
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

