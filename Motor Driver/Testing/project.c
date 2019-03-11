#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"

#define InvertPort 	GPIO_PORTK_BASE
#define InvertPin 	GPIO_PIN_7

#define ADC_BASE		ADC0_BASE

void EnableClock(void);
void EnablePeripherals(void);

void SetPWMDutyCycle(float DutyCycle){
	uint32_t PWMRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * DutyCycle;
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMRegisterValue);
}

float SampleCurrentFeedback(void);

int main(void)
{
	bool isTurningClockwise = true;
	EnableClock();
	EnablePeripherals();

  while(1)
  {
		if(SampleCurrentFeedback() > 1.0){
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);
		}
		else{
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0);
		}
		
		if(isTurningClockwise){
			//Turn the motor clockwise
		}
		else{
			//Turn the motor counter clockwise
		}
		
		//If the current has exceeded 2 amps, we will assume the joint hit its limit
		// and we will reverse the direction
		
	}
}

float SampleCurrentFeedback(void){
	uint32_t ui32ADCValue = 0;
	float Voltage = 0;
	float Current = 0;
	
	//Trigger a conversion
	ADCProcessorTrigger(ADC_BASE, 3);
	
	//Wait for conversion to finish
	while(!ADCIntStatus(ADC_BASE, 3, false)){
	}
	
	//Clear the interrupt flag
	ADCIntClear(ADC_BASE, 3);
	
	//Read the value
	ADCSequenceDataGet(ADC_BASE, 3, &ui32ADCValue);
	
	//Convert to a voltage
	Voltage = ui32ADCValue * 3.3 / 4096;
	
	//Convert to current
	Current = 1.2;//(1.0/0.525) * Voltage;	//Approximately 525mv per amp.
	
	Voltage = 0;
	
	return Current;
}

void EnablePWM(void);
void EnableStatusFlag(void);
void EnableInvert(void);
void EnableCurrentFeedback(void);

void EnableClock(void){
	SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                      SYSCTL_OSC_MAIN |
                      SYSCTL_USE_PLL |
                      SYSCTL_CFG_VCO_480), 20000000);
}

/*
	Enables all peripherals needed for this motor driver test
*/
void EnablePeripherals(void){
	EnablePWM();
	EnableStatusFlag();
	EnableInvert();
	EnableCurrentFeedback();
}

/*
	Enables PWM pin PF3
*/
void EnablePWM(void){
	//Set the PWM clock
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	
	//Enable the PWM peripheral and wait
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	
	//Enable GPIO Port F
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	//Wait for the clock to stabilize
	SysCtlDelay(1);
	
	//Configure the GPIO pin to be muxed to a PWM signal.
	GPIOPinConfigure(GPIO_PF3_M0PWM3);
	
	//Configure the GPIO for PWM function
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
	
	//Conifgure PWM0 to count up/down without synchronization
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	
	//Set the PWM frequency to 250Hz
	//N = (1 / f) * SysClk, where N is the function parameter and f is the frequency
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 64000);
	
	//Set the duty cycle to 25%
	//Duty cycle is a function of the period, use PWMGenPeriodGet()
	//For a frequency of 25%, Calculation is N * 25%
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);
	
	//Enable the PWM Bit3(PF3) signal
	PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
	
	//Enable the PWM generator block
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

/*
	Enables the status flag pin as an input (PK6)
*/
void EnableStatusFlag(void){
}

/*
	Enables the invert pin as an output (PK7)
*/
void EnableInvert(void){
	// Enable the GPIO module.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlDelay(1);

	// Configure PK7 as an output
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);
	
	//Default the pin low
	GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0);
}

/*
	Enables the current feedback using an ADC (PK0)
*/
void EnableCurrentFeedback(void){
	//Enable ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	
	//Enable PortK
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlDelay(1);
	
	//Select the analog ADC function
	GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);
	
	//Enable sample sequence 3 with a processor signal trigger
	//Sequencer 3 will do a single sample
	ADCSequenceConfigure(ADC_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	
	ADCSequenceStepConfigure(ADC_BASE, 3, 0, ADC_CTL_CH16 | ADC_CTL_IE | ADC_CTL_END);
	
	//Enable sequence 3
	ADCSequenceEnable(ADC_BASE, 3);
	
	//Set reference
	ADCReferenceSet(ADC_BASE, ADC_REF_INT);
	
	//Clear the interrupt flag
	ADCIntClear(ADC_BASE, 3);
}
