#include "MotorDriver.h"

//Standard includes
#include <stdint.h>

//Project includes

//Tivaware includes
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"

void MD_Initialize(void){
    uint8_t i;

    //Set the PWM Clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //Enable the PWM0 Peripheral and wait for the clock to stabilize
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlDelay(1);

    //Configure PWM0 to count up/down without synchronization
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the PWM frequency to 20KHz
    //TODO: Adjust this to a more suitable PWM frequency later
    //N = (1 / f) * SysClk, where n is the function parameter and f is the desired frequency
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 6000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 6000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 6000);

    //Default the duty cycle for each joint to 0%
    //Disable the motor
    //Set the direction to clockwise
    for(i = 0; i < JOINT_COUNT; i++){
        JOINT_POSITION Joint = (JOINT_POSITION)(i);
        MD_EnableMotor(Joint, false);
        MD_SetMotorDirection(Joint, true);
        MD_SetMotorDutyCycle(Joint, 0);
    }

    //Enable the PWM signal
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);

    //Enable the PWM Generator Block
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

void MD_EnableMotor(JOINT_POSITION Joint, bool bEnabled){
    if(bEnabled == true){
        switch(Joint){
            case JOINT1:
                GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_PIN_3);
                break;
            case JOINT2:
                GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_3, GPIO_PIN_3);
                break;
            case JOINT3:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, GPIO_PIN_5);
                break;
            case JOINT4:
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
                break;
            case JOINT5:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);
                break;
            case JOINT6:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
                break;
            default:
                //Invalid joint, do nothing
                break;
        }
    }
    else{
        switch(Joint){
            case JOINT1:
                GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_3, 0);
                break;
            case JOINT2:
                GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_3, 0);
                break;
            case JOINT3:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, 0);
                break;
            case JOINT4:
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0);
                break;
            case JOINT5:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0);
                break;
            case JOINT6:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);
                break;
            default:
                //Invalid joint, do nothing
                break;
        }
    }
}

void MD_SetMotorDutyCycle(JOINT_POSITION Joint, float DutyCycle){
    if(DutyCycle >= 0.999){
        DutyCycle = 0.999;
    }
    else if(DutyCycle < 0){
        DutyCycle = 0;
    }
    uint32_t PulseWidthRegisterValue = 0;
    switch(Joint){
        case JOINT1:
            PulseWidthRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * DutyCycle;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PulseWidthRegisterValue);
            break;
        case JOINT2:
            PulseWidthRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * DutyCycle;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PulseWidthRegisterValue);
            break;
        case JOINT3:
            PulseWidthRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * DutyCycle;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PulseWidthRegisterValue);
            break;
        case JOINT4:
            PulseWidthRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) * DutyCycle;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PulseWidthRegisterValue);
            break;
        case JOINT5:
            PulseWidthRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) * DutyCycle;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PulseWidthRegisterValue);
            break;
        case JOINT6:
            PulseWidthRegisterValue = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2) * DutyCycle;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PulseWidthRegisterValue);
            break;
        default:
            //Invalid joint, do nothing
            break;
    }
}

void MD_SetMotorDirection(JOINT_POSITION Joint, bool Clockwise){
    if(Clockwise == true){
        switch(Joint){
            case JOINT1:
                GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_2, 0);
                break;
            case JOINT2:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, 0);
                break;
            case JOINT3:
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
                break;
            case JOINT4:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0);
                break;
            case JOINT5:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0);
                break;
            case JOINT6:
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
                break;
            default:
                //Invalid joint, do nothing
                break;
        }
    }
    else{
        switch(Joint){
            case JOINT1:
                GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_PIN_2);
                break;
            case JOINT2:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_4, GPIO_PIN_4);
                break;
            case JOINT3:
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
                break;
            case JOINT4:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);
                break;
            case JOINT5:
                GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, GPIO_PIN_2);
                break;
            case JOINT6:
                GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
                break;
            default:
                //Invalid joint, do nothing
                break;
        }
    }
}

void MD_EnableBrake(bool bEnabled){
    if(bEnabled == true){
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, 0);
    }
    else{
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_PIN_4);
    }
}
