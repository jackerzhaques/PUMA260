#include "EncoderInterface.h"

/*
 * Standard includes
 */

/*
 * Project includes
 */
#include "Globals.h"

/*
 * Tivaware includes
 */
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "utils/uartstdio.h"
#include <driverlib/sysctl.h>
#include <EIB/OldSPI.h>

/*
 * Pin defines
 */
#define SS0_BASE    GPIO_PORTM_BASE
#define SS1_BASE    GPIO_PORTM_BASE
#define SS2_BASE    GPIO_PORTH_BASE

#define SS0         GPIO_PIN_1
#define SS1         GPIO_PIN_2
#define SS2         GPIO_PIN_0

typedef struct int24_t_tag{
    int32_t data : 24;
} int24_t;

//Forward declarations
void SelectEncoderChannel(EncoderDeviceSelect Encoder);
void SetChipSelect(bool Enabled);

void EI_Initialize(void){
    uint8_t i = 0;

    SPI_Initialize();

    //Deactivate chip select
    SetChipSelect(false);

    //Default the slave select channel to 0
    SelectEncoderChannel(ENC_SEL_1);

    SysCtlDelay(CLOCK_FREQ / 100);

    //Initialize each channel
    for(i = 0; i < ENC_SEL_COUNT; i++){
        uint8_t RegisterReadbackValue;
        EncoderDeviceSelect Encoder = (EncoderDeviceSelect)(i);

        //Write to MDR0
        EI_WriteRegister(WRITE_MDR0,
                         FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX4,
                         Encoder);

        //Verify the contents of MDR0
        //If the contents do not match, print to the UART that there is an error
        RegisterReadbackValue = EI_ReadRegister(READ_MDR0, Encoder);
        if(RegisterReadbackValue !=
                (FILTER_2 | DISABLE_INDX | FREE_RUN | QUADRX4)){
            //UARTprintf("Unable to connect to encoder %u\n", i + 1);
        }
        else{
            //UARTprintf("Successfully connected to encoder %u\n", i + 1);
        }

        //Write to MDR1
        EI_WriteRegister(WRITE_MDR1, BYTE_3 | EN_CNTR, Encoder);

        //Clear the encoder data register
        EI_ClearEncoder(Encoder);
    }
}

void EI_WriteRegister(uint8_t OpCode, uint8_t Data, EncoderDeviceSelect Encoder){
    SelectEncoderChannel(Encoder);
    SetChipSelect(true);

    SPI_Write(OpCode);
    SPI_Write(Data);

    SetChipSelect(false);
}

void EI_ClearEncoder(EncoderDeviceSelect Encoder){
    SelectEncoderChannel(Encoder);
    SetChipSelect(true);
    SPI_Write(CLR_CNTR);
    SetChipSelect(false);
}

uint8_t EI_ReadRegister(uint8_t OpCode, EncoderDeviceSelect Encoder){
    uint8_t Data = 0;

    SelectEncoderChannel(Encoder);
    SetChipSelect(true);

    SPI_Write(OpCode);
    Data = SPI_Read();

    SetChipSelect(false);

    return Data;
}

int32_t EI_ReadEncoderValue(EncoderDeviceSelect Encoder){
    int32_t EncoderValue;
    int24_t CastVariable;   //Struct used to cast from 24 to 32 bit.
    uint32_t Byte1, Byte2, Byte3;

    if(Encoder == ENC_SEL_2){
        uint8_t i = 0;
        i = 10;
    }

    SelectEncoderChannel(Encoder);
    SetChipSelect(true);

    SPI_Write(READ_CNTR);
    Byte1 = SPI_Read();
    Byte2 = SPI_Read();
    Byte3 = SPI_Read();

    SetChipSelect(false);

    CastVariable.data = (Byte1 << 16) | (Byte2 << 8) | Byte3;

    EncoderValue = CastVariable.data;

    return EncoderValue;
}

void SelectEncoderChannel(EncoderDeviceSelect Encoder){
    switch(Encoder){
        case ENC_SEL_1:
            GPIOPinWrite(SS0_BASE, SS0, 0);
            GPIOPinWrite(SS1_BASE, SS1, 0);
            GPIOPinWrite(SS2_BASE, SS2, 0);
        break;
        case ENC_SEL_2:
            GPIOPinWrite(SS0_BASE, SS0, SS0);
            GPIOPinWrite(SS1_BASE, SS1, 0);
            GPIOPinWrite(SS2_BASE, SS2, 0);
            break;
        case ENC_SEL_3:
            GPIOPinWrite(SS0_BASE, SS0, 0);
            GPIOPinWrite(SS1_BASE, SS1, SS1);
            GPIOPinWrite(SS2_BASE, SS2, 0);
            break;
        case ENC_SEL_4:
            GPIOPinWrite(SS0_BASE, SS0, SS0);
            GPIOPinWrite(SS1_BASE, SS1, SS1);
            GPIOPinWrite(SS2_BASE, SS2, 0);
            break;
        case ENC_SEL_5:
            GPIOPinWrite(SS0_BASE, SS0, 0);
            GPIOPinWrite(SS1_BASE, SS1, 0);
            GPIOPinWrite(SS2_BASE, SS2, SS2);
            break;
        case ENC_SEL_6:
            GPIOPinWrite(SS0_BASE, SS0, SS0);
            GPIOPinWrite(SS1_BASE, SS1, 0);
            GPIOPinWrite(SS2_BASE, SS2, SS2);
            break;
        default:
            //incorrect encoder value, do nothing
            break;
    }
}

void SetChipSelect(bool Enabled){
    if(Enabled){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    }
    else{
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    }
}
