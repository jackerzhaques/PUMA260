#include "SPI.h"

//Project includes

//Standard includes

//Tivaware includes
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"

//Forward declarations

void SPI_Initialize(void){
    uint32_t ui32SysClock;
    SysCtlDelay(1);

    //Configure the QSSI Interface
    /*
     * Mode:        SPI Master
     * Frequency:   100KHz
     * Data Size:   8bits
     */
    ui32SysClock = SysCtlClockGet();
    SSIConfigSetExpClk(SSI3_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    //Enable the module
    SSIEnable(SSI3_BASE);
}

void SPI_Write(uint8_t Data){
    uint32_t DummyData;
    //Write the data
    SSIDataPut(SSI3_BASE, Data);

    //Wait for transfer to complete
    while(SSIBusy(SSI3_BASE)){

    }

    //The SPI read buffer needs to be cleared after every write
    SSIDataGet(SSI3_BASE, &DummyData);
}

uint8_t SPI_Read(void){
    uint8_t ui8ReceivedData;
    uint32_t ui32ReadData;

    //Wait for any existing transfer to complete
    while(SSIBusy(SSI3_BASE)){

    }

    //Write a dummy byte
    SSIDataPut(SSI3_BASE, 0x00);
    //Clock in the data and return
    SSIDataGet(SSI3_BASE, &ui32ReadData);
    ui8ReceivedData = (uint8_t)(ui32ReadData);

    return ui8ReceivedData;
}
