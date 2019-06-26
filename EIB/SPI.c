#include <EIB/OldSPI.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"

//Forward declarations
void SPI_ISR(void);

void InitializeSPI(uint32_t sysClk){

    //Configure the QSSI Interface
    /*
     * Mode:        SPI Master
     * Frequency:   100KHz
     * Data Size:   8bits
     */
    SSIConfigSetExpClk(SSI3_BASE, sysClk, SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 250000, 8);

    //Register the interrupt handler
    SSIIntRegister(SSI3_BASE, SPI_ISR);

    //Enable transmission interrupts
    SSIIntEnable(SSI3_BASE, SSI_TXEOT);

    //Enable the module
    SSIEnable(SSI3_BASE);
}

void SPI_ISR(){

}

void SPIWriteBuffer(uint8_t *buffer, uint8_t nBytes){

}
