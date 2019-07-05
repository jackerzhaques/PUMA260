#include "EIB/Spi.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"

//Typedefs
typedef struct Buffer_tag{
    uint8_t current;
    uint8_t start;
    uint8_t data[BUFFER_SIZE];
} Buffer;

//File variables


//Forward declarations
void SPI_ISR(void);
bool AddByteToBuffer(Buffer* buffer, uint8_t byte);
void TransmitBuffer(void);
bool BufferFull(Buffer *buffer);
bool BufferEmpty(Buffer *buffer);
uint8_t GetNextByte(Buffer* buffer);
bool Peak(Buffer* buffer, uint8_t c);

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

    SSIDataPutNonBlocking(SSI3_BASE, 0x01);
    SSIDataPutNonBlocking(SSI3_BASE, 0x01);
    SSIDataPutNonBlocking(SSI3_BASE, 0x01);
    SSIDataPutNonBlocking(SSI3_BASE, 0x01);
}

void SPI_ISR(){
    if(SSIIntStatus(SSI3_BASE, false) & SSI_TXEOT){
        SSIIntClear(SSI3_BASE, SSI_TXEOT);
        SSIDataPutNonBlocking(SSI3_BASE, 0xAA);
        SSIDataPutNonBlocking(SSI3_BASE, 0xAA);
        SSIDataPutNonBlocking(SSI3_BASE, 0xAA);
    }

    if(SSIIntStatus(SSI3_BASE, false) & (SSI_RXFF | SSI_RXTO)){
        SSIIntClear(SSI3_BASE, SSI_RXFF | SSI_RXTO);
    }

}

void SPIWriteBuffer(uint8_t *buffer, uint8_t nBytes){

}

bool BufferFull(Buffer *buffer){
    bool isFull = false;

    if(INCREMENT_INDEX(buffer->current) == buffer->start){
        isFull = true;
    }
    else{
        isFull = false;
    }

    return isFull;
}

bool BufferEmpty(Buffer *buffer){
    bool isFull = false;

    if(buffer->start == buffer->current){
        isFull = true;
    }
    else{
        isFull = false;
    }

    return isFull;
}


bool AddByteToBuffer(Buffer* buffer, uint8_t byte){
    bool bufferHasRoom = false;

    if(INCREMENT_INDEX(buffer->current) != buffer->start){
        buffer->data[buffer->current] = byte;
        buffer->current = INCREMENT_INDEX(buffer->current);
        bufferHasRoom = true;
    }
    else{
        //Buffer is full
    }

    return bufferHasRoom;
}

uint32_t AddBytesToBuffer(uint8_t *bytes, uint32_t nBytes){
    uint32_t i = 0;
    bool bufferHasRoom = true;


    while(i < nBytes && bufferHasRoom){
        bufferHasRoom = AddByteToBuffer(&OutgoingBuffer, bytes[i]);

        if(bufferHasRoom){
            i++;
        }
    }

    TransmitBuffer();

    return i;
}

void TransmitBuffer(void){
    //TODO: Refactor this to use a different method than UARTSpaceAvail
    //SSI has no equivalent
    //Will probably have to try and add a byte and see if any could be added
    //Remember, the function GetNextByte will remove the byte from the buffer! We can't do that!
    while(OutgoingBuffer.start != OutgoingBuffer.current &&
            SSISpaceAvail(SSI3_BASE))
    {
        if(UARTSpaceAvail(UART0_BASE)){
            UARTCharPutNonBlocking(
                    UART0_BASE,
                    OutgoingBuffer.data[OutgoingBuffer.start]
            );

            OutgoingBuffer.start = INCREMENT_INDEX(OutgoingBuffer.start);
        }

    }
}

uint8_t GetNextByte(Buffer* buffer){
    uint8_t byte = 0x00;

    if(buffer->start != buffer->current){
        byte = buffer->data[buffer->start];
        buffer->start = INCREMENT_INDEX(buffer->start);
    }

    return byte;
}

bool Peak(Buffer* buffer, uint8_t c){
    bool charFound = false;
    uint32_t start = buffer->start;

    while(start != buffer->current && !charFound){
        if(buffer->data[start] == c){
            charFound = true;
        }
        else{
            //Char not found, keep traversing
            start = INCREMENT_INDEX(start);
        }
    }

    return charFound;
}
