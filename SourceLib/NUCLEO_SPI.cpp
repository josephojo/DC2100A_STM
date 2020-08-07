/*
 Linear Technology DC2100A Demonstration Board.
 PIC18 Driver File for SPI port.
 All datasheet references in this file refer to Microchip Technology Inc. document 39964B.pdf.

 @verbatim
 This code provides an interface to the SPI port on the PIC18 used to communicate with the LTC6804-2.
 DMA is used so that the other software modules can perform other tasks (such as CRC calculation) as
 bytes as sent to and received from the SPI bus.  The SPI driver can be configured to different baud
 rates, so that communication for each IC can occur at its maximum rate.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 750 $
 $Date: 2014-09-17 19:27:28 -0400 (Wed, 17 Sep 2014) $

 Copyright (c) 2013, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of Linear Technology Corp.

*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "NUCLEO_SPI.h"
#include "../mbed.h"



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SPI spi(D11, D12, D13); // mosi, miso, sclk
int8 spi_dummy_tx; // Cosmetic only, byte is only necessary to ensure MOSI is low while reading from SPI



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes SPI pins, SPI module, and timer used for SPI baud rate generator
void NUCLEO_SPI_Init(int16 baud_khz, NUCLEO_SPI_MODE mode)
{
    spi.format(8,mode);

    // Set the baud rate, which will also start the timer and SPI module
    NUCLEO_SPI_Set_Baud(baud_khz);

    spi_dummy_tx = 0x00;        // Set low so that logic analyzer pictures will be pretty during read
    spi.set_default_write_value(spi_dummy_tx); 	
    spi.set_dma_usage(DMA_USAGE_ALLOCATED); // DMA_USAGE_ALLOCATED means it doesn't release the DMA channel after a transfer. The alternative is DMA_USAGE_TEMPORARY_ALLOCATED
    return;
}

// Initializes SPI baud rate and starts the SPI module
// This is called regularly during code operation, as several different baud rates are used by the peripheral ICs on DC2100A.
void NUCLEO_SPI_Set_Baud(int16 baud_khz)
{
    spi.frequency(baud_khz/1000);
    return;
}

/* // Checks if the current SPI transmission or reception is complete.
inline BOOLEAN NUCLEO_SPI_Buffer_Done(void)
{
    // Implementation does not exit in mbed library
}*/


/*// Begins the transmission of number of bytes to the SPI via DMA.
// Does not wait for transmission to complete.
inline void NUCLEO_SPI_Buffer_Send_Start(int8* write_buffer, int8 num_bytes)
{
    
    return;
} */

// Begins the transmission of number of bytes to the SPI via DMA.
// Does not wait for transmission to complete.
void NUCLEO_SPI_Buffer_Send_Start(unsigned int8* write_buffer, int8 num_bytes)
{
    // Send Data
    for (int i=0; i< num_bytes; i++)
    {
        spi.write(write_buffer[i]);
    }
    return;
}


/*// Begins the reception of a number of bytes to the SPI via DMA.
// Does not wait for reception to complete. Coming SOON!!
inline void NUCLEO_SPI_Buffer_Receive_Start(int8* read_buffer, int8 num_bytes)
{

} */


// Begins the reception of a number of bytes to the SPI.
// Waits for reception to complete.
void NUCLEO_SPI_Buffer_Receive_Start(unsigned int8* read_buffer, int8 num_bytes)
{
    // Read Data
    for (int i = 0; i< num_bytes; i++)
    {
        // Need to convert it back from int (32bits) to int8
        read_buffer [i] = (int8)(spi.write(spi_dummy_tx));
    }
    return;
}

//// Returns the number of bytes left to be sent out the SPI port.
//inline int8 NUCLEO_SPI_Buffer_Receive_Bytes_Available(int8* write_buffer_ptr)
//{
//    // Might be usefull in the future. #FutureUse // NeededNow??? - Commenting this out in the meantime
//}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

