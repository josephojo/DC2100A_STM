/*
 Linear Technology DC2100A Demonstration Board.
 Driver Configuration Header File for LTC6804-2 Multicell Battery Monitors.
 All datasheet references in this file refer to Linear Technology Corp. document 680412fa.pdf.

 @verbatim
 The LTC6804 is a 3rd generation multicell battery stack
 monitor that measures up to 12 series connected battery
 cells with a total measurement error of less than 1.2mV. The
 cell measurement range of 0V to 5V makes the LTC6804
 suitable for most battery chemistries. All 12 cell voltages
 can be captured in 290us, and lower data acquisition rates
 can be selected for high noise reduction.
 Multiple LTC6804 devices can be connected in series,
 permitting simultaneous cell monitoring of long, high voltage
 battery strings. Each LTC6804 has an isoSPI interface
 for high speed, RF-immune, local area communications.
 Using the LTC6804-2, multiple devices are connected in
 a daisy-chain with one host processor connection for all
 devices. Using the LTC6804-2, multiple devices are connected
 in parallel to the host processor, with each device
 individually addressed.
 Additional features include passive balancing for each cell,
 an onboard 5V regulator, and 5 general purpose I/O lines.
 In sleep mode, current consumption is reduced to 4uA.
 The LTC6804 can be powered directly from the battery,
 or from an isolated supply.
 @endverbatim

 http://www.linear.com/product/LTC6804

 http://www.linear.com/product/LTC6804#demoboards

 REVISION HISTORY
 $Revision: 699 $
 $Date: 2014-09-09 16:02:09 -0400 (Tue, 09 Sep 2014) $

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

/*! @file
    @ingroup LTC6804-2
    Driver Configuration Header File for LTC6804-2 Multicell Battery Monitors.
*/

#ifndef __LTC6804_CONFIG_H__
#define __LTC6804_CONFIG_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "DC2100A.h"                // HW definition for Board
#include "NUCLEO_SPI.h"             // Interface for LTC3300s on DC2100A is through SPI
#include "NUCLEO_Timer.h"           // Interface for Timers and Delays
#include "System.h"                 // The number and addresses of the LTC6804s are tracked by the system monitor code module.
#include "Error.h"                  // Errors are reported through Error Handler Task
//#include "../mbed.h"                // MBed library for pinnames, 

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=




//! Configures Free Running Timer used to determine if the LTC6804 needs a LTC6804_TWAKE or LTC6804_TREADY duration wakeup pulse
#define LTC6804_CONFIG_TIMER                NUCLEO_Timer_Update()


//! Function for delaying a specified number of us.  This is used to generate wakeup signals for LTC6804.
//! - Parameters
//!     + us: The number of us to delay.
#define LTC6804_CONFIG_DELAY_US(us)         NUCLEO_Timer_Delay_us(us)

//! The number of LTC6804 addresses used in this system.  This is used for loops over all available LTC6804s.
#define LTC6804_CONFIG_NUM_BOARDS           System_Num_Boards // #NeededNow??? - Joseph #NeededNow??? #FutureUse
		
//! Returns the physical address for a LTC6804 in the system given its logical address.
#define LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num) \
        ((board_num == LTC6804_BROADCAST) ? LTC6804_COMMAND_CODE_BROADCAST_ADDRESS : System_Address_Table[board_num] | LTC6804_COMMAND_CODE_ADDRESSED_BIT)

//! Configures the function called to set the baud rate to the LTC6804.  This is used to control the baud rate of the I2C and SPI Buses implemented with the LTC6804 GPIO
//! - Parameters
//!     + baud_khz: The baud rate in kHz.
#define LTC6804_CONFIG_SPI_SET_BAUD(baud_khz) \
        NUCLEO_SPI_Set_Baud(baud_khz)

//! Note - This LTC6804 driver is designed for a buffered SPI peripheral (DMA, Interrupt driven, etc) to maximize throughput.
//!        If a simple, non-buffered SPI driver is all that's available, while loops will need to be incorporated in these #defines.

//! Configures the function called to start receiving a string of bytes to the LTC6804.
//! - Parameters
//!     + buffer: Pointer where to store the received bytes.
//!     + num_bytes: The number of bytes to receive.
#define LTC6804_CONFIG_SPI_BUFFER_RECEIVE_START(buffer, num_bytes) \
        NUCLEO_SPI_Buffer_Receive_Start(buffer, num_bytes)

//! Configures the function called to start sending a string of bytes to the LTC6804.
//! - Parameters
//!     + buffer: Pointer to the bytes to send.
//!     + num_bytes: The number of bytes to send.
#define LTC6804_CONFIG_SPI_BUFFER_SEND_START(buffer, num_bytes) \
        NUCLEO_SPI_Buffer_Send_Start(buffer, num_bytes)

/* //! Configures the function called to check if the last SPI communication to LTC6804 is done and SPI is ready to send/receive more data.
#define LTC6804_CONFIG_SPI_BUFFER_DONE() \
        NUCLEO_SPI_Buffer_Done() // #NeededNow??? - Commenting this out in the meantime */

/* // ! Configures the function called to return the number of bytes received in a buffer.
// ! - Parameters
// !     + buffer: Pointer where the received bytes are being stored.
#define LTC6804_CONFIG_SPI_BUFFER_RECEIVE_BYTES_AVAILABLE(buffer) \
        NUCLEO_SPI_Buffer_Receive_Bytes_Available(buffer) // #NeededNow??? - Commenting this out in the meantime */

//! Configures interface through which LTC6804-2 driver module reports its CRC errors.
//! - Parameters
//!     + address: The logical address for the PCB containing this LTC6804-2 IC.
//!     + command: LTC6804 Command which generated CRC error.
//!     + data_ptr: Pointer to bytes read from LTC6804-2.
//!     + num_bytes: Number of bytes to be read via SPI.
//! - Usage Examples
//!     + Example 1: If DC2100A Error.c/.h code module is defined for a system, framing the data and passing to that error reporting module would be here.
//!     + Example 2: if stdout is defined for a system, this macro would format the data into a printf() statement.
//!     + Example 3: If LTC6804-2 CRC errors are to be ignored, this macro could be defined as blank.
#define LTC6804_CONFIG_ERROR_CRC(address, command, data_ptr, num_bytes)                         \
        {                                                                                       \
            int8 temp_data[ERROR_DATA_SIZE];                                                    \
            unsigned int8 byte_num;                                                                      \
            temp_data[0] = address;                                                             \
            temp_data[1] = UPPER_BYTE(command);                                                 \
            temp_data[2] = LOWER_BYTE(command);                                                 \
            for (byte_num = 3; byte_num < MIN(num_bytes + 3, ERROR_DATA_SIZE); byte_num++)      \
            {                                                                                   \
                temp_data[byte_num] = data_ptr[byte_num - 3];                                   \
            }                                                                                   \
            Error_Data_Set(ERROR_CODE_LTC6804_CRC, temp_data, num_bytes + 3);                   \
        }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
