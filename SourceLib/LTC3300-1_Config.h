/*
 Linear Technology DC2100A Demonstration Board.
 Driver Configuration Header File for LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer.
 This file configures the interface of the LTC3300-1 Driver to the hardware in which it's used.

 @verbatim
 The LTC3300-1 is a fault-protected controller IC for
 transformer-based bidirectional active balancing of multicell
 battery stacks. All associated gate drive circuitry,
 precision current sensing, fault detection circuitry and a
 robust serial interface with built-in watchdog timer are
 integrated.
 Each LTC3300-1 can balance up to 6 series-connected battery
 cells with an input common mode voltage up to 36V.
 Charge from any selected cell can be transferred at high
 efficiency to or from 12 or more adjacent cells. A unique
 level-shifting SPI-compatible serial interface enables
 multiple LTC3300-1 devices to be connected in series,
 without opto-couplers or isolators, allowing for balancing
 of every cell in a long string of series-connected batteries.
 When multiple LTC3300-1 devices are connected in series
 they can operate simultaneously, permitting all cells in
 the stack to be balanced concurrently and independently.
 Fault protection features include readback capability, cyclic
 redundancy check (CRC) error detection, maximum
 on-time volt-second clamps, and overvoltage shutoffs.
 @endverbatim

 http://www.linear.com/product/LTC3300

 http://www.linear.com/product/LTC3300#demoboards

 REVISION HISTORY
 $Revision: 692 $
 $Date: 2014-09-08 11:51:35 -0400 (Mon, 08 Sep 2014) $

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
    @ingroup LTC3300-1
    Driver Configuration Header File for LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer
*/

#ifndef __LTC3300_CONFIG_H__
#define __LTC3300_CONFIG_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "DC2100A.h"                // HW definition for Board
#include "LTC6804-2.h"              // Interface for LTC3300s on DC2100A is through LTC6804-2
#include "Error.h"                  // Errors are reported through Error Handler Task
#include "../mbed.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Configures the number of LTC3300-1 ICs in a chain at each logical address.
#define LTC3300_CONFIG_NUM_ICS_PER_ADDRESS              DC2100A_NUM_LTC3300

//! Configures interface through which LTC3300-1 driver module sends SPI bytes to a chain of LTC3300-1 ICs.
//! - Parameters
//!     + address: The logical address for the PCB containing these LTC3300-1 ICs.
//!     + data_ptr: Pointer to LTC3300-1 bytes to be written via SPI.
//!     + num_bytes: Number of bytes to be written via SPI.
//!     + baud_khz: The baud rate of the SPI communication in kHz.
//! - Usage Examples
//!     + If communication is through an LTC6804-2, this macro would call the LTC6804-2 driver's SPI Write function.
//!     + If communication is direct from the microcontroller, this macro would call the microcontroller's SPI Write function.
#define LTC3300_CONFIG_SPI_WRITE(address, data_ptr, num_bytes, baud_khz)       \
        LTC6804_SPI_Write(address, TRUE, TRUE, data_ptr, num_bytes, baud_khz)

//! Configures interface through which LTC3300-1 driver module receives SPI bytes from a chain of LTC3300-1 ICs.
//! - Parameters
//!     + address: The logical address for the PCB containing these LTC3300-1 ICs.
//!     + data_ptr: Pointer to where LTC3300-1 bytes read via SPI will be written.
//!     + num_bytes: Number of bytes to be read via SPI.
//!     + baud_khz: The baud rate of the SPI communication in kHz.
//! - Usage Examples
//!     + Example 1: If communication is through an LTC6804-2, this macro would call the LTC6804-2 driver's SPI Read function.
//!     + Example 2: If communication is direct from the microcontroller, this macro would call the microcontroller's SPI Read function.
#define LTC3300_CONFIG_SPI_READ(address, data_ptr, num_bytes, baud_khz)        \
        LTC6804_SPI_Read(address, TRUE, TRUE, data_ptr, num_bytes, baud_khz)

//! Configures interface through which LTC3300-1 driver module reports its CRC errors.
//! - Parameters
//!     + address: The logical address for the PCB containing these LTC3300-1 ICs.
//!     + data_ptr: Pointer to bytes read from LTC3300-1.
//!     + num_bytes: Number of bytes read from LTC3300-1.
//! - Usage Examples
//!     + Example 1: If DC2100A Error.c/.h code module is defined for a system, framing the data and passing to that error reporting module would be here.
//!     + Example 2: if stdout is defined for a system, this macro would format the data into a printf() statement.
//!     + Example 3: If LTC3300-1 CRC errors are to be ignored, this macro could be defined as blank.
#define LTC3300_CONFIG_ERROR_CRC(address, ic_num, command, data_ptr, num_bytes)                 \
        {                                                                                       \
            int8 temp_data[ERROR_DATA_SIZE];                                                    \
            int8 byte_num;                                                                      \
            temp_data[0] = address;                                                             \
            temp_data[1] = ic_num;                                                              \
            temp_data[2] = command;                                                             \
            for (byte_num = 3; byte_num < MIN(num_bytes + 2, ERROR_DATA_SIZE); byte_num++)      \
            {                                                                                   \
                temp_data[byte_num] = data_ptr[byte_num - 2];                                   \
            }                                                                                   \
            Error_Data_Set(ERROR_CODE_LTC3300_CRC, temp_data, num_bytes + 2);                   \
                                                                                                \
        }    
            //printf("\t ####  Error ###\r\n");                                                   \
            //printf("Board Num: %d\r\n", address);                                               \
            //printf("IC Num: %d\r\n", ic_num);                                                   \
            //printf("Command: %d\r\n", command);                                                 \
            //for (int i=3; i< MIN(num_bytes + 2, ERROR_DATA_SIZE);i++)                           \
            //{                                                                                   \
            //    printf("Data[%d]: %d\r\n", i, temp_data[i]);                                    \
            //}                                                                                   \
            //printf("--------------------------------------------------\n\n");                   \


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
