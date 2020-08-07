/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Interface to 24AA64 EEPROM through the LTC6804-2 Battery Monitor on the DC2100A PCB.
 All datasheet references in this file refer to Microchip Technology Inc. document 21711J.pdf.

 @verbatim
 The Microchip Technology Inc. 24AA64/24LC64
 (24XX64*) is a 64 Kbit Electrically Erasable PROM.
 The device is organized as a single block of 8K x 8-bit
 memory with a 2-wire serial interface. Low-voltage
 design permits operation down to 1.8V, with standby
 and active currents of only 1 uA and 1 mA,
 respectively. It has been developed for advanced, lowpower
 applications such as personal communications
 or data acquisition. The 24XX64 also has a page write
 capability for up to 32 bytes of data. Functional address
 lines allow up to eight devices on the same bus, for up
 to 512 Kbits address space. The 24XX64 is available in
 the standard 8-pin PDIP, surface mount SOIC, TSSOP
 and MSOP packages.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 542 $
 $Date: 2014-07-31 11:57:59 -0400 (Thu, 31 Jul 2014) $

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
    @ingroup EEPROM
    Reference Application File for Interface to 24AA64 EEPROM through the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "24AA64.h"
#include "LTC6804-2.h"
#include "NUCLEO_Timer.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// I2C address and format from datasheet Figure 3-3
#define EEPROM_24AA64_BASE_ADDRESS          0x50
#define EEPROM_24AA64_WRITE_BIT             0
#define EEPROM_24AA64_READ_BIT              1

typedef struct
{
    int8 control_byte;
    int8 word_address[2];
} EEPROM_24AA64_COMMAND_TYPE;

// Timing characteristics from datasheet Table 1-2
#define EEPROM_24AA64_BAUD_RATE             400 // in kHz, Max Clock Frequency when 2.5V <= VCC <= 5.5V (FCLK in datasheet)

// Timing characteristics from datasheet Table 1-2
#define EEPROM_24AA64_TWC                   5 // in ms, max write cycle time

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Writes a series of bytes to the 24AA64 EEPROM, one page at a time, following datasheet Figure 6-2
void Eeprom_24AA64_Write(int16 board_num, int16 address, int8* data_ptr, int16 num_bytes)
{
    EEPROM_24AA64_COMMAND_TYPE command;
    int8 buf[sizeof(EEPROM_24AA64_COMMAND_TYPE)];

    unsigned int8 bytes_to_send;

    while (num_bytes)
    {
        // Send control byte to EEPROM
        command.control_byte    = buf[0] = (EEPROM_24AA64_BASE_ADDRESS << 1) | EEPROM_24AA64_WRITE_BIT;
        command.word_address[0] = buf[1] = UPPER_BYTE(address);
        command.word_address[1] = buf[2] = LOWER_BYTE(address);

        //LTC6804_I2C_Write(board_num, TRUE, FALSE, &command, sizeof(command), EEPROM_24AA64_BAUD_RATE);
        LTC6804_I2C_Write(board_num, TRUE, FALSE, buf, sizeof(command), EEPROM_24AA64_BAUD_RATE);

        // Write either as much data as necessary to fill an EEPROM page, or the rest of the bytes
        bytes_to_send = MIN(EEPROM_24AA64_PAGE_SIZE - (address % EEPROM_24AA64_PAGE_SIZE), num_bytes);

        // Send the data to the EEPROM.
        LTC6804_I2C_Write(board_num, FALSE, TRUE, data_ptr, bytes_to_send, EEPROM_24AA64_BAUD_RATE);

        // Update counters to reflect data sent.
        num_bytes -= bytes_to_send;
        address += bytes_to_send;
        data_ptr += bytes_to_send;

        // Wait for the data to be written
        NUCLEO_Timer_Delay_ms(EEPROM_24AA64_TWC);
    }
    return;
}

// Reads a series of bytes to the 24AA64 EEPROM, following datasheet Figure 8-3
void Eeprom_24AA64_Read(int16 board_num, int16 address, int8* data_ptr, int16 num_bytes)
{
    EEPROM_24AA64_COMMAND_TYPE command;
    int8 buf[sizeof(EEPROM_24AA64_COMMAND_TYPE)];

    while(num_bytes)
    {
        // Send control byte to EEPROM to write the start address.
        command.control_byte    = buf[0] = (EEPROM_24AA64_BASE_ADDRESS << 1) | EEPROM_24AA64_WRITE_BIT;
        command.word_address[0] = buf[1] = UPPER_BYTE(address);
        command.word_address[1] = buf[2] = LOWER_BYTE(address);

        //LTC6804_I2C_Write(board_num, TRUE, FALSE, &command, sizeof(command), EEPROM_24AA64_BAUD_RATE);
        LTC6804_I2C_Write(board_num, TRUE, FALSE, buf, sizeof(command), EEPROM_24AA64_BAUD_RATE);

        // You can not read from I2C without first writing an address to read from.
        // Use first location where data is to be stored for the address
        *data_ptr = (EEPROM_24AA64_BASE_ADDRESS << 1) | EEPROM_24AA64_READ_BIT;

        // Read the data from the EEPROM.
        LTC6804_I2C_Read(board_num, TRUE, TRUE, data_ptr, num_bytes+1, EEPROM_24AA64_BAUD_RATE);

        // We can read all at once;  no page limitations for reading
        num_bytes = 0;
    }
    return;
}

// Erases the full contents of the 24AA64 EEPROM
void Eeprom_24AA64_Erase(int16 board_num)
{
    int8 erase_data[EEPROM_24AA64_PAGE_SIZE];
    int16 erase_address = 0;

    memset(erase_data, 0xFF, sizeof(erase_data));

    while (erase_address < EEPROM_24AA64_SIZE)
    {
        Eeprom_24AA64_Write(board_num, erase_address, erase_data, sizeof(erase_data));
        erase_address += EEPROM_24AA64_PAGE_SIZE;
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
