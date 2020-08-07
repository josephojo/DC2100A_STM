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

#ifndef __24AA64_H__
#define __24AA64_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name 24AA64 EEPROM Properties
//! @{
//! EEPROM capacity and page size from datasheet Description on page 1
#define EEPROM_24AA64_SIZE                  8192    //!< Number of bytes in the EEPROM
#define EEPROM_24AA64_PAGE_SIZE             32      //!< Number of bytes that can be operated on at a time.
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Writes a series of bytes to the 24AA64 EEPROM
//! @return void
void Eeprom_24AA64_Write(int16 board_num, //!< The logical address for the PCB containing this EEPROM.
                         int16 address,  //!< The address in the EEPROM.
                         int8* data_ptr, //!< Pointer to the data to write.
                         int16 num_bytes //!< The number of bytes to write.
                         );

//! Reads a series of bytes to the 24AA64 EEPROM
//! @return void
void Eeprom_24AA64_Read(int16 board_num, //!< The logical address for the PCB containing this EEPROM.
                        int16 address,  //!< The address in the EEPROM.
                        int8* data_ptr, //!< Pointer where to store the read data.
                        int16 num_bytes //!< The number of bytes to read.
                        );

//! Erases the full contents of the 24AA64 EEPROM
//! @return void
void Eeprom_24AA64_Erase(int16 board_num //!< The logical address for the PCB containing this EEPROM.
                        );

#endif
