/*
 Linear Technology DC2100A Demonstration Board.
 Driver Configuration Header File for LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.

 @verbatim
 The LTC1380/LTC1393 are CMOS analog multiplexers with
 SMBus compatible digital interfaces. The LTC1380 is a
 single-ended 8-channel multiplexer, while the LTC1393 is a
 differential 4-channel multiplexer. The SMBus digital interface
 requires only two wires (SCL and SDA). Both the
 LTC1380 and the LTC1393 have four hard-wired SMBus
 addresses, selectable with two external address pins. This
 allows four devices, each with a unique SMBus address, to
 coexist on one system and for four devices to be synchronized
 with one stop bit.
 The supply current is typically 10mA. Both digital interface
 pins are SMBus compatible over the full operating supply
 voltage range. The LTC1380 analog switches feature a
 typical RON of 35W (±5V supplies), typical switch leakage of
 20pA and guaranteed break-before-make operation. Charge
 injection is ±1pC typical.
 The LTC1380/LTC1393 are available in 16-lead SO and GN
 packages. Operation is fully specified over the commercial
 and industrial temperature ranges.
 @endverbatim

 http://www.linear.com/product/LTC1380

 REVISION HISTORY
 $Revision: 541 $
 $Date: 2014-07-30 18:54:03 -0400 (Wed, 30 Jul 2014) $

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
    @ingroup LTC1380
    Driver Configuration Header File for LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.
*/

#ifndef __LTC1380_CONFIG_H__
#define __LTC1380_CONFIG_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "LTC6804-2.h"                      // Interface for LTC1380s on DC2100A is through LTC6804-2
#include "DC2100A.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Configures the number of LTC1380 ICs at each logical address.
#define LTC1380_CONFIG_NUM_ICS_PER_ADDRESS              DC2100A_NUM_MUXES

//! Configures interface through which LTC1380 driver module sends I2C bytes to an LTC1380 IC.
//! - Parameters
//!     + address: The logical address for the PCB containing this LTC1380 IC.
//!     + data_ptr: Pointer to LTC1380 bytes to be written via I2C.
//!     + num_bytes: Number of bytes to be written via I2C.
//!     + baud_khz: The baud rate of the I2C communication in kHz.
//! - Usage Examples
//!     + If communication is through an LTC6804-2, this macro would call the LTC6804-2 driver's I2C Write function.
//!     + If communication is direct from the microcontroller, this macro would call the microcontroller's I2C Write function.
#define LTC1380_CONFIG_I2C_WRITE(address, data_ptr, num_bytes, baud_khz)        \
        LTC6804_I2C_Write(address, TRUE, TRUE, data_ptr, num_bytes, baud_khz);


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
