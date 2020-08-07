/*
 Linear Technology DC2100A Demonstration Board.
 API Header File for LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.
 All datasheet references in this file refer to Linear Technology document 138093f.pdf.

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
 $Revision: 687 $
 $Date: 2014-09-05 14:51:22 -0400 (Fri, 05 Sep 2014) $

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
    API Header File for LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.
*/

#ifndef __LTC1380_H__
#define __LTC1380_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name LTC3300-1 Driver Properties
//! @{
#define LTC1380_NUM_CHANNELS            8   //! Num addresses and channels from datasheet Description on page 1
#define LTC1380_BAUD_RATE               100 //! in kHz, Max SMBus Operating Frequency (fSMB from datasheet page 3)
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Commands an LTC1380 mux to connect one channel to its output.
//! @return void
void LTC1380_Set_Channel(int16 board_num,            //!< The logical address for the PCB containing this LTC1380 IC.
                         int8 mux_num,              //!< The number for the LTC1380 IC, must be less than LTC1380_CONFIG_NUM_ICS_PER_ADDRESS.
                         int8 channel_num           //!< The channel number to set for the LTC1380 IC.
                         );

//! Commands an LTC1380 mux to disconnect all channels from its output.
//! @return void
void LTC1380_All_Off(int16 board_num,                //!< The logical address for the PCB containing this LTC1380 IC.,
                     int8 mux_num                   //!< The number for the LTC1380 IC, must be less than LTC1380_CONFIG_NUM_ICS_PER_ADDRESS.
                     );

#endif
