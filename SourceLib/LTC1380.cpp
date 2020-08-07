/*
 Linear Technology DC2100A Demonstration Board.
 Driver File for LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.
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
 $Revision: 608 $
 $Date: 2014-08-20 14:34:15 -0400 (Wed, 20 Aug 2014) $

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

//! @defgroup LTC1380 LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.

/*! @file
    @ingroup LTC1380
    Driver File for LTC1380 Single-Ended 8-Channel/Differential 4-Channel Analog Multiplexer with SMBus Interface.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "LTC1380.h"
#include "LTC1380_Config.h"
#include "NUCLEO_Timer.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// I2C address and format from datasheet page 8
#define LTC1380_BASE_ADDRESS               0x48

typedef struct
{
    int8 address_byte;
    int8 command_byte;
} LTC1380_COMMAND_TYPE;

// command byte format from datasheet table 2
#define LTC1380_EN_BIT                      0x08
#define LTC1380_CHANNEL_MASK                0x07

// Timing characteristics from datasheet page 3
#define LTC1380_TON                         2        // in us, max value

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

// Commands an LTC1380 mux to connect one channel to its output
void LTC1380_Set_Channel(int16 board_num, int8 mux_num, int8 channel_num)
{
    LTC1380_COMMAND_TYPE command;

    if((LTC1380_CONFIG_NUM_ICS_PER_ADDRESS <= mux_num) ||
       (LTC1380_NUM_CHANNELS <= channel_num))
    {
        return;
    }

    // Build command to control the mux
    command.address_byte = (LTC1380_BASE_ADDRESS | mux_num) << 1;
    command.command_byte = LTC1380_EN_BIT | channel_num;

    // Send command to control the mux
    int8 buf[sizeof(LTC1380_COMMAND_TYPE)];
    memcpy(buf, &command, sizeof(LTC1380_COMMAND_TYPE));
    LTC1380_CONFIG_I2C_WRITE(board_num, buf, sizeof(buf), LTC1380_BAUD_RATE);
    //LTC1380_CONFIG_I2C_WRITE(board_num, &command, sizeof(command), LTC1380_BAUD_RATE); // #Changed - Replacing Struct with an array of its contents due to the compile error: "cannot convert 'LTC1380_COMMAND_TYPE*' to 'char*' "

    // Wait for the channel to connect
    NUCLEO_Timer_Delay_us(LTC1380_TON);

    return;
}

// Commands an LTC1380 mux to disconnect all channels from its output
void LTC1380_All_Off(int16 board_num, int8 mux_num)
{
    LTC1380_COMMAND_TYPE command;

    if(LTC1380_NUM_CHANNELS <= mux_num)
    {
        return;
    }

    // Build command to control the mux
    command.address_byte = (LTC1380_BASE_ADDRESS | mux_num) << 1;
    command.command_byte = ~LTC1380_EN_BIT;

    // Send command to control the mux
    int8 buf[sizeof(LTC1380_COMMAND_TYPE)];
    memcpy(buf, &command, sizeof(LTC1380_COMMAND_TYPE));
    LTC1380_CONFIG_I2C_WRITE(board_num, buf, sizeof(buf), LTC1380_BAUD_RATE);
    //LTC1380_CONFIG_I2C_WRITE(board_num, &command, sizeof(command), LTC1380_BAUD_RATE); // #Changed - Replacing Struct with an array of its contents due to the compile error: "cannot convert 'LTC1380_COMMAND_TYPE*' to 'char*' "

    return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
