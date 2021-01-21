/*
 Linear Technology DC2100A Demonstration Board.
 Driver File for LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer.
 All datasheet references in this file refer to Linear Technology Corp. document 33001fa.pdf.

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
 $Revision: 1395 $
 $Date: 2015-05-28 16:12:35 -0400 (Thu, 28 May 2015) $

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

//! @defgroup LTC3300-1 LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer
/*! @file
 @ingroup LTC3300-1
 Driver File for LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer
 */

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "LTC3300-1.h"
#include "LTC3300-1_Config.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//// Command Byte format and address specified in datasheet Table 2 // Changed - Moved the next 16 lines from LTC3300-1.cpp to LTC3300-1.h
//#define LTC3300_COMMAND_SIZE                    sizeof(int8)   // 8 bit command
//#define LTC3300_COMMAND(address, cmd, parity)   ((address << 3) + (cmd <<1) + parity)
//#define LTC3300_ADDRESS                         0x15
//
//// Command Bit codes specified in datasheet Table 3
//#define LTC3300_COMMAND_BALANCE_WRITE       LTC3300_COMMAND(LTC3300_ADDRESS, 0x0 ,1) // Write Balance Command (without Executing)
//#define LTC3300_COMMAND_BALANCE_READ        LTC3300_COMMAND(LTC3300_ADDRESS, 0x1 ,0) // Readback Balance Command
//#define LTC3300_COMMAND_STATUS_READ         LTC3300_COMMAND(LTC3300_ADDRESS, 0x2 ,0) // Read Balance Status
//#define LTC3300_COMMAND_EXECUTE             LTC3300_COMMAND(LTC3300_ADDRESS, 0x3 ,1) // Execute Balance Command
//#define LTC3300_COMMAND_SUSPEND             LTC3300_COMMAND(LTC3300_ADDRESS, 0x3 ,0) // Suspend Balance Command
//// Balance Command Register specified in datasheet Table 4
//// Read Status Register specified in datasheet Table 6
//#define LTC3300_REGISTER_SIZE               sizeof(int16)   // 16 bit balance command and status registers
//#define LTC3300_CRC_SIZE                    4               // 4 bit CRC used for registers
//#define LTC3300_REGISTER_BITS               (LTC3300_REGISTER_SIZE*BITS_PER_BYTE - LTC3300_CRC_SIZE)

// Cell Balancer Control Bits specified in datasheet Table 5
#define LTC3300_BALANCER_CONTROL_SIZE       2   // 2 bits for DnA and DnB
#define LTC3300_BALANCER_CONTROL_MASK       ((1 << LTC3300_BALANCER_CONTROL_SIZE) - 1)

// Status Bits specified in datasheet Table 6
#define LTC3300_STATUS_GATE_DRIVE_OK_SIZE           LTC3300_NUM_CELLS   // number of Gate Drive OK status bits
#define LTC3300_STATUS_GATE_DRIVE_OK_POSITION       10                  // Gate Drive OK bit positions in status register
#define LTC3300_STATUS_GATE_DRIVE_OK_MASK           MASK(LTC3300_STATUS_GATE_DRIVE_OK_SIZE, LTC3300_STATUS_GATE_DRIVE_OK_POSITION)
#define LTC3300_STATUS_CELLS_NOT_OV_SIZE            1                   // number of Cells Not OV status bits
#define LTC3300_STATUS_CELLS_NOT_OV_POSITION        9                   // Cells Not OV bit position in status register
#define LTC3300_STATUS_CELLS_NOT_OV_MASK            MASK(LTC3300_STATUS_CELLS_NOT_OV_SIZE, LTC3300_STATUS_CELLS_NOT_OV_POSITION)
#define LTC3300_STATUS_STACK_NOT_OV_SIZE            1                   // number of Stack Not OV status bits
#define LTC3300_STATUS_STACK_NOT_OV_POSITION        8                   // Stack Not OV bit positions in status register
#define LTC3300_STATUS_STACK_NOT_OV_MASK            MASK(LTC3300_STATUS_STACK_NOT_OV_SIZE, LTC3300_STATUS_STACK_NOT_OV_POSITION)
#define LTC3300_STATUS_TEMP_OK_SIZE                 1                   // number of Temp OK status bits
#define LTC3300_STATUS_TEMP_OK_POSITION             7                   // Temp OK bit position in status register
#define LTC3300_STATUS_TEMP_OK_MASK                 MASK(LTC3300_STATUS_TEMP_OK_SIZE, LTC3300_STATUS_TEMP_OK_POSITION)

// Baud Rate from datasheet page 5
#define LTC3300_BAUD_RATE                           1000     // in kHz, Clock Frequency (FCLK in datasheet)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int8 connectedLTC3300s[DC2100A_MAX_BOARDS];

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Table implements Telecommunication Union CRC-4 polynomial from datasheet figure 16
const unsigned int8 ltc3300_crc_table[1 << BITS_PER_NIBBLE] = {0x00, 0x13, 0x26, 0x35, 0x4C, 0x5F, 0x6A, 0x79,
                                                               0x9B, 0x88, 0xBD, 0xAE, 0xD7, 0xC4, 0xF1, 0xE2};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned int8 ltc3300_crc_calc(int8* ltc3300_data);
BOOLEAN ltc3300_crc_check(int8* ltc3300_data);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the LTC3300-1 code module.
void LTC3300_Init(void)
{
    memset(connectedLTC3300s, LTC3300_CONFIG_NUM_ICS_PER_ADDRESS, sizeof(connectedLTC3300s));
}

// Writes the balancer control bits for a number of cells controlled by a chain of LTC3300-1 ICs at a specific logical address.
void LTC3300_Command_Write(int16 board_num, int8* balancer_command_ptr)
{
    int8 ltc3300_num;
    int8 balancer_num;
    int16 balancer_command;
    int8 ltc3300_data[LTC3300_COMMAND_SIZE+ LTC3300_CONFIG_NUM_ICS_PER_ADDRESS * LTC3300_REGISTER_SIZE]; 
    int8* ltc3300_data_ptr; 

    // Build Command as specified in datasheet Table 4.  Note that last LTC3300 in chain gets the command bytes sent first,
    // since bytes are passed serially and the highest potential cells must be connected to the last LTC3300 in the series
    ltc3300_data_ptr = LAST_BYTE(ltc3300_data) ;
    for (ltc3300_num = 0; ltc3300_num < LTC3300_CONFIG_NUM_ICS_PER_ADDRESS; ltc3300_num++)
    {
        // Start with command byte cleared.
        balancer_command = 0;

        for (balancer_num = 0; balancer_num < LTC3300_NUM_CELLS; balancer_num++)
        {
            balancer_command <<= LTC3300_BALANCER_CONTROL_SIZE;
            balancer_command |= *balancer_command_ptr++;
        }

        // Adjust for CRC position
        balancer_command <<= LTC3300_CRC_SIZE;

        // Ensure endianess is correct for the LTC3300.
        *ltc3300_data_ptr-- = LOWER_BYTE(balancer_command);
        *ltc3300_data_ptr = UPPER_BYTE(balancer_command);

        // Add CRC to command byte
        *(ltc3300_data_ptr + 1) |= ltc3300_crc_calc(ltc3300_data_ptr);
        ltc3300_data_ptr--;
    }

    *ltc3300_data_ptr = LTC3300_COMMAND_BALANCE_WRITE;

    //    Send the command to the LTC3300s
    LTC3300_CONFIG_SPI_WRITE(board_num, ltc3300_data, sizeof(ltc3300_data), LTC3300_BAUD_RATE);

    return;
}

// Reads the balancer control bits for a number of cells controlled by a chain of LTC3300-1 ICs at a specific logical address.
BOOLEAN LTC3300_Command_Read(int16 board_num, int8* balancer_command_ptr)
{
    BOOLEAN success = TRUE;
    int8 ltc3300_num;
    int8 balancer_num;
    int16 balancer_command;
    int8 ltc3300_data[LTC3300_CONFIG_NUM_ICS_PER_ADDRESS * LTC3300_REGISTER_SIZE];
    int8* ltc3300_data_ptr;

    // Broadcast reads are not possible through the 6804
    if(board_num == LTC6804_BROADCAST)
    {
        return FALSE;
    }

    // Build Command.
    ltc3300_data[0] = LTC3300_COMMAND_BALANCE_READ;

    // Send the command to the LTC3300s
    LTC3300_CONFIG_SPI_READ(board_num, ltc3300_data, sizeof(ltc3300_data) + 1, LTC3300_BAUD_RATE);

    // Read Balance Command uses the Write Balance Command specified in datasheet Table 4
    // Note that last LTC3300 in chain sends its command bytes last, since bytes are
    // passed serially and the highest potential cells must be connected to the last LTC3300 in the series
    ltc3300_data_ptr = ltc3300_data;

    //for (int i = 0; i < sizeof(ltc3300_data); i++)
    //{
    //    printf("ltc3300_data[%d] = %d\r\n", i, ltc3300_data[i]);
    //}
    //printf("\n");

    
    for (ltc3300_num = 0; ltc3300_num < LTC3300_CONFIG_NUM_ICS_PER_ADDRESS; ltc3300_num++)
    {
        // Verify data, and extract it if valid.
        if( (ltc3300_crc_check(ltc3300_data_ptr) == TRUE) || (ltc3300_num >= connectedLTC3300s[board_num]) ) // 2nd statement prevents the throwing of LTC3300CRC errors when device is not powered
        {
            // Extract balancer command in an endianess independent way.
            balancer_command = (((int16) (*ltc3300_data_ptr)) << 8) + *(ltc3300_data_ptr + 1);

            // Adjust for CRC position
            balancer_command >>= LTC3300_CRC_SIZE;
            //printf("Sucessful?????? %d\r\n", success);

            // Read out balancer command information, with highest cell first
            for (balancer_num = LTC3300_NUM_CELLS; balancer_num > 0; balancer_num--)
            {
                *(balancer_command_ptr + balancer_num - 1) = (balancer_command & LTC3300_BALANCER_CONTROL_MASK);
                balancer_command >>= LTC3300_BALANCER_CONTROL_SIZE;
            }
        }
        else
        {
            LTC3300_CONFIG_ERROR_CRC(board_num, ltc3300_num, LTC3300_COMMAND_BALANCE_READ, ltc3300_data_ptr, LTC3300_REGISTER_SIZE);
            success = FALSE;
        }

        // point to data locations for balancer commands from next ltc3300.
        balancer_command_ptr += LTC3300_NUM_CELLS;
        ltc3300_data_ptr += LTC3300_REGISTER_SIZE;
    }

    return success;
}

// Reads the status bits for a chain of LTC3300-1 ICs at a specific logical address.
BOOLEAN LTC3300_Status_Read(int16 board_num, int8* gate_drive_ok, int8* cells_ov_ok, int8* stack_ov_ok, int8* temp_ok)
{
    BOOLEAN success = TRUE;
    int8 ltc3300_num;
    int8 cell_num;
    int16 status;
    int16 status_mask;
    int8 gate_drive_ok_mask;
    int8 ltc3300_data[LTC3300_CONFIG_NUM_ICS_PER_ADDRESS * LTC3300_REGISTER_SIZE]; 
    int8* ltc3300_data_ptr; 

    // Broadcast reads are not possible through the 6804
    if(board_num == LTC6804_BROADCAST)
    {
        return FALSE;
    }

    // Build Command.
    ltc3300_data[0] = LTC3300_COMMAND_STATUS_READ;

    // Send the command to the LTC3300s
    LTC3300_CONFIG_SPI_READ(board_num, ltc3300_data, sizeof(ltc3300_data) + 1, LTC3300_BAUD_RATE);

    // Note that last LTC3300 in chain sends its command bytes last, since bytes are
    // passed serially and the highest potential cells must be connected to the last LTC3300 in the series
    ltc3300_data_ptr = ltc3300_data;


    for (ltc3300_num = 0; ltc3300_num < LTC3300_CONFIG_NUM_ICS_PER_ADDRESS; ltc3300_num++)
    {
        // Verify data, and extract it if valid.
        if(ltc3300_crc_check(ltc3300_data_ptr) == TRUE)
        {
            // Extract balancer command in an endianess independent way.
            status = (((int16) (*ltc3300_data_ptr)) << 8) + *(ltc3300_data_ptr + 1);

            // return gate_drive_ok bits ordered such that cell 1 is in bit position 0
            status_mask = (1L << (LTC3300_STATUS_GATE_DRIVE_OK_POSITION + LTC3300_STATUS_GATE_DRIVE_OK_SIZE - 1));
            gate_drive_ok_mask = 1;
            *gate_drive_ok = 0;
            for (cell_num = 0; cell_num < LTC3300_NUM_CELLS; cell_num++)
            {
                if((status & status_mask) != 0)
                {
                    *gate_drive_ok |= gate_drive_ok_mask;
                }
                gate_drive_ok_mask <<= 1;
                status_mask >>= 1;
            }
            gate_drive_ok++;

            *cells_ov_ok++ = (status & LTC3300_STATUS_CELLS_NOT_OV_MASK) >> LTC3300_STATUS_CELLS_NOT_OV_POSITION;
            *stack_ov_ok++ = (status & LTC3300_STATUS_STACK_NOT_OV_MASK) >> LTC3300_STATUS_STACK_NOT_OV_POSITION;
            *temp_ok++ = (status & LTC3300_STATUS_TEMP_OK_MASK) >> LTC3300_STATUS_TEMP_OK_POSITION;
        }
        else
        {
            // #Changed - Using a variable "connectedLTC3300s" that is modified based on the number of cells connected. 
            // Prevents the throwing of LTC3300CRC errors when device is not powered
            if (ltc3300_num < connectedLTC3300s[board_num])
            {
                LTC3300_CONFIG_ERROR_CRC(board_num, ltc3300_num, LTC3300_COMMAND_STATUS_READ, ltc3300_data_ptr, LTC3300_REGISTER_SIZE);
                success = FALSE;
            }
        }

        ltc3300_data_ptr += LTC3300_REGISTER_SIZE;
    }

    return success;
}

// Commands a chain of LTC3300s at a specific logical address to execute their balance commands.
void LTC3300_Execute(int16 board_num)
{
    int8 ltc3300_data[LTC3300_COMMAND_SIZE];  

    // Build Command.  Note that last LTC3300 in chain gets the command bytes sent first, since bytes are
    // passed serially and the highest potential cells must be connected to the last LTC3300 in the series
    ltc3300_data[0] = LTC3300_COMMAND_EXECUTE;

    //    Send the command to the LTC3300s
    LTC3300_CONFIG_SPI_WRITE(board_num, ltc3300_data, sizeof(ltc3300_data), LTC3300_BAUD_RATE);

    return;
}

// Commands a chain of LTC3300s at a specific logical address to suspend their balance commands.
void LTC3300_Suspend(int16 board_num)
{
    int8 ltc3300_data[LTC3300_COMMAND_SIZE]; 

    // Build Command.  Note that last LTC3300 in chain gets the command bytes sent first, since bytes are
    // passed serially and the highest potential cells must be connected to the last LTC3300 in the series
    ltc3300_data[0] = LTC3300_COMMAND_SUSPEND;

    //    Send the command to the LTC3300s
    LTC3300_CONFIG_SPI_WRITE(board_num, ltc3300_data, sizeof(ltc3300_data), LTC3300_BAUD_RATE);

    return;
}

// Sends a benign command to all chains of LTC3300s at all logical addresses to reset their watchdog timers.
void LTC3300_Watchdog_Kick(void)
{
    int8 ltc3300_data[LTC3300_COMMAND_SIZE]; 

    // Build Command.
    ltc3300_data[0] = LTC3300_COMMAND_STATUS_READ;

    // Send the command to the LTC3300s.  Do not bother reading back the registers, as it will just be using time.
    LTC3300_CONFIG_SPI_WRITE(LTC6804_BROADCAST, ltc3300_data, LTC3300_COMMAND_SIZE, LTC3300_BAUD_RATE);

    return;
}

// Sends a raw string of bytes to a chain of LTC3300s at a specific logical address.
void LTC3300_Raw_Write(int16 board_num, int8* ltc3300_data, int8 num_bytes)  
{
    //    Send the command to the LTC3300s
    LTC3300_CONFIG_SPI_WRITE(board_num, ltc3300_data, num_bytes, LTC3300_BAUD_RATE);

    return;
}

// Receives a raw string of bytes from a chain of LTC3300s at a specific logical address.
void LTC3300_Raw_Read(int16 board_num, int8* ltc3300_data, int8 num_bytes)  
{
    //    Send the command to the LTC3300s and read the results
    LTC3300_CONFIG_SPI_READ(board_num, ltc3300_data, num_bytes + 1, LTC3300_BAUD_RATE);

    return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Calculates the CRC for an LTC3300-1 Register
// return the LTC3300-1 CRC calculated for these bytes.
unsigned int8 ltc3300_crc_calc(int8* ltc3300_data    // The bytes to be written to an LTC3300-1 Register
        )
{
    unsigned int8 nybble_num;
    unsigned int8 addr;
    unsigned int8 data[LTC3300_BALANCER_CONTROL_SIZE*LTC3300_NUM_CELLS/BITS_PER_NIBBLE];
    unsigned int8 CRC_val = 0;

    data[0] = UPPER_NIBBLE(ltc3300_data[0]);
    data[1] = LOWER_NIBBLE(ltc3300_data[0]);
    data[2] = UPPER_NIBBLE(ltc3300_data[1]);

    for (nybble_num = 0; nybble_num < LTC3300_BALANCER_CONTROL_SIZE*LTC3300_NUM_CELLS/BITS_PER_NIBBLE; nybble_num++)
    {
        addr = ((CRC_val >> (LTC3300_CRC_SIZE - BITS_PER_NIBBLE)) ^ data[nybble_num]) & MASK(BITS_PER_NIBBLE, 0);   // calculate table address
        CRC_val = (CRC_val << BITS_PER_NIBBLE) ^ ltc3300_crc_table[addr];                                               // get value from table
    }

    return (~CRC_val) & 0x0F;
}

// Calculates the CRC for an LTC3300-1 register and compares it to the CRC recieved with the register.
// return True is the CRC matches, False if the CRC does not match.
BOOLEAN ltc3300_crc_check(int8* ltc3300_data)
{
    // Calc and check CRC
    if(ltc3300_crc_calc(ltc3300_data) == (ltc3300_data[1] & 0x0F))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

