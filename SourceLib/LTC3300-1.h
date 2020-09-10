/*
 Linear Technology DC2100A Demonstration Board.
 API Header File to LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer.
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
 $Revision: 693 $
 $Date: 2014-09-08 13:52:04 -0400 (Mon, 08 Sep 2014) $

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
    API Header File for LTC3300-1 High Efficiency Bidirectional Multicell Battery Balancer
*/

#ifndef __LTC3300_H__
#define __LTC3300_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"               // Definition of basic data types
#include "DC2100A.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name LTC3300-1 Driver Properties
//! @{
#define LTC3300_NUM_CELLS           6   //!< Number of cells controlled by one LTC3300
#define LTC3300_TWD1                750 //!< in ms, min Watchdog Timer Timeout Period WDT Assertion Measured from Last Valid Command Byte (from datasheet page 5)
#define LTC3300_V_SUPPLY_VOLTAGE    6500//!< mV per bit, Minimum supply voltage to ensure LTC3300s can communicate.

//! @}

// Changed - Moved the next 16 lines from LTC3300-1.cpp to LTC3300-1.h
// Command Byte format and address specified in datasheet Table 2
#define LTC3300_COMMAND_SIZE                    sizeof(int8)   // 8 bit command
#define LTC3300_COMMAND(address, cmd, parity)   ((address << 3) + (cmd <<1) + parity)
#define LTC3300_ADDRESS                         0x15

// Command Bit codes specified in datasheet Table 3
#define LTC3300_COMMAND_BALANCE_WRITE       LTC3300_COMMAND(LTC3300_ADDRESS, 0x0 ,1) // Write Balance Command (without Executing)
#define LTC3300_COMMAND_BALANCE_READ        LTC3300_COMMAND(LTC3300_ADDRESS, 0x1 ,0) // Readback Balance Command
#define LTC3300_COMMAND_STATUS_READ         LTC3300_COMMAND(LTC3300_ADDRESS, 0x2 ,0) // Read Balance Status
#define LTC3300_COMMAND_EXECUTE             LTC3300_COMMAND(LTC3300_ADDRESS, 0x3 ,1) // Execute Balance Command
#define LTC3300_COMMAND_SUSPEND             LTC3300_COMMAND(LTC3300_ADDRESS, 0x3 ,0) // Suspend Balance Command
// Balance Command Register specified in datasheet Table 4
// Read Status Register specified in datasheet Table 6
#define LTC3300_REGISTER_SIZE               sizeof(int16)   // 16 bit balance command and status registers
#define LTC3300_CRC_SIZE                    4               // 4 bit CRC used for registers
#define LTC3300_REGISTER_BITS               (LTC3300_REGISTER_SIZE*BITS_PER_BYTE - LTC3300_CRC_SIZE)



//! @name Balancer Control Codes
//! @{
//! Bit definitions for control codes written to LTC3300 Balance Control Registers.
//! Balancer Control Codes specified in datasheet Table 5
#define LTC3300_BALANCER_CONTROL_CODE_NONE               0x0 //!< Balancing Action: None
#define LTC3300_BALANCER_CONTROL_CODE_DISCHARGE_NONSYNC  0x1 //!< Balancing Action: Discharge Cell n (Nonsynchronous)
#define LTC3300_BALANCER_CONTROL_CODE_DISCHARGE_SYNC     0x2 //!< Balancing Action: Discharge Cell n (Synchronous)
#define LTC3300_BALANCER_CONTROL_CODE_CHARGE             0x3 //!< Balancing Action: Charge Cell n
#define LTC3300_BALANCER_NUM_CONTROL_CODES               0x4 //!< Number of Balancing Actions
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

extern int8 connectedLTC3300s[DC2100A_MAX_BOARDS];

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Initializes the LTC3300-1 code module.
//! @return void
void LTC3300_Init(void);

//! Writes the balancer control bits for a number of cells controlled by a chain of LTC3300-1 ICs at a specific logical address.
//! Number of ICs in chain is set at compile time by LTC3300_CONFIG_NUM_ICS_PER_ADDRESS.
//! Number of cells controlled by chain = LTC3300_CONFIG_NUM_ICS_PER_ADDRESS * LTC3300_NUM_CELLS.
//! @return void
void LTC3300_Command_Write(int16 board_num,                  //!< The logical address for the PCB containing these LTC3300-1 ICs.
                           int8* balancer_command_ptr       //!< Pointer to array of Balancer Control Bits to write to LTC3300-1 Balancer Command Registers.
                          );

//! Reads the balancer control bits for a number of cells controlled by a chain of LTC3300-1 ICs at a specific logical address.
//! Number of ICs in chain is set at compile time by LTC3300_CONFIG_NUM_ICS_PER_ADDRESS.
//! Number of cells controlled by chain = LTC3300_CONFIG_NUM_ICS_PER_ADDRESS * LTC3300_NUM_CELLS.
//! @return void
BOOLEAN LTC3300_Command_Read(int16 board_num,                //!< The logical address for the PCB containing these LTC3300-1 ICs.
                             int8* balancer_command_ptr     //!< Pointer to array where Balancer Control Bits read from LTC3300-1 Balancer Command Registers are to be returned.
                            );

//! Reads the status bits for a chain of LTC3300-1 ICs at a specific logical address.
//! Number of ICs in chain is set at compile time by LTC3300_CONFIG_NUM_ICS_PER_ADDRESS.
//! @return void
BOOLEAN LTC3300_Status_Read(int16 board_num,                 //!< The logical address for the PCB containing these LTC3300-1 ICs.
                            int8* gate_drive_ok,            //!< Pointer to array where Gate Drive OK bits read from LTC3300-1 Status Registers are to be returned.
                            int8* cells_ov_ok,              //!< Pointer to array where Cells Overvoltage OK bits read from LTC3300-1 Status Registers are to be returned.
                            int8* stack_ov_ok,              //!< Pointer to array where Stack Overvoltage OK bits read from LTC3300-1 Status Registers are to be returned.
                            int8* temp_ok                   //!< Pointer to array where Temperature OK bits read from LTC3300-1 Status Registers are to be returned.
                            );

//! Commands a chain of LTC3300s at a specific logical address to execute their balance commands.
//! @return void
void LTC3300_Execute(int16 board_num                         //!< The logical address for the PCB containing these LTC3300-1 ICs.
                    );

//! Commands a chain of LTC3300s at a specific logical address to suspend their balance commands.
//! @return void
void LTC3300_Suspend(int16 board_num                         //!< The logical address for the PCB containing these LTC3300-1 ICs.
                    );

//! Sends a benign command to all chains of LTC3300s at all logical addresses to reset their watchdog timers.
//! Uses the Read Status command as a benign command that resets the watchdog in the LTC3300 without resulting in a balancing action.
//! @return void
void LTC3300_Watchdog_Kick(void);

//! Sends a raw string of bytes to a chain of LTC3300s at a specific logical address.
//! This function is configured by the num_bytes parameter instead of LTC3300_CONFIG_NUM_ICS_PER_ADDRESS.
//! It is not recommended to use this function.  It primarily exists for the DC2100A GUI to display the raw communication to/from the LTC3300s.
//! @return void
void LTC3300_Raw_Write(int16 board_num,                      //!< The logical address for the PCB containing these LTC3300-1 ICs.
                       int8* ltc3300_data,                  //!< Pointer to the raw bytes to send to the chain of LTC3300-1 ICs. 
                       int8 num_bytes                       //!< The number of raw bytes to send to the chain of LTC3300-1 ICs.
                      );

//! Receives a raw string of bytes from a chain of LTC3300s at a specific logical address.
//! This function is configured by the num_bytes parameter instead of LTC3300_CONFIG_NUM_ICS_PER_ADDRESS.
//! It is not recommended to use this function.  It primarily exists for the DC2100A GUI to display the raw communication to/from the LTC3300s.
//! @return void
void LTC3300_Raw_Read(int16 board_num,                       //!< The logical address for the PCB containing these LTC3300-1 ICs.
                      int8* ltc3300_data,                   //!< Pointer to where the raw bytes read from the chain of LTC3300-1 ICs should be stored. 
                      int8 num_bytes                        //!< The number of raw bytes to read from the chain of LTC3300-1 ICs.
                     );

#endif
