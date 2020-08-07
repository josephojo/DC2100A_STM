/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for Detecting all DC2100A boards, managing them, and indicating the state of the system.

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
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "../mbed.h"
//#include "PIC18F47J53_registers.h" // #ComeBack - Commented out Temporarily
#include "EEPROM.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


#define STATUS_TASK_RATE             100    // in ms, the rate at which the state is monitored
#define DETECT_TASK_RATE            1000    // in ms, the rate at which new boards are detected after the init state


// The possible states for the DC2100A firwmare
typedef enum {
    SYSTEM_STATE_OFF,                       // The PIC is not powered (HW state)
    SYSTEM_STATE_NUCLEO_BOARD_INIT,            // The PIC is powered and is being initialized.  The PIC must establish comm with the LTC6804-2 on its PCB to leave this state.
    SYSTEM_STATE_INIT,                      // The PIC has initialized the LTC6804-2 on its PCB, and is now searching for attached DC2100A-D PCBs.
    SYSTEM_STATE_AWAKE,                     // The PIC has monitoring and controlling all of the DC2100A in the system.
    SYSTEM_STATE_SLEEP,                     // todo - The PIC is allowing the DC2100A to enter low power mode.
    SYSTEM_NUM_STATES
} SYSTEM_STATE_TYPE;

// The settings specific to the SuperCap Demo system using the DC2100A // Not using this, but have it nonetheless since the System_Cap_Demo var is sent to host PC
typedef struct {
    int8 demo_present       : 1;            // This DC2100A is part of a SuperCap Demo system
    int8 charging           : 1;            // Turn on the charger in the SuperCap Demo system
    int8 discharging        : 1;            // Turn on the discharger in the SuperCap Demo system
    int8 unused             : 5;
} SYSTEM_CAP_DEMO_TYPE;

// Key needed to completely reset the manufacturing data in the PIC Flash and EEPROM on a DC2100A PCB.
#define SYSTEM_RESET_KEY            "system_reset"
#define SYSTEM_RESET_KEY_SIZE       (sizeof(SYSTEM_RESET_KEY) - 1)

// Key needed to make the PIC reboot into the bootloader mode.
#define SYSTEM_BOOTLOAD_KEY        "bootload"
#define SYSTEM_BOOTLOAD_KEY_SIZE    (sizeof(SYSTEM_BOOTLOAD_KEY) - 1)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
extern int16 System_Num_Boards;                          // The number of boards attached to this DC2100A system.
extern int8 System_Address_Table[DC2100A_MAX_BOARDS];   // The physical address (set with JP1-JP4) for each board attached to this DC2100A system.

//// Manufacturing Data contained in the PIC to allow the proper model and serial number to be passed during USB enumeration. // #Changed - Commented out, #Scrapping - Does not seemed needed in the forseeable future
//extern EEPROM_MFG_DATA_TYPE System_Pic_Board_Mfg_Data;
//extern BOOLEAN System_Pic_Board_Mfg_Data_Valid;

extern char System_Model[DC2100A_MAX_BOARDS];          // The model of each DC2100A board in this system

SYSTEM_STATE_TYPE returnSysState();


// Manufacturing Data contained in the PIC to allow the proper model and serial number to be passed during USB enumeration.
extern SYSTEM_CAP_DEMO_TYPE System_Cap_Demo;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void System_Init(void);                                 // Initializes the System code module.
void System_Status_Task(void);                          // Manages the DC2100 state and indicates via LED
void System_Detect_Task(void);                          // Detect new boards as they wake up in the system.
BOOLEAN System_Powered_Up(void);                        // Returns if all of the 6804-2 ICs in the system are powered up and communicating with the PIC.

BOOLEAN System_Mfg_Data_Get(EEPROM_MFG_DATA_TYPE* mfg_data);  // Gets the manufacturing data for the DC2100A board with the PIC.
void System_Mfg_Data_Set(EEPROM_MFG_DATA_TYPE* mfg_data);     // Sets the manufacturing data for the DC2100A board with the PIC.
void System_Mfg_Data_Reset(char* reset_key);                  // Resets the manufacturing data for the DC2100A board with the PIC.


#endif
