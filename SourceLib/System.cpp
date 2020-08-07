/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for Detecting all DC2100A boards, managing them, and indicating the state of the system.

 This code detects and manages the state of the DC2100A system and indicates it through the Green LED D15.  The states of the system
 are defined in SYSTEM_STATE_TYPE.  Most of the tasks in the DC2100A system will only begin to operate once the system has reached the
 SYSTEM_STATE_AWAKE state.

 A DC2100A system can contain different numbers of boards, and these boards can have 16 different physical addresses set by JP1-JP4.
 The number of boards and the mapping between the logical and physical addresses are maintained by this code module and made available
 to the rest of the FW and SW as System_Num_Boards and System_Address_Table[].  Note that the DC2100A containing the PIC must have a
 special address as defined by DC2100A_NUCLEO_BOARD_NUM, where the remaining physical addresses are free for the DC2100A-D boards connected
 to the system.

 As boards are detected they are initialized and their manufacturing data is retrieved.  If the boards cease communication, they are
 assumed to still be present but in a state where the 6804-2 does not have sufficient voltage for communication.  This code module detects
 this conditions and reinitialize the boards as they regain communication with the PIC18.

 http://www.linear.com/solutions/5126

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


Library Adopted and Modified for use by Joseph Ojo (March 2020)

*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h" 
//#include "PIC18F47J53_registers.h" // #Scrapping - Not going to use this
//#include "./bootloader/boot.inc" // #Scrapping - Not going to use this
#include "DC2100A.h"
#include "System.h"
#include "LTC6804-2.h"
#include "LTC3300-1.h"
#include "Eeprom.h"
#include "Voltage.h"
#include "Temperature.h"
//#include "Balancer.h"
#include "USB_Parser.h"        // #NeededNow??? - Might need this for USB Communication
#include "Error.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define SYSTEM_SHUTDOWN_VOLTAGE     (MAX2(LTC6804_V_SUPPLY_VOLTAGE, LTC3300_V_SUPPLY_VOLTAGE*DC2100A_NUM_LTC3300)*MV_PER_V/LTC6804_SOC_RESOLUTION)  // Voltage below which LTC6804 and LTC3300s are not certain to be operational.
#define SYSTEM_SHUTDOWN_HYSTERESIS  (100*MV_PER_V/LTC6804_SOC_RESOLUTION)
#define SYSTEM_WAKEUP_VOLTAGE       (SYSTEM_SHUTDOWN_VOLTAGE + SYSTEM_SHUTDOWN_HYSTERESIS)      // Voltage at which LTC6804 is certain to be operational, and initialization is performed.
#define SYSTEM_CRC_LIMIT            (2)                                                         // The number of CRC errors per task execution, before the board will re-initialize

DigitalOut ledPin(D8);
#define LED_STATE_PIN               ledPin  // The pin for the LED indicator


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
SYSTEM_STATE_TYPE System_State;                 // The state of the DC2100A stacked system
int16 System_Num_Boards;                         // The number of DC2100A boards stacked together in this system
int8 System_Address_Table[DC2100A_MAX_BOARDS];  // Table to map DC2100A logical addresses to LTC6804-2 physical addresses
char System_Model[DC2100A_MAX_BOARDS]; // #Changed - Add this here
SYSTEM_CAP_DEMO_TYPE System_Cap_Demo;           // The settings specific to the Cap Demo implementation of the DC2100A // #Scrapping - Not going to use this


//// RAM storage for the manufacturing data in PIC board.  Since this data is needed for the USB String Descriptors,        // #NeededNow??? - Might need this for USB Communication
//// and the String Descriptors are fetched in an interrupt, there is value not making them get read out of Flash + a CRC calculation each time they're read.
//EEPROM_MFG_DATA_TYPE System_Pic_Board_Mfg_Data;
//BOOLEAN System_Pic_Board_Mfg_Data_Valid;
//
//BOOT_CONTROL_TYPE DC2100A_Reboot_Control;               // Control for the reboot of the Application into the Bootloader
//#LOCATE DC2100A_Reboot_Control = BOOT_RAM_KEY_LOCATION  // Locate at a specific location, so that the same RAM is used as a key for the bootloader

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/*//// Storage for manufacturing data in flash.  This is necessary since we wish SuperCap Demo boards to identify themselves   // #NeededNow??? - Might need this for USB Communication
//// properly before the cells (and LTC6804 + EEPROM) are powered.
//#ORG APP_FLASH_DATA
//rom char system_mfg_data_flash[sizeof(EEPROM_MFG_DATA_TYPE) + LTC6804_PEC_SIZE] = \
//{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
//  0xFF, \
//  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
//  0xFF, 0xFF
//};
//
//#ORG (APP_FLASH_DATA + sizeof(system_mfg_data_flash) + sizeof(system_mfg_data_flash)%2), APP_ROM_END {}*/

// Period in ms at which the heartbeat LED will be flashed to indicate board status
const unsigned int16 system_state_led_period[SYSTEM_NUM_STATES] = {  0,         // SYSTEM_STATE_OFF
                                                                     200,       // SYSTEM_STATE_NUCLEO_BOARD_INIT
                                                                     200,       // SYSTEM_STATE_INIT
                                                                     1000,      // SYSTEM_STATE_AWAKE
                                                                     10000 };   // SYSTEM_STATE_SLEEP

// Timer to control flashing of LED to indicate state.
unsigned int16 system_state_led_timer;

// The current board address being detected
unsigned int16 system_address_to_detect;

// Flag set when a 6804 has a voltage below its operational threshold, and may be resetting the values that are cleared upon powerup/sleep
BOOLEAN system_voltage_low_flag;
BOOLEAN system_crc_count_flag;
unsigned int16 system_crc_count;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
BOOLEAN system_detect_task_ltc6804_address(int8 address_to_detect);
void system_board_detected_init(void);
BOOLEAN system_wakeup_init(void);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the System code module.
// Retrieves the manufacturing info for the PIC board.  If not present, use default settings as a cap demo board in case the charger is
// needed to power the cells.
void System_Init(void)
{
    //EEPROM_MFG_DATA_TYPE mfg_data; // #NeededNow??? - Might need this for USB Communication
    //BOOLEAN mfg_data_valid; // #NeededNow??? - Might need this for USB Communication

    // Initialize system variables.
    System_State = SYSTEM_STATE_NUCLEO_BOARD_INIT;
    //System_Num_Boards = 1;  // #Changed - Changed this from 0 to 1 cuz currently don't want to deal with it
    System_Num_Boards = 0;  
    system_state_led_timer = 0;
    system_address_to_detect = 0;
    system_voltage_low_flag = TRUE;  
    //system_voltage_low_flag = FALSE;  // #Changed - Changed this from true to false cuz I currently don't want to deal with implementing what sets it to false
    system_crc_count_flag = TRUE;  
    //system_crc_count_flag = FALSE;  // #Changed - Changed this from true to false cuz I currently don't want to deal with implementing what sets it to false
    system_crc_count = 0;

    // Initialize address table to LTC6804_MAX_BOARDS (16). Since the max address possible is 15 on JP1 - JP4. Can't use zero since the intial board is address = 0
    memset(System_Address_Table, LTC6804_MAX_BOARDS, sizeof(System_Address_Table));

    System_Model[DC2100A_NUCLEO_BOARD_NUM] = 'C'; // #Changed - Added this in place of how it was previously done by retreiving it from the PIC's memory

    // #Changed - Added this in place of the block below that decides these values depending on the Mfg Data on MCU. Our MCU isn't setup to store that data.
    System_Cap_Demo.demo_present = 0;
    System_Cap_Demo.charging = 0;
    System_Cap_Demo.discharging = 0;


    //// Check if the PIC has manufacturing data in its Flash Data Page. // #NeededNow??? - Might need this for USB Communication
    //mfg_data_valid = System_Mfg_Data_Get(&mfg_data);
    //
    //// If manufacturing data is available, initialize such that USB enumerates as the correct model/sn
    //if(mfg_data_valid == TRUE)
    //{
    //    // Store in RAM so that USB Get String Descriptor requests can be answered quickly.
    //    System_Pic_Board_Mfg_Data_Valid = TRUE;
    //    System_Pic_Board_Mfg_Data = mfg_data;
    //
    //    // Model is the last character in the model string.
    //    System_Model[DC2100A_NUCLEO_BOARD_NUM] = mfg_data.model_num[DC2100A_MODEL_NUM_SIZE - 1];
    //
    //    //if(mfg_data.cap_demo == 'T') // #Scrapping - Not going to use this
    //    //{
    //    //    System_Cap_Demo.demo_present = 1;
    //    //
    //    //    // Initialize charger and discharger to be outputs, and initialize to be off
    //    //    CHARGER_OUT_PIN_SETUP;
    //    //    DISCHARGER_OUT_PIN_SETUP;
    //    //    System_Cap_Demo.charging = 0;
    //    //    System_Cap_Demo.discharging = 0;
    //    //}
    //    //else
    //    //{
    //    //    System_Cap_Demo.demo_present =  0;
    //    //}
    //
    //    System_Cap_Demo.demo_present = 0;
    //}
    //else
    //{
    //    // Store in RAM so that USB Get String Descriptor requests can be answered quickly.
    //    System_Pic_Board_Mfg_Data_Valid = FALSE;
    //    System_Pic_Board_Mfg_Data.model_num = DC2100A_MODEL_NUM_DEFAULT;
    //    System_Pic_Board_Mfg_Data.cap_demo = DC2100A_CAP_DEMO_DEFAULT;
    //    System_Pic_Board_Mfg_Data.serial_num = DC2100A_SERIAL_NUM_DEFAULT;
    //
    //    // If manufacturing data is not available, initialize to be unknown board that's part of cap demo system
    //    System_Model[DC2100A_NUCLEO_BOARD_NUM] = '?';
    //    System_Cap_Demo.demo_present = 1;
    //
    //    // Initialize charger and discharger to be outputs, and initialize to be off
    //    CHARGER_OUT_PIN_SETUP;
    //    DISCHARGER_OUT_PIN_SETUP;
    //    System_Cap_Demo.charging = 0;
    //    System_Cap_Demo.discharging = 0;
    //}
    //

    // Initialize models for non-PIC boards to be unknown.
    memset(&System_Model[1], '?', sizeof(System_Model) - 1);
    
    //// Initialize to not reboot to the bootloader
    //DC2100A_Reboot_Control._app_reboot_flags = 0;
}

SYSTEM_STATE_TYPE returnSysState()
{
    return System_State;
}

// Manages the DC2100 state and indicates via LED
 void System_Status_Task(void)
{
    int16 board_num;
    unsigned int16 system_crc_count_next = Error_Count_Get(ERROR_CODE_LTC6804_CRC);
    BOOLEAN system_crc_count_flag_next = system_crc_count_flag;
    BOOLEAN system_voltage_low_flag_next = system_voltage_low_flag;
    SYSTEM_STATE_TYPE system_state_next = System_State;

    // Strobe the LED to indicate the DC2100A state.
    if(STATUS_TASK_RATE < system_state_led_timer)
    {
        system_state_led_timer -= STATUS_TASK_RATE;
    }
    else
    {
        //TOGGLE(DEBUG0_OUT_PIN);  // #ComeBack - We should have LED indication
        TOGGLE(LED_STATE_PIN); 
        system_state_led_timer = system_state_led_period[System_State];
    }

    //// If configured as a demo system, manage the charger and discharger. // #Scrapping - Not going to use this
    //if(System_Cap_Demo.demo_present)
    //{
    //    CHARGER_OUT_PIN = System_Cap_Demo.charging ? 1 : 0;
    //    DISCHARGER_OUT_PIN = System_Cap_Demo.discharging ? 0 : 1;
    //}
    //
    //// If bootload is requested and no tasks are disallowing it, set RAM key for bootload operation and reset.
    //if(DC2100A_Reboot_Control._app_reboot_flags & BOOT_CONTROL_BOOTLOAD_REQUESTED)
    //{
    //    if((DC2100A_Reboot_Control._app_reboot_flags & BOOT_CONTROL_DISALLOW_FLAGS) == 0)
    //    {
    //        memcpy(&DC2100A_Reboot_Control._boot_ram_key, boot_ram_code, sizeof(boot_ram_code));
    //        reset_cpu();
    //    }
    //}

    // Operate system depending upon the operating state.
    switch (System_State)
    {
        case SYSTEM_STATE_NUCLEO_BOARD_INIT:

            // Do not attempt to detect any boards until the PIC board is powered up.
            if(system_detect_task_ltc6804_address(DC2100A_NUCLEO_BOARD_NUM) == TRUE)
            {
                system_board_detected_init();
            }

            // Enter the Awake State when any boards are detected.
            if(System_Num_Boards != 0)
            {
                system_state_next = SYSTEM_STATE_INIT;
            }
            break;

        case SYSTEM_STATE_INIT:
            // Detect if any more boards are powered up in the system
            for (system_address_to_detect = DC2100A_NUCLEO_BOARD_NUM + 1; system_address_to_detect < LTC6804_MAX_BOARDS; system_address_to_detect++)
            {
                if(system_detect_task_ltc6804_address(system_address_to_detect) == TRUE)
                {
                    system_board_detected_init();
                }

                // Check all of the possible addresses for the LTC6804, but do not detect more than the DC2100A FW can handle
                if(System_Num_Boards >= DC2100A_MAX_BOARDS)
                {
                    break;
                }
            }

            // Enter the Awake State when any boards are detected.
            system_state_next = SYSTEM_STATE_AWAKE;
            break;

        case SYSTEM_STATE_AWAKE:
            // Test for voltages to be below the 6804 threshold.
            system_voltage_low_flag_next = FALSE;
            for (board_num = 0; board_num < System_Num_Boards; board_num++)
            {
                if (system_voltage_low_flag == FALSE)
                {
                    // If system voltage was not previously low, then any board below the threshold can make it low
                    if(voltage_sum[board_num] < SYSTEM_SHUTDOWN_VOLTAGE)
                    {
                        system_voltage_low_flag_next = TRUE;
                    }
                }
                else
                {
                    // If system voltage was previously low, then all boards must be above the threshold to clear it
                    if(voltage_sum[board_num] < SYSTEM_WAKEUP_VOLTAGE)
                    {
                        system_voltage_low_flag_next = TRUE;
                    }
                }
            }

            // Check for too many CRC errors that could indicate a need to reinitialize the 6804, once reliable communication has restarted.
            if(system_crc_count == system_crc_count_next)
            {
                system_crc_count_flag_next = FALSE;
            }
            else if((system_crc_count_next - system_crc_count) > SYSTEM_CRC_LIMIT)
            {
                system_crc_count_flag_next = TRUE;
            }
            system_crc_count = system_crc_count_next;

            // todo - control going to sleep to demonstrate low power capability of LTC3300 and LTC6804
//            if()
//            {
//                system_state_next = SYSTEM_STATE_SLEEP;
//            }

            break;

        case SYSTEM_STATE_SLEEP:
            // todo - control waking from sleep to demonstrate low power capability of LTC3300 and LTC6804
            system_state_next = SYSTEM_STATE_AWAKE;
            break;
    }

    // If entering the awake state, or recovering from a time when the 6804 could not operate, perform the actions necessary when waking up.
    if(((System_State != SYSTEM_STATE_AWAKE) && (system_state_next == SYSTEM_STATE_AWAKE)) ||
       ((system_voltage_low_flag == TRUE) && (system_voltage_low_flag_next == FALSE)) ||
       ((system_crc_count_flag == TRUE) && (system_crc_count_flag_next == FALSE)))
    {
        if (system_wakeup_init() == TRUE) //#Changed - Not sure why, but the is here was "If" not "if". I switched it back in the meantime but hovering on it gives a desc
        {
            // Change the state if the wakeup init was successful
            System_State = system_state_next;
            system_voltage_low_flag = system_voltage_low_flag_next;
            system_crc_count_flag = system_crc_count_flag_next;
        }
    }
    else
    {
        // Not waking up, set flags to detect wakeup condition next time task is run.
        System_State = system_state_next;
        system_voltage_low_flag = system_voltage_low_flag_next;
        system_crc_count_flag = system_crc_count_flag_next;
    }

    return;
}

// Detect new boards as they wake up in the system.
// Necessary when the PIC is powered up before the cells so that they can not all be detected at init time.
void System_Detect_Task(void)
{
    // Check all of the possible addresses for the LTC6804, but do not detect more than the DC2100A FW can handle
    if((System_State == SYSTEM_STATE_AWAKE) && (System_Num_Boards < DC2100A_MAX_BOARDS))
    {
        // Increment to the next board address to check.
        if(system_address_to_detect < (LTC6804_MAX_BOARDS - 1))
        {
            system_address_to_detect++;
        }
        else
        {
            system_address_to_detect = 0;
        }

        // Check if this board was added to the system after init state.
        if(system_detect_task_ltc6804_address(system_address_to_detect) == TRUE)
        {
            system_board_detected_init();
            system_wakeup_init();
            //// Send an async to notify the GUI that the board configuration has changed.
            USB_Parser_System_Data_Async(); 
        }
    }

    return;
}

// Returns if all of the 6804-2 ICs in the system are powered up and communicating with the PIC.
BOOLEAN System_Powered_Up(void)
{
    if((system_voltage_low_flag == TRUE) || (system_crc_count_flag == TRUE))
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

//// Gets the manufacturing data for the DC2100A board with the PIC.  // #NeededNow??? - This currently reads mfg data from PIC MCU to recognize it. It also uses system_mfg_data_flash which is commented out
//// Returns true if the manufacturing data is valid.
//BOOLEAN System_Mfg_Data_Get(EEPROM_MFG_DATA_TYPE* mfg_data)
//{
//    unsigned int16 crc_read, crc_calc;
//    int32 address = system_mfg_data_flash;
//
//    // read the data out of program memory
//    read_program_memory(address, mfg_data, sizeof(EEPROM_MFG_DATA_TYPE));
//    read_program_memory(address + sizeof(EEPROM_MFG_DATA_TYPE), &crc_read, LTC6804_PEC_SIZE);
//
//    // verify the crc
//    crc_calc = LTC6804_PEC_Calc((char*)mfg_data, sizeof(EEPROM_MFG_DATA_TYPE));
//    if(crc_calc == crc_read)
//    {
//        return TRUE;
//    }
//    else
//    {
//        return FALSE;
//    }
//}

//// Sets the manufacturing data for the DC2100A board with the PIC. // #NeededNow??? - This currently sets mfg data from PIC MCU to recognize it. It also uses system_mfg_data_flash which is commented out
//void System_Mfg_Data_Set(EEPROM_MFG_DATA_TYPE* mfg_data)
//{
//    unsigned int16 crc_calc;
//    int32 address = system_mfg_data_flash;
//
//    // Erase the mfg data in Flash.  This is necessary as boards have to be reconfigured by the factory as cap demo boards,
//    // which will happen without the cells powered up.
//    // This is a strangely named CCS function since it's not erasing EEPROM, but it seems to work
//    erase_program_eeprom(APP_FLASH_DATA);
//
//    // Flash is blank.  Write the manufacturing data too it.
//    write_program_memory(address, mfg_data, sizeof(EEPROM_MFG_DATA_TYPE));
//    crc_calc = LTC6804_PEC_Calc((char*)mfg_data, sizeof(EEPROM_MFG_DATA_TYPE));
//    write_program_memory(address + sizeof(EEPROM_MFG_DATA_TYPE), (int8*)&crc_calc, LTC6804_PEC_SIZE);
//
//    // Save copy of new data to RAM.
//    System_Pic_Board_Mfg_Data_Valid = TRUE;
//    System_Pic_Board_Mfg_Data = *mfg_data;
//
//    // Configure charger and discharger pins if reconfigured to be a cap demo system.
//    if(mfg_data->cap_demo == 'T')
//    {
//        System_Cap_Demo.demo_present = 1;
//
//        // Initialize charger and discharger to be outputs, and initialize to be off
//        CHARGER_OUT_PIN_SETUP;
//        DISCHARGER_OUT_PIN_SETUP;
//        System_Cap_Demo.charging = 0;
//        System_Cap_Demo.discharging = 0;
//    }
//    else
//    {
//        System_Cap_Demo.demo_present =  0;
//    }
//
//    return;
//}

//// Resets the manufacturing data for the DC2100A board with the PIC. // #NeededNow??? - This currently resets the mfg data on the PIC MCU to recognize it. It also uses APP_FLASH_DATA which is commented out
//void System_Mfg_Data_Reset(char* reset_key)
//{
//    char reset_string[SYSTEM_RESET_KEY_SIZE] = SYSTEM_RESET_KEY;
//
//    if(memcmp(reset_key, reset_string, SYSTEM_RESET_KEY_SIZE) == 0)
//    {
//        // This is a strangely named CCS function since it's not erasing EEPROM, but it seems to work
//        erase_program_eeprom(APP_FLASH_DATA);
//
//        System_Model[DC2100A_NUCLEO_BOARD_NUM] = '?';
//        System_Cap_Demo.demo_present = 1;
//    }
//
//    return;
//
//}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Detects if a physical address is present in the the system
// Returns true if the address is detected for its first time.
BOOLEAN system_detect_task_ltc6804_address(int8 address_to_detect)
{
    BOOLEAN revision_get_successful;
    unsigned int8 revision;

    // If this board address has already been entered into the address table, do not check for it again.
    for (int16 board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        if(System_Address_Table[board_num] ==  address_to_detect)
        {
            // Reply false to indicate the board was previously detected.
            return FALSE;
        }
    }

    // Use the last open space in the address table to look for this board
    System_Address_Table[System_Num_Boards] = address_to_detect;

    // Ignore PEC failures, since we're not sure if a board is present at this address or not.
    Error_Code_LTC6804_CRC_Ignore = TRUE;

    // Request the revision of the board to detect if it is present
    revision_get_successful = LTC6804_Revision_Get(System_Num_Boards, &revision);

    if(revision_get_successful == FALSE)
    {
        System_Address_Table[System_Num_Boards] = LTC6804_MAX_BOARDS;  // Clear address table used to look for board that wasn't found.
    }
    Error_Code_LTC6804_CRC_Ignore = FALSE;

    // If the board replies with its revision, return that the board was found.
    return revision_get_successful;
}

// Initializes the last board detected by the system.
void system_board_detected_init(void)
{
    EEPROM_MFG_DATA_TYPE mfg_data;
    BOOLEAN mfg_data_valid;

    // If a new board is detected, read its EEPROM Data and increment the board count.
    // Now that a new board is detected, System_Num_Boards contains the index to the data for the most recently detected board.

    mfg_data_valid = Eeprom_Mfg_Data_Get(System_Num_Boards, &mfg_data);

    // Save the model of the detected board.
    System_Model[System_Num_Boards] = mfg_data.model_num[DC2100A_MODEL_NUM_SIZE - 1];

    //// If this is the first board detected, check if it's configured to be cap demo board. // #Scrapped - This snippet rewrites the mfg data if it appears incorrect for the purposes of the mfg's SuperCap demo. 
    //if(System_Num_Boards == DC2100A_NUCLEO_BOARD_NUM)
    //{
    //    // If mfg_data in the EEPROM was valid, but the mfg_data in flash was not, then somebody blew away our S/N with an ICD3.  Fix this...
    //    // If mfg_data is valid in both, but not equal, then somebody most likely changed a regular board into a cap demo board while the cells were powered down.
    //    if(mfg_data_valid == TRUE)
    //    {
    //        EEPROM_MFG_DATA_TYPE flash_mfg_data;
    //        mfg_data_valid = System_Mfg_Data_Get(&flash_mfg_data);
    //        if(mfg_data_valid == FALSE)
    //        {
    //            System_Mfg_Data_Set(&mfg_data);
    //        }
    //        else if(memcmp(&flash_mfg_data, &mfg_data, sizeof(mfg_data)) != 0)
    //        {
    //            mfg_data = flash_mfg_data;
    //            Eeprom_Mfg_Data_Set(DC2100A_NUCLEO_BOARD_NUM, &mfg_data);
    //        }
    //    }
    //}

    Eeprom_Cap_Load(System_Num_Boards, 0);
    Eeprom_Current_Load(System_Num_Boards, 0);

    // Increment the board count
    System_Num_Boards++;

}

// Re-initialize the code modules that lose their configuration upon sleep.
// Return true if initialization was successful.
BOOLEAN system_wakeup_init(void)
{
    BOOLEAN success = TRUE;

    success &= Temperature_Wakeup_Init();
    success &= Voltage_Wakeup_Init();
    //success &= Balancer_Wakeup_Init();        // todo - do we need to clear out the balance commands upon wakeup? #Changed - Commented out in the mean time

    return success;
}
