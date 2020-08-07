/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for EEPROM Data Storage through the LTC6804-2 Battery Monitor.

 @verbatim
 This file provides access to reading, writing, and setting defaults for EEPROM data stored on each DC2100A PCB.

 See "EEPROM" Worksheet in DC2100A_Design.xlsm for map of Data in EEPROM.

 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 750 $
 $Date: 2014-09-17 19:27:28 -0400 (Wed, 17 Sep 2014) $

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

//! @defgroup EEPROM Reference Application File for Interface to 24AA64 EEPROM through the LTC6804-2 Battery Monitor on the DC2100A PCB.

/*! @file
    @ingroup EEPROM
    Reference Application File for Interface to 24AA64 EEPROM through the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"
#include "Eeprom.h"
#include "24AA64.h"
#include "SOC.h"        // For default capacities
#include "Balancer.h"   // For default currents
#include "LTC6804-2.h"  // PEC calculation for LTC6804 used To validate EEPROM
#include "System.h"
#include "../mbed.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

////! @name EEPROM Keys
////! @{
////! Keys used to restrict special operations on the EEPROM.
////const char* EEPROM_RESET_KEY = "EEPROM_RESET";                     //!< Key allows reset of EEPROM to blank values.;
//#define EEPROM_RESET_KEY  "EEPROM_RESET"                     //!< Key allows reset of EEPROM to blank values.;
//#define EEPROM_RESET_KEY_SIZE  sizeof(EEPROM_RESET_KEY) - 1     //!< Size of EEPROM_RSTKEY
//#define EEPROM_MFG_KEY  0x86                               //!< Key allows access for Linear factory calibrated values.
////! @}



//// Define addresses for EEPROM data // // #ComeBack - This seems to have an issue compiling due to the sizeof(...) causing a  missing binary operator before token "("  error
//#define EEPROM_MFG_DATA_ADDRESS                 0       // EEPROM address for first byte of Manufacturing Board ID Data
//#define EEPROM_MFG_DATA_CRC_ADDRESS             (EEPROM_MFG_DATA_ADDRESS + sizeof(EEPROM_MFG_DATA_TYPE))
//#define EEPROM_MFG_DATA_END                     (EEPROM_MFG_DATA_CRC_ADDRESS + sizeof(int16) - 1)
//
//#define EEPROM_MFG_CAP_ADDRESS                  32      // EEPROM address for first byte of Linear factory calibrated capacity values.
//#define EEPROM_MFG_CAP_CRC_ADDRESS              (EEPROM_MFG_CAP_ADDRESS + sizeof(EEPROM_CAP_TYPE))
//#define EEPROM_MFG_CAP_END                      (EEPROM_MFG_CAP_CRC_ADDRESS + sizeof(int16) - 1)
//#if EEPROM_MFG_CAP_ADDRESS <= EEPROM_MFG_DATA_END
//#error "EEPROM_MFG_CAP_ADDRESS" overlaps "EEPROM_MFG_DATA_END"
//#endif
//
//#define EEPROM_MFG_CURRENT_ADDRESS              80      // EEPROM address for first byte of Linear factory calibrated balance current values.
//#define EEPROM_MFG_CURRENT_CRC_ADDRESS          (EEPROM_MFG_CURRENT_ADDRESS + sizeof(EEPROM_CURRENT_TYPE))
//#define EEPROM_MFG_CURRENT_END                  (EEPROM_MFG_CURRENT_CRC_ADDRESS + sizeof(int16) - 1)
//#if EEPROM_MFG_CURRENT_ADDRESS <= EEPROM_MFG_CAP_END
//#error "EEPROM_MFG_CURRENT_ADDRESS" overlaps "EEPROM_MFG_CAP_END"
//#endif
//
//#define EEPROM_USER_CAP_ADDRESS                 128     // EEPROM address for first byte of User entered capacity values.
//#define EEPROM_USER_CAP_CRC_ADDRESS             (EEPROM_USER_CAP_ADDRESS + sizeof(EEPROM_CAP_TYPE))
//#define EEPROM_USER_CAP_END                     (EEPROM_USER_CAP_CRC_ADDRESS + sizeof(int16) - 1)
//#if EEPROM_USER_CAP_ADDRESS <= EEPROM_MFG_CURRENT_END
//#error "EEPROM_USER_CAP_ADDRESS" overlaps "EEPROM_MFG_CURRENT_END"
//#endif
//
//#define EEPROM_USER_CURRENT_ADDRESS             176     // EEPROM address for first byte of User entered balance current values.
//#define EEPROM_USER_CURRENT_CRC_ADDRESS         (EEPROM_USER_CURRENT_ADDRESS + sizeof(EEPROM_CURRENT_TYPE))
//#define EEPROM_USER_CURRENT_END                 (EEPROM_USER_CURRENT_CRC_ADDRESS + sizeof(int16) - 1)
//#if EEPROM_USER_CURRENT_ADDRESS <= EEPROM_USER_CAP_END
//#error "EEPROM_USER_CURRENT_ADDRESS" overlaps "EEPROM_USER_CAP_END"
//#endif
//
//#define EEPROM_PACK_CURRENT_ADDRESS             224     // EEPROM address for first byte of Pack Current Calibration, and Charger/Discharger settings
//#define EEPROM_PACK_CURRENT_CRC_ADDRESS         (EEPROM_PACK_CURRENT_ADDRESS + sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE))
//#define EEPROM_PACK_CURRENT_END                 (EEPROM_PACK_CURRENT_CRC_ADDRESS + sizeof(int16) - 1)
//#if EEPROM_PACK_CURRENT_ADDRESS <= EEPROM_USER_CURRENT_END
//#error "EEPROM_PACK_CURRENT_ADDRESS" overlaps "EEPROM_USER_CURRENT_END"
//#endif

// Since the preprocessor can not use the sizeof function, compilable variables are used instead
constexpr auto EEPROM_MFG_DATA_TYPE_SIZE = sizeof(EEPROM_MFG_DATA_TYPE);
constexpr auto EEPROM_CAP_TYPE_SIZE = sizeof(EEPROM_CAP_TYPE);
constexpr auto EEPROM_CURRENT_TYPE_SIZE = sizeof(EEPROM_CURRENT_TYPE);


// Define addresses for EEPROM data
constexpr auto INT16_SIZE = sizeof(int16);
#define EEPROM_MFG_DATA_ADDRESS                 0       // EEPROM address for first byte of Manufacturing Board ID Data
#define EEPROM_MFG_DATA_CRC_ADDRESS             (EEPROM_MFG_DATA_ADDRESS + EEPROM_MFG_DATA_TYPE_SIZE)
#define EEPROM_MFG_DATA_END                     (EEPROM_MFG_DATA_CRC_ADDRESS + INT16_SIZE - 1)

#define EEPROM_MFG_CAP_ADDRESS                  32      // EEPROM address for first byte of Linear factory calibrated capacity values.
#define EEPROM_MFG_CAP_CRC_ADDRESS              (EEPROM_MFG_CAP_ADDRESS + EEPROM_CAP_TYPE_SIZE)
#define EEPROM_MFG_CAP_END                      (EEPROM_MFG_CAP_CRC_ADDRESS + INT16_SIZE - 1)
#if EEPROM_MFG_CAP_ADDRESS <= EEPROM_MFG_DATA_END
#error "EEPROM_MFG_CAP_ADDRESS" overlaps "EEPROM_MFG_DATA_END"
#endif

#define EEPROM_MFG_CURRENT_ADDRESS              80      // EEPROM address for first byte of Linear factory calibrated balance current values.
#define EEPROM_MFG_CURRENT_CRC_ADDRESS          (EEPROM_MFG_CURRENT_ADDRESS + EEPROM_CURRENT_TYPE_SIZE)
#define EEPROM_MFG_CURRENT_END                  (EEPROM_MFG_CURRENT_CRC_ADDRESS + INT16_SIZE - 1)
#if EEPROM_MFG_CURRENT_ADDRESS <= EEPROM_MFG_CAP_END
#error "EEPROM_MFG_CURRENT_ADDRESS" overlaps "EEPROM_MFG_CAP_END"
#endif

#define EEPROM_USER_CAP_ADDRESS                 128     // EEPROM address for first byte of User entered capacity values.
#define EEPROM_USER_CAP_CRC_ADDRESS             (EEPROM_USER_CAP_ADDRESS + EEPROM_CAP_TYPE_SIZE)
#define EEPROM_USER_CAP_END                     (EEPROM_USER_CAP_CRC_ADDRESS + INT16_SIZE - 1)
#if EEPROM_USER_CAP_ADDRESS <= EEPROM_MFG_CURRENT_END
#error "EEPROM_USER_CAP_ADDRESS" overlaps "EEPROM_MFG_CURRENT_END"
#endif

#define EEPROM_USER_CURRENT_ADDRESS             176     // EEPROM address for first byte of User entered balance current values.
#define EEPROM_USER_CURRENT_CRC_ADDRESS         (EEPROM_USER_CURRENT_ADDRESS + EEPROM_CURRENT_TYPE_SIZE)
#define EEPROM_USER_CURRENT_END                 (EEPROM_USER_CURRENT_CRC_ADDRESS + INT16_SIZE - 1)
#if EEPROM_USER_CURRENT_ADDRESS <= EEPROM_USER_CAP_END
#error "EEPROM_USER_CURRENT_ADDRESS" overlaps "EEPROM_USER_CAP_END"
#endif

#define EEPROM_PACK_CURRENT_ADDRESS             224     // EEPROM address for first byte of Pack Current Calibration, and Charger/Discharger settings
constexpr auto EEPROM_PACK_CURRENT_CONFIG_TYPE_SIZE = sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE);
#define EEPROM_PACK_CURRENT_CRC_ADDRESS         (EEPROM_PACK_CURRENT_ADDRESS + EEPROM_PACK_CURRENT_CONFIG_TYPE_SIZE)
#define EEPROM_PACK_CURRENT_END                 (EEPROM_PACK_CURRENT_CRC_ADDRESS + INT16_SIZE - 1)
#if EEPROM_PACK_CURRENT_ADDRESS <= EEPROM_USER_CURRENT_END
#error "EEPROM_PACK_CURRENT_ADDRESS" overlaps "EEPROM_USER_CURRENT_END"
#endif


// Define interface to read from EEPROM hardware
#define eeprom_read(board_num, address, data_ptr, num_bytes)                \
        {                                                                   \
            Eeprom_24AA64_Read(board_num, address, data_ptr, num_bytes);    \
            eeprom_number_reads += num_bytes;                               \
        }

// Define interface to write to EEPROM hardware
#define eeprom_write(board_num, address, data_ptr, num_bytes) \
        {                                                                   \
            Eeprom_24AA64_Write(board_num, address, data_ptr, num_bytes);   \
            eeprom_number_writes += num_bytes;                              \
        }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Shadow RAM for cell data stored in DC2100A EEPROM.
EEPROM_CAP_TYPE Eeprom_Cap_Values[DC2100A_MAX_BOARDS];          // Copy of capacity value allows quick SOC calculations.
EEPROM_CURRENT_TYPE Eeprom_Current_Values[DC2100A_MAX_BOARDS];  // Copy of balance currents values allows quick balance calculations.
EEPROM_PACK_CURRENT_CONFIG_TYPE Eeprom_Pack_Current_Config;     // Pack currents calibration only stored for DC2100A_PIC_BOARD_NUM board, as it's expected that all boards see the same pack currents. // #NeededNow???

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int16 eeprom_number_reads;                                          // debug data - number of times eeprom has been read.
int16 eeprom_number_writes;                                         // debug data - number of times eeprom has been written.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void eeprom_write_with_crc(int16 board_num, int8* data_ptr, int16 address, int16 num_bytes);
BOOLEAN eeprom_read_with_crc(int16 board_num, int8* data_ptr, int16 address, int16 num_bytes);
void eeprom_cap_use_defaults(int16 board_num);
void eeprom_current_use_defaults(int16 board_num);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the EEPROM code module.
void Eeprom_Init(void)
{

    // Init debug data
    eeprom_number_reads = 0;
    eeprom_number_writes = 0;

    memset(Eeprom_Cap_Values, 0, sizeof(Eeprom_Cap_Values));
    memset(Eeprom_Current_Values, 0, sizeof(Eeprom_Current_Values));
}

// Resets the EEPROM to blank values.
// Note this will take a long time!
BOOLEAN Eeprom_Reset(int16 board_num, char* reset_key)
{
    char reset_string[EEPROM_RESET_KEY_SIZE];
    memcpy(reset_string, &EEPROM_RESET_KEY, EEPROM_RESET_KEY_SIZE);

    if(memcmp(reset_key, reset_string, EEPROM_RESET_KEY_SIZE) == 0)
    {
        Eeprom_24AA64_Erase(board_num);
        return TRUE;
    }

    return FALSE;
}

// Loads user cap values from EEPROM and uses if populated.  If not populated, uses nominal values.
// If user values are not populated or if mfg code is specified, load the mfg values from EEPROM and use if they're populated.
// If mfg values are not populated, use nominal values.
void Eeprom_Cap_Load(int16 board_num, int8 mfg_key)
{
    int8 buf[sizeof(EEPROM_CAP_TYPE)];
    //if((FALSE == eeprom_read_with_crc(board_num, &Eeprom_Cap_Values[board_num], EEPROM_USER_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE))) ||
    if((FALSE == eeprom_read_with_crc(board_num, buf, EEPROM_USER_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE))) ||
       (EEPROM_MFG_KEY == mfg_key))
    {
        //if(FALSE == eeprom_read_with_crc(board_num, &Eeprom_Cap_Values[board_num], EEPROM_MFG_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE)))
        if(FALSE == eeprom_read_with_crc(board_num, buf, EEPROM_MFG_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE)))
        {
            eeprom_cap_use_defaults(board_num);
            return;
        }
    }
    for (int i = 0; i < DC2100A_NUM_CELLS; i++)
    {
        Eeprom_Cap_Values[board_num].cap[i] = buf[(i * 2) + 1] | buf[i * 2] << 8;
    }
}

// Loads user balance current values from EEPROM and use if they're populated.  If not populated, uses nominal values.
// If user values are not populated or if mfg code is specified, load the mfg values from EEPROM and use if they're populated.
// If mfg values are not populated, use nominal values.
void Eeprom_Current_Load(int16 board_num, int8 mfg_key)
{
    int8 buf[sizeof(EEPROM_CURRENT_TYPE)];
    //if((FALSE == eeprom_read_with_crc(board_num, &Eeprom_Current_Values[board_num], EEPROM_USER_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE))) ||
    if((FALSE == eeprom_read_with_crc(board_num, buf, EEPROM_USER_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE))) ||
       (EEPROM_MFG_KEY == mfg_key))
    {
        //if(FALSE == eeprom_read_with_crc(board_num, &Eeprom_Current_Values[board_num], EEPROM_MFG_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE)))
        if(FALSE == eeprom_read_with_crc(board_num, buf, EEPROM_MFG_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE)))
        {
            eeprom_current_use_defaults(board_num);
            return;
        }
    }
    for (int i = 0; i < DC2100A_NUM_CELLS; i++)
    {
        Eeprom_Current_Values[board_num].current[i].charge = buf[i * 2];
        Eeprom_Current_Values[board_num].current[i].discharge = buf[(i * 2) + 1];
    }
}

// Saves the capacity values to the eeprom
// If mfg code is specified, this is saved as mfg cap and user cap values.
// If mfg_key is not specified, this is saved as user cap values.
void Eeprom_Cap_Save(int16 board_num, int8 mfg_key)
{
    int8 buf[sizeof(EEPROM_CAP_TYPE)];
    for (int i = 0; i < DC2100A_NUM_CELLS; i++)
    {
        buf[i*2] = UPPER_BYTE(Eeprom_Cap_Values[board_num].cap[i]);
        buf[(i*2) + 1] = LOWER_BYTE(Eeprom_Cap_Values[board_num].cap[i]);
    }

    if(EEPROM_MFG_KEY == mfg_key)
    {
        //eeprom_write_with_crc(board_num, &Eeprom_Cap_Values[board_num], EEPROM_MFG_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE));
        eeprom_write_with_crc(board_num, buf, EEPROM_MFG_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE));
    }
    //eeprom_write_with_crc(board_num, &Eeprom_Cap_Values[board_num], EEPROM_USER_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE));
    eeprom_write_with_crc(board_num, buf, EEPROM_USER_CAP_ADDRESS, sizeof(EEPROM_CAP_TYPE));
 
    Eeprom_Cap_Load(board_num, mfg_key);  // Reload to verify that they were saved correctly.
}

// Saves the balance current values to the eeprom
// If mfg code is specified, this is saved as mfg balance current and user balance current values.
// If mfg_key is not specified, this is saved as user balance current values.
void Eeprom_Current_Save(int16 board_num, int8 mfg_key)
{
    int8 buf[sizeof(EEPROM_CURRENT_TYPE)];
    memcpy(buf, &Eeprom_Current_Values[board_num], sizeof(EEPROM_CURRENT_TYPE));
    if(EEPROM_MFG_KEY == mfg_key)
    {
        //eeprom_write_with_crc(board_num, &Eeprom_Current_Values[board_num], EEPROM_MFG_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE));
        eeprom_write_with_crc(board_num, buf, EEPROM_MFG_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE));
    }
    //eeprom_write_with_crc(board_num, &Eeprom_Current_Values[board_num], EEPROM_USER_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE));
    eeprom_write_with_crc(board_num, buf, EEPROM_USER_CURRENT_ADDRESS, sizeof(EEPROM_CURRENT_TYPE));
    Eeprom_Current_Load(board_num, mfg_key);  // Reload to verify that they were saved correctly.
}

// Loads (but does not save in the EEPROM) the capacity values to their defaults
// If mfg code is specified, the nominal capacity values are loaded.
// If mfg_key is not specified, the mfg capacity values are loaded.
void Eeprom_Cap_Load_Defaults(int16 board_num, int8 mfg_key)
{
    if(EEPROM_MFG_KEY == mfg_key)
    {
        eeprom_cap_use_defaults(board_num);
    }
    else
    {
        Eeprom_Cap_Load(board_num, EEPROM_MFG_KEY);
    }
}

// Loads (but does not save in the EEPROM) the balance current values to their defaults
// If mfg code is specified, the nominal balance current values are loaded.
// If mfg_key is not specified, the mfg balance current values are loaded.
void Eeprom_Current_Load_Defaults(int16 board_num, int8 mfg_key)
{
    if(EEPROM_MFG_KEY == mfg_key)
    {
        eeprom_current_use_defaults(board_num);
    }
    else
    {
        Eeprom_Current_Load(board_num, EEPROM_MFG_KEY);
    }
}

// Sets and saves the capacity values to their defaults
// If mfg code is specified, the nominal capacity values are saved as the mfg and user capacity values.
// If mfg_key is not specified, the mfg capacity values are saved as the user capacity values.
void Eeprom_Cap_Save_Defaults(int16 board_num, int8 mfg_key)
{
    if(EEPROM_MFG_KEY == mfg_key)
    {
        eeprom_cap_use_defaults(board_num);
        Eeprom_Cap_Save(board_num, 0);
        Eeprom_Cap_Save(board_num, EEPROM_MFG_KEY);
    }
    else
    {
        Eeprom_Cap_Load(board_num, EEPROM_MFG_KEY);
        Eeprom_Cap_Save(board_num, 0);
    }
}

// Sets and saves the balance current values to their defaults
// If mfg code is specified, the nominal balance current values are saved as the mfg and user balance current values.
// If mfg_key is not specified, the mfg balance current values are saved as the user balance current values.
void Eeprom_Current_Save_Defaults(int16 board_num, int8 mfg_key)
{
    if(EEPROM_MFG_KEY == mfg_key)
    {
        eeprom_current_use_defaults(board_num);
        Eeprom_Current_Save(board_num, 0);
        Eeprom_Current_Save(board_num, EEPROM_MFG_KEY);
    }
    else
    {
        Eeprom_Current_Load(board_num, EEPROM_MFG_KEY);
        Eeprom_Current_Save(board_num, 0);
    }
}

// Loads the Manufacturing Board ID Data from EEPROM into global shadow RAM.
BOOLEAN Eeprom_Mfg_Data_Get(int16 board_num, EEPROM_MFG_DATA_TYPE* mfg_data)
{
    int8 buf[sizeof(EEPROM_MFG_DATA_TYPE)];
    char resp = eeprom_read_with_crc(board_num, buf, EEPROM_MFG_DATA_ADDRESS, sizeof(EEPROM_MFG_DATA_TYPE));
    memcpy(mfg_data, buf, sizeof(buf));
// //  ---------------  Debugging Code -------------------------- //
    //printf("buf = ");
    //for (int i = 0; i < sizeof(buf); i++)
    //{
    //    printf("%c", buf[i]);
    //    if (i == 8 || i==10)
    //    {
    //        printf(" ");
    //    }
    //}
    //printf("\n");
// // ------------------------------------------------------------ //
    return resp;

    // # Changed - Replaced line below with memcpy and the rest above due to compilation issues with passing a struct in place of a char array
    //return eeprom_read_with_crc(board_num, mfg_data, EEPROM_MFG_DATA_ADDRESS, sizeof(EEPROM_MFG_DATA_TYPE)); 

}

// Saves the Manufacturing Board ID Data from global shadow RAM into EEPROM.
void Eeprom_Mfg_Data_Set(int16 board_num, EEPROM_MFG_DATA_TYPE* mfg_data)
{
    int8 buf[sizeof(EEPROM_MFG_DATA_TYPE)];
    memcpy(buf, mfg_data, sizeof(EEPROM_MFG_DATA_TYPE));
    //eeprom_write_with_crc(board_num, mfg_data, EEPROM_MFG_DATA_ADDRESS, sizeof(EEPROM_MFG_DATA_TYPE));
    eeprom_write_with_crc(board_num, buf, EEPROM_MFG_DATA_ADDRESS, sizeof(EEPROM_MFG_DATA_TYPE));
}

// Loads the Pack Current Data from EEPROM into global shadow RAM.
// Pack currents calibration only stored for DC2100A_PIC_BOARD_NUM board, as it's expected that all boards see the same pack currents.
BOOLEAN Eeprom_Pack_Current_Config_Get(void)
{
    int8 buf[sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE)];
    BOOLEAN resp = eeprom_read_with_crc(DC2100A_NUCLEO_BOARD_NUM, buf, EEPROM_PACK_CURRENT_ADDRESS, sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE));
    memcpy(&Eeprom_Pack_Current_Config, buf, sizeof(buf));
    return resp;
}

// Saves the Pack Current Data from global shadow RAM into EEPROM.
// Pack currents calibration only stored for DC2100A_PIC_BOARD_NUM board, as it's expected that all boards see the same pack currents.
void Eeprom_Pack_Current_Config_Set(void)
{
    int8 buf[sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE)];
    memcpy(buf, &Eeprom_Pack_Current_Config, sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE));
    eeprom_write_with_crc(DC2100A_NUCLEO_BOARD_NUM, buf, EEPROM_PACK_CURRENT_ADDRESS, sizeof(buf));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Write a series of bytes to the EEPROM with crc to a DC2100A board
void eeprom_write_with_crc(int16 board_num, int8* data_ptr, int16 address, int16 num_bytes)
{
    unsigned int16 crc;
    unsigned int16 crc_address;

    eeprom_write(board_num, address, data_ptr, num_bytes);

    // write the crc at the end
    crc = LTC6804_PEC_Calc(data_ptr, num_bytes); // todo - how much is gained by combining crc calc with eeprom_write loop?
    crc_address = address + num_bytes;

    eeprom_write(board_num, crc_address, (int8*)&crc, sizeof(crc));
}

// Read a series of bytes from the EEPROM with crc to a DC2100A board.
// Returns TRUE if crc matches, FALSE if crc does not match.
BOOLEAN eeprom_read_with_crc(int16 board_num, int8* data_ptr, int16 address, int16 num_bytes)
{
    unsigned int16 crc_read, crc_calc;
    unsigned int16 crc_address;

    eeprom_read(board_num, address, data_ptr, num_bytes);

    // verify the crc
    crc_calc = LTC6804_PEC_Calc(data_ptr, num_bytes); // todo - how much is gained by combining with eeprom_read loop?
    crc_address = address + num_bytes;
    eeprom_read(board_num, crc_address, (int8*)&crc_read, sizeof(crc_read));

    if(crc_calc == crc_read)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
// Sets the capacitance values to their defaults depending upon the model number and whether board was shipped as part of a cap demo.
// These are the valid combinations:
// DC2100A-A Cap Demo Board
// DC2100A-A, Not Cap Demo Board
// DC2100A-B, Not Cap Demo Board
// DC2100A-C Cap Demo Board
// DC2100A-C, Not Cap Demo Board
// DC2100A-D, Not Cap Demo Board
// todo - can probably optimize this for space better.
void eeprom_cap_use_defaults(int16 board_num)
{
    unsigned int8 cell_num;

    if(System_Cap_Demo.demo_present == 1)
    {
        for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
        {
            Eeprom_Cap_Values[board_num].cap[cell_num] = SOC_CAP_DEMO_DEFAULT;
        }
        Eeprom_Cap_Values[board_num].cap[5] = SOC_CAP_DEMO_CELL_6_DEFAULT;
        Eeprom_Cap_Values[board_num].cap[6] = SOC_CAP_DEMO_CELL_7_DEFAULT;
    }
    else
    {
        if((System_Model[board_num] == 'A') || (System_Model[board_num] == 'B'))
        {
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                Eeprom_Cap_Values[board_num].cap[cell_num] = SOC_CAPACITY_AB_DEFAULT;
            }

        }
        else
        {
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                Eeprom_Cap_Values[board_num].cap[cell_num] = SOC_CAPACITY_CD_DEFAULT;
            }
        }
    }
}

// Sets the current values to their defaults depending upon the model number and whether board was shipped as part of a cap demo.
// These are the valid combinations:
// DC2100A-A Cap Demo Board
// DC2100A-A, Not Cap Demo Board
// DC2100A-B, Not Cap Demo Board
// DC2100A-C Cap Demo Board
// DC2100A-C, Not Cap Demo Board
// DC2100A-D, Not Cap Demo Board
void eeprom_current_use_defaults(int16 board_num)
{
    unsigned int8 cell_num;
    int8 charge_default_12cell;
    int8 discharge_default_12cell;

    // The bottom of stack defaults, are for the charge/discharge currents for 12 cells.
    if((System_Model[board_num] == 'A') || (System_Model[board_num] == 'B'))
    {
        charge_default_12cell = BALANCER_CURRENT_SCALE_CALC(BALANCER_AB_CURRENT_CHARGE_12CELL, BALANCER_AB_CURRENT_CHARGE_6CELL);
        discharge_default_12cell = BALANCER_CURRENT_SCALE_CALC(BALANCER_AB_CURRENT_DISCHARGE_12CELL, BALANCER_AB_CURRENT_DISCHARGE_6CELL);
    }
    else
    {
        charge_default_12cell = BALANCER_CURRENT_SCALE_CALC(BALANCER_CD_CURRENT_CHARGE_12CELL, BALANCER_CD_CURRENT_CHARGE_6CELL);
        discharge_default_12cell = BALANCER_CURRENT_SCALE_CALC(BALANCER_CD_CURRENT_DISCHARGE_12CELL, BALANCER_CD_CURRENT_DISCHARGE_6CELL);
    }

    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS / 2; cell_num++)
    {
        Eeprom_Current_Values[board_num].current[cell_num].charge = charge_default_12cell;
        Eeprom_Current_Values[board_num].current[cell_num].discharge = discharge_default_12cell;
    }

    // The bottom of stack defaults, are for the charge/discharge currents for 6 cells.
    for (cell_num = DC2100A_NUM_CELLS / 2; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        Eeprom_Current_Values[board_num].current[cell_num].charge = 0;
        Eeprom_Current_Values[board_num].current[cell_num].discharge = 0;
    }

}
