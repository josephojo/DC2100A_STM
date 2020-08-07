/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for EEPROM Data Storage through the LTC6804-2 Battery Monitor.

 @verbatim
 This file provides access to reading, writing, and setting defaults for EEPROM data stored on each DC2100A PCB.

 The items that are stored in the EEPROM are:  Manufacturing (Mfg) Board ID Data, Cell Capacity Values, and Balancer Current Values.

 Some items have two copies in the EEPROM:  The Mfg value is written in the Linear factory when the board is manufactured, and the User value is available for general use.

 Data is checked for consistency when it is retrieved from the EEPROM.  If invalid data is retrieved, it is replaced in this order:
 User EEPROM values are replaced by Linear factory calibrated EERPOM values, which is then replaced by nominal values from Flash.

 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 752 $
 $Date: 2014-09-18 13:02:07 -0400 (Thu, 18 Sep 2014) $

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
    Reference Application File for EEPROM Data specific to the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

#ifndef __EEPROM_H__
#define __EEPROM_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name EEPROM Keys
//! @{
//! Keys used to restrict special operations on the EEPROM.
//const char* EEPROM_RESET_KEY = "EEPROM_RESET";                     //!< Key allows reset of EEPROM to blank values.;
//constexpr int16 EEPROM_RESET_KEY_SIZE = sizeof(EEPROM_RESET_KEY) - 1;     //!< Size of EEPROM_RSTKEY
//int16 EEPROM_MFG_KEY = 0x86;                              //!< Key allows access for Linear factory calibrated values.
//! @}

#define EEPROM_RESET_KEY  "EEPROM_RESET"                     //!< Key allows reset of EEPROM to blank values.;
#define EEPROM_RESET_KEY_SIZE  sizeof(EEPROM_RESET_KEY) - 1     //!< Size of EEPROM_RSTKEY
#define EEPROM_MFG_KEY  0x86                               //!< Key allows access for Linear factory calibrated values.


//! Structure containing manufacturing data for one DC2100A PCB
typedef struct {
    char model_num[DC2100A_MODEL_NUM_SIZE];     //!< DC2100A model number
    char cap_demo;                              //!< True if DC2100A was manufactured into a SuperCap Demo System
    char serial_num[DC2100A_SERIAL_NUM_SIZE];   //!< DC2100A serial number
} EEPROM_MFG_DATA_TYPE;

//! Structure containing capacity data for one DC2100A PCB
typedef struct {
    int16 cap[DC2100A_NUM_CELLS]; //!< cell capacity in SOC_CAP_SCALE_FACTOR units
} EEPROM_CAP_TYPE;

//! Structure containing balance current calibration factors for one DC2100A PCB
typedef struct {
    struct {
        signed int8 charge;                //!< charge current calibration factor
        signed int8 discharge;             //!< discharge current calibration factor
    } current [DC2100A_NUM_CELLS];         //!< current calibration factor in BALANCER_CURRENT_SCALE units
} EEPROM_CURRENT_TYPE;

//! Structure containing pack current calibration factors for DC2100A System
typedef struct {
    struct {
        int8 output : 1;                 //!< enables control of charger and discharger outputs.
        int8 output_inverted : 1;        //!< enables inversion of discharger output (available as inverted and non-inverted on J21).
        int8 input  : 1;                 //!< enables monitoring of charger and discharger inputs.
        int8 analog : 1;                 //!< enables monitoring of analog pack current measurement.
        int8 unused : 4;
    } enable;                            //!< enables to use of pack current inputs and outputs
    int8 sample_time_us;                 //!< pack current sample time in us
    unsigned int16 offset;               //!< pack current offset from ADC
    unsigned int16 charge_calibration;   //!< pack current calibration factor when charging
    unsigned int16 discharge_calibration;//!< pack current calibration factor when discharging
} EEPROM_PACK_CURRENT_CONFIG_TYPE;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @ EEPROM Shadow RAM // #Scrapping - Don't thing we're going to use this
//! @{
//! Shadow RAM for cell data stored in DC2100A EEPROM.
extern EEPROM_CAP_TYPE Eeprom_Cap_Values[DC2100A_MAX_BOARDS];          //!< Copy of capacity value allows quick SOC calculations.
extern EEPROM_CURRENT_TYPE Eeprom_Current_Values[DC2100A_MAX_BOARDS];  //!< Copy of balance currents values allows quick balance calculations.
extern EEPROM_PACK_CURRENT_CONFIG_TYPE Eeprom_Pack_Current_Config;     //!< Pack currents calibration only stored for DC2100A_PIC_BOARD_NUM board, as it's expected that all boards see the same pack currents. 
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Initializes the EEPROM code module.
//! @return void
void Eeprom_Init(void);

//! Resets the EEPROM to blank values.
//! @return TRUE if the board is reset.
BOOLEAN Eeprom_Reset(int16 board_num,        //!< The logical address for the PCB containing this EEPROM data.
                     char* reset_key        //!< Key allows reset of EEPROM to blank values.
                    );

//! Loads the customer saved or Linear factory calibrated capacity values from EEPROM into global shadow RAM.
//! @return void
void Eeprom_Cap_Load(int16 board_num,        //!< The logical address for the PCB containing this EEPROM data.
                     int8 mfg_key           //!< Key allows access for Linear factory calibrated values.
                    );

//! Saves the customer saved or Linear factory calibrated capacity values from global shadow RAM into EEPROM.
//! @return void
void Eeprom_Cap_Save(int16 board_num,        //!< The logical address for the PCB containing this EEPROM data.
                     int8 mfg_key           //!< Key allows access for Linear factory calibrated values.
                     );

//! Loads Linear factory calibrated or nominal capacity values.
//! @return void
void Eeprom_Cap_Load_Defaults(int16 board_num,       //!< The logical address for the PCB containing this EEPROM data.
                              int8 mfg_key          //!< Key allows access for Linear factory calibrated values.
                              );

//! Resets customer saved capacity values to Linear factory calibrated capacity values, or factory calibrated capacity values to nominal.
//! @return void
void Eeprom_Cap_Save_Defaults(int16 board_num,       //!< The logical address for the PCB containing this EEPROM data.
                              int8 mfg_key          //!< Key allows access for Linear factory calibrated values.
                              );

//! Loads the customer saved or Linear factory calibrated balance current values from EEPROM into global shadow RAM.
//! @return void
void Eeprom_Current_Load(int16 board_num,    //!< The logical address for the PCB containing this EEPROM data.
                         int8 mfg_key       //!< Key allows access for Linear factory calibrated values.
                         );

//! Saves the customer saved or Linear factory calibrated balance current values from global shadow RAM into EEPROM.
//! @return void
void Eeprom_Current_Save(int16 board_num,        //!< The logical address for the PCB containing this EEPROM data.
                         int8 mfg_key           //!< Key allows access for Linear factory calibrated values.
                         );

//! Loads Linear factory calibrated or nominal balance current values.
//! @return void
void Eeprom_Current_Load_Defaults(int16 board_num,        //!< The logical address for the PCB containing this EEPROM data.
                                  int8 mfg_key           //!< Key allows access for Linear factory calibrated values.
                                  );

//! Resets customer saved balance current values to Linear factory calibrated balance current values, or factory calibrated balance current values to nominal.
//! @return void
void Eeprom_Current_Save_Defaults(int16 board_num,       //!< The logical address for the PCB containing this EEPROM data.
                                  int8 mfg_key          //!< Key allows access for Linear factory calibrated values.
                                  );

//! Loads the Manufacturing Board ID Data from EEPROM into global shadow RAM.
//! @return True if Mfg Board ID Data is valid.
BOOLEAN Eeprom_Mfg_Data_Get(int16 board_num,                 //!< The logical address for the PCB containing this EEPROM data.
                            EEPROM_MFG_DATA_TYPE* mfg_data  //!< Pointer where read data should be stored.
                            );

//! Saves the Manufacturing Board ID Data from global shadow RAM into EEPROM.
//! @return void
void Eeprom_Mfg_Data_Set(int16 board_num,                    //!< The logical address for the PCB containing this EEPROM data.
                         EEPROM_MFG_DATA_TYPE* mfg_data     //!< Pointer to data to write.
                         );


//! Loads the Pack Current Data from EEPROM into global shadow RAM. // #Scrapping - Don't thing we're going to use this
//! Pack currents calibration only stored for DC2100A_PIC_BOARD_NUM board, as it's expected that all boards see the same pack currents.
//! @return True if Pack Current Data is valid.
BOOLEAN Eeprom_Pack_Current_Config_Get(void);

//! Saves the Pack Current Data from global shadow RAM into EEPROM.
//! Pack currents calibration only stored for DC2100A_PIC_BOARD_NUM board, as it's expected that all boards see the same pack currents.
//! @return void
void Eeprom_Pack_Current_Config_Set(void);

#endif
