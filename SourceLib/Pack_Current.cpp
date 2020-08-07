/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for controlling the charger and discharger in a SuperCap Demo System, monitoring inputs that can be used to make the FW operate
 in a charging vs discharging mode, and monitoring an ADC input which represents the Pack Current.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 741 $
 $Date: 2014-09-12 15:38:24 -0400 (Fri, 12 Sep 2014) $

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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"
#include "Pack_Current.h"
#include "NUCLEO_Timer.h"
#include "System.h"
#include "Eeprom.h"
#include "Balancer.h"
#include <string.h>
#include "../mbed.h"


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define NUCLEO_ADC_RESOLUTION                           12
#define PACK_CURRENT_DIVISOR_SHIFT                      (8)     // bits
#define PACK_CURRENT_SAMPLE_TIME_US_DEFAULT             (25)    // us
#define PACK_CURRENT_OFFSET_DEFAULT                     (1L << (NUCLEO_ADC_RESOLUTION - 1))
#define PACK_CURRENT_CHARGE_CALIBRATION_DEFAULT         ((12 * MA_PER_A) << PACK_CURRENT_DIVISOR_SHIFT) / (1L << (NUCLEO_ADC_RESOLUTION) - 1 - PACK_CURRENT_OFFSET_DEFAULT)
#define PACK_CURRENT_DISCHARGE_CALIBRATION_DEFAULT      PACK_CURRENT_CHARGE_CALIBRATION_DEFAULT

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
PACK_CURRENT_IO_TYPE Pack_Current_IO;       // Bitmap for digital inputs/outputs used to monitor/control an external charger or discharger.
unsigned int16 Pack_Current_ADC_Value;      // Raw Value read from ADC.
signed int32 Pack_Current;                  // Pack Current in mA, with discharging > 0 and charging < 0.
unsigned int32 pack_current_timestamp;      // Timestamp taken when current measurement was started.
unsigned int8  pack_current_balancestamp;   // 1 if balancers were on when this measurement were taken, otherwise 0.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
AnalogIn   NUCLEO_AIN0(A0);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the parts of the Pack Current Module, that need to be initialized upon power-up of the PIC.
void Pack_Current_Init(void)
{
    // Get the pack configuration from the EEPROM
    Pack_Current_Config_Get();

    Pack_Current_ADC_Value = 0;
    Pack_Current = 0;
    pack_current_timestamp = NUCLEO_Timer_Update();
    pack_current_balancestamp = 0;
}


// Gets the Pack Current Configuration from the EEPROM, and sets defaults if invalid
void Pack_Current_Config_Get(void)
{
    if(Eeprom_Pack_Current_Config_Get() == FALSE)
    {
        // The configuration was not good.  Use defaults.
        Pack_Current_Config_Defaults();
    }

    //// If SuperCap Demo System is present, let that code module control the charger and discharger.
    //// todo - these 3 bits are redunday, but it allows the EEPROM and Pack Current modules to have different values in case a SuperCap Demo System is present.
    ////        It might be simpler someday to only do this in one place, but for now I want to leave the old system intact.
    //if(System_Cap_Demo.demo_present == TRUE)
    //{
    //    Pack_Current_IO.output_enabled = 0;
    //    Pack_Current_IO.input_enabled = 0;
    //    Pack_Current_IO.analog_enabled = 0;
    //}
    //else
    //{
        Pack_Current_IO.output_enabled = Eeprom_Pack_Current_Config.enable.output;
        Pack_Current_IO.input_enabled = Eeprom_Pack_Current_Config.enable.input;
        Pack_Current_IO.analog_enabled = Eeprom_Pack_Current_Config.enable.analog;
    //}
    Pack_Current_IO.output_inverted = Eeprom_Pack_Current_Config.enable.output_inverted;

}

// Resets Pack Current Configuration to Defaults
void Pack_Current_Config_Defaults(void)
{
    Eeprom_Pack_Current_Config.enable.output = 1;
    Eeprom_Pack_Current_Config.enable.output_inverted = 1;
    Eeprom_Pack_Current_Config.enable.input = 1;
    Eeprom_Pack_Current_Config.enable.analog = 1;
    Eeprom_Pack_Current_Config.sample_time_us = PACK_CURRENT_SAMPLE_TIME_US_DEFAULT;
    Eeprom_Pack_Current_Config.offset = PACK_CURRENT_OFFSET_DEFAULT;
    Eeprom_Pack_Current_Config.charge_calibration = PACK_CURRENT_CHARGE_CALIBRATION_DEFAULT;
    Eeprom_Pack_Current_Config.discharge_calibration = PACK_CURRENT_DISCHARGE_CALIBRATION_DEFAULT;
}

void Pack_Current_Monitor_Task(void)
{
    signed int32 temp_int32;

    // Start the ADC conversion.
    if(Pack_Current_IO.analog_enabled)
    {
        //// Start the ADC Conversion // #Scrapping - Don't need this
        //NUCLEO_ADC_Start();

        // Update Timer and store timestamp for this sample.
        pack_current_timestamp = NUCLEO_Timer_Update();

        // Add stamp for whether balancing was active during these voltage samples. 
        pack_current_balancestamp = (Balancer_Is_Balancing() == TRUE) ? 0x1 : 0x0;
    }

    // Drive the charger/discharger outputs.
    if(Pack_Current_IO.output_enabled)
    {
	    //CHARGER_OUT_PIN = Pack_Current_IO.charging_output ? 1 : 0; // #Changed #NeededNow??? - Commented this out cuz we currently don't have an implementation / need for this
    	//DISCHARGER_OUT_PIN = (Pack_Current_IO.discharging_output ^ Pack_Current_IO.output_inverted) ? 0 : 1; // #Changed #NeededNow??? - Commented this out cuz we currently don't have an implementation / need for this
    }
	else
	{
        Pack_Current_IO.charging_output = 0;
        Pack_Current_IO.discharging_output = 0;
	}

    // Read the charger/discharger inputs.
    if(Pack_Current_IO.input_enabled)
    {
        //Pack_Current_IO.charging_input = CHARGER_IN_PIN; // #Changed #NeededNow??? - Commented this out cuz we currently don't have an implementation / need for this
        //Pack_Current_IO.discharging_input = DISCHARGER_IN_PIN; // #Changed #NeededNow??? - Commented this out cuz we currently don't have an implementation / need for this
    }
    else
    {
        Pack_Current_IO.charging_input = 0;
        Pack_Current_IO.discharging_input = 0;
    }

    // Wait for ADC conversion to complete and read the results.
    if(Pack_Current_IO.analog_enabled)
    {
        // #Changed #NeededNow??? - Commented this out cuz we currently don't have an implementation / need for this
        //DEBUG5_OUT_PIN = 0; 
        //while (NUCLEO_ADC_Read(&Pack_Current_ADC_Value) == FALSE)
        //{
        //    // todo - make some way to break out of this loop if the PIC A/D somehow breaks
        //}
        //DEBUG5_OUT_PIN = 1;

        Pack_Current_ADC_Value = NUCLEO_AIN0.read_u16();

        temp_int32 = (int32)Pack_Current_ADC_Value - Eeprom_Pack_Current_Config.offset; // #ComeBack - I don't think this calculation is correct as I have just used what they had and included the line before. Need to confirm calculation
        if(temp_int32 > 0)
        {
            temp_int32 *= Eeprom_Pack_Current_Config.discharge_calibration;
        }
        else if (temp_int32 < 0)
        {
            temp_int32 *= Eeprom_Pack_Current_Config.charge_calibration;
        }

        Pack_Current = SIGNED_RIGHT_SHIFT_WITH_ROUND(temp_int32, PACK_CURRENT_DIVISOR_SHIFT);
    }
    else
    {
        Pack_Current_ADC_Value = 0;
        Pack_Current = 0;
    }

    return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

