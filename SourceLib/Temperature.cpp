/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Monitoring Temperature Sensors through the LTC6804-2 Battery Monitor on the DC2100A PCB.
 All datasheet references in this file refer to Vishay document number: 33011.

 @verbatim

 This code contains a task to read the temperature sensors in the DC2100A System at a rate set by TEMPERATURE_TASK_RATE.

 J17 of the DC2100A board has connections for DC2100A_NUM_TEMPS thermistors.  The DC2100A hardware can only connect to one thermistor
 at a time, as it is connected to the LTC6804 GPIO channel through one of two LTC1380 analog muxes.  The task reads one thermistor each
 execution, and allows the large time constants that result when switching mux channels to settle in between task executions.  Therefore,
 it takes TEMPERATURE_TASK_RATE * DC2100A_NUM_TEMPS time for all of the thermistor inputs to be read.

 This code module assumes the resistors are of the 100kOhm Curve Type 1 from Vishay document number 33011.  The voltages are sampled,
 accounting for the large time constants when performing the ADC conversion, with the results stored as to minimize RAM usage. The temperatures for
 each DC2100A board can be retrieved with a function that converts them to °C.  See "Temperature" worksheet on DC2100A_Design.xlsm for details of
 conversion from ADC counts to °C.

 The raw ADC values for one DC2100A board can be stored for retrieval by the DC2100A GUI.

 @endverbatim

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

//! @defgroup Temperature Reference Application File for Monitoring Temperature Sensors through the LTC6804-2 Battery Monitor on the DC2100A PCB.

/*! @file
    @ingroup Temperature
    Reference Application File for Monitoring Temperature Sensors through the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"
#include "Temperature.h"
#include "System.h"
#include "LTC6804-2.h"
#include "NUCLEO_Timer.h"           // Interface for Timers and Delays // #ComeBack - Temporarily added this since the depndency still exists
#include "LTC1380.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define TEMPERATURE_TABLE_OFFSET        -56     // in °C, offset used to store temperatures in 8 bit number even though range exceeds 8 bits.
#define TEMPERATURE_TABLE_RESOLUTION    8       // temperatures are stored in 1/8°C

#define TEMPERATURE_TABLE_IDX_MAX       (sizeof(Temperature_Table)/sizeof(int16) - 1)

#define TEMPERATURE_ADC_VALUE_INVALID   0xFFFF  // Code used to indicate that temperature adc value has not been read yet.

// properties needed to relate a temperature number to an LTC1380 mux and channel
typedef struct {
    int8 mux_num;
    int8 channel_num;
} TEMPERATURE_CHANNEL_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned int32 temperature_timestamp;                                    // Timestamp taken when last temperature measurement was started.
unsigned int8  temperature_balancestamp;                                 // 1 if balancers were on when these measurements were taken, otherwise 0

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Temperatures measured by the thermistor switched by the LTC1380 on the DC2100A board
unsigned int8 temperatures[DC2100A_MAX_BOARDS][DC2100A_NUM_TEMPS];


// Relationship between 10kOhm NTHS01N1002JE thermistor and temperature from Curve 1 of datasheet, assuming 35Ohm RON in series due to LTC1380 Multiplexer
const unsigned int16 Temperature_Table[] = { 29715,  // -56°C
                                             29495,  // -48°C
                                             29136,  // -40°C
                                             28578,  // -32°C
                                             27745,  // -24°C
                                             26563,  // -16°C
                                             24978,  // -8°C
                                             22979,  // 0°C
                                             20622,  // 8°C
                                             18034,  // 16°C
                                             15380,  // 24°C
                                             12830,  // 32°C
                                             10513,  // 40°C
                                             8507,   // 48°C
                                             6830,   // 56°C
                                             5466,   // 64°C
                                             4376,   // 72°C
                                             3515,   // 80°C
                                             2841,   // 88°C
                                             2312,   // 96°C
                                             1898,   // 104°C
                                             1573,   // 112°C
                                             1318,   // 120°C
                                             1116,   // 128°C
                                             955,    // 136°C
                                             826,    // 144°C
                                             723,    // 152°C
                                             639 } ; // 160°C

// Relationship between thermistor number, and LTC1380 mux/channel
const TEMPERATURE_CHANNEL_TYPE Temperature_Channel_Table[DC2100A_NUM_TEMPS] = { 0, 0,   // RT1
                                                                        0, 1,   // RT2
                                                                        0, 2,   // RT3
                                                                        0, 3,   // RT4
                                                                        0, 4,   // RT5
                                                                        0, 5,   // RT6
                                                                        0, 6,   // RT7
                                                                        0, 7,   // RT8
                                                                        1, 0,   // RT9
                                                                        1, 1,   // RT10
                                                                        1, 2,   // RT11
                                                                        1, 3};  // RT12

int16 temperature_in_process;                                // The temperature channel currently being monitored.
BOOLEAN temperature_skip;                                   // Flag to skip the first thermistor reading after wakeup of the LTCH6804, as it is likely junk.
int16 temperature_adc_values[DC2100A_NUM_TEMPS];   // For debug and manufacturing mostly, tracks the adc values for one board.
int8 temperature_board_for_adc_values;                      // The board currently having its temperature adc values being tracked

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void temperature_channel_setup(void);
unsigned int8 temperature_lookup(unsigned int16 adc_value);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the parts of the Temperature Module, that need to be initialized upon power-up of the PIC.
void Temperature_Init(void)
{
    // Init temperatures to room temperature
    memset(temperatures, (25 - TEMPERATURE_TABLE_OFFSET), sizeof(temperatures));

    // Init to start monitoring first channel
    temperature_in_process = 0;

    temperature_board_for_adc_values = 0;

    temperature_timestamp = NUCLEO_Timer_Update();;
    temperature_balancestamp = 0;

    return;
}

// Initializes the parts of the Temperature Module, that need to be initialized upon wakeup of the LTC6804.
BOOLEAN Temperature_Wakeup_Init(void)
{
    BOOLEAN success = TRUE;
    unsigned int8 refon_temp; // #changed - Changed this from int8 to unsigned int8
    int16 board_num;

    // Turn on the Reference.
    LTC6804_Refon_Set(LTC6804_BROADCAST, TRUE);
    // Check to see if reference was turned on for each board.
    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        refon_temp = FALSE;
        success &= LTC6804_Refon_Get(board_num, &refon_temp);
        if(refon_temp == 0)
        {
            success = FALSE;
        }
    }

    // Set Up GPIO2 for Analog Input
    // No way to check if these bits are actually set in the 6804 as read value and write value have different meanings.
    LTC6804_GPIO_Set(LTC6804_BROADCAST, 0x1E);

    temperature_skip = TRUE;

    return success;
}

// Executes the Temperature monitor task.
// - Measures one thermistor value on each DC2100 board in the system.
// - Calculates the temperature from the thermistor ADC value..
// - Sets up LTC1380 analog mux channel for the next thermistor measurement.
// - This task must be executed at a maximum of TEMPERATURE_THERMISTOR_DELAY, to allow circuit to settled after analog mux is switched.
void Temperature_Monitor_Task(void)
{
    int16 board_num;
    int16 adc_value;

    // Clear ADC results, so that it can be detected if LTC6804_Cell_ADC_Start() command is not successful.
    LTC6804_GPIO_ADC_Clear(LTC6804_BROADCAST);

    // Convert and read adc value of temperature measurement started at the end of last task
    LTC6804_GPIO_ADC_Start(LTC6804_BROADCAST, LTC6804_CONVERSION_2KHZ_MODE, LTC6804_CHG_GPIO2);

    // Update Timer and store timestamp for these samples.
    temperature_timestamp = NUCLEO_Timer_Update();

    //// Add stamp for whether balancing was active during these temperature samples. // #ComeBack - Commented this in the meantime
    //temperature_balancestamp = (Balancer_Is_Balancing() == TRUE) ? 0x1 : 0x0;

    // Due to the massive time constants involved with switching to the temperature channel the first reading
    // is likely to be junk upon wakeup of the LTC6804.
    if(temperature_skip == TRUE)
    {
        temperature_skip = FALSE;
    }
    else
    {
        // Wait for conversion to be complete.
        // Note - It is worthwhile to perform some other action while waiting for the ADC conversion to be complete.
        NUCLEO_Timer_Delay_us(LTC6804_CONVERSION_2KHZ_DELAY);    // Wait for conversion to complete

        // Read adc values and convert to temperatures.
        // Note that broadcast read commands can not be performed by the LTC6804-2, so a loop must be used.
        for (board_num = 0; board_num < System_Num_Boards; board_num++)
        {
            // Read the adc result
            if(LTC6804_GPIO_ADC_Read(board_num, LTC6804_CHG_GPIO2, &adc_value) == TRUE)
            {
                if((int32)adc_value != LTC6804_ADC_CLEAR)
                {
                    if(board_num == temperature_board_for_adc_values)
                    {
                        temperature_adc_values[temperature_in_process] = adc_value;
                    }

                    // Convert adc value to a temperature
                    temperatures[board_num][temperature_in_process] = temperature_lookup(adc_value);
                }
            }
        }

        // Switch to the next temperature
        if(temperature_in_process < (DC2100A_NUM_TEMPS - 1))
        {
            temperature_in_process++;
        }
        else
        {
            temperature_in_process = 0;
        }
    }

    // Setup LTC1380s to connect next thermistor to be read to the adc
    temperature_channel_setup();

    // This delay is massive.  Do not wait here, but time task such that delay will be done the next time the task is entered.
     //NUCLEO_Timer_Delay_ms(TEMPERATURE_THERMISTOR_DELAY);

    return;
}

// Gets one temperature from one DC2100A PCB in °C
int16 Temperature_Get(int16 board_num, int8 temperature_num)
{
    int16 temp_int16;

    temp_int16 = (int16)temperatures[board_num][(int16)temperature_num];
    temp_int16 += TEMPERATURE_TABLE_OFFSET;

    return temp_int16;
}

// Gets the raw ADC values for all of the thermistors on one DC2100A board.
// Returns a pointer to the thermistor ADC values for this board.  Returns NULL if all ADC values have not yet been taken.
int16* Temperature_Adc_Value_Get(int16 board_num)
{
    int16 temp_num;

    // If this is the board that was previously selected, then check to see if all adc values have been stored yet.
    if(board_num == temperature_board_for_adc_values)
    {
        for (temp_num = 0; temp_num < DC2100A_NUM_TEMPS; temp_num++)
        {
            if((int32)temperature_adc_values[temp_num] == TEMPERATURE_ADC_VALUE_INVALID)
            {
                // All adc values have not been stored yet.  Return NULL
                return NULL;
            }
        }
    }
    // A new board is having its adc values requested.  Prepare to store the adc values.
    else
    {
        temperature_board_for_adc_values = board_num;
        for (temp_num = 0; temp_num < DC2100A_NUM_TEMPS; temp_num++)
        {
            temperature_adc_values[(int16)temp_num] = TEMPERATURE_ADC_VALUE_INVALID;
        }
        // No adc values have not been stored yet.  Return NULL
        return NULL;
    }

    return temperature_adc_values;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Sets up a channel to be converted.
void temperature_channel_setup(void)
{
    int8 mux_num = Temperature_Channel_Table[temperature_in_process].mux_num;
    int8 channel_num = Temperature_Channel_Table[temperature_in_process].channel_num;

    // Turn off channels on mux that is not being used
    LTC1380_All_Off(LTC6804_BROADCAST, (DC2100A_NUM_MUXES - 1) - mux_num);

    // Turn on the channel on the mux for temperature to be measured
    LTC1380_Set_Channel(LTC6804_BROADCAST, mux_num, channel_num);
}

// Converts a 16 bit adc value value to a temperature by performing a binary search of Temperature_Table[]
// Note that Temperature_Table[] is reverse ordered due to the inverse relationship between temperature and resistance.
unsigned int8 temperature_lookup(unsigned int16 adc_value)
{
    unsigned int8 idx, idx_min, idx_max;
    unsigned int16 temp_uint16, temp_uint16b;

    if(Temperature_Table[0] <= adc_value)
    {
        // Temperature is too low for the table, return min value
        idx =  0;
    }
    else if(Temperature_Table[TEMPERATURE_TABLE_IDX_MAX] >= adc_value)
    {
        // Temperature is too high for the table, return max value
        idx = TEMPERATURE_TABLE_IDX_MAX*TEMPERATURE_TABLE_RESOLUTION;
    }
    else
    {
        idx_min = 0;
        idx_max = TEMPERATURE_TABLE_IDX_MAX;
        idx = TEMPERATURE_TABLE_IDX_MAX/2;

        // Temperature is in the table.  Find the closest values and interpolate
        while (idx_min < idx_max)
        {
            temp_uint16 = Temperature_Table[idx];

            if(temp_uint16 == adc_value)
            {
                // If temperature is exactly value in the table, stop searching
                break;
            }
            else if(temp_uint16 < adc_value)
            {
                // If temperature is lower than tested table value, adjust the max
                idx_max = idx;
            }
            else
            {
                // If temperature is higher than tested table value, adjust the min
                idx_min = idx + 1;
            }

            idx = (idx_max + idx_min) >> 1;
        }

        // idx is the value in the table that's higher than the measured temperature.  Interpolate to nearest degree.
        temp_uint16 = (Temperature_Table[idx - 1] - Temperature_Table[idx]);
        temp_uint16b = (Temperature_Table[idx - 1] - adc_value);
        idx = ((idx - 1) * TEMPERATURE_TABLE_RESOLUTION) + (temp_uint16b * TEMPERATURE_TABLE_RESOLUTION + temp_uint16/2) / temp_uint16;
    }

    return idx;
}
