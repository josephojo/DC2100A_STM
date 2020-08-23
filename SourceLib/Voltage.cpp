/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Monitoring Voltages from the LTC6804-2 Battery Monitor on the DC2100A PCB.

 @verbatim
 This file contains a task to read the voltages in the DC2100A System at a rate set by VOLTAGE_TASK_RATE.

 The task measures all cell voltages in the system, and calculates the sum-of-cells voltages for each LTC6804 in the system.
 Each cell is monitored for under-voltage (UV) and over-voltage (OV), and balancing is automatically stopped if these conditions occur.

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

//! @defgroup Voltage Reference Application File for Monitoring Voltages from the LTC6804-2 Battery Monitor on the DC2100A PCB.

/*! @file
    @ingroup Voltage
    Reference Application File for Monitoring Voltages from the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"
#include "Voltage.h"
#include "System.h"
#include "LTC6804-2.h"
#include "Balancer.h"   
#include "USB_Parser.h"  
#include "NUCLEO_Timer.h"           // Interface for Timers and Delays // #ComeBack - Temporarily added this since the depndency still exists
#include "Error.h" 
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define VOLTAGE_VUV_DEFAULT         (2.50 * UV_PER_V/LTC6804_VOLTAGE_RESOLUTION)
#define VOLTAGE_VOV_DEFAULT         (4.30 * UV_PER_V/LTC6804_VOLTAGE_RESOLUTION)
#define VOLTAGE_UVOV_CONVERSION     (LTC6804_UVOV_RESOLUTION/LTC6804_VOLTAGE_RESOLUTION)
#define VOLTAGE_UVOV_ROUND_MASK     (VOLTAGE_UVOV_CONVERSION - 1)

#define VOLTAGE_PROCESSING_DURING_DELAY     50         // Estimated number of microseconds that we can avoid waiting, due to processing done during ADC conversion

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned int16 voltage_cell[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];      // Measured cell voltages for each board in LTC6804_VOLTAGE_RESOLUTION.
unsigned int16 voltage_sum[DC2100A_MAX_BOARDS];                          // Measured sum of the cell voltages for this board in LTC6804_SOC_RESOLUTION.
unsigned int32 voltage_timestamp;                                        // Timestamp taken when voltage measurements were started.
unsigned int8  voltage_balancestamp;                                     // 1 if balancers were on when these measurements were taken, otherwise 0

unsigned int16 voltage_vov_threshold;                                    // over-voltage threshold in LTC6804_VOLTAGE_RESOLUTION.
unsigned int16 voltage_vuv_threshold;                                    // under-voltage threshold in LTC6804_VOLTAGE_RESOLUTION.
unsigned int16 voltage_ov_flags[DC2100A_MAX_BOARDS];                     // Bitmap indicating if a cell input on the LTC6804 is over-voltage (1) or not (0).
unsigned int16 voltage_uv_flags[DC2100A_MAX_BOARDS];                     // Bitmap indicating if a cell input on the LTC6804 is under-voltage (1) or not (0).
VOLTAGE_CELL_PRESENT_TYPE voltage_cell_present_flags[DC2100A_MAX_BOARDS];// Bitmap indicating if a cell input on the LTC6804 is unpopulated (0), such that UV conditions should be ignored.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void voltage_error_adc_clear(int16 board_num);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the parts of the Voltage Module, that need to be initialized upon power-up of the PIC.
void Voltage_Init(void)
{
    // Init all measurements to be clear.
    memset(voltage_cell, 0, sizeof(voltage_cell));
    memset(voltage_sum, 0, sizeof(voltage_sum));
    voltage_timestamp = NUCLEO_Timer_Update();
    voltage_balancestamp = 0;

    // Assume all cells to be present
    memset(voltage_cell_present_flags, 0xFF, sizeof(voltage_uv_flags));

    // Init all cells to be UV until we've seen an LTC6804 communicate and reply back with voltages.
    memset(voltage_ov_flags, 0, sizeof(voltage_ov_flags));
    memset(voltage_uv_flags, 0xFF, sizeof(voltage_uv_flags));

    // Init thresholds to defaults.
    voltage_vov_threshold = VOLTAGE_VOV_DEFAULT;
    voltage_vuv_threshold = VOLTAGE_VUV_DEFAULT;

}

// Initializes the parts of the Voltage Module, that need to be initialized upon wakeup of the LTC6804.
BOOLEAN Voltage_Wakeup_Init(void)
{
    // Initialize the UV and OV threshold registers in the LTC6804.
    return Voltage_UVOV_Thresholds_Set(voltage_vuv_threshold, voltage_vov_threshold);
}

// Executes the Voltage monitor task.
// - Measures all cell voltages in the system.  Note that all DC2100A voltages are started simultaneously and read sequentially.
// - A timestamp is attached to each voltage for mathematical operations to be performed on the samples.
// - The state of the balancers is saved, as the cell voltage measurements are affected by the large DC2100A balance currents.
// - Calculates the sum-of-cells voltages for each LTC6804 in the system.
// - Monitors each cell for UV and OV.  Balancing is stopped if UV or OV occurs, and in USB message is sent to the GUI
void Voltage_Monitor_Task(void)
{
    int16 board_num, cell_num;                           // loop variables.
    unsigned int16 voltage_temp[DC2100A_NUM_CELLS];     // temp storage for reading voltages, so that last results are not overwritten if ADC fails to start.
    int16 vov_flags, vuv_flags;                         // temp storage so changes can be detected in flags.
    BOOLEAN suspend_sent;                               // flag so that only one suspend command is sent if any cell voltages are vov or vuv.
    BOOLEAN results_clear;                              // flag that results were not received for this board.
    int32 cell_sum;                                     // temp storage for summing cell voltages before scaling.

    // Clear ADC results, so that it can be detected if LTC6804_Cell_ADC_Start() command is not successful.
    LTC6804_Cell_ADC_Clear(LTC6804_BROADCAST);
    //memset(voltage_cell, 0, sizeof(voltage_cell));

    // Start converting all of the cell ADC values
    // Broadcast Start Cell Voltage ADC Start in Normal Mode (7KHz)
    LTC6804_Cell_ADC_Start(LTC6804_BROADCAST, LTC6804_CONVERSION_7KHZ_MODE, LTC6804_CH_ALL, TRUE);

    // Update Timer and store timestamp for these samples.
    voltage_timestamp = NUCLEO_Timer_Update();

    // Add stamp for whether balancing was active during these voltage samples. // #ComeBack - Commented this in the meantime
    voltage_balancestamp = (Balancer_Is_Balancing() == TRUE) ? 0x1 : 0x0;

    // If under-voltage or over-voltage condition found, suspend balancing but only send once for all boards.
    suspend_sent = FALSE;

    // Wait for conversion to be complete.
    // Note - It is worthwhile to perform some other action while waiting for the ADC conversion to be complete.
    if (LTC6804_CONVERSIONS_ALL_7KHZ_DELAY > VOLTAGE_PROCESSING_DURING_DELAY)
    {
        NUCLEO_Timer_Delay_us(LTC6804_CONVERSIONS_ALL_7KHZ_DELAY - VOLTAGE_PROCESSING_DURING_DELAY);
    }

    // Read cell voltage ADC values.
    // Note that broadcast read commands can not be performed by the LTC6804-2, so a loop must be used.
    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        // Start with sum cleared, and flag indicating ADC results were received.
        results_clear = FALSE;
        cell_sum = 0;

        // Read the ADC results
        if(LTC6804_Cell_ADC_Read(board_num, LTC6804_CH_ALL, voltage_temp) == TRUE)
        {
            // Sum to get the voltage for the full board and copy to global variables.
            // If any results are clear, log an error but do not update the sum of cells.
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                if(voltage_temp[cell_num] != LTC6804_ADC_CLEAR)
                {
                    voltage_cell[board_num][cell_num] = voltage_temp[cell_num];
                    cell_sum += voltage_cell[board_num][cell_num];
                }
                else
                {
                    results_clear = TRUE;
                }
            }

            // Scale sum and save.
            // If any results were clear, send an error.
            if(results_clear == FALSE)
            {
                voltage_sum[board_num] = UNSIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(cell_sum, LTC6804_SOC_RESOLUTION/LTC6804_VOLTAGE_RESOLUTION);
            }
            else
            {
                voltage_error_adc_clear(board_num);
            }

            // Check UV and OV flags after system has initialized the thresholds and the ADC results are not clear.
            if((System_Powered_Up() == TRUE) && (results_clear == FALSE))
            {
                // Init temporary storage to current flag values, so that failed reads to not appear as changes
                vuv_flags = voltage_uv_flags[board_num];
                vov_flags = voltage_ov_flags[board_num];

                // Read the under-voltage and over-voltage conditions, which are only updated after the ADC conversion is complete.
                if(LTC6804_UVOV_Flags_Get(board_num, &vuv_flags, &vov_flags) == TRUE)
                {

                    // Ignore UV and OV if the cells are not present
                    vov_flags &= voltage_cell_present_flags[board_num];
                    vuv_flags &= voltage_cell_present_flags[board_num];

                    if(vov_flags || vuv_flags)
                    {
                        // todo - Using the balancers to correct and overvoltage or undervoltage condition would be better than suspending,
                        // but requires more knowledge of how the cells are connected.
                        if(suspend_sent == FALSE)
                        {
                            Balancer_Suspend(); 
                            suspend_sent = TRUE;
                        }
                    }

                    // Send Async USB response if UV or OV condition changed.  
                    if((vuv_flags != voltage_uv_flags[board_num]) || (vov_flags != voltage_ov_flags[board_num]))
                    {
                        voltage_uv_flags[board_num] = vuv_flags;
                        voltage_ov_flags[board_num] = vov_flags;
                        USB_Parser_Board_Vov_Vuv_Async(board_num);
                    }
                }
            }
        }
    }
    return;
}

// Sets the under-voltage and over-voltage thresholds in all DC2100A in the system.
BOOLEAN Voltage_UVOV_Thresholds_Set(unsigned int16 vuv_value, unsigned int16 vov_value)
{
    int16 board_num;
    BOOLEAN write_successful;
    unsigned int16 vuv_value_temp,  vov_value_temp;

    // Convert from resolution used by voltage module to the resolution used by the LTC6804 to store VUV and VOV thresholds.
    // LTC6804_UVOV_RESOLUTION doesn't allow for nice round decimal numbers, so select "closest value lower than or equal to" for vov,
    // and "closest value greater than or equal to" for vuv.
    vov_value = (vov_value / VOLTAGE_UVOV_CONVERSION) - (vov_value & VOLTAGE_UVOV_ROUND_MASK ? 1 : 0);
    vuv_value = (vuv_value / VOLTAGE_UVOV_CONVERSION) + (vuv_value & VOLTAGE_UVOV_ROUND_MASK ? 1 : 0);

    // Write the new values to the LTC6804s
    LTC6804_UVOV_Thresholds_Set(LTC6804_BROADCAST, vuv_value, vov_value);

    // Verify that the values were changed for all boards
    write_successful = TRUE;
    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        LTC6804_UVOV_Thresholds_Get(board_num, &vuv_value_temp, &vov_value_temp);
        if((vuv_value_temp != vuv_value) || (vov_value_temp != vov_value))
        {
            write_successful = FALSE;
        }
    }

    // Convert back to resolution that can be used by FW for comparison against cell voltage measurements.
    voltage_vov_threshold = vov_value * VOLTAGE_UVOV_CONVERSION;
    voltage_vuv_threshold = vuv_value * VOLTAGE_UVOV_CONVERSION;

    return write_successful;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Reports that the adc results were clear when we attempted to read, indicating the LTC6804 did not receive the Start command.
void voltage_error_adc_clear(int16 board_num)
{
    int8 temp_data[ERROR_DATA_SIZE];
    temp_data[0] = board_num;
    temp_data[1] = UPPER_BYTE(UPPER_WORD(voltage_timestamp));
    temp_data[2] = LOWER_BYTE(UPPER_WORD(voltage_timestamp));
    temp_data[3] = UPPER_BYTE(LOWER_WORD(voltage_timestamp));
    temp_data[4] = LOWER_BYTE(LOWER_WORD(voltage_timestamp));
    Error_Data_Set(ERROR_CODE_LTC6804_ADC_CLEAR, temp_data, 5);
}
