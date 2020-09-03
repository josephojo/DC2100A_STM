/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Monitoring Voltages from the LTC6804-2 Battery Monitor on the DC2100A PCB.

 @verbatim
 This file contains the interface to a task to read the voltages in the DC2100A System at a rate set by VOLTAGE_TASK_RATE.

 The task measures all cell voltages in the system, and calculates the sum-of-cells voltages for each LTC6804 in the system.
 Each cell is monitored for under-voltage (UV) and over-voltage (OV), and balancing is automatically stopped if these conditions occur.

 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 565 $
 $Date: 2014-08-13 16:11:06 -0400 (Wed, 13 Aug 2014) $

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
    @ingroup Voltage
    Reference Application File for Monitoring Voltages from the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

#ifndef __VOLTAGE_H__
#define __VOLTAGE_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "LTC6804-2.h"
#include "DC2100A.h"


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name Voltage Monitor Module Constants
//! @{
#define VOLTAGE_TASK_RATE                   100  // 200  //       //!< in ms, the rate at which the voltage monitor task is executed
#define VOLTAGE_CELL_BITS_PER_MV            (UV_PER_V/MV_PER_V/LTC6804_VOLTAGE_RESOLUTION)  //!< number of bits per mV in cell voltage measurements
//! @}

typedef unsigned int16 VOLTAGE_CELL_PRESENT_TYPE;      //!< Bitmap indicating if a cell input on the LTC6804 is populated or shorted.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name Voltage Measurements
//! @{
extern unsigned int16 voltage_cell[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];     //!< Measured cell voltages for each board in LTC6804_VOLTAGE_RESOLUTION.
extern unsigned int16 voltage_sum[DC2100A_MAX_BOARDS];                         //!< Measured sum of the cell voltages for this board in LTC6804_SOC_RESOLUTION.
extern unsigned int32 voltage_timestamp;                                        //!< Timestamp taken when voltage measurements were started.
extern unsigned int8  voltage_balancestamp;                                     //!< 1 if balancers were on when these measurements were taken, otherwise 0
//! @}

//! @name Under-Voltage and Over-Voltage Monitoring
//! @{
extern unsigned int16 voltage_vov_threshold;                                    //!< over-voltage threshold in LTC6804_VOLTAGE_RESOLUTION.
extern unsigned int16 voltage_vuv_threshold;                                    //!< under-voltage threshold in LTC6804_VOLTAGE_RESOLUTION.
extern unsigned int16 voltage_ov_flags[DC2100A_MAX_BOARDS];                     //!< Bitmap indicating if a cell input on the LTC6804 is over-voltage (1) or not (0).
extern unsigned int16 voltage_uv_flags[DC2100A_MAX_BOARDS];                     //!< Bitmap indicating if a cell input on the LTC6804 is under-voltage (1) or not (0).
extern VOLTAGE_CELL_PRESENT_TYPE voltage_cell_present_flags[DC2100A_MAX_BOARDS];//!< Bitmap indicating if a cell input on the LTC6804 is unpopulated (0), such that UV conditions should be ignored.
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Initializes the parts of the Voltage Module, that need to be initialized upon power-up of the PIC.
//! @return void
void Voltage_Init(void);

//! Initializes the parts of the Voltage Module, that need to be initialized upon wakeup of the LTC6804.
//! @return TRUE if initialization was successful.
BOOLEAN Voltage_Wakeup_Init(void);

//! Executes the Voltage Monitor task.
//! - Measures all cell voltages in the system.  Note that all DC2100A voltages are started simultaneously and read sequentially.
//! - A timestamp is attached to each voltage for mathematical operations to be performed on the samples.
//! - The state of the balancers is saved, as the cell voltage measurements are affected by the large DC2100A balance currents.
//! - Calculates the sum-of-cells voltages for each LTC6804 in the system.
//! - Monitors each cell for UV and OV.  Balancing is stopped if UV or OV occurs, and in USB message is sent to the GUI
//! @return void
void Voltage_Monitor_Task(void);

//! Sets the under-voltage and over-voltage thresholds in all DC2100A in the system.
//! @return TRUE if voltage thresholds successfully set.
BOOLEAN Voltage_UVOV_Thresholds_Set(unsigned int16 vuv_value,   //!< over-voltage threshold in LTC6804_VOLTAGE_RESOLUTION.
                                    unsigned int16 vov_value    //!< under-voltage threshold in LTC6804_VOLTAGE_RESOLUTION.
                                    );

#endif
