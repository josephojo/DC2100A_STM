/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for controlling the charger and discharger in a SuperCap Demo System, monitoring inputs that can be used to make the FW operate
 in a charging vs discharging mode, and monitoring an ADC input which represents the Pack Current.

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

#ifndef __PACK_CURRENT_H__
#define __PACK_CURRENT_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define PACK_CURRENT_TASK_RATE                   100    // in ms, the rate at which the current monitor task is executed

// Configures the usage of the IO available for monitoring the pack current, as well as control/monitoring of an external charger/discharger
typedef struct {
    int8 output_enabled     : 1;            // Set if the IO are to be used to control an external charger and discharger.
    int8 output_inverted    : 1;            // Inverts discharger output (available as inverted and non-inverted on J21).
    int8 charging_output    : 1;            // Turn on the charger in the SuperCap Demo system.
    int8 discharging_output : 1;            // Turn on the discharger in the SuperCap Demo system.
    int8 input_enabled      : 1;            // Set if the IO are to be used to monitor external charging and discharging signals.
    int8 charging_input     : 1;            // State of the charging input pin.
    int8 discharging_input  : 1;            // State of the discharging input pin.
    int8 analog_enabled     : 1;            // Set if the ADC pin measurement is to be used as a current signal.
} PACK_CURRENT_IO_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
extern PACK_CURRENT_IO_TYPE Pack_Current_IO;       // Bitmap for digital inputs/outputs used to monitor/control an external charger or discharger.
extern unsigned int16 Pack_Current_ADC_Value;      // Raw Value read from ADC.
extern signed int32 Pack_Current;                  // Pack Current in mA, with discharging > 0 and charging < 0.
extern unsigned int32 pack_current_timestamp;      // Timestamp taken when current measurement was started.
extern unsigned int8  pack_current_balancestamp;   // 1 if balancers were on when this measurement were taken, otherwise 0.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the parts of the Pack Current Module, that need to be initialized upon power-up of the PIC.
// return void
void Pack_Current_Init(void);
void Pack_Current_Config_Get(void);
void Pack_Current_Config_Defaults(void);
void Pack_Current_Monitor_Task(void);

#endif
