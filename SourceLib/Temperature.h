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

/*! @file
    @ingroup Temperature
    Reference Application File for Monitoring Temperature Sensors through the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name Temperature Monitor Module Constants
//! @{
//! On DC2100A board, R35 and C30 form a 100ms time constant (thermistor resistance is insignificant to R35 at 0°C).
//! Wait 6 time constants before attempting to read thermistors.  3 time constants was determined to give unacceptable
//! error at temperatures below -20°C.
#define TEMPERATURE_THERMISTOR_DELAY     600                            //!< in ms, the delay to wait between thermistor measurements
#define TEMPERATURE_TASK_RATE            TEMPERATURE_THERMISTOR_DELAY   //!< in ms, the rate at which the temperature monitor task is executed.
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
extern unsigned int32 temperature_timestamp;                                   //!< Timestamp taken when last temperature measurement was started.
extern unsigned int8  temperature_balancestamp;                                 // 1 if balancers were on when these measurements were taken, otherwise 0
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Initializes the parts of the Temperature Module, that need to be initialized upon power-up of the PIC.
//! @return void
void Temperature_Init(void);

//! Initializes the parts of the Temperature Module, that need to be initialized upon wakeup of the LTC6804.
//! @return TRUE if initialization was successful.
BOOLEAN Temperature_Wakeup_Init(void);


//! Executes the Temperature Monitor task.
//! - Measures one thermistor value on each DC2100 board in the system.
//! - Calculates the temperature from the thermistor ADC value..
//! - Sets up LTC1380 analog mux channel for the next thermistor measurement.
//! - This task must be executed at a maximum of TEMPERATURE_THERMISTOR_DELAY, to allow circuit to settled after analog mux is switched.
//! @return void
void Temperature_Monitor_Task(void);

//! Gets one temperature from one DC2100A PCB.
//! @return the temperature in °C.
int16 Temperature_Get(int16 board_num,                      //!< The logical address for the PCB containing this Temperature.
                      int8 temperature_num                 //!< The temperature number to get from this PCB.
                      );

//! Gets the raw ADC values for all of the thermistors on one DC2100A board.
//! @return a pointer to the thermistor ADC values for this board.  Returns NULL if all ADC values have not yet been taken.
int16* Temperature_Adc_Value_Get(int16 board_num            //!< The logical address for the PCB containing this Temperature.
                                );

#endif
