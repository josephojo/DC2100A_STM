/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.

 @verbatim
 This file contains a task to control the balancers in the DC2100A System with a resolution set by BALANCER_TASK_RATE, as well as functions
 to control and monitor the balancer task.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 687 $
 $Date: 2014-09-05 14:51:22 -0400 (Fri, 05 Sep 2014) $

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
    @ingroup Balancer
    Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

#ifndef __BALANCER_H__
#define __BALANCER_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name Balancer Control Module Constants
//! @{
//#define BALANCER_TASK_RATE                  62.5  //!< in ms, the rate at which the balancer control task is executed. #Changed - Reducing Balancer Rate to 62.5ms // Can't define floating point numbers
constexpr auto BALANCER_TASK_RATE = 62.5;       //!< in ms, the rate at which the balancer control task is executed. #Changed - Reducing Balancer Rate to 62.5ms;
//! @}

#define MPC_STIME 4000                          //!< in ms, the rate at which the high level sends balance charge commands to the balancer
#define PWM_ON_PERIOD 1                         //!< in multiples of BALANCER_TASK_RATE. How many task rates should the balancer be active/ON (timed) when in PWM Mode

//! @name Balancer Control States
//! @{
//! The Balancer Control Task can operate in one of the following states.
typedef enum
{
    BALANCER_CONTROL_OFF,                       //!< Balancing is not active. ICs are allowed to go to sleep.
    BALANCER_CONTROL_GUI,                       //!< Watchdog is being activated to prevent LTC3300s from going to sleep, but raw commands are being sent to ICs from the GUI.
    BALANCER_CONTROL_SETUP,                     //!< Balancing commands are being loaded. ICs are prevented from going to sleep.
    BALANCER_CONTROL_PWM_PAUSE,                 //!< Balancing commands are halted in the frequently to simulate a pwm signal and to prevent voltage spike. ICs are prevented from going to sleep.
    BALANCER_CONTROL_ON,                        //!< Balancing commands are being executed for a timed value for each cell. ICs are prevented from going to sleep.  State automatically transitions to OFF when balancing is complete.
    BALANCER_CONTROL_SUSPEND                    //!< Balancing commands are suspended.  ICs are prevented from going to sleep.
} BALANCER_CONTROL_STATE_TYPE;
//! @}

//! @name Balancer Active State Structures
//! @{
//! This C structure contains the active balance command to be executed by the Balancer task in the BALANCER_CONTROL_ON state.
//! Note that CCS gives the error "Number of bits is out of range" with the structure defined, so definitions are used to define
//! the BALANCER_ACTIVE_STATE_TYPE packed structure manually.
//! @verbatim
//!  typedef struct{
//!     int16 command : 1;       // 0 for charge, 1 for discharge
//!     int16 time : 15;         // time to balance with BALANCER_TASK_RATE resolution.
//! } BALANCER_ACTIVE_STATE_TYPE;
//! @endverbatim
typedef int16 BALANCER_ACTIVE_STATE_TYPE;                       //!< Size of structure used for balancer active state.
#define BALANCER_ACTIVE_STATE_COMMAND_BITS    1                 //!< Size of bit field used to indicate charge and discharge
#define BALANCER_ACTIVE_STATE_COMMAND_SHIFT   15                //!< Position of bit field used to indicate charge and discharge
#define BALANCER_ACTIVE_STATE_COMMAND_MASK    MASK(BALANCER_ACTIVE_STATE_COMMAND_BITS, BALANCER_ACTIVE_STATE_COMMAND_SHIFT) //!< Mask for bit field used to indicate charge and discharge (1000 0000 0000 0000)
#define BALANCER_ACTIVE_STATE_TIME_BITS       15                //!< Size of bit field used to hold time to balance
#define BALANCER_ACTIVE_STATE_TIME_SHIFT      0                 //!< Position of bit field used to hold time to balance
#define BALANCER_ACTIVE_STATE_TIME_MASK       MASK(BALANCER_ACTIVE_STATE_TIME_BITS, BALANCER_ACTIVE_STATE_TIME_SHIFT) //!< Mask for bit field used to hold time to balance (0111 1111 1111 1111)
#define BALANCER_ACTIVE_STATE_CHARGE          0                 //!< 0 for charge
#define BALANCER_ACTIVE_STATE_DISCHARGE       1                 //!< 1 for discharge
#define BALANCER_ACTIVE_STATE_COMMAND_SET(command, time)       ((command << BALANCER_ACTIVE_STATE_COMMAND_SHIFT) + (time << BALANCER_ACTIVE_STATE_TIME_SHIFT)) //!< Macro to set active balancer command.
//! @}

//! @name Balancer Passive State Structure
//! @{
typedef int16 BALANCER_PASSIVE_STATE_TYPE;      //!< Bitmap for LTC6804_NUM_CELLV_ADC passive balancers on one DC2100A, 1 = ON and 0 = Off, bit 0 = cell 0
//! @}

//! @name Balancer Charge Transfer Structure
//! @{
typedef signed int32 BALANCER_DELTA_Q_TYPE;     //!< Data Type for amount of charge to move from a cell in mAs.
//! @}

//! @name Balancer Current Constants
//! @{
//! Nominal balance currents for DC2100A models, which can be A, B, C, or D.
//! Note that calibrated balance currents use a scaled version of nominal to reduce RAM usage.
#define BALANCER_AB_CURRENT_CHARGE_12CELL      2600  //!< in mA, nominal charge balance current for cells with 12 cell secondary connections on a DC2100A-A or DC2100A-B
#define BALANCER_AB_CURRENT_DISCHARGE_12CELL   2400  //!< in mA, nominal discharge balance current for cells with 12 cell secondary connections on a DC2100A-A or DC2100A-B
#define BALANCER_AB_CURRENT_CHARGE_6CELL       2200  //!< in mA, nominal charge balance current for cells with 6 cell secondary connections on a DC2100A-A or DC2100A-B
#define BALANCER_AB_CURRENT_DISCHARGE_6CELL    2400  //!< in mA, nominal discharge balance current for cells with 6 cell secondary connections on a DC2100A-A or DC2100A-B
#define BALANCER_CD_CURRENT_CHARGE_12CELL      4000  //!< in mA, nominal charge balance current for cells with 12 cell secondary connections on a DC2100A-C or DC2100A-D
#define BALANCER_CD_CURRENT_DISCHARGE_12CELL   4300  //!< in mA, nominal discharge balance current for cells with 12 cell secondary connections on a DC2100A-C or DC2100A-D
#define BALANCER_CD_CURRENT_CHARGE_6CELL       3400  //!< in mA, nominal charge balance current for cells with 6 cell secondary connections on a DC2100A-C or DC2100A-D
#define BALANCER_CD_CURRENT_DISCHARGE_6CELL    4000  //!< in mA, nominal discharge balance current for cells with 6 cell secondary connections on a DC2100A-C or DC2100A-D
#define BALANCER_CURRENT_SCALE_FACTOR          (1L << BALANCER_CURRENT_SCALE_FACTOR_SHIFT) //!< scale that calibration values are divided by to get a % of the nominal current value.
#define BALANCER_CURRENT_SCALE_FACTOR_SHIFT    8     //!< number of bits to shift for equivalant of division by BALANCER_CURRENT_SCALE_FACTOR
#define BALANCER_CURRENT_SCALE_CALC(desired_current, base_current) ((BALANCER_CURRENT_SCALE_FACTOR*(desired_current - base_current) + base_current/2) / base_current)  //!< calculation of calibration current values
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//! @name Active Cell Balancer State Variables
//! @{
extern BALANCER_ACTIVE_STATE_TYPE Balancer_Active_State[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS]; //!< The state of each active cell balancer on each DC2100A in the system.
extern BALANCER_ACTIVE_STATE_TYPE Balancer_Active_Time_Max;                                     //!< The longest active balance time in the DC2100A system.
extern BALANCER_ACTIVE_STATE_TYPE Balancer_Active_Time_Next_Stop;                               //!< The shortest, yet non-zero, active balance time in the DC2100A system.
extern int8 Balancer_Active_Board_Max;                                                          //!< The board with the longest remaining balance time
extern int8 Balancer_Active_Board_Next_Stop;                                                    //!< The cell with the shortest, yet non-zero, remaining balance time
//extern bool Balancer_USE_PWM[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];                            //!< The method to use for controlling balancing. When true, balancing for each cell is paused periodically to emulate pwm
//! @}

//! @name Passive Cell Balancer States
//! @{
extern BALANCER_PASSIVE_STATE_TYPE Balancer_Passive_State[DC2100A_MAX_BOARDS];  //!< Bitmap for LTC6804_NUM_CELLV_ADC passive balancers on one DC2100A, 1 = ON and 0 = Off, bit 0 = cell 0
//! @}

//! @name Battery cell Balance Variables
//! @{
extern signed int32 unapplied_charge[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];
extern signed int16 Current_Commands[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Initializes the parts of the Balancer Module, that need to be initialized upon power-up of the PIC.
//! @return void
void Balancer_Init(void);

//! Initializes the parts of the Balancer Module, that need to be initialized upon wakeup of the LTC3300-1.
//! @return TRUE if initialization was successful.
BOOLEAN Balancer_Wakeup_Init(void);

//! Executes the Balancer Control task.
//! - Sends watchdog commands to the LTC3300-1 ICs to keep them awake, unless in a state which allows the LTC3300-1 ICs to sleep.
//! - If in the BALANCER_CONTROL_ON state, each active cell balancer command has its timer decremented while sending the appropriate commands
//!   to the LTC3300-1 ICs.  The max and min (yet non-zero) balance times are tracked in this state.
//! - Turns on/off the passive balancers.
//! @return void
void Balancer_Control_Task(void);

//! Executes the Balancer Control task, but uses the PWM Feature.
//! - Sends watchdog commands to the LTC3300-1 ICs to keep them awake, unless in a state which allows the LTC3300-1 ICs to sleep.
//! - If in the BALANCER_CONTROL_ON state, each active cell balancer command has its timer decremented while sending the appropriate commands
//!   to the LTC3300-1 ICs.  The max and min (yet non-zero) balance times are tracked in this state.
//! - Turns on/off the passive balancers.
//! - Pauses Balancing after every 2 task runs / sample times, emulating a pwm signal. The feature is coined as Variable Pulse Length Modulation (VPLM)
//! @return void
void Balancer_Control_Task_PWM(void);

//! Places Balancer Control Task in the BALANCER_CONTROL_SETUP state.
//! Does not change the active cell balancer states.
//! @return void
void Balancer_Set(void);

// Places Balancer Control Task in the BALANCER_CONTROL_SETUP state.
// Loads the desired active cell balancer states for DC2100A_NUM_CELLS cells on one board.
// return void
void Balancer_Set(
    int16 board_num,                               // The logical address for the PCB containing this Balancer.
    BALANCER_ACTIVE_STATE_TYPE* cell_state_ptr    // Pointer to the desired active cell balancer states for DC2100A_NUM_CELLS cells.
);

//! Places Balancer Control Task in the BALANCER_CONTROL_SETUP state.
// Calculates and loads the optimal active cell balancer states to achieve the desired amount of charge to move for each cell.
// See "CapDemo Balancing Algorithm" worksheet in DC2100A_Design.xlsm for a model of this function.
// Note - this function is currently only implemented for a single DC2100A board.
// return void
void Balancer_Set(
    BALANCER_DELTA_Q_TYPE* charge_target_ptr      // Pointer to the desired charge to move in for DC2100A_NUM_CELLS cells in mAs.
);

//! Places Balancer Control Task in the BALANCER_CONTROL_ON state.  Note that balancing is not started until the next state execution,
//! to ensure that the balance times are accurately controlled to the BALANCER_TASK_RATE resolution.
//! @return void
void Balancer_Start(void);

//! Places Balancer Control Task in the BALANCER_CONTROL_OFF state immediately, as it's possible there's a catastrophic reason why we
//! need to suspend,and resets the Balancer Control Task.
//! @return void
void Balancer_Stop(void);

//! Places Balancer Control Task in the BALANCER_CONTROL_SUSPEND state immediately as it's possible there's a catastrophic reason why we
//! need to suspend.
//! @return void
void Balancer_Suspend(void);

//! Places Balancer Control Task in the BALANCER_CONTROL_GUI state, stopping all control of balancers by Balancer Control Task,
//! and allows full control of LTC3300 ICs through direct commands from GUI.
//! @return void
void Balancer_GUI(void);

//! Returns if any balancer is actively balancing.
//! @return TRUE if any balancer is actively balancing.
BOOLEAN Balancer_Is_Balancing(void);

//! Sets Balancer Control Task for synchronous or asynchronous mode.
//! @return void
void Balancer_Synchronous_Mode_Set(BOOLEAN synchronous_mode     //!< TRUE for synchronous mode, FALSE for asynchronous mode.
                                   );

//! Returns if Balancer Control Task is configured for synchronous mode.
//! @return TRUE if Balancer Control Task is configured for synchronous mode.
BOOLEAN Balancer_Synchronous_Mode_Get(void);

//! Forces recalculation of the longest and shortest (yet non-zero) active balance times in the DC2100A system.
//! @return void
void Balancer_Max_and_Nextstop_Find(void);

#endif
