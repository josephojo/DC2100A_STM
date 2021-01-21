/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.

 @verbatim
 This file contains a task to control the balancers in the DC2100A System with a resolution set by BALANCER_TASK_RATE, as well as functions
 to control and monitor the balancer task.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 1837 $
 $Date: 2015-10-14 15:13:12 -0400 (Wed, 14 Oct 2015) $

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

//! @defgroup Balancer Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.

/*! @file
    @ingroup Balancer
    Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"
#include "LTC3300-1.h"
#include "System.h"
#include "Balancer.h"
#include "LTC6804-2.h"
#include "Eeprom.h"
#include "Error.h"
#include "NUCLEO_Timer.h"
//#include "USB_Parser.h"
#include <string.h>
#include <assert.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Constants for configuration of Balancer_Set(BALANCER_DELTA_Q_TYPE* charge_target_ptr) function
// See DC2100A_Balance_Algorithm_Design.xlsm, available upon request from LTC, for a model of this function, and the meaning of these constants.
#define BALANCER_ALGORITHM_NUM_BOARDS       1       // Note - the Balancer_Set(BALANCER_DELTA_Q_TYPE* charge_target_ptr) function is limited to this many boards.
#define BALANCER_ALGORITHM_PASSES           1      // The number of iterations algorithm performs to determine optimal active balance states to achieve desired delta Q.

//#define BALANCER_TIME_RESOLUTION_SHIFT      2           // Division  #Changed - Not currently in use
#define BALANCER_TIME_RESOLUTION            (1000 / BALANCER_TASK_RATE)         // resolution = 100 Task Runs per second #Changed - Using any integer for resolution now, removed limitation due to bit shifting (1L << BALANCER_TIME_RESOLUTION_SHIFT)

#if BALANCER_TIME_RESOLUTION != (MS_PER_S/BALANCER_TASK_RATE)  // Cannot divide by floating point numbers. Sees them as zero for some reason.
#error The balancer task must be called at the frequency necessary to provide the desired resolution in balance time.
#endif

//constexpr void checkBalTimeResolution(void)
//{
//    assert(BALANCER_TIME_RESOLUTION == (MS_PER_S / BALANCER_TASK_RATE)); // The balancer task must be called at the frequency necessary to provide the desired resolution in balance time.
//}


#define BALANCER_CHARGE_EFFICIENCY                  92 //77 //92      // in %, efficiency of balancer when charging a cell #Changed - Changed Efficiency to match what was physically measured
#define BALANCER_DISCHARGE_EFFICIENCY               92 //65 //92      // in %, efficiency of balancer when discharging a cell  #Changed - Changed Efficiency to match what was physically measured
#define BALANCER_CELL_CHARGE_ERROR_DAMPING_SHIFT    1       // Damping factor for cell charge error feedback term in iterative calaculation
#define BALANCER_CELL_CHARGE_ERROR_DAMPING          (1L << BALANCER_CELL_CHARGE_ERROR_DAMPING_SHIFT)
#define BALANCER_HALF_STACK_EEROR_DAMPING_SHIFT     1
#define BALANCER_HALF_STACK_EEROR_DAMPING           (1L << BALANCER_HALF_STACK_EEROR_DAMPING_SHIFT)
//! @}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BALANCER_ACTIVE_STATE_TYPE Balancer_Active_State[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS]; // The state of each active cell balancer on each DC2100A in the system.
//bool Balancer_USE_PWM[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];                            // The method to use for controlling balancing. When true, balancing for each cell is paused periodically to emulate pwm
BALANCER_ACTIVE_STATE_TYPE Balancer_Active_Time_Max;                                     // The longest active balance time in the DC2100A system.
BALANCER_ACTIVE_STATE_TYPE Balancer_Active_Time_Next_Stop;                               // The shortest, yet non-zero, active balance time in the DC2100A system.
int8 Balancer_Active_Board_Max;                                                          // The board with the longest remaining balance time
int8 Balancer_Active_Board_Next_Stop;                                                    // The cell with the shortest, yet non-zero, remaining balance time
BALANCER_PASSIVE_STATE_TYPE Balancer_Passive_State[DC2100A_MAX_BOARDS];                  // Bitmap for LTC6804_NUM_CELLV_ADC passive balancers on one DC2100A, 1 = ON and 0 = Off, bit 0 = cell 0

//signed int32 unapplied_charge[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];
signed int16 Current_Commands[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];
unsigned int16 pwmOffFlag = 0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
BALANCER_CONTROL_STATE_TYPE balancer_control_state; // State in which the balancer task is currently operating
int16 balancer_watchdog_counter;                     // Counter for how often balancer task needs to send watchdog commands to prevent LTC3300-1 from goign to sleep.
BOOLEAN balancer_synchronous_mode;                  // TRUE if LTC3300-1 are to be operated in Synchronous mode.
unsigned int16 balancer_gate_drive_ok[DC2100A_MAX_BOARDS];   // Bitmap for gate drive signal status bits, where bit 0 is cell 1.  Note that these are 0 if balancer is not on, as well as if gate drive is not ok.
unsigned int16 balancer_cells_ov_ok;                         // Bitmap for cells overvoltage status bits, where bit 0 is board 0.  1 indicates that cells are not overvoltaged.
unsigned int16 balancer_stack_ov_ok;                         // Bitmap for stack overvoltage status bits, where bit 0 is board 0.  1 indicates that stack is not overvoltaged.
unsigned int16 balancer_temperature_ok;                      // Bitmap for temperature ok, where bit 0 is board 0.  1 indicates that stack is not overtemperature.

// For Testing
string str = "";

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void balancer_command_update(int16 board_num);
void balancer_nextstop_update(int16 board_num, int16 bal_timer);
void balancer_max_and_nextstop_update(int16 board_num, BALANCER_ACTIVE_STATE_TYPE balancer_state);
void balancer_status_update(int16 board_num);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the parts of the Balancer Module, that need to be initialized upon power-up of the PIC.
void Balancer_Init(void)
{
    memset(Balancer_Active_State, 0, sizeof(Balancer_Active_State));
    memset(Balancer_Passive_State, 0, sizeof(Balancer_Passive_State));

    balancer_control_state = BALANCER_CONTROL_OFF;
    balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;

    Balancer_Active_Time_Max = 0;
    Balancer_Active_Board_Max = 0;
    Balancer_Active_Time_Next_Stop = 0;
    Balancer_Active_Board_Next_Stop = 0;

    memset(balancer_gate_drive_ok, 0, sizeof(balancer_gate_drive_ok));
    balancer_cells_ov_ok = 0;
    balancer_stack_ov_ok = 0;
    balancer_temperature_ok = 0;

    balancer_synchronous_mode = TRUE;

}

// Initializes the parts of the Balancer Module, that need to be initialized upon wakeup of the LTC3300-1.
BOOLEAN Balancer_Wakeup_Init(void)
{
    BOOLEAN success = TRUE;
    int16 cell_num;
    int8 balancer_command[DC2100A_NUM_CELLS];

    // Balance commands are not cleared in the LTC3300, after waking from sleep.
    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        // If timer has expired, then turn balancer off
        balancer_command[cell_num] = LTC3300_BALANCER_CONTROL_CODE_NONE;
    }

    // Clear balance commands to the LTC3300s for the boards in this system
    LTC3300_Command_Write(LTC6804_BROADCAST, balancer_command);

    return success;
}

// Executes the Balancer Control task.
// - Sends watchdog commands to the LTC3300-1 ICs to keep them awake, unless in a state which allows the LTC3300-1 ICs to sleep.
// - If in the BALANCER_CONTROL_ON state, each active cell balancer command has its timer decremented while sending the appropriate commands
//   to the LTC3300-1 ICs.  The max and min (yet non-zero) balance times are tracked in this state.
// - Turns on/off the passive balancers.
// - Monitors the state of the balancers.
void Balancer_Control_Task(void)
{
    int16 board_num;
    int16 cell_num;
    int16 bal_timer;

    // Update the watchdog counter.  The reset will be handled depending upon the case.
    if(balancer_watchdog_counter != 0)
    {
        balancer_watchdog_counter--;
    }

    switch (balancer_control_state)
    {
        default:
        case BALANCER_CONTROL_OFF:
            // Not balancing, do not send commands to allow LTC3300 to turn off.
            balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
            break;

        case BALANCER_CONTROL_GUI:
            // Kick WDT but nothing else, to allow GUI directly talk to the LTC3300s
            if(balancer_watchdog_counter == 0)
            {
                LTC3300_Watchdog_Kick();
                balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
            }
            break;

        case BALANCER_CONTROL_SETUP:
            // Kick WDT.
            if(balancer_watchdog_counter == 0)
            {
                LTC3300_Suspend(LTC6804_BROADCAST);
                balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
            }

            for (board_num = 0; board_num < System_Num_Boards; board_num++)
            {
                // Update the balancer commands in the LTC3300s
                balancer_command_update(board_num);
            }
            break;

        case BALANCER_CONTROL_ON:
            // Send commands to the LTC3300.  Update first, in case the BALANCER_CONTROL_SETUP state was skipped.
            // Worst case we need enough communication time for when all of the cells change balance state at once.
            // A simple way to ensure this is to update all of the cells all of the time.
            for (board_num = 0; board_num < System_Num_Boards; board_num++)
            {
                balancer_command_update(board_num);
            }

            // Execute the loaded command.  Note that loaded commands don't change the balancer output until a subsequent Execute command is received.
            LTC3300_Execute(LTC6804_BROADCAST);

            // The next time that this task is run, all of the times will be zero.
            // Move to suspend so that the state reflects if we are actually balancing.
            if(Balancer_Active_Time_Max == 0)
            {
                balancer_control_state = BALANCER_CONTROL_GUI; // BALANCER_CONTROL_OFF; #Changed
            }
            else
            {
                // decrement the max and next stop timers.
                Balancer_Active_Time_Max--;
                if(Balancer_Active_Time_Next_Stop != 0)
                {
                    Balancer_Active_Time_Next_Stop--;
                }

                // Count down the balancing time for each board, and update the balance commands.
                // Note that the balancers are actually turned off next time task is run.
                for (board_num = 0; board_num < System_Num_Boards; board_num++)
                {
                    // decrement the count for all cells.
                    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
                    {

                        bal_timer = (Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_TIME_MASK);
                        if(bal_timer == 0)
                        {
                            Balancer_Active_State[board_num][cell_num] &= ~BALANCER_ACTIVE_STATE_COMMAND_MASK;
                        }
                        else
                        {
                            // If timer not expired, decrement it
                            Balancer_Active_State[board_num][cell_num] -= (1L << BALANCER_ACTIVE_STATE_TIME_SHIFT);

                            // Look for a new next stop.  Note that the maximum does not need to constantly be updated, as the board with the
                            // maximum balance time will remain the max until the end of balancing.
                            balancer_nextstop_update(board_num, bal_timer - 1);
                        }
                    }
                }
            }
            break;

        case BALANCER_CONTROL_SUSPEND:
            // Kick WDT.
            if(balancer_watchdog_counter == 0)
            {
                LTC3300_Suspend(LTC6804_BROADCAST);
                balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
            }
            break;
    }

    // Update status bits
    if(balancer_control_state ==  BALANCER_CONTROL_OFF)
    {
        // If LTC3300 are being allowed to turn off, then their gate drives can not be on.
        memset(balancer_gate_drive_ok, 0, sizeof(balancer_gate_drive_ok));
    }
    else
    {
        // In all other states, update the status bits.
        for (board_num = 0; board_num < System_Num_Boards; board_num++)
        {
            balancer_status_update(board_num);
        }
    }

    // Manage passive balancers
    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        LTC6804_Dischargers_Set(board_num, Balancer_Passive_State[board_num], 0);
    }

    return;

}

// Executes the Balancer Control task.
// - Sends watchdog commands to the LTC3300-1 ICs to keep them awake, unless in a state which allows the LTC3300-1 ICs to sleep.
// - If in the BALANCER_CONTROL_ON state, each active cell balancer command has its timer decremented while sending the appropriate commands
//   to the LTC3300-1 ICs.  The max and min (yet non-zero) balance times are tracked in this state.
// - Turns on/off the passive balancers.
// - Monitors the state of the balancers.
void Balancer_Control_Task_PWM(void)
{
    int16 board_num;
    int16 cell_num;
    int16 bal_timer;

    // Update the watchdog counter.  The reset will be handled depending upon the case.
    if (balancer_watchdog_counter != 0)
    {
        balancer_watchdog_counter--;
    }

    switch (balancer_control_state)
    {
    default:
    case BALANCER_CONTROL_OFF:
        // Not balancing, do not send commands to allow LTC3300 to turn off.
        balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
        break;

    case BALANCER_CONTROL_GUI:
        // Kick WDT but nothing else, to allow GUI directly talk to the LTC3300s
        if (balancer_watchdog_counter == 0)
        {
            LTC3300_Watchdog_Kick();
            balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
        }
        break;

    case BALANCER_CONTROL_SETUP:
        // Kick WDT.
        if (balancer_watchdog_counter == 0)
        {
            LTC3300_Suspend(LTC6804_BROADCAST);
            balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
        }

        for (board_num = 0; board_num < System_Num_Boards; board_num++)
        {
            // Update the balancer commands in the LTC3300s
            balancer_command_update(board_num);
        }
        break;

    case BALANCER_CONTROL_PWM_PAUSE:
        pwmOffFlag += 1;

        if ((pwmOffFlag == 1) && (pwmOffFlag == PWM_OFF_PERIOD))
        {
            LTC3300_Suspend(LTC6804_BROADCAST);
            balancer_control_state = BALANCER_CONTROL_ON;
            pwmOffFlag = 0;
        }
        else if (pwmOffFlag == 1)
        {
            LTC3300_Suspend(LTC6804_BROADCAST);
        }
        else if (pwmOffFlag == PWM_OFF_PERIOD)
        {
            balancer_control_state = BALANCER_CONTROL_ON;
            pwmOffFlag = 0;
        }
        //str += "OFF\n";
        break;

    case BALANCER_CONTROL_ON:
        // Send commands to the LTC3300.  Update first, in case the BALANCER_CONTROL_SETUP state was skipped.
        // Worst case we need enough communication time for when all of the cells change balance state at once.
        // A simple way to ensure this is to update all of the cells all of the time.
        for (board_num = 0; board_num < System_Num_Boards; board_num++)
        {
            balancer_command_update(board_num);
        }

        // Execute the loaded command.  Note that loaded commands don't change the balancer output until a subsequent Execute command is received.
        LTC3300_Execute(LTC6804_BROADCAST);

        // The next time that this task is run, all of the times will be zero.
        // Move to suspend so that the state reflects if we are actually balancing.
        if (Balancer_Active_Time_Max == 0)
        {
            balancer_control_state = BALANCER_CONTROL_GUI; // BALANCER_CONTROL_OFF; #Changed
        }
        else
        {
            // decrement the max and next stop timers.
            Balancer_Active_Time_Max--;
            if (Balancer_Active_Time_Next_Stop != 0)
            {
                Balancer_Active_Time_Next_Stop--;
            }

            // Count down the balancing time for each board, and update the balance commands.
            // Note that the balancers are actually turned off next time task is run.
            for (board_num = 0; board_num < System_Num_Boards; board_num++)
            {
                // decrement the count for all cells.
                for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
                {

                    bal_timer = (Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_TIME_MASK);
                    if (bal_timer == 0)
                    {
                        Balancer_Active_State[board_num][cell_num] &= ~BALANCER_ACTIVE_STATE_COMMAND_MASK;
                    }
                    else
                    {
						// If timer not expired, decrement it
						Balancer_Active_State[board_num][cell_num] -= (1L << BALANCER_ACTIVE_STATE_TIME_SHIFT);

						// Look for a new next stop.  Note that the maximum does not need to constantly be updated, as the board with the
						// maximum balance time will remain the max until the end of balancing.
						balancer_nextstop_update(board_num, bal_timer - 1);
                    }
                }
            }
            // For the VPLM/PWM feature: If max balancer time has been ON for the max PWM period, make sure the next control state is a rest/pause
            if (Balancer_Active_Time_Max % PWM_ON_PERIOD == 0) 
            {
                balancer_control_state = BALANCER_CONTROL_PWM_PAUSE;

                //// Turn off and back on here? // === Does not work. Equivalent to Without VPLM
                //LTC3300_Suspend(LTC6804_BROADCAST); // Pause all balancing
                //NUCLEO_Timer_Delay_us(500);
                //LTC3300_Execute(LTC6804_BROADCAST); // Resume Balancing
            }
        }
        //str += "ON\n";
        break;

    case BALANCER_CONTROL_SUSPEND:
        // Kick WDT.
        if (balancer_watchdog_counter == 0)
        {
            LTC3300_Suspend(LTC6804_BROADCAST);
            balancer_watchdog_counter = LTC3300_TWD1 / BALANCER_TASK_RATE;
        }
        break;
    }

    // Update status bits
    if (balancer_control_state == BALANCER_CONTROL_OFF)
    {
        // If LTC3300 are being allowed to turn off, then their gate drives can not be on.
        memset(balancer_gate_drive_ok, 0, sizeof(balancer_gate_drive_ok));
    }
    else
    {
        // In all other states, update the status bits.
        for (board_num = 0; board_num < System_Num_Boards; board_num++)
        {
            balancer_status_update(board_num);
        }
    }

    // Manage passive balancers
    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        LTC6804_Dischargers_Set(board_num, Balancer_Passive_State[board_num], 0);
    }

    return;

}


// Places Balancer Control Task in the BALANCER_CONTROL_SETUP state.
// Does not change the active cell balancer states.
void Balancer_Set(void)
{
    balancer_control_state = BALANCER_CONTROL_SETUP;

    return;
}

// Places Balancer Control Task in the BALANCER_CONTROL_SETUP state.
// Loads the desired active cell balancer states.
void Balancer_Set(int16 board_num, BALANCER_ACTIVE_STATE_TYPE* cell_state_ptr)
{
    int16 cell_num;

    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        Balancer_Active_State[board_num][cell_num] = cell_state_ptr[cell_num];
        balancer_max_and_nextstop_update(board_num, Balancer_Active_State[board_num][cell_num]);
    }

    Balancer_Set();

    return;
}

// Places Balancer Control Task in the BALANCER_CONTROL_SETUP state.
// Calculates and loads the optimal active cell balancer states to achieve the desired amount of charge to move for each cell.
// See DC2100A_Balance_Algorithm_Design.xlsm, available upon request from LTC, for a model of this function.
// Note - this function is currently only implemented for a single DC2100A board.
void Balancer_Set(BALANCER_DELTA_Q_TYPE* charge_target_ptr)
{
    int8 pass_num;
    int16 board_num;
    int16 cell_num;

    signed int32 signed_temp;
    signed int32 xfr_s_SUM;
    signed int32 xfr_s_HALFSUM;
    struct
    {
        int16 charge_current;           // in mA
        int16 discharge_current;        // in mA
        signed int32 primary_charge;    // in mAs
        signed int32 total_charge;      // in mAs
    } cell[BALANCER_ALGORITHM_NUM_BOARDS][DC2100A_NUM_CELLS];

    if(System_Num_Boards == BALANCER_ALGORITHM_NUM_BOARDS)
    {
        board_num = DC2100A_NUCLEO_BOARD_NUM;

        // Initialize variables before iteration.
        // Balance currents are calibrated values from EEPROM..
        // Initial guess at primary charge is the value passed into the function.
        {
            int16 base_charge_current;
            int16 base_discharge_current;

            // Pick the base current, depending upon the model
            if((System_Model[board_num] == 'A') || (System_Model[board_num] == 'B'))
            {
                base_charge_current = BALANCER_AB_CURRENT_CHARGE_6CELL;
                base_discharge_current = BALANCER_AB_CURRENT_DISCHARGE_6CELL;
            }
            else // if((System_Model[board_num] == 'C') || (System_Model[board_num] == 'D'))
            {
                base_charge_current = BALANCER_CD_CURRENT_CHARGE_6CELL;
                base_discharge_current = BALANCER_CD_CURRENT_DISCHARGE_6CELL;
            }

            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)    //fill w/ voltages and capacitances for test case
            {
                // Store and scale the charge current
                signed_temp = base_charge_current;
                signed_temp *= Eeprom_Current_Values[board_num].current[cell_num].charge;
                signed_temp = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_CURRENT_SCALE_FACTOR_SHIFT);
                cell[board_num][cell_num].charge_current = base_charge_current + signed_temp;

                // Store and scale the discharge current
                signed_temp = base_discharge_current;
                signed_temp *= Eeprom_Current_Values[board_num].current[cell_num].discharge;
                signed_temp = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_CURRENT_SCALE_FACTOR_SHIFT);
                cell[board_num][cell_num].discharge_current = base_discharge_current + signed_temp;

                // Scale to time resolution used by balancer algorithm.
                //charge_target_ptr[cell_num] <<= BALANCER_TIME_RESOLUTION_SHIFT; // #Changed - Due to resolution being multiples of 2
                charge_target_ptr[cell_num] *= BALANCER_TIME_RESOLUTION;

                // Start with the primary charge moved equal to the total charge requested to be moved.
                cell[board_num][cell_num].primary_charge = charge_target_ptr[cell_num];
            }
        }

        // Iterate to find the balancer commands and times that best move the amount of target charge per cell.
        for (pass_num = 0; pass_num < BALANCER_ALGORITHM_PASSES; pass_num++)
        {
            // Calculate the amount of charge moved into the stack by the bottom stack secondary currents.
            xfr_s_HALFSUM = 0;
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS/2; cell_num++)
            {
                if(cell[board_num][cell_num].primary_charge >= 0)
                {
                    signed_temp = cell[board_num][cell_num].primary_charge * BALANCER_DISCHARGE_EFFICIENCY; // Amount of charge actually moved into the stack (secondary side)
                    signed_temp += (DC2100A_NUM_CELLS * PERCENT_MAX) / 2; // 
                    signed_temp /= (DC2100A_NUM_CELLS * PERCENT_MAX);
                }
                else
                {
                    signed_temp = cell[board_num][cell_num].primary_charge * PERCENT_MAX;
                    signed_temp -= (DC2100A_NUM_CELLS * BALANCER_CHARGE_EFFICIENCY) / 2; // 
                    signed_temp /= (DC2100A_NUM_CELLS * BALANCER_CHARGE_EFFICIENCY);
                }
                xfr_s_HALFSUM += signed_temp;
            }

            // Calculate the amount of charge moved into the top of the stack by the top stack secondary currents.
            xfr_s_SUM = 0;
            for (cell_num = DC2100A_NUM_CELLS/2; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                if(cell[board_num][cell_num].primary_charge >= 0)
                {
                    signed_temp = cell[board_num][cell_num].primary_charge * BALANCER_DISCHARGE_EFFICIENCY;
                    signed_temp += (DC2100A_NUM_CELLS/2 * PERCENT_MAX) / 2;
                    signed_temp /= (DC2100A_NUM_CELLS/2 * PERCENT_MAX);
                }
                else
                {
                    signed_temp = cell[board_num][cell_num].primary_charge * PERCENT_MAX;
                    signed_temp -= (DC2100A_NUM_CELLS/2 * BALANCER_CHARGE_EFFICIENCY) / 2;
                    signed_temp /= (DC2100A_NUM_CELLS/2 * BALANCER_CHARGE_EFFICIENCY);
                }
                xfr_s_SUM += signed_temp;
            }

            // Calculate the total amount of charge moved (primary and secondary currents)
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)         //new voltage computation
            {
                cell[board_num][cell_num].total_charge = cell[board_num][cell_num].primary_charge - xfr_s_HALFSUM;

                if(cell_num >= DC2100A_NUM_CELLS/2)
                {
                    cell[board_num][cell_num].total_charge -= xfr_s_SUM;
                }
            }

            // Calculate the average discrepancy between the charge requested to be moved into the bottom of the stack and charge that actually was.
            xfr_s_HALFSUM = 0;
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS/2; cell_num++)
            {
                xfr_s_HALFSUM += charge_target_ptr[cell_num] - cell[board_num][cell_num].total_charge;
            }
            xfr_s_HALFSUM = SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(xfr_s_HALFSUM, DC2100A_NUM_CELLS/2);

            // Calculate the average discrepancy between the charge requested to be moved into the top of the stack and charge that actually was.
            xfr_s_SUM = 0;
            for (cell_num = DC2100A_NUM_CELLS/2; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                xfr_s_SUM += charge_target_ptr[cell_num] - cell[board_num][cell_num].total_charge;
            }
            xfr_s_SUM = SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(xfr_s_SUM, DC2100A_NUM_CELLS/2);

            // Calculate next guess for primary currents
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)         //new voltage computation
            {
                signed int32 cell_charge_feedback_term;
                signed int32 half_stack_charge_feedback_term;

                // Calculate feedback term for cell charge moved.
                cell_charge_feedback_term = charge_target_ptr[cell_num] - cell[board_num][cell_num].total_charge;
                cell_charge_feedback_term -= SIGNED_RIGHT_SHIFT_WITH_ROUND(xfr_s_HALFSUM + xfr_s_SUM, 1);    // average losses over all cells
                cell_charge_feedback_term = SIGNED_RIGHT_SHIFT_WITH_ROUND(cell_charge_feedback_term, BALANCER_CELL_CHARGE_ERROR_DAMPING_SHIFT);

                // Calculate feedback term for balancing half stacks.
                if(cell_num < DC2100A_NUM_CELLS/2)
                {
                    half_stack_charge_feedback_term = (xfr_s_HALFSUM - xfr_s_SUM);
                    half_stack_charge_feedback_term = SIGNED_RIGHT_SHIFT_WITH_ROUND(half_stack_charge_feedback_term, BALANCER_HALF_STACK_EEROR_DAMPING_SHIFT);
                }
                else
                {
                    // Cells in the top half stack can only move charge within their own half stack.
                    half_stack_charge_feedback_term = 0;
                }

                // Calculate next guess at primary charge using feedback terms.
                cell[board_num][cell_num].primary_charge += cell_charge_feedback_term;
                cell[board_num][cell_num].primary_charge += half_stack_charge_feedback_term;
            }
        }

        // Build balancer command with times and directions.
        for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)      //compute total adjusted times (difference between net chg/dchg times)
        {
            if(cell[board_num][cell_num].primary_charge >= 0)
            {
                signed_temp = cell[board_num][cell_num].primary_charge;
                signed_temp += (cell[board_num][cell_num].discharge_current >> 1);
                signed_temp /= cell[board_num][cell_num].discharge_current;
                Balancer_Active_State[board_num][cell_num] = (int16)signed_temp;
                if(signed_temp != 0 )
                {
                    Balancer_Active_State[board_num][cell_num] |= BALANCER_ACTIVE_STATE_COMMAND_MASK;  // Set bit to indicate discharging
                }
                else
                {
                    Balancer_Active_State[board_num][cell_num] &= ~BALANCER_ACTIVE_STATE_COMMAND_MASK;  // Clear bit to indicate charging (or None)
                }

                // Calculate actual charge moved and scale back to mAs for SOC algorithm.
                //signed_temp = (signed_temp) * cell[board_num][cell_num].discharge_current; // todo - the xls feeds back the theoretical Delta-Q, not what actually is actually moved due to the quantitized balancer times.
                signed_temp = cell[board_num][cell_num].total_charge;
                //charge_target_ptr[cell_num] = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_TIME_RESOLUTION_SHIFT);
                charge_target_ptr[cell_num] = SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(signed_temp, BALANCER_TIME_RESOLUTION);
            }
            else
            {
                signed_temp = -cell[board_num][cell_num].primary_charge;
                signed_temp += (cell[board_num][cell_num].charge_current >> 1);
                signed_temp /= cell[board_num][cell_num].charge_current;
                Balancer_Active_State[board_num][cell_num] = (int16)signed_temp;
                Balancer_Active_State[board_num][cell_num] &= ~BALANCER_ACTIVE_STATE_COMMAND_MASK;  // Clear bit to indicate charging (or None)

                // Calculate actual charge moved and scale back to mAs for SOC algorithm.
                //signed_temp = (-signed_temp) * cell[board_num][cell_num].charge_current; // todo - the xls feeds back the theoretical Delta-Q, not what actually is actually moved due to the quantitized balancer times.
                signed_temp = cell[board_num][cell_num].total_charge;
                //charge_target_ptr[cell_num] = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_TIME_RESOLUTION_SHIFT);
                charge_target_ptr[cell_num] = SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(signed_temp, BALANCER_TIME_RESOLUTION);
            }

            balancer_max_and_nextstop_update(board_num, Balancer_Active_State[board_num][cell_num]);

            /*int16 bal_timer;
            bal_timer = Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_TIME_MASK;
            printf("Cell[%d]\nCommand = %d\tTime = %d\n\n", cell_num + 1, (int16)((Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_COMMAND_MASK) >> 15),
                bal_timer);*/
        }

    }

    Balancer_Set();

    return;
}


// Places Balancer Control Task in the BALANCER_CONTROL_ON state.  Note that balancing is not started until the next state execution,
// to ensure that the balance times are accurately controlled to the BALANCER_TASK_RATE resolution.
void Balancer_Start(void)
{
    balancer_watchdog_counter = 0;
    balancer_control_state = BALANCER_CONTROL_ON;
}

// Places Balancer Control Task in the BALANCER_CONTROL_OFF state immediately, as it's possible there's a catastrophic reason why we,
// need to suspend,and resets the Balancer Control Task.
void Balancer_Stop(void)
{
    memset(Balancer_Active_State, 0, sizeof(Balancer_Active_State));

    Balancer_Active_Time_Max = 0;
    Balancer_Active_Board_Max = 0;
    Balancer_Active_Time_Next_Stop = 0;
    Balancer_Active_Board_Next_Stop = 0;

    LTC3300_Suspend(LTC6804_BROADCAST);
    balancer_control_state = BALANCER_CONTROL_OFF;
}

// Places Balancer Control Task in the BALANCER_CONTROL_SUSPEND state immediately as it's possible there's a catastrophic reason why we
// need to suspend.
void Balancer_Suspend(void)
{
    LTC3300_Suspend(LTC6804_BROADCAST);
    balancer_control_state = BALANCER_CONTROL_SUSPEND;
}

// Places Balancer Control Task in the BALANCER_CONTROL_GUI state, stopping all control of balancers by Balancer Control Task,
// and allows full control of LTC3300 ICs through direct commands from GUI.
void Balancer_GUI(void)
{
    balancer_control_state = BALANCER_CONTROL_GUI;
}

// Returns if any balancer is actively balancing.
BOOLEAN Balancer_Is_Balancing(void)
{
    int16 board_num;

    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        if(balancer_gate_drive_ok[board_num] != 0)
        {
            return TRUE;
        }
    }

    return FALSE;
}

// Sets Balancer Control Task for synchronous or asynchronous mode.
void Balancer_Synchronous_Mode_Set(BOOLEAN synchronous_mode)
{
    balancer_synchronous_mode = synchronous_mode;
}

// Returns if Balancer Control Task is configured for synchronous mode.
BOOLEAN Balancer_Synchronous_Mode_Get(void)
{
    return balancer_synchronous_mode;
}

// Forces recalculation of the longest and shortest (yet non-zero) active balance times in the DC2100A system.
void Balancer_Max_and_Nextstop_Find(void)
{
    int16 board_num;
    int16 cell_num;

    // Reset the Max and Nextstop, then search through all boards to find the new Max and Nextstop
    Balancer_Active_Time_Max = 0;
    Balancer_Active_Time_Next_Stop = 0;

    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
        {
            balancer_max_and_nextstop_update(board_num, Balancer_Active_State[board_num][cell_num]);
        }
    }

    return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Sends a balancer command to the LTC3300s
void balancer_command_update(int16 board_num)
{
    int16 cell_num;
    int8 charge_code;
    int8 discharge_code;
    int8 balancer_command_write[DC2100A_NUM_CELLS];
    int8 balancer_command_read[DC2100A_NUM_CELLS];
    BOOLEAN write_success;

    // Use the appropriate codes, depending upon whether set for synchronous or non-synchronous mode
    if(balancer_synchronous_mode == TRUE)
    {
        charge_code = LTC3300_BALANCER_CONTROL_CODE_CHARGE;
        discharge_code = LTC3300_BALANCER_CONTROL_CODE_DISCHARGE_SYNC;
    }
    else
    {
        charge_code = LTC3300_BALANCER_CONTROL_CODE_NONE;
        discharge_code = LTC3300_BALANCER_CONTROL_CODE_DISCHARGE_NONSYNC;
    }

    // Loop through cells to convert timer values and balance directions into LTC3300 balancer control codes.
    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        if((Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_TIME_MASK) == 0)
        {
            // If timer has expired, then turn balancer off
            balancer_command_write[cell_num] = LTC3300_BALANCER_CONTROL_CODE_NONE;
        }
        else
        {
            if(Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_COMMAND_MASK)
            {
                // If discharging, use appropriate discharge code
                balancer_command_write[cell_num] = discharge_code;
            }
            else
            {
                // If charging, use appropriate charge code
                balancer_command_write[cell_num] = charge_code;
            }
        }
    }

    // Write the balance command to the LTC3300s for this board
    LTC3300_Command_Write(board_num, balancer_command_write);

    // Read the balance command to the LTC3300s for this board
    if( LTC3300_Command_Read(board_num, balancer_command_read) == TRUE)
    {
        // Check if the write was successful
        write_success = TRUE;   // Start by assuming the write was successful
        for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
        {
            if(balancer_command_write[cell_num] != balancer_command_read[cell_num])
            {
                write_success = FALSE;   // Start by assuming the write was successful
                break;
            }
        }

        // If the write wasn't successful, create an error log entry
        if(write_success != TRUE)
        {
            int8 temp_data[ERROR_DATA_SIZE];
            memset(temp_data, 0, sizeof(temp_data));
            temp_data[0] = board_num;

            // Build a bitmap of the written and read balancer commands
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                *((int32*)(temp_data + 1)) |= ((int32)balancer_command_write[cell_num] << (cell_num * NBITS((LTC3300_BALANCER_NUM_CONTROL_CODES - 1))));
                *((int32*)(temp_data + 4)) |= ((int32)balancer_command_read[cell_num] << (cell_num * NBITS((LTC3300_BALANCER_NUM_CONTROL_CODES - 1))));
            }
            Error_Data_Set(ERROR_CODE_LTC3300_FAILED_CMD_WRITE, temp_data, sizeof(temp_data));
        }
    }

    return;
}

// Updates the longest and shortest (yet non-zero) active balance times and board numbers, if the active balancer state argument is the longest or
// shortest (yet non-zero) active balance time.
void balancer_max_and_nextstop_update(int16 board_num, BALANCER_ACTIVE_STATE_TYPE balancer_state)
{
    int16 bal_timer;

    bal_timer = balancer_state & BALANCER_ACTIVE_STATE_TIME_MASK;
    if(bal_timer > Balancer_Active_Time_Max)
    {
        Balancer_Active_Time_Max = bal_timer;
        Balancer_Active_Board_Max = board_num;
    }

    if(bal_timer != 0)
    {
        balancer_nextstop_update(board_num, bal_timer);
    }

    return;
}

// Updates the shortest (yet non-zero) active balance time and board number, if the bal_timer argument is the shortest (yet non-zero) active balance time.
void balancer_nextstop_update(int16 board_num, int16 bal_timer)
{
    if((Balancer_Active_Time_Next_Stop == 0) || (bal_timer < Balancer_Active_Time_Next_Stop))
    {
        Balancer_Active_Time_Next_Stop = bal_timer;
        Balancer_Active_Board_Next_Stop = board_num;
    }

    return;
}

// Updates the status bits for the balancers on one board.
void balancer_status_update(int16 board_num)
{
    int16 bal_num;
    int8 gate_drive_ok[DC2100A_NUM_LTC3300];
    int8 cells_ov_ok[DC2100A_NUM_LTC3300];
    int8 stack_ov_ok[DC2100A_NUM_LTC3300];
    int8 temp_ok[DC2100A_NUM_LTC3300];

    // Get the data for this board
    if(LTC3300_Status_Read(board_num, gate_drive_ok, cells_ov_ok, stack_ov_ok, temp_ok) == TRUE)
    {

        // Store data retrieved from LTC3300.

        // Init status bits to be ok, and then clear if any of the bits are not ok for this board.
        balancer_cells_ov_ok |= MASK(1, board_num);
        balancer_stack_ov_ok |= MASK(1, board_num);
        balancer_temperature_ok |= MASK(1, board_num);

        // Init gate drive bits to be not ok, as that also means off.
        balancer_gate_drive_ok[board_num] = 0;

        for (bal_num = 0; bal_num < DC2100A_NUM_LTC3300; bal_num++)
        {
            balancer_gate_drive_ok[board_num] |= ((unsigned int16) gate_drive_ok[bal_num] << (LTC3300_NUM_CELLS * bal_num));
            if(!cells_ov_ok[bal_num])
                balancer_cells_ov_ok &= ~MASK(1, board_num);
            if(!stack_ov_ok[bal_num])
                balancer_stack_ov_ok &= ~MASK(1, board_num);
            if(!temp_ok[bal_num])
                balancer_temperature_ok &= ~MASK(1, board_num);
        }
    }
    return;
}
