/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for calculating the SOC in a SuperCap Demo system and setting the LTC3300 to balance the charge.

 This code calculates the charge imbalance in the SuperCap Demo system used by the Linear Sales team.  It is very simple, using
 the relationship between charge and voltage on a capacitor:  delta Q = C * delta V. The charge imbalance is then passed to the
 Balancer module to calculate the optimal time for each balancer to be turned on for the SuperCap Demo system to balance.  See
 the section on the Balancer.c/.h code module for the details about how the charge imbalance is translated into balancer commands.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 748 $
 $Date: 2014-09-16 17:45:44 -0400 (Tue, 16 Sep 2014) $

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
#include "SOC.h"
#include "Voltage.h"
#include "Eeprom.h"
#include "Balancer.h"
#include "System.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define SOC_ALGORITHM_PASSES    2  // The number of times to recalculate the delta Q to move in each cell before starting balancing.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void SOC_Balance(void)
{

    if (System_Cap_Demo.demo_present == 1)
    {
        // If Cap Demo system, calculate the relative SOC for each cell using the Voltage/Charge relationship for capacitors.
        int16 pass_num;
        int16 cell_num;
        unsigned int32 average_voltage;
        signed int32 temp;
        signed int32 target_charge[DC2100A_NUM_CELLS];

        // Start assuming that no charge will be moved.
        memset(target_charge, 0, sizeof(target_charge));

        // Calculate the delta Q to move SOC_ALGORITHM_PASSES times.  Since the battery balancers are not 100% efficient,
        // the balancer algorithm will evenly spread the losses resulting from the desired delta Q across each cell.
        // If all cells have the same capacity, this is the correct way to distribute the losses.  If the cells have different
        // capacities, however, then less of the losses should be applied to the cells with less capacity.
        for (pass_num = 0; pass_num < SOC_ALGORITHM_PASSES; pass_num++)
        {
            // Calculate the average cell voltage, if the target charge was moved from each cell.
            average_voltage = 0;
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                average_voltage += voltage_cell[DC2100A_NUCLEO_BOARD_NUM][cell_num];
                temp = (target_charge[cell_num] * VOLTAGE_CELL_BITS_PER_MV * SOC_CAP_SCALE_FACTOR);
                average_voltage -= SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(temp, Eeprom_Cap_Values[DC2100A_NUCLEO_BOARD_NUM].cap[cell_num]);
            }
            average_voltage = (average_voltage + DC2100A_NUM_CELLS / 2) / DC2100A_NUM_CELLS;

            // Calculate the amount of charge needed to bring each cell to this average cell voltage
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                target_charge[cell_num] = voltage_cell[DC2100A_NUCLEO_BOARD_NUM][cell_num] - average_voltage;
                target_charge[cell_num] *= Eeprom_Cap_Values[DC2100A_NUCLEO_BOARD_NUM].cap[cell_num];
                target_charge[cell_num] = SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(target_charge[cell_num], VOLTAGE_CELL_BITS_PER_MV * SOC_CAP_SCALE_FACTOR);
            }

            // Calculate the amount of charge actually moved when attempting to move the target_charge.
            Balancer_Set(target_charge);
        }
    }
    else
    {
        // Note - a battery balance algorithm would go here.

        // Unlike the charge calculation algorithm above, we are not trying to balance the cells fully just once.
        // We get the current values that are then converted to the required amount of energy/"charge" (mAs) 
        // the higher level algorithm expects to be moved between each cell during it's sample time. Once this 
        // has occured the time to move that charge is then calculated with "Balancer_Set(BALANCER_DELTA_Q_TYPE* charge_target_ptr)"

        signed int32 target_charge[DC2100A_NUM_CELLS];
        for (int cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
        {
            target_charge[cell_num] =  Current_Commands[DC2100A_NUCLEO_BOARD_NUM][cell_num];
        }


        Balancer_Set(target_charge);
        Balancer_Start();




  //      // Alternative for Balancer_Set(target_charge)
  //      int16 board_num;
  //      int16 cell_num;

		//signed int32 signed_temp;
		//struct
		//{
		//	int16 charge_current;           // in mA
		//	int16 discharge_current;        // in mA
		//	signed int32 primary_charge;    // in mAs
		//	signed int32 total_charge;      // in mAs
		//} cell[DC2100A_MAX_BOARDS][DC2100A_NUM_CELLS];


		//board_num = DC2100A_NUCLEO_BOARD_NUM;

		//// Initialize variables before iteration.
		//// Balance currents are calibrated values from EEPROM..
		//// Initial guess at primary charge is the value passed into the function.
		//{
		//	int16 base_charge_current;
		//	int16 base_discharge_current;

		//	// Pick the base current, depending upon the model
		//	if ((System_Model[board_num] == 'A') || (System_Model[board_num] == 'B'))
		//	{
		//		base_charge_current = BALANCER_AB_CURRENT_CHARGE_6CELL;
		//		base_discharge_current = BALANCER_AB_CURRENT_DISCHARGE_6CELL;
		//	}
		//	else // if((System_Model[board_num] == 'C') || (System_Model[board_num] == 'D'))
		//	{
		//		base_charge_current = BALANCER_CD_CURRENT_CHARGE_6CELL;
		//		base_discharge_current = BALANCER_CD_CURRENT_DISCHARGE_6CELL;
		//	}

		//	for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)    //fill w/ voltages and capacitances for test case
		//	{
		//		// Store and scale the charge current
		//		signed_temp = base_charge_current;
		//		signed_temp *= Eeprom_Current_Values[board_num].current[cell_num].charge;
		//		signed_temp = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_CURRENT_SCALE_FACTOR_SHIFT);
		//		cell[board_num][cell_num].charge_current = base_charge_current + signed_temp;

		//		// Store and scale the discharge current
		//		signed_temp = base_discharge_current;
		//		signed_temp *= Eeprom_Current_Values[board_num].current[cell_num].discharge;
		//		signed_temp = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_CURRENT_SCALE_FACTOR_SHIFT);
		//		cell[board_num][cell_num].discharge_current = base_discharge_current + signed_temp;
		//	}
		//}

        
		//// Build balancer command with times and directions.
		//for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)      //compute total adjusted times (difference between net chg/dchg times)
		//{
		//	if (Current_Commands[board_num][cell_num].primary_charge >= 0)
		//	{
		//		signed_temp = cell[board_num][cell_num].primary_charge;
		//		signed_temp += (cell[board_num][cell_num].discharge_current >> 1);
		//		signed_temp /= cell[board_num][cell_num].discharge_current;
		//		Balancer_Active_State[board_num][cell_num] = signed_temp;
		//		if (signed_temp != 0)
		//		{
		//			Balancer_Active_State[board_num][cell_num] |= BALANCER_ACTIVE_STATE_COMMAND_MASK;  // Set bit to indicate discharging
		//		}
		//		else
		//		{
		//			Balancer_Active_State[board_num][cell_num] &= ~BALANCER_ACTIVE_STATE_COMMAND_MASK;  // Clear bit to indicate charging (or None)
		//		}

		//		// Calculate actual charge moved and scale back to mAs for SOC algorithm.
		//		//signed_temp = (signed_temp) * cell[board_num][cell_num].discharge_current; // todo - the xls feeds back the theoretical Delta-Q, not what actually is actually moved due to the quantitized balancer times.
		//		signed_temp = cell[board_num][cell_num].total_charge;
		//		charge_target_ptr[cell_num] = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_TIME_RESOLUTION_SHIFT);
		//	}
		//	else
		//	{
		//		signed_temp = -cell[board_num][cell_num].primary_charge;
		//		signed_temp += (cell[board_num][cell_num].charge_current >> 1);
		//		signed_temp /= cell[board_num][cell_num].charge_current;
		//		Balancer_Active_State[board_num][cell_num] = signed_temp;
		//		Balancer_Active_State[board_num][cell_num] &= ~BALANCER_ACTIVE_STATE_COMMAND_MASK;  // Clear bit to indicate charging (or None)

		//		// Calculate actual charge moved and scale back to mAs for SOC algorithm.
		//		//signed_temp = (-signed_temp) * cell[board_num][cell_num].charge_current; // todo - the xls feeds back the theoretical Delta-Q, not what actually is actually moved due to the quantitized balancer times.
		//		signed_temp = cell[board_num][cell_num].total_charge;
		//		charge_target_ptr[cell_num] = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_TIME_RESOLUTION_SHIFT);
		//	}

		//	balancer_max_and_nextstop_update(board_num, Balancer_Active_State[board_num][cell_num]);
		//}

  //      for (int i = 0; i < 12; i++)
  //      {
  //          serial.printf("Current_Commands[%d] = %d\n", i, Current_Commands[i]);
  //      }

    }

    return;
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
