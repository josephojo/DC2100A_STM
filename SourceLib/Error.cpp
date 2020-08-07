/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for collecting error data and sending to the DC2100A GUI.

 This code collects information about errors in the DC2100A system so that they can be recorded in the GUI Software’s Event Log.

 It is very minimal, providing only an error code and 12 bytes specific to the error.  It was primarily intended to expose communication
 issues with the ICs on the DC2100A through the GUI.

 The types of errors recognized are defined in ERROR_CODE_TYPE.  Each type of error is counted, but the full data is only collected
 for one error per ERROR_TASK_RATE ms. This prevents this code module from hijacking all of the USB bandwidth when a fault occurs.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 724 $
 $Date: 2014-09-11 11:02:17 -0400 (Thu, 11 Sep 2014) $

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
#include "DC2100A.h" // #ComeBack - Commented out
#include "Error.h"
#include "LTC6804-2.h"
#include "LTC3300-1.h"
#include "USB_Parser.h"
#include <string.h>
#include "System.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Codes used to test for errata in early LTC6804 silicon where configuration registers do not write successfully.
#define ERROR_LTC6804_FAILED_CFG_TEST_VUV       0x000
#define ERROR_LTC6804_FAILED_CFG_TEST_VOV       0xFFF

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
BOOLEAN Error_Code_LTC6804_CRC_Ignore;          // Flag to ignore LTC6804_CRC errors as the DC2100A PIC scans for attached boards.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
ERROR_CODE_TYPE error_code_current;             // The error code for which the error data was set.
int8 error_data[ERROR_DATA_SIZE];               // Error data for set error code to be passed to the DC2100A GUI.
int16 error_count[ERROR_NUM_CODES];             // Count of how many times each error code has been set.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void error_detect_ltc6804_failed_cfg_write(void);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the error code module.
void Error_Init(void)
{
    memset(error_count, 0, sizeof(error_count));
    Error_Code_LTC6804_CRC_Ignore = FALSE;
    Error_Data_Ptr_Clear();
}

// Task monitors for errors reported by the system. 				
// Sends error data for one error code to the DC2100A GUI.
// If there is no error code to send, monitors for errata in early LTC6804 silicon where configuration registers do not write successfully.
void Error_Monitor_Task(void)
{
    if(error_code_current != ERROR_CODE_NONE)
    {
        // If another task already set the error code, then send it asynchronously over USB
        USB_Parser_Error_Data_Async(); 
        //if (error_code_current == ERROR_CODE_LTC6804_CRC) // #ComeBack - This is temporary 
        //{
        //    printf("There's been an Error : ERROR_CODE_LTC6804_CRC - Could not write to device!\n");
        //}
        //else
        //{
        //    printf("There's been an Error!\n");
        //}
    }
    else if(returnSysState() == SYSTEM_STATE_AWAKE)
    {
        // No other error reported, do additional diagnostic checks
        error_detect_ltc6804_failed_cfg_write();
    }
} 

// Gets the error data for the currently set error code.
ERROR_CODE_TYPE Error_Data_Get(int8** data_ptr)
{
    *data_ptr = error_data;
    return error_code_current;
}

// Sets the error data for one error code.
void Error_Data_Set(ERROR_CODE_TYPE code, int8* data_ptr, int8 num_bytes)
{
    // Ignore 6804 CRC errors if the System Monitor Module is attempting to detect new boards attached to the system.
    if((ERROR_CODE_LTC6804_CRC == code) && (Error_Code_LTC6804_CRC_Ignore == TRUE))
    {
        return;
    }

    // Ignore 6804 and 3300 CRC errors if the System Monitor Module is not above the powered-up threshold. 
    if(System_Powered_Up() == FALSE)
    {
        if((ERROR_CODE_LTC6804_CRC == code) ||
           (ERROR_CODE_LTC6804_FAILED_CFG_WRITE == code) ||
           (ERROR_CODE_LTC6804_ADC_CLEAR == code) ||
           (ERROR_CODE_LTC3300_CRC == code) ||
           (ERROR_CODE_LTC3300_FAILED_CMD_WRITE == code))
        {
            return;
        }
    } 

    // Do not overwrite an existing code that hasn't been sent out yet.
    if(ERROR_CODE_NONE == error_code_current)
    {
        error_code_current = code;
        memcpy(error_data, data_ptr, MIN(num_bytes, sizeof(error_data)));
    }
    error_count[code]++;
    return;
}

// Gets the count for an error code.  This is useful for tasks that want to operate differently when ICs are not communicating reliably.
unsigned int16 Error_Count_Get(ERROR_CODE_TYPE code)
{
    return error_count[code];
}

// Clears the error data.
void Error_Data_Ptr_Clear(void)
{
    error_code_current = ERROR_CODE_NONE;
    memset(error_data, 0, sizeof(error_data));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Function to test for errata in early LTC6804 silicon where configuration registers do not write successfully.
void error_detect_ltc6804_failed_cfg_write(void)
{
    unsigned int16 vuv_value_old, vov_value_old;
    unsigned int16 vuv_value_test, vov_value_test;
    unsigned int16 board_num;
	
	int System_Num_Boards = 1; // #ComeBack - Added this temporariliy
	
    for (board_num = 0; board_num < System_Num_Boards; board_num++)
    {
        // Store old vuv and vov values
        LTC6804_UVOV_Thresholds_Get(board_num, &vuv_value_old, &vov_value_old);

        // Trying writing benign vuv and vov values to the CFG register
        LTC6804_UVOV_Thresholds_Set(board_num, ERROR_LTC6804_FAILED_CFG_TEST_VUV, ERROR_LTC6804_FAILED_CFG_TEST_VOV);

        // Read back the benign vuv and vov values from the CFG register
        LTC6804_UVOV_Thresholds_Get(board_num, &vuv_value_test, &vov_value_test);

        // Test to make sure the write was successful.
        if((vuv_value_test != ERROR_LTC6804_FAILED_CFG_TEST_VUV) || (vov_value_test != ERROR_LTC6804_FAILED_CFG_TEST_VOV))
        {
            int8 temp_data[ERROR_DATA_SIZE];
            temp_data[0] = board_num;
            temp_data[1] = UPPER_BYTE(vuv_value_test);
            temp_data[2] = LOWER_BYTE(vuv_value_test);
            temp_data[3] = UPPER_BYTE(ERROR_LTC6804_FAILED_CFG_TEST_VUV);
            temp_data[4] = LOWER_BYTE(ERROR_LTC6804_FAILED_CFG_TEST_VUV);
            temp_data[5] = UPPER_BYTE(vov_value_test);
            temp_data[6] = LOWER_BYTE(vov_value_test);
            temp_data[7] = UPPER_BYTE(ERROR_LTC6804_FAILED_CFG_TEST_VOV);
            temp_data[8] = LOWER_BYTE(ERROR_LTC6804_FAILED_CFG_TEST_VOV);
            Error_Data_Set(ERROR_CODE_LTC6804_FAILED_CFG_WRITE, temp_data, sizeof(temp_data));
        }

        // Write back old vuv and vov values
        LTC6804_UVOV_Thresholds_Set(board_num, vuv_value_old, vov_value_old);
    }

    return;
}

