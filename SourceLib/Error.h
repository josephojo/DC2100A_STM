/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for collecting error data and sending to the DC2100A GUI.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 692 $
 $Date: 2014-09-08 11:51:35 -0400 (Mon, 08 Sep 2014) $

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

#ifndef __ERROR_H__
#define __ERROR_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define ERROR_TASK_RATE                   1000  // in ms, the rate at which the voltage measurement task is executed
#define ERROR_DATA_SIZE                   12    // in bytes

// From "Error Codes" Sheet in DC2100A_Design.xlsm
// Definition of error codes passed from the App FW to the GUI
typedef enum
{
    ERROR_CODE_NONE,                        // No Error is present in DC2100A System.
    ERROR_CODE_TEST,                        // Test Code for sending ERROR_DATA_SIZE raw bytes to the DC2100A GUI.
    ERROR_CODE_LTC6804_FAILED_CFG_WRITE,    // Errata in early LTC6804 silicon was detected, where configuration registers do not write successfully.
    ERROR_CODE_LTC6804_CRC,                 // An LTC6804 response had an incorrect CRC.
    ERROR_CODE_LTC3300_CRC,                 // An LTC3300 response had an incorrect CRC.
    ERROR_CODE_LTC6804_ADC_CLEAR,           // An LTC6804 ADC conversion returned clear, indicating that the command to start the conversion was not received.
    ERROR_CODE_LTC3300_FAILED_CMD_WRITE,    // An LTC3300 Balancer Command Read did not match the last value written.
    ERROR_NUM_CODES
} ERROR_CODE_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
extern BOOLEAN Error_Code_LTC6804_CRC_Ignore;   // Flag to ignore LTC6804_CRC errors as the DC2100A PIC scans for attached boards.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Error_Init(void);                                  // Initializes the error code module.
void Error_Monitor_Task(void);                          // Task monitors for errors reported by the system.
ERROR_CODE_TYPE Error_Data_Get(int8** error_data_ptr);  // Gets the error data for the currently set error code.
void Error_Data_Set(ERROR_CODE_TYPE code, int8* data_ptr, int8 num_bytes); // Sets the error data for one error code.
unsigned int16 Error_Count_Get(ERROR_CODE_TYPE code);   // Gets the count for an error code.  This is useful for tasks that want to operate differently when ICs are not communicating reliably.
void Error_Data_Ptr_Clear(void);                        // Clears the error data.

#endif
