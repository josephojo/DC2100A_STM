/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for receiving commands from and sending responses to the DC2100A GUI.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 750 $
 $Date: 2014-09-17 19:27:28 -0400 (Wed, 17 Sep 2014) $

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

#ifndef __USB_PARSER__
#define __USB_PARSER__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "../mbed.h"
#include "../BufferedSerial.h"
//#include "USB_Descriptors.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

extern DigitalOut LED_COMM_PIN;
extern BufferedSerial serial;


#define USB_TASK_RATE                       50     // in ms, the rate at which the USB bulk pipe traffic is serviced.

// USB Commands defined by “USB Parser” worksheet in the file DC2100A_Design.xlsm.
#define USB_PARSER_MFG_COMMAND              'O'    // Read/Write Board Manufacturing Data.
#define USB_PARSER_SYSTEM_COMMAND           's'    // Read System Data*/
#define USB_PARSER_VOLTAGE_COMMAND          'v'    // Read Board Voltage Data.
#define USB_PARSER_TEMPERATURE_COMMAND      't'    // Read Board Temperature Data.
#define USB_PARSER_TEMP_ADC_COMMAND         'l'    // Read Board Temperature Adc Values.
#define USB_PARSER_PASSIVE_BALANCE_COMMAND  'M'    // Board Passive Balancers.
#define USB_PARSER_PACK_CURRENT_COMMAND     'P'    // Read System Pack Current Data
#define USB_PARSER_CELL_PRESENT_COMMAND     'n'    // Board Cell Present.
#define USB_PARSER_TIMED_BALANCE_COMMAND    'm'    // Board Timed Balance.
#define USB_PARSER_UVOV_COMMAND             'V'    // Read Board Over-Voltage and Under-Voltage Conditions.
#define USB_PARSER_ERROR_COMMAND            'o'    // Read System Error Data.
#define USB_PARSER_LTC3300_COMMAND          'k'    // LTC3300 Raw Write via LTC6804.
#define USB_PARSER_EEPROM_COMMAND           'g'    // Read/Write/Default EEPROM.
#define USB_PARSER_UVOV_THRESHOLDS_COMMAND  'L'    // Sets Over and Under Voltage Thresholds.
#define USB_PARSER_CAP_DEMO_COMMAND         'p'    // Charge/Discharge the Cap Board.
#define USB_PARSER_ALGORITHM_COMMAND        'j'    // Timed Balance Incorporating Algorithm.
#define USB_PARSER_HELLO_COMMAND            'H'    // Reply with Hello String.  Mostly useful for testing..
#define USB_PARSER_IDSTRING_COMMAND         'i'    // Read controller ID and firmware rev, this supports legacy functions.
#define USB_PARSER_IDSTRING_COMMAND_2       'I'    // Read controller ID and firmware rev, this supports legacy functions.
#define USB_PARSER_DEFAULT_COMMAND                 // By default anything not specified is a no-op.
#define USB_PARSER_BOOT_MODE_COMMAND        'r'    // Enter Bootload Mode.
#define USB_PARSER_EMERGENCY_STOP_COMMAND   'z'    // Actuate Emergency Stop .

// EEPROM Items that are available through USB_PARSER_EEPROM_COMMAND.
typedef enum
{
    USB_PARSER_EEPROM_ITEM_NUM_CAP,
    USB_PARSER_EEPROM_ITEM_NUM_CURRENT,
    USB_PARSER_EEPROM_PACK_CURRENT_CONFIG,
    USB_PARSER_EEPROM_NUM_ITEMS
} USB_PARSER_EEPROM_ITEM_NUM_TYPE;

// String returned for any unrecognized command.
#define USB_PARSER_DEFAULT_STRING           "Not a recognized command!"
#define USB_PARSER_DEFAULT_STRING_SIZE      (sizeof(USB_PARSER_DEFAULT_STRING) - 1)

// Structure defining queues used by USB Parser
// todo - This should be private.  It's pretty gross for main() to need to access it directly.
typedef struct
{
    int8 ReadIndex;
    int8 WriteIndex;
    int8 Length;
    int8 BufferSize;
    char* buffer;
} USB_PARSER_QUEUE_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Queues used by USB Parser
// todo - These should be private.  It's pretty gross for main() to need to access them directly.
extern USB_PARSER_QUEUE_TYPE usb_parser_async_queue;            // Queue for USB responses sent asynchronously by the FW.  It operates similarly to the receive queue, that receives command from other code modules instead of the USB.
extern USB_PARSER_QUEUE_TYPE usb_parser_receive_queue;          // Queue for USB commands received from the USB.
extern USB_PARSER_QUEUE_TYPE usb_parser_transmit_queue;         // Queue for USB responses to write to the USB.

// Storage used to maintain variables while rtos_await() is called.  Note - It seems like these should be maintained already by the RTOS?
// todo - These should be private.  It's pretty gross for main() to need to access them directly.
extern char usb_parser_command;                                 // The the command being processed.
extern char usb_parser_subcommand;                              // The subcommand for the command being processed.
extern int8 usb_parser_board_num;                               // The board for which the command being processed is targeted.
extern int8 usb_parser_num_bytes;                               // The number of bytes for the command being processed.
extern int8 usb_parser_item_num;                                // The number of item number for the command being processed.

//extern const int8 usb_parser_eeprom_num_bytes; // This doesn't work for some older gnu versions
extern const int8 usb_parser_eeprom_num_bytes[USB_PARSER_EEPROM_NUM_ITEMS];

//// Length of EEPROM Items that are available through USB_PARSER_EEPROM_COMMAND. //#Scrapping - Don't need this since the CCS compiler was not used
//#if getenv("VERSION") < 5.024
//// This line causes an error now after upgrading the CCS compiler.
//extern const int8 usb_parser_eeprom_num_bytes[USB_PARSER_EEPROM_NUM_ITEMS];
//#endif

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void USB_Parser_Init(void);         // Initializes the USB Parser code module.

// Manages the asynchronous messages sent by the FW without first receiving a command from the GUI.
void USB_Parser_Async_Response(void);
void USB_Parser_Board_Vov_Vuv_Async(int16 board_num);
void USB_Parser_System_Data_Async(void);
void USB_Parser_Error_Data_Async(void);

// Packs and Unpacks the ASCII USB characters, and performs the FW functions for each USB command.
//void USB_Parser_Bootload_Command(void);
void USB_Parser_Error_Data_Response(void);
void USB_Parser_System_Data_Response(void);
void USB_Parser_System_UVOV_Command(void);
void USB_Parser_Board_Mfg_Data_Command(int16 board_num);
void USB_Parser_Board_Mfg_Data_Reset(int16 board_num);
void USB_Parser_Board_Mfg_Data_Response(int16 board_num);
void USB_Parser_Board_Passive_Balancer_Command(int16 board_num);
void USB_Parser_Board_Passive_Balancer_Response(int16 board_num);
void USB_Parser_Board_Cell_Present_Command(int16 board_num);
void USB_Parser_Board_Cell_Present_Response(int16 board_num);
void USB_Parser_Board_Active_Balancer_Command(int16 board_num);
void USB_Parser_Board_Active_Balancer_Response(int16 board_num);
void USB_Parser_Balancer_Algorithm_Command(int16 board_num);
void USB_Parser_Board_Temperature_Data_Response(int16 board_num);
void USB_Parser_Pack_Current_Data_Response(void);  
void USB_Parser_Board_Temperature_Adc_Value_Response(int16 board_num);
void USB_Parser_Board_Voltage_Data_Response(int16 board_num);
void USB_Parser_Board_LTC3300_Write_Command(int16 board_num, int8 num_bytes);
void USB_Parser_Board_LTC3300_Read_Response(int16 board_num, int8 num_bytes);
void USB_Parser_Board_EEPROM_Default_Load_Command(int16 board_num, int8 item_num, int8 mfg_key);
void USB_Parser_Board_EEPROM_Default_Save_Command(int16 board_num, int8 item_num, int8 mfg_key);
void USB_Parser_Board_EEPROM_Write_Command(int16 board_num, int8 item_num, int8 mfg_key);
void USB_Parser_Board_EEPROM_Read_Response(int16 board_num, int8 item_num);
void USB_Parser_Board_Vov_Vuv_Response(int16 board_num);
void USB_Parser_Hello_Response(void);
void USB_Parser_IDString_Response(void);

// Functions to manage the queues used by USB Parser.
// todo - These should be private.  It's pretty gross for main() to need to access them directly.
void USB_Parser_Check_Incoming(void);
void USB_Parser_Check_Outgoing(void);
BOOLEAN USB_Parser_Buffer_Put(USB_PARSER_QUEUE_TYPE* queue, char *data, int8 num_chars);
BOOLEAN USB_Parser_Buffer_Get(USB_PARSER_QUEUE_TYPE* queue, char *data, int8 num_chars);
char USB_Parser_Buffer_Get_Char(USB_PARSER_QUEUE_TYPE* queue);
BOOLEAN USB_Parser_Buffer_Put_Char(char character);
int8 USB_Parser_Buffer_Length_Used(USB_PARSER_QUEUE_TYPE* queue);
int8 USB_Parser_Buffer_Length_Remaining(USB_PARSER_QUEUE_TYPE* queue);


BOOLEAN sendUSBmessage();

// todo - if we start transmitting in binary, most of these can go away.
int8 ASCIItonybble(char character);
char nybbletoASCII(int8 Hexnybble);
BOOLEAN putUSBint8_ASCII(int8 num);
BOOLEAN putUSBint16_ASCII(int16 num);
BOOLEAN putUSBint32_ASCII(int32 num);
BOOLEAN putUSBint16(int16 num);
BOOLEAN putUSBint12_todo_remove(int16 num);
int16 getUSBint16_ASCII(void);
int8 getUSBint8_ASCII(void);

#endif
