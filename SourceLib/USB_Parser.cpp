/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for receiving commands from and sending responses to the DC2100A GUI.

 The current implementation is not ideal and is minimally documented here.  The “USB Parser” worksheet in the file DC2100A_Design.xlsm,
 is all that is provided for the interface at this time.  A future implementation of this code module will address throughput issues when
 many boards are in the system, as well as ensuring data consistency through atomically de-signed messages.  This future implementation will
 be fully documented to allow customers to create their own software to interface to the DC2100A.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 752 $
 $Date: 2014-09-18 13:02:07 -0400 (Thu, 18 Sep 2014) $

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
//#include "Typedefs.h"
//#include "./bootloader/boot.inc"
#include "DC2100A.h"
//#include "USB_Descriptors.h"
//#include "../mbed.h"
#include "../BufferedSerial.h"
#include "USB_Parser.h"
#include "System.h" 
#include "SOC.h"
#include "Balancer.h"
#include "Voltage.h"
#include "Temperature.h"
#include "Nucleo_Timer.h"
//#include "Eeprom.h"
#include "LTC3300-1.h"
#include "Error.h"
#include "Pack_Current.h"

#include <string.h>


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define USB_PARSER_ASYNC_BUFFER_SIZE  (DC2100A_MAX_BOARDS*2)
#define USB_PARSER_BUFFER_SIZE        2 * 64 //2 * MaxMessageSize
#define HELLOSTRING                   "Hello\n"
#define USB_PARSER_SEND_TIMEOUT       50                    // in ms

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Queues used by USB Parser
// todo - These should be private.  It's pretty gross for main() to need to access them directly.
USB_PARSER_QUEUE_TYPE usb_parser_async_queue;            // Queue for USB responses sent asynchronously by the FW.  It operates similarly to the receive queue, that receives command from other code modules instead of the USB.
USB_PARSER_QUEUE_TYPE usb_parser_receive_queue;          // Queue for USB commands received from the USB.
USB_PARSER_QUEUE_TYPE usb_parser_transmit_queue;         // Q ueue for USB responses to write to the USB.

// Storage used to maintain variables while rtos_await() is called.  Note - It seems like these should be maintained already by the RTOS?
// todo - These should be private.  It's pretty gross for main() to need to access them directly.
char usb_parser_command;                                 // The the command being processed.
char usb_parser_subcommand;                              // The subcommand for the command being processed.
int8 usb_parser_board_num;                               // The board for which the command being processed is targeted.
int8 usb_parser_num_bytes;                               // The number of bytes for the command being processed.
int8 usb_parser_item_num;                                // The number of item number for the command being processed.

// Length of EEPROM Items that are available through USB_PARSER_EEPROM_COMMAND.
const int8 usb_parser_eeprom_num_bytes[USB_PARSER_EEPROM_NUM_ITEMS] = { sizeof(EEPROM_CAP_TYPE) * ASCII_PER_BYTE,
                                                                        sizeof(EEPROM_CURRENT_TYPE) * ASCII_PER_BYTE,
                                                                        sizeof(EEPROM_PACK_CURRENT_CONFIG_TYPE) * ASCII_PER_BYTE};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Used by nybbletoASCII to convert a binary nybble to appropriate ASCII character, by using the binary nybble as the array index

BYTE const bin2hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

char usb_parser_async_buffer[USB_PARSER_ASYNC_BUFFER_SIZE];     // Allocate async buffer for async queue.
char usb_parser_receive_buffer[USB_PARSER_BUFFER_SIZE];         // Allocate receive buffer for receive queue.
char usb_parser_transmit_buffer[USB_PARSER_BUFFER_SIZE];        // Allocate transmit buffer for transmit queue..

BufferedSerial serial(USBTX, USBRX);
DigitalOut LED_COMM_PIN(D9);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the USB Parser code module.
void USB_Parser_Init(void)
{
    // Start with buffers cleared
    memset(usb_parser_receive_buffer, 0, sizeof(usb_parser_receive_buffer));
    memset(usb_parser_transmit_buffer, 0, sizeof(usb_parser_transmit_buffer));

    // Init Async Buffer
    usb_parser_async_queue.BufferSize = USB_PARSER_ASYNC_BUFFER_SIZE;
    usb_parser_async_queue.Length = 0;
    usb_parser_async_queue.ReadIndex = 0;
    usb_parser_async_queue.WriteIndex = 0;
    usb_parser_async_queue.buffer = usb_parser_async_buffer;

    // Init Receive Buffer
    usb_parser_receive_queue.BufferSize = USB_PARSER_BUFFER_SIZE;
    usb_parser_receive_queue.Length = 0;
    usb_parser_receive_queue.ReadIndex = 0;
    usb_parser_receive_queue.WriteIndex = 0;
    usb_parser_receive_queue.buffer = usb_parser_receive_buffer;

    // Init Transmit Buffer
    usb_parser_transmit_queue.BufferSize = USB_PARSER_BUFFER_SIZE;
    usb_parser_transmit_queue.Length = 0;
    usb_parser_transmit_queue.ReadIndex = 0;
    usb_parser_transmit_queue.WriteIndex = 0;
    usb_parser_transmit_queue.buffer = usb_parser_transmit_buffer;

    // Send Hello message to the host
    USB_Parser_Hello_Response();

    serial.baud(115200); // BufferedSerial BaudRate
}

// Returns the number of bytes that are written in the buffer
int8 USB_Parser_Buffer_Length_Used(USB_PARSER_QUEUE_TYPE* queue)
{
    return queue->Length;
}

// Returns the number of bytes that can be written to the buffer
int8 USB_Parser_Buffer_Length_Remaining(USB_PARSER_QUEUE_TYPE* queue)
{
    return (queue->BufferSize - queue->Length);
}

// Puts a series of bytes into a buffer if it is not full
BOOLEAN USB_Parser_Buffer_Put(USB_PARSER_QUEUE_TYPE* queue, char *data, int8 num_chars)
{
    BOOLEAN success;

    if(num_chars <= (queue->BufferSize - queue->Length))
    {
        // move the data and report it was moved.
        memcpy(queue->buffer + queue->WriteIndex, data, num_chars);
        queue->WriteIndex += num_chars;
        queue->Length += num_chars;
        success = TRUE;
    }
    else
    {
        // Do not put anything into the buffer if not enough room for everything.
        success = FALSE;
    }

    return success;
}

// Gets a series of bytes from a buffer
BOOLEAN USB_Parser_Buffer_Get(USB_PARSER_QUEUE_TYPE* queue, char *data, int8 num_chars)
{
    BOOLEAN success;

    if(num_chars <= queue->Length)
    {
        // move the data and report it was moved.
        memcpy(data, queue->buffer + queue->ReadIndex, num_chars);

        if(queue->Length == num_chars)
        {
            // If all characters read, reset the buffer
            queue->Length = 0;
            queue->ReadIndex = 0;
            queue->WriteIndex = 0;
        }
        else
        {
            // If not all characters read, increment pointers
            queue->ReadIndex += num_chars;
            queue->Length -= num_chars;
        }
        success = TRUE;
    }
    else
    {
        // Do not get anything from the buffer if not enough to provide requested.
        success = FALSE;
    }

    return success;
}

// Gets one character from a buffer.
char USB_Parser_Buffer_Get_Char(USB_PARSER_QUEUE_TYPE* queue)
{
    char retval;

    retval = queue->buffer[(int16)(queue->ReadIndex)];

    if(queue->Length == 1)
    {
        // If all characters read, reset the buffer
        queue->Length = 0;
        queue->ReadIndex = 0;
        queue->WriteIndex = 0;
    }
    else
    {
        // If not all characters read, increment pointers
        queue->ReadIndex++;
        queue->Length--;
    }

    return retval;
}

// Puts one character in a buffer.
BOOLEAN USB_Parser_Buffer_Put_Char(char character)
{
    return USB_Parser_Buffer_Put(&usb_parser_transmit_queue, &character, sizeof(character));
}

// Checks USB Driver for bytes, and copies to receive queue for processing by the USB Parser.
void USB_Parser_Check_Incoming(void)
{
    unsigned int16 ReceiveSize;

    // Wait until the USB has received something
    //if(usb_kbhit(1)) // #Changed - From Previous Implementation #Scrapping - Not useful for our use case. See replacement below.
    if(serial.readable())
    {
        // Get how much space is available in the receive buffer.
        ReceiveSize = USB_Parser_Buffer_Length_Remaining(&usb_parser_receive_queue);

        // Get as many bytes from the USB as will fit in the USB parser queue
        // todo - once this is all local, this will be much less gross
        //ReceiveSize = usb_gets(1, &usb_parser_receive_queue.buffer[usb_parser_receive_queue.WriteIndex], ReceiveSize, 0); // #Changed - From Previous Implementation #Scrapping - Not useful for our use case. See replacement below.
        
        int ind = 0; // #Changed - Added This after commenting lines above
        while(ind < ReceiveSize && serial.readable()) // #Changed - Added This after commenting lines above
        {
            usb_parser_receive_queue.buffer[usb_parser_receive_queue.WriteIndex + ind] = serial.getc(); // #Changed - Added This after commenting lines above
            ind++; // #Changed - Added This after commenting lines above
        }
        ReceiveSize = ind; // #Changed - Added This after commenting lines above
        usb_parser_receive_queue.Length += ReceiveSize;
        usb_parser_receive_queue.WriteIndex += ReceiveSize;
    }
}

// Copies bytes from transmit queue to USB if it is able to accept bytes.
void USB_Parser_Check_Outgoing(void)
{   // Data in transmit queue and Bulk Out port buffer is empty?
    //if((usb_parser_transmit_queue.Length != 0) && usb_tbe(1)) // #Changed - Temporarily removed usb_tbe which checks to see if the transmit buffer(Serial buffer) is empty. #Scrapping - can't implement "usb_tbe"

    // Data in transmit queue and Bulk Out port buffer is empty?
    if((usb_parser_transmit_queue.Length != 0)) // #Changed - Added This after commenting lines above. "usb_tbe" looks like the equivalent for serial.writeable which always returns 1
    {
        sendUSBmessage();  // Flush transmit queue to Bulk Out port.
    }
}

// Process queue for USB responses sent asynchronously by the FW.
// This async queue is handled similarly to the receive queue.  The other FW code modules load this queue
// with commands that look just like they came from the USB.
void USB_Parser_Async_Response(void)
{
    // Fetch current buffer character and Parse it
    switch (USB_Parser_Buffer_Get_Char(&usb_parser_async_queue))
    {
        case USB_PARSER_ERROR_COMMAND: /* Read Error Data*/
            USB_Parser_Error_Data_Response();
            break;

        case USB_PARSER_SYSTEM_COMMAND: /* Send System Data*/
            USB_Parser_System_Data_Response();
            break;

        case USB_PARSER_UVOV_COMMAND: /* Send Over-Voltage and Under-Voltage Conditions for one board */
            usb_parser_board_num = USB_Parser_Buffer_Get_Char(&usb_parser_async_queue);
            USB_Parser_Board_Vov_Vuv_Response(usb_parser_board_num);
            break;
    }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Load async queue with commands that look just like they came from the USB.
void USB_Parser_System_Data_Async(void)
{
    char data[1];
    data[0] = USB_PARSER_SYSTEM_COMMAND;
    USB_Parser_Buffer_Put(&usb_parser_async_queue, data, sizeof(data));
}

void USB_Parser_Error_Data_Async(void)
{
    char data[1];
    data[0] = USB_PARSER_ERROR_COMMAND;
    USB_Parser_Buffer_Put(&usb_parser_async_queue, data, sizeof(data));
}

void USB_Parser_Board_Vov_Vuv_Async(int16 board_num)
{
    char data[2];
    data[0] = USB_PARSER_UVOV_COMMAND;
    data[1] = board_num;
    USB_Parser_Buffer_Put(&usb_parser_async_queue, data, sizeof(data));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Pack and Unpack the ASCII USB characters, and performs the FW functions for each USB command.
// USB Commands defined by “USB Parser” worksheet in the file DC2100A_Design.xlsm

void USB_Parser_Hello_Response(void)
{
    //char res;
    for (unsigned int i = 0; i < sizeof(HELLOSTRING)-1; i++)
    {
        USB_Parser_Buffer_Put_Char(HELLOSTRING[i]);
    }

    if (useUSBTerminator == true)
    {
        USB_Parser_Buffer_Put_Char('\n');
    }
    //serial.printf("%d", res);
}

void USB_Parser_IDString_Response(void)
{
    //char id_string[DC2100A_IDSTRING_SIZE] = DC2100A_IDSTRING;
    char id_string[DC2100A_IDSTRING_SIZE];
    memcpy(id_string, DC2100A_IDSTRING, DC2100A_IDSTRING_SIZE);

    //if(System_Pic_Board_Mfg_Data_Valid == TRUE)
    //{
    //    USB_Parser_Buffer_Put(&usb_parser_transmit_queue, System_Pic_Board_Mfg_Data.model_num, DC2100A_MODEL_NUM_SIZE);
    //    USB_Parser_Buffer_Put(&usb_parser_transmit_queue, &id_string[DC2100A_MODEL_NUM_SIZE], DC2100A_IDSTRING_SIZE - DC2100A_MODEL_NUM_SIZE);
    //}
    //else
    //{
    USB_Parser_Buffer_Put(&usb_parser_transmit_queue, id_string, DC2100A_IDSTRING_SIZE);
    //}

}

//void USB_Parser_Bootload_Command(void)
//{
//    char bootload_key[SYSTEM_BOOTLOAD_KEY_SIZE];
//    char key_string[SYSTEM_BOOTLOAD_KEY_SIZE] = SYSTEM_BOOTLOAD_KEY;
//
//    USB_Parser_Buffer_Get(&usb_parser_receive_queue, bootload_key, SYSTEM_BOOTLOAD_KEY_SIZE); // #Changed - &bootload_key to bootload_key
//
//    if(memcmp(bootload_key, key_string, SYSTEM_BOOTLOAD_KEY_SIZE) == 0)
//    {
//        DC2100A_Reboot_Control._app_reboot_flags |= BOOT_CONTROL_BOOTLOAD_REQUESTED;
//
//        USB_Parser_Buffer_Put_Char('r');
//        USB_Parser_Buffer_Put(&usb_parser_transmit_queue, key_string, sizeof(key_string));
//    }
//
//    return;
//}

// Returns the manufacturing information for a board
void USB_Parser_Board_Mfg_Data_Response(int16 board_num)
{
    EEPROM_MFG_DATA_TYPE mfg_data;
    BOOLEAN mfg_data_valid;
    union
    {
        char model_num[DC2100A_MODEL_NUM_SIZE];
        char serial_num[DC2100A_SERIAL_NUM_SIZE];
        char fw_rev[APP_FW_STRING_SIZE];
    } temp_string;

    // Send the response chat with the board number
    USB_Parser_Buffer_Put_Char(USB_PARSER_MFG_COMMAND);
    putUSBint8_ASCII(board_num);

    // Read the data from the EEPROM
    if(board_num < System_Num_Boards)
    {
        mfg_data_valid = Eeprom_Mfg_Data_Get(board_num, &mfg_data);
    }
    else
    {
        mfg_data_valid = FALSE;
    }

    // #Changed - Commented out in the meantime. #NeedNow??? - Might not need this
    //// Handle board DC2100A_NUCLEO_BOARD_NUM separately since it's possible (and likely for cap demos) for the PIC to be powered by the 6804 (and EEPROM) to not be powered.
    //if((mfg_data_valid == FALSE) && (board_num == DC2100A_NUCLEO_BOARD_NUM))
    //{
    //    mfg_data_valid = System_Pic_Board_Mfg_Data_Valid;
    //    mfg_data = System_Pic_Board_Mfg_Data;
    //}

    // Send the model number, whether the board is a cap demo or not, and the serial number
    if(mfg_data_valid == TRUE)
    {
        //USB_Parser_Buffer_Put(&usb_parser_transmit_queue, &mfg_data, sizeof(mfg_data)); // #Changed - Commented out since compiler can't convert struct to char array. Using 3 lines below instead
        int8 buf[sizeof(EEPROM_MFG_DATA_TYPE)];
        memcpy(buf, &mfg_data, sizeof(EEPROM_MFG_DATA_TYPE));
        USB_Parser_Buffer_Put(&usb_parser_transmit_queue, buf, sizeof(mfg_data));
    }
    else
    {
        memcpy(temp_string.model_num, DC2100A_MODEL_NUM_DEFAULT, sizeof(temp_string.model_num));
        USB_Parser_Buffer_Put(&usb_parser_transmit_queue, temp_string.model_num, sizeof(temp_string.model_num));
        USB_Parser_Buffer_Put_Char(DC2100A_CAP_DEMO_DEFAULT);
        memcpy(temp_string.serial_num, DC2100A_SERIAL_NUM_DEFAULT, sizeof(temp_string.serial_num));
        USB_Parser_Buffer_Put(&usb_parser_transmit_queue, temp_string.serial_num, sizeof(temp_string.serial_num));
    }

    // Send charge and discharge base currents
    if((System_Model[board_num] == 'A') || (System_Model[board_num] == 'B'))
    {
        putUSBint16_ASCII(BALANCER_AB_CURRENT_CHARGE_12CELL);
        putUSBint16_ASCII(BALANCER_AB_CURRENT_DISCHARGE_12CELL);
        putUSBint16_ASCII(BALANCER_AB_CURRENT_CHARGE_6CELL);
        putUSBint16_ASCII(BALANCER_AB_CURRENT_DISCHARGE_6CELL);
    }
    else if((System_Model[board_num] == 'C') || (System_Model[board_num] == 'D'))
    {
        putUSBint16_ASCII(BALANCER_CD_CURRENT_CHARGE_12CELL);
        putUSBint16_ASCII(BALANCER_CD_CURRENT_DISCHARGE_12CELL);
        putUSBint16_ASCII(BALANCER_CD_CURRENT_CHARGE_6CELL);
        putUSBint16_ASCII(BALANCER_CD_CURRENT_DISCHARGE_6CELL);
    }
    else
    {
        putUSBint16_ASCII(0);
        putUSBint16_ASCII(0);
        putUSBint16_ASCII(0);
        putUSBint16_ASCII(0);
    }

    // Send firmware revision, but only for the boards that contain a PIC.
    if(board_num == DC2100A_NUCLEO_BOARD_NUM)
    {
        putUSBint8_ASCII(APP_FW_REV_MAJOR);
        USB_Parser_Buffer_Put_Char('.');
        putUSBint8_ASCII(APP_FW_REV_MINOR);
        USB_Parser_Buffer_Put_Char('.');
        putUSBint8_ASCII(APP_FW_REV_SUB_MINOR);

        // Only include a beta revision if it exists.  Customers should never receive beta revisions.
#if(APP_FW_REV_BETA != 0)
        USB_Parser_Buffer_Put_Char('b');
        putUSBint8_ASCII(APP_FW_REV_BETA);
#else
        USB_Parser_Buffer_Put_Char(' ');
        USB_Parser_Buffer_Put_Char(' ');
        USB_Parser_Buffer_Put_Char(' ');
#endif
    }
    else
    {
        memcpy(temp_string.fw_rev, APP_FW_STRING_DEFAULT, sizeof(temp_string.fw_rev));
        USB_Parser_Buffer_Put(&usb_parser_transmit_queue, temp_string.fw_rev, sizeof(temp_string.fw_rev));
    }
}

// Writes the manufacturing information for a board
void USB_Parser_Board_Mfg_Data_Command(int16 board_num)
{
    EEPROM_MFG_DATA_TYPE mfg_data;
    //int8 demo_present_last; // #Scrapping - Commented out indefiniitely  #Changed

    int8 buf[sizeof(EEPROM_MFG_DATA_TYPE)];
    USB_Parser_Buffer_Get(&usb_parser_receive_queue, buf, sizeof(mfg_data));
    memcpy(&mfg_data, buf, sizeof(buf));


    Eeprom_Mfg_Data_Set(board_num, &mfg_data);

    // Detect if the cap demo setting changed // #Scrapping - Commented out indefiniitely  #Changed
    //demo_present_last = System_Cap_Demo.demo_present;

    // Save the model of the detected board.
    System_Model[board_num] = mfg_data.model_num[DC2100A_MODEL_NUM_SIZE - 1];

    //// If storing to board DC2100A_NUCLEO_BOARD_NUM, then also save in flash  // #Scrapping - Commented out indefiniitely  #Changed
    //if(board_num == DC2100A_NUCLEO_BOARD_NUM)
    //{
    //    System_Mfg_Data_Set(&mfg_data);
    //}
    //else
    //{
    //    // If multiple boards are detected, the system can not be a cap board demo system.
    //    //System_Cap_Demo.demo_present = 0; // #Scrapping - Commented out indefiniitely  #Changed
    //}

    //// If the cap demo setting changed, reset the capacity/capacitance data
    //if(demo_present_last != System_Cap_Demo.demo_present) // #Scrapping - Commented out indefiniitely  #Changed
    //{
    //    Eeprom_Cap_Save_Defaults(DC2100A_NUCLEO_BOARD_NUM, EEPROM_MFG_KEY);
    //}

    return;
}

// Resets the manufacturing information for a board
void USB_Parser_Board_Mfg_Data_Reset(int16 board_num)
{
    char eeprom_key[EEPROM_RESET_KEY_SIZE];
    char system_key[SYSTEM_RESET_KEY_SIZE];

    USB_Parser_Buffer_Get(&usb_parser_receive_queue, eeprom_key, sizeof(eeprom_key));
    USB_Parser_Buffer_Get(&usb_parser_receive_queue, system_key, sizeof(system_key));

    //if(Eeprom_Reset(board_num, eeprom_key) == TRUE) // #Scrapping - Commented out indefiniitely  #Changed
    //{
    //    if(board_num == DC2100A_NUCLEO_BOARD_NUM)
    //    {
    //        System_Mfg_Data_Reset(system_key);
    //    }
    //}

    return;
}

// Writes the passive balancer states for one board.
void USB_Parser_Board_Passive_Balancer_Command(int16 board_num)
{

    Balancer_Passive_State[board_num] = getUSBint16_ASCII();

    return;
}

// Returns the passive balancer states for one board.
void USB_Parser_Board_Passive_Balancer_Response(int16 board_num)
{
    // Send the response char with the board number
    USB_Parser_Buffer_Put_Char(USB_PARSER_PASSIVE_BALANCE_COMMAND);
    putUSBint8_ASCII(board_num);

    putUSBint16_ASCII(Balancer_Passive_State[board_num]);

    return;
}

// Writes the passive balancer states for one board.
void USB_Parser_Board_Cell_Present_Command(int16 board_num)
{

    voltage_cell_present_flags[board_num] = getUSBint16_ASCII();
    if (voltage_cell_present_flags[board_num] > (1 << LTC3300_NUM_CELLS))
    {
        connectedLTC3300s[board_num] = 2;
    }
    else
    {
        connectedLTC3300s[board_num] = 1;
    }

    return;
}

// Returns the passive balancer states for one board.
void USB_Parser_Board_Cell_Present_Response(int16 board_num)
{
    // Send the response char with the board number
    USB_Parser_Buffer_Put_Char(USB_PARSER_CELL_PRESENT_COMMAND);
    putUSBint8_ASCII(board_num);

    putUSBint16_ASCII(voltage_cell_present_flags[board_num]);

    return;
}

// Writes the active balancer states for one board.
void USB_Parser_Board_Active_Balancer_Command(int16 board_num)
{
    int16 cell_num;

    // Send the balance timers for the selected board.
    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        Balancer_Active_State[board_num][cell_num] = getUSBint16_ASCII();
    }

    /*
    // Send the balance timers for the selected board.
    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        serial.printf("Cell[%d]\tCommand = %d\tTime = %i\n", cell_num + 1, (int)((Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_COMMAND_MASK) >> 15), \
            ((int)Balancer_Active_State[board_num][cell_num] & BALANCER_ACTIVE_STATE_TIME_MASK));
    }
    */

    Balancer_Max_and_Nextstop_Find();

    return;
}

// Returns the active balancer states for one board, and the max/min balance times for the whole system.
void USB_Parser_Board_Active_Balancer_Response(int16 board_num)
{
    int16 cell_num;

    // Send the response char with the board number
    USB_Parser_Buffer_Put_Char(USB_PARSER_TIMED_BALANCE_COMMAND);
    putUSBint8_ASCII(board_num);

    // Send the balance timers for the selected board.
    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        putUSBint16_ASCII(Balancer_Active_State[board_num][cell_num]);
    }

    // Send the next stop time and time remaining for the whole system.
    putUSBint16_ASCII(Balancer_Active_Time_Max);
    putUSBint8_ASCII(Balancer_Active_Board_Max);
    putUSBint16_ASCII(Balancer_Active_Time_Next_Stop);
    putUSBint8_ASCII(Balancer_Active_Board_Next_Stop);

    return;
}

void USB_Parser_Balancer_Algorithm_Command(int16 board_num)
{
    int8 cell_num;
    unsigned int16 balanceActions;

    // Receives Current commands, converts them to the charge amount (mAs) and then computes how much time to balance each cell for
    balanceActions = getUSBint16_ASCII();

    // Receives Current commands, converts them to the charge amount (mAs) and then computes how much time to balance each cell for
    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        if (BITVAL(balanceActions, cell_num) == 1) // If action is 1, make sure target charge is positive signify to discharge
        {
            Current_Commands[board_num][cell_num] = getUSBint16_ASCII();
        }
        else
        {
            Current_Commands[board_num][cell_num] = -1 * getUSBint16_ASCII(); // If action is 1, set target charge as negative signify to discharge
        }
        //serial.printf("Current[%d] = %d\n", cell_num + 1, Current_Commands[board_num][cell_num]);
    }

    SOC_Balance();
    Balancer_Max_and_Nextstop_Find();
}

// Send one error message
void USB_Parser_Error_Data_Response(void)
{
    int8 byte_num;
    ERROR_CODE_TYPE error_code;
    int8 *error_data_ptr;

    error_code = Error_Data_Get(&error_data_ptr);

    // Send the current error data
    USB_Parser_Buffer_Put_Char(USB_PARSER_ERROR_COMMAND);
    putUSBint8_ASCII(error_code);
    for (byte_num = 0; byte_num < ERROR_DATA_SIZE; byte_num++)
    {
        putUSBint8_ASCII(*error_data_ptr++);
    }

    // Clear data after it has been sent
    Error_Data_Ptr_Clear();
}

// Returns the information about the system
void USB_Parser_System_Data_Response(void)
{
    int16 board_num;

    USB_Parser_Buffer_Put_Char(USB_PARSER_SYSTEM_COMMAND);
    putUSBint8_ASCII(System_Num_Boards);
    for (board_num = 0; board_num < DC2100A_MAX_BOARDS; board_num++) // #Changed - Changed the number of boards from LTC6804_MAX_BOARDS=16 to DC2100A_MAX_BOARDS=8 since the max size of System_Adress_Table=8
    {
        putUSBint8_ASCII(System_Address_Table[board_num]);
    }

    putUSBint16_ASCII(voltage_vuv_threshold);
    putUSBint16_ASCII(voltage_vov_threshold);
    //putUSBint8_ASCII((int8) System_Cap_Demo); //# Changed - Commented this out for next 3 lines due to error: USB_Parser.cpp@674,29: invalid cast from type 'SYSTEM_CAP_DEMO_TYPE' to type 'char'
    int8 buf[sizeof(SYSTEM_CAP_DEMO_TYPE)];
    memcpy(buf, &System_Cap_Demo, sizeof(buf));
    putUSBint8_ASCII(buf[0]);
}

void USB_Parser_Board_Vov_Vuv_Response(int16 board_num)
{
    // Send the response chat with the board number
    USB_Parser_Buffer_Put_Char(USB_PARSER_UVOV_COMMAND);
    putUSBint8_ASCII(board_num);

    putUSBint12_todo_remove(voltage_ov_flags[board_num]);
    putUSBint12_todo_remove(voltage_uv_flags[board_num]);
}

// Returns the voltage information for one board
void USB_Parser_Board_Voltage_Data_Response(int16 board_num)
{
    int16 cell_num;

    USB_Parser_Buffer_Put_Char(USB_PARSER_VOLTAGE_COMMAND);
    putUSBint8_ASCII(board_num);
    if(returnSysState() != SYSTEM_STATE_AWAKE)
    {
        voltage_timestamp = NUCLEO_Timer_Update();
    }
    putUSBint32_ASCII(voltage_timestamp);

    for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
    {
        putUSBint16_ASCII(voltage_cell[board_num][cell_num]);
    }

    // Add stamp for whether balancing was active during these voltage samples.
    putUSBint8_ASCII(voltage_balancestamp);
}

// Returns the temperature information for one board
void USB_Parser_Board_Temperature_Data_Response(int16 board_num)
{
    int16 temp_num;

    USB_Parser_Buffer_Put_Char(USB_PARSER_TEMPERATURE_COMMAND);
    putUSBint8_ASCII(board_num);
    if(returnSysState() != SYSTEM_STATE_AWAKE)
    {
        temperature_timestamp = NUCLEO_Timer_Update();
    }
    putUSBint32_ASCII(temperature_timestamp);

    for (temp_num = 0; temp_num < DC2100A_NUM_TEMPS; temp_num++)
    {
        putUSBint16_ASCII(Temperature_Get(board_num, temp_num));
    }

    // Add stamp for whether balancing was active during these voltage samples.
    putUSBint8_ASCII(temperature_balancestamp);

    return;
}

void USB_Parser_Pack_Current_Data_Response(void)
{
    USB_Parser_Buffer_Put_Char(USB_PARSER_PACK_CURRENT_COMMAND);
    if(returnSysState() != SYSTEM_STATE_AWAKE)
    {
        pack_current_timestamp = NUCLEO_Timer_Update();
    }
    putUSBint32_ASCII(pack_current_timestamp);

    //putUSBint8_ASCII((int8)Pack_Current_IO);
    int8 buf[sizeof(PACK_CURRENT_IO_TYPE)];
    memcpy(buf, &Pack_Current_IO, sizeof(buf));
    putUSBint8_ASCII(buf[0]);

    putUSBint16_ASCII((int16)Pack_Current_ADC_Value);
    putUSBint32_ASCII((int32)Pack_Current);
    putUSBint8_ASCII(pack_current_balancestamp);
    return;
}

// Returns the temperature adc value information for one board if it is available
void USB_Parser_Board_Temperature_Adc_Value_Response(int16 board_num)
{
    int8 temp_num;
    int16* adc_value_ptr;

    adc_value_ptr = Temperature_Adc_Value_Get(board_num);

    if(adc_value_ptr != NULL)
    {
        USB_Parser_Buffer_Put_Char(USB_PARSER_TEMP_ADC_COMMAND);
        putUSBint8_ASCII(board_num);
        for (temp_num = 0; temp_num < DC2100A_NUM_TEMPS; temp_num++)
        {
            putUSBint16_ASCII(*adc_value_ptr++);
        }
    }

    return;
}

void USB_Parser_System_UVOV_Command(void)
{
    int16 vuv_value_new;
    int16 vov_value_new;

    vov_value_new = getUSBint16_ASCII();
    vuv_value_new = getUSBint16_ASCII();
    Voltage_UVOV_Thresholds_Set(vuv_value_new, vov_value_new);
}

void USB_Parser_Board_LTC3300_Write_Command(int16 board_num, int8 num_bytes)
{
    int8 ltc3300_data[LTC3300_COMMAND_SIZE + DC2100A_NUM_LTC3300 * LTC3300_REGISTER_SIZE];
    unsigned int16 byte_num;

    Balancer_GUI();     // Switch balancers to GUI control

    byte_num = 0;

    while (num_bytes--)
    {
        if(byte_num < sizeof(ltc3300_data))
        {
            ltc3300_data[byte_num++] = getUSBint8_ASCII();
        }
        else
        {
            // If more bytes sent than we can accept, dump them from the USB buffer.
            getUSBint8_ASCII();
        }
    }

    LTC3300_Raw_Write(board_num, ltc3300_data, byte_num);

    return;
}

void USB_Parser_Board_LTC3300_Read_Response(int16 board_num, int8 num_bytes)
{
    int8 ltc3300_data[DC2100A_NUM_LTC3300 * LTC3300_REGISTER_SIZE]; // #Changed - Made signed from unsigned 
    int8 command;
    int8 num_data_bytes;
    int16 byte_num;

    // Get command to send.
    command = getUSBint8_ASCII();

    ltc3300_data[0] = command;      // Note that the same buffer used for reading data back is also for the command to write?
    num_data_bytes = num_bytes - 1;
    LTC3300_Raw_Read(board_num, ltc3300_data, num_data_bytes);

    // Pass response to USB.  Let GUI determine if CRC error occurred, and record error

    // Build response header
    USB_Parser_Buffer_Put_Char(USB_PARSER_LTC3300_COMMAND);
    putUSBint8_ASCII(board_num);

    // Put number of bytes read and command back in response so that GUI can distinguish between different responses.
    USB_Parser_Buffer_Put_Char(nybbletoASCII(num_bytes));
    putUSBint8_ASCII(command);

    for (byte_num = 0; byte_num < num_data_bytes; byte_num++)
    {
        putUSBint8_ASCII(ltc3300_data[byte_num]);
    }

    return;
}

void USB_Parser_Board_EEPROM_Default_Load_Command(int16 board_num, int8 item_num, int8 mfg_key)
{
    switch (item_num)
    {
        case USB_PARSER_EEPROM_ITEM_NUM_CAP:
            Eeprom_Cap_Load_Defaults(board_num, mfg_key);
            break;

        case USB_PARSER_EEPROM_ITEM_NUM_CURRENT:
            Eeprom_Current_Load_Defaults(board_num, mfg_key);
            break;

        case USB_PARSER_EEPROM_PACK_CURRENT_CONFIG:
            Pack_Current_Config_Defaults();
            break;

        default:
            break;
    }

    return;
}

void USB_Parser_Board_EEPROM_Default_Save_Command(int16 board_num, int8 item_num, int8 mfg_key)
{
    switch (item_num)
    {
        case USB_PARSER_EEPROM_ITEM_NUM_CAP:
            Eeprom_Cap_Save_Defaults(board_num, mfg_key);
            break;

        case USB_PARSER_EEPROM_ITEM_NUM_CURRENT:
            Eeprom_Current_Save_Defaults(board_num, mfg_key);
            break;

        case USB_PARSER_EEPROM_PACK_CURRENT_CONFIG: 
            Pack_Current_Config_Defaults(); 
            Eeprom_Pack_Current_Config_Set();
            break;

        default:
            break;
    }

    return;
}

void USB_Parser_Board_EEPROM_Write_Command(int16 board_num, int8 item_num, int8 mfg_key)
{
    int16 cell_num;

    switch (item_num)
    {
        case USB_PARSER_EEPROM_ITEM_NUM_CAP:
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                Eeprom_Cap_Values[board_num].cap[cell_num] = getUSBint16_ASCII();
            }
            Eeprom_Cap_Save(board_num, mfg_key);
            break;

        case USB_PARSER_EEPROM_ITEM_NUM_CURRENT:
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                Eeprom_Current_Values[board_num].current[cell_num].charge = getUSBint8_ASCII();
                Eeprom_Current_Values[board_num].current[cell_num].discharge = getUSBint8_ASCII();
            }
            Eeprom_Current_Save(board_num, mfg_key);
            break;

        case USB_PARSER_EEPROM_PACK_CURRENT_CONFIG:
            *((int8*)&Eeprom_Pack_Current_Config.enable) = getUSBint8_ASCII();
            Eeprom_Pack_Current_Config.sample_time_us = getUSBint8_ASCII();
            Eeprom_Pack_Current_Config.offset = getUSBint16_ASCII();
            Eeprom_Pack_Current_Config.charge_calibration = getUSBint16_ASCII();
            Eeprom_Pack_Current_Config.discharge_calibration = getUSBint16_ASCII();
            Eeprom_Pack_Current_Config_Set();

            // Reload the configuration into the Pack Current Module and modify the sample time if necessary
            Pack_Current_Config_Get();

            //if(Pack_Current_IO.analog_enabled)   //#Changed - Commenting this out temporarily while we don't need it
            //{
            //    NUCLEO_ADC_Init(Eeprom_Pack_Current_Config.sample_time_us);
            //}

            break;

        default:
            break;
    }

    return;
}

void USB_Parser_Board_EEPROM_Read_Response(int16 board_num, int8 item_num)
{
    int16 cell_num;

    // Build response header
    USB_Parser_Buffer_Put_Char(USB_PARSER_EEPROM_COMMAND);
    putUSBint8_ASCII(board_num);

    // Put item number back in response so that GUI can distinguish between different responses.
    USB_Parser_Buffer_Put_Char(nybbletoASCII(item_num));

    switch (item_num)
    {
        case USB_PARSER_EEPROM_ITEM_NUM_CAP:
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                putUSBint16_ASCII(Eeprom_Cap_Values[board_num].cap[cell_num]);
            }
            break;

        case USB_PARSER_EEPROM_ITEM_NUM_CURRENT:
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                putUSBint8_ASCII(Eeprom_Current_Values[board_num].current[cell_num].charge);
                putUSBint8_ASCII(Eeprom_Current_Values[board_num].current[cell_num].discharge);
            }
            break;

        case USB_PARSER_EEPROM_PACK_CURRENT_CONFIG:
            putUSBint8_ASCII(Eeprom_Pack_Current_Config.enable.output);
            putUSBint8_ASCII(Eeprom_Pack_Current_Config.enable.output_inverted);
            putUSBint8_ASCII(Eeprom_Pack_Current_Config.enable.input);
            putUSBint8_ASCII(Eeprom_Pack_Current_Config.enable.analog);
            putUSBint8_ASCII(Eeprom_Pack_Current_Config.enable.unused);
            putUSBint8_ASCII(Eeprom_Pack_Current_Config.sample_time_us);
            putUSBint16_ASCII(Eeprom_Pack_Current_Config.offset);
            putUSBint16_ASCII(Eeprom_Pack_Current_Config.charge_calibration);
            putUSBint16_ASCII(Eeprom_Pack_Current_Config.discharge_calibration);
            break;

        default:
            break;
    }

    return;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Flushes the transmit queue to the USB Bulk Out buffer.
// Returns 'TRUE' if successful, 'FALSE' if unsuccessful.
BOOLEAN sendUSBmessage()
{
    BOOLEAN retval;

    //// Send bytes from the buffer
    //if(TRUE == usb_puts(1, &(usb_parser_transmit_queue.buffer[usb_parser_transmit_queue.ReadIndex]),
    //                         usb_parser_transmit_queue.Length, USB_PARSER_SEND_TIMEOUT))
        
    // Send bytes from the buffer
    if(serial.write(&(usb_parser_transmit_queue.buffer[(int16)(usb_parser_transmit_queue.ReadIndex)]), usb_parser_transmit_queue.Length))
    {
        if (useUSBTerminator == true)
        {
            serial.printf("\n");
        }
        usb_parser_transmit_queue.ReadIndex = 0;
        usb_parser_transmit_queue.WriteIndex = 0;
        usb_parser_transmit_queue.Length = 0;
        retval = TRUE;
    }
    else
    {
        retval = FALSE;
    }

    return retval;
}

// Endian independent function for transmitting 16 bit number to USB highest-byte to lowest-byte in ASCII
BOOLEAN putUSBint8_ASCII(int8 num)
{
    BOOLEAN retval = TRUE;

    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num >> 4));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num & 0x0F));

    return retval;
}

// Endian independent function for transmitting 32 bit number to USB highest-byte to lowest-byte in ASCII
BOOLEAN putUSBint32_ASCII(int32 num)
{
    BOOLEAN retval = TRUE;

// This looks more elegant, but it's not endian independent and doesn't seem to execute any faster (~120us)
//    int8* byte_ptr = (int*)(&num) + sizeof(num);
//    int8  byte_num;
//    for (byte_num = 0; byte_num < sizeof(num); byte_num++)
//    {
//        byte_ptr--;
//        retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII(*byte_ptr >> 4));
//        retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII(*byte_ptr & 0x0F));
//    }

    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 28) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 24) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 20) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 16) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 12) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 8) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num >> 4));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num & 0x0F));

    return retval;
}

// Endian independent function for transmitting 16 bit number to USB highest-byte to lowest-byte in ASCII
BOOLEAN putUSBint16_ASCII(int16 num)
{
    BOOLEAN retval = TRUE;

    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII(num >> 12));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 8) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num >> 4));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num & 0x0F));

    return retval;
}

BOOLEAN putUSBint16(int16 num)
{
    BOOLEAN retval = TRUE;

    retval &= USB_Parser_Buffer_Put_Char(num >> 12);
    retval &= USB_Parser_Buffer_Put_Char((num >> 8) & 0x0F);
    retval &= USB_Parser_Buffer_Put_Char((int8) num >> 4);
    retval &= USB_Parser_Buffer_Put_Char((int8) num & 0x0F);

    return retval;
}

BOOLEAN putUSBint12_todo_remove(int16 num)
{
    BOOLEAN retval = TRUE;

    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((num >> 8) & 0x0F));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num >> 4));
    retval &= USB_Parser_Buffer_Put_Char(nybbletoASCII((int8) num & 0x0F));

    return retval;
}

// Endian independent function for receiving 16 bit number to USB highest-byte to lowest-byte in ASCII
int16 getUSBint16_ASCII(void)
{
    char ascii_buffer[4];  // 4 ASCII characters per int16
    int8 high_byte, low_byte;

    USB_Parser_Buffer_Get(&usb_parser_receive_queue, ascii_buffer, sizeof(ascii_buffer));

    high_byte = ASCIItonybble(ascii_buffer[0]) << 4;
    high_byte += ASCIItonybble(ascii_buffer[1]);
    low_byte = ASCIItonybble(ascii_buffer[2]) << 4;
    low_byte += ASCIItonybble(ascii_buffer[3]);

    return ((int16) high_byte << 8) + low_byte;
}

int16 getUSBint12_todo_remove(void)
{
    char ascii_buffer[3];  // 3 ASCII characters per int12
    int8 high_byte, low_byte;

    USB_Parser_Buffer_Get(&usb_parser_receive_queue, ascii_buffer, sizeof(ascii_buffer));

    high_byte = ASCIItonybble(ascii_buffer[0]);
    low_byte = ASCIItonybble(ascii_buffer[1]) << 4;
    low_byte += ASCIItonybble(ascii_buffer[2]);

    return ((int16) high_byte << 8) + low_byte;
}

// Endian independent function for receiving 16 bit number to USB highest-byte to lowest-byte in ASCII
int8 getUSBint8_ASCII(void)
{
    char ascii_buffer[2];  // 2 ASCII characters per int8

    USB_Parser_Buffer_Get(&usb_parser_receive_queue, ascii_buffer, sizeof(ascii_buffer));

    return (ASCIItonybble(ascii_buffer[0]) << 4) + ASCIItonybble(ascii_buffer[1]);
}

// Converts ASCII input character to hex nybble.
int8 ASCIItonybble(char character)
{
    if((character >= '0') && (character <= '9'))
    {
        character -= '0';
    }
    else if((character >= 'A') && (character <= 'F'))
    {
        character -= ('A' - 0x0A);
    }
    else if((character >= 'a') && (character <= 'f'))
    {
        character -= ('a' - 0x0a);
    }
    else
    {
        character = 0x00;  // If not '0' ~ 9 or 'A'/'a' ~ 'F'/'f' then NAK
    }

    return character;
}

// Converts a hex nybble to an ASCII character
char nybbletoASCII(int8 Hexnybble)
{
    Hexnybble &= 0x0F;
    return bin2hex[(int16)(Hexnybble)];
}
