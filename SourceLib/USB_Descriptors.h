/*
 Linear Technology DC2100A Demonstration Board.
 This code contains the USB descriptors used by the DC2100A to identify itself to the connected PC.

 This file is in the format required by the CCS USB driver.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 1836 $
 $Date: 2015-10-14 15:07:45 -0400 (Wed, 14 Oct 2015) $

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

#ifndef __USB_DESCRIPTORS__
#define __USB_DESCRIPTORS__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Note - these must be defined before the header file for the CCS USB driver is included.
#define USB_HID_DEVICE      FALSE                       // Disable HID device functionality.
#define USB_CDC_DEVICE      FALSE                       // Disable COM port device functionality.
#define MaxMessageSize      64
#define USB_EP1_RX_ENABLE   USB_ENABLE_BULK             // Turn on EP1(EndPoint1) for OUT bulk/interrupt transfers.
#define USB_EP1_RX_SIZE     MaxMessageSize              // Size to allocate for the rx endpoint 1 buffer.
#define USB_EP1_TX_ENABLE   USB_ENABLE_BULK             // Turn on EP1(EndPoint1) for IN bulk/interrupt transfers.
#define USB_EP1_TX_SIZE     MaxMessageSize              // Size to allocate for the tx endpoint 1 buffer.

#include "USB.h"
#include "Typedefs.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define USB_REVISION            0x0200         // USB specification v2.0 coded as BCD.
#define USB_VID_LINEAR          0x1272         // LTC Vendor ID.
#define USB_PID_DC2100A         0x8004         // DC2100A Product ID.
#define USB_DEVICE_VERSION      0x0100         // DC2100A Device Version coded as BCD.

typedef enum
{
    USB_DESCRIPTOR_STRING_LANGUAGE,
    USB_DESCRIPTOR_STRING_MANUFACTURER,
    USB_DESCRIPTOR_STRING_PRODUCT,
    USB_DESCRIPTOR_STRING_SERIAL_NUMBER,
    USB_DESCRIPTOR_NUM_STRINGS
} USB_DESCRIPTOR_STRING_ENUM;

#define USB_TOTAL_CONFIG_LEN    32            // Configuration + interface + class + endpoint

//NOTE: DO TO A LIMITATION OF THE CCS CODE, ALL HID INTERFACES MUST START AT 0 AND BE SEQUENTIAL
//      FOR EXAMPLE, IF YOU HAVE 2 HID INTERFACES THEY MUST BE INTERFACE 0 AND INTERFACE 1
#define USB_NUM_HID_INTERFACES  0

//the maximum number of interfaces seen on any config
//for example, if config 1 has 1 interface and config 2 has 2 interfaces you must define this as 2
#define USB_MAX_NUM_INTERFACES  1


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
///////////////////////////////////////////////////////////////////////////
///                                                                     ///
///   start config descriptor                                           ///
///   right now we only support one configuration descriptor.           ///
///   the config, interface, class, and endpoint goes into this array.  ///
///                                                                     ///
///////////////////////////////////////////////////////////////////////////

//configuration descriptor
char const USB_CONFIG_DESC[] = {
//config_descriptor for config index 1
USB_DESC_CONFIG_LEN,                // Length of descriptor
USB_DESC_CONFIG_TYPE,               // Constant CONFIGURATION (0x02)
USB_TOTAL_CONFIG_LEN, 0,            // Size of all data returned for this config
1,                                  // Number of interfaces this device supports
0x01,                               // Identifier for this configuration.  (IF we had more than one configurations)
0x00,                               // Index of string descriptor for this configuration
0xC0,                               // Bit 6=1 if self powered, bit 5=1 if supports remote wakeup (we don't), bits 0-4 reserved and bit7=1
0x32,                               // Maximum bus power required (maximum milliamperes/2)  (0x32 = 100mA)

//interface descriptor 0 alt 0
USB_DESC_INTERFACE_LEN,             // Length of descriptor
USB_DESC_INTERFACE_TYPE,            // Constant INTERFACE (0x04)
0x00,                               // Number defining this interface (IF we had more than one interface)
0x00,                               // Alternate setting
2,                                  // Number of endpoints, not counting endpoint 0.
0xFF,                               // Class code, 0xFF = Vendor Specific
0x00,                               // Subclass code
0x00,                               // Protocol code
0x00,                               // Index of string descriptor for interface

//endpoint descriptor
USB_DESC_ENDPOINT_LEN,              // Length of descriptor
USB_DESC_ENDPOINT_TYPE,             // Constant ENDPOINT (0x05)
0x81,                               // Endpoint number and direction (0x81 = EP1 IN)
0x02,                               // Transfer type supported (0 is control, 1 is iso, 2 is bulk, 3 is interrupt)
USB_EP1_TX_SIZE, 0x00,              // Maximum packet size supported
0x01,                               // Polling interval in ms. (for interrupt transfers ONLY)

//endpoint descriptor
USB_DESC_ENDPOINT_LEN,              // Length of descriptor
USB_DESC_ENDPOINT_TYPE,             // Constant ENDPOINT (0x05)
0x01,                               // Endpoint number and direction (0x01 = EP1 OUT)
0x02,                               // Transfer type supported (0 is control, 1 is iso, 2 is bulk, 3 is interrupt)
USB_EP1_RX_SIZE, 0x00,              // Maximum packet size supported
0x01,                               // Polling interval in ms. (for interrupt transfers ONLY)

};

///////////////////////////////////////////////////////////////////////////
///                                                                     ///
///   since we can't make pointers to constants in certain pic16s,      ///
///   this is an offset table to find a specific descriptor in the      ///
///   above table.                                                      ///
///                                                                     ///
///////////////////////////////////////////////////////////////////////////

//define how many interfaces there are per config.  [0] is the first config, etc.
const char USB_NUM_INTERFACES[USB_NUM_CONFIGURATIONS] = { 1 };

#if (sizeof(USB_CONFIG_DESC) != USB_TOTAL_CONFIG_LEN)
#error USB_TOTAL_CONFIG_LEN not defined correctly
#endif

///////////////////////////////////////////////////////////////////////////
///   start device descriptors
///
//////////////////////////////////////////////////////////////////

//device descriptor
char const USB_DEVICE_DESC[] =
{
    USB_DESC_DEVICE_LEN,                    // The length of this descriptor
    0x01,                                   // Constant DEVICE (0x01)
    LOWER_BYTE(USB_REVISION),               // USB version in BCD
    UPPER_BYTE(USB_REVISION),               //
    0x00,                                   // Class code (if 0, interface defines class.  FF is vendor defined)
    0x00,                                   // Subclass code
    0x00,                                   // Protocol code
    USB_MAX_EP0_PACKET_LENGTH,              // Max packet size for endpoint 0. (SLOW SPEED SPECIFIES 8)
    LOWER_BYTE(USB_VID_LINEAR),             // Vendor ID
    UPPER_BYTE(USB_VID_LINEAR),             //
    LOWER_BYTE(USB_PID_DC2100A),            // Product ID
    UPPER_BYTE(USB_PID_DC2100A),            //
    LOWER_BYTE(USB_DEVICE_VERSION),         // Device Version
    UPPER_BYTE(USB_DEVICE_VERSION),         //
    USB_DESCRIPTOR_STRING_MANUFACTURER,     // Index of string description of manufacturer. therefore we point to string_1 array (see below)
    USB_DESCRIPTOR_STRING_PRODUCT,          // Index of string descriptor of the product
    USB_DESCRIPTOR_STRING_SERIAL_NUMBER,    // Index of string descriptor of serial number
    USB_NUM_CONFIGURATIONS                  // Number of possible configurations
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned int16 Usb_Descriptor_String_Setup(unsigned int8 string_num, unsigned int16* usb_mem_ptr);  // Setup structure to track the characters in a string descriptor as they are sent to the USB driver.
char Usb_Descriptor_String_Get_Char(unsigned int16* usb_mem_ptr);                                   // Gets one character from a string descriptor.

#endif
