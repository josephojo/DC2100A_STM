/*
 Linear Technology DC2100A Demonstration Board.
 Driver Registers for LTC6804-2 Multicell Battery Monitors.
 All datasheet references in this file refer to Linear Technology Corp. document 680412fa.pdf.

 @verbatim
 The LTC6804 is a 3rd generation multicell battery stack
 monitor that measures up to 12 series connected battery
 cells with a total measurement error of less than 1.2mV. The
 cell measurement range of 0V to 5V makes the LTC6804
 suitable for most battery chemistries. All 12 cell voltages
 can be captured in 290us, and lower data acquisition rates
 can be selected for high noise reduction.
 Multiple LTC6804 devices can be connected in series,
 permitting simultaneous cell monitoring of long, high voltage
 battery strings. Each LTC6804 has an isoSPI interface
 for high speed, RF-immune, local area communications.
 Using the LTC6804-2, multiple devices are connected in
 a daisy-chain with one host processor connection for all
 devices. Using the LTC6804-2, multiple devices are connected
 in parallel to the host processor, with each device
 individually addressed.
 Additional features include passive balancing for each cell,
 an onboard 5V regulator, and 5 general purpose I/O lines.
 In sleep mode, current consumption is reduced to 4uA.
 The LTC6804 can be powered directly from the battery,
 or from an isolated supply.
 @endverbatim

 http://www.linear.com/product/LTC6804

 http://www.linear.com/product/LTC6804#demoboards

 REVISION HISTORY
 $Revision: 556 $
 $Date: 2014-08-08 11:34:04 -0400 (Fri, 08 Aug 2014) $

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
    @ingroup LTC6804-2
    Driver Registers for LTC6804-2 Multicell Battery Monitors.
*/

#ifndef __LTC6804_REGISTERS_H__
#define __LTC6804_REGISTERS_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// LTC6803 Command Codes are addressed, defined by datasheet Table 32 and Table 33
#define LTC6804_COMMAND_CODE_BROADCAST_ADDRESS              0x00    // Address used for an LTC6804 command to be broadcast to all boards
#define LTC6804_COMMAND_CODE_ADDRESSED_BIT                  0x10    // Bit set in address used for an LTC6804 command to be addressed to one board
#define LTC6804_COMMAND_CODE_ADDRESS_MASK                   0x0F
#define LTC6804_COMMAND_CODE_ADDRESS(command_code, address)   ((command_code & 0x7FF) | ((int16)(address & 0x1F) << 11))

// LTC6803 Command Codes Base Address, as defined by datasheet Table 34
#define LTC6804_COMMAND_CODE_BASE_WRCFG     0x001 // Write Configuration Register Group
#define LTC6804_COMMAND_CODE_BASE_RDCFG     0x002 // Read Configuration Register Group
#define LTC6804_COMMAND_CODE_BASE_RDCVA     0x004 // Read Cell Voltage Register Group A
#define LTC6804_COMMAND_CODE_BASE_RDCVB     0x006 // Read Cell Voltage Register Group B
#define LTC6804_COMMAND_CODE_BASE_RDCVC     0x008 // Read Cell Voltage Register Group C
#define LTC6804_COMMAND_CODE_BASE_RDCVD     0x00A // Read Cell Voltage Register Group D
#define LTC6804_COMMAND_CODE_BASE_RDAUXA    0x00C // Read Auxiliary Register Group A
#define LTC6804_COMMAND_CODE_BASE_RDAUXB    0x00E // Read Auxiliary Register Group B
#define LTC6804_COMMAND_CODE_BASE_RDSTATA   0x010 // Read Status Register Group A
#define LTC6804_COMMAND_CODE_BASE_RDSTATB   0x012 // Read Status Register Group B
#define LTC6804_COMMAND_CODE_BASE_ADCV      0x260 // Start Cell Voltage ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_ADOW      0x228 // Start Open Wire ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_CVST      0x207 // Start Self-Test Cell Voltage Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_ADAX      0x460 // Start GPIOs ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_AXST      0x407 // Start Self-Test GPIOs Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_ADSTAT    0x468 // Start Status group ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_STATST    0x40F // Start Self-Test Status group Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_ADCVAX    0x46F // Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
#define LTC6804_COMMAND_CODE_BASE_CLRCELL   0x711 // Clear Cell Voltage Register Group
#define LTC6804_COMMAND_CODE_BASE_CLRAUX    0x712 // Clear Auxiliary Register Group
#define LTC6804_COMMAND_CODE_BASE_CLRSTAT   0x713 // Clear Status Register Group
#define LTC6804_COMMAND_CODE_BASE_PLADC     0x714 // Poll ADC Conversion Status
#define LTC6804_COMMAND_CODE_BASE_DIAGN     0x715 // Diagnose MUX and Poll Status
#define LTC6804_COMMAND_CODE_BASE_WRCOMM    0x721 // Write COMM Register Group
#define LTC6804_COMMAND_CODE_BASE_RDCOMM    0x722 // Read COMM Register Group
#define LTC6804_COMMAND_CODE_BASE_STCOMM    0x723 // Start I2C/SPI Communication

// LTC6803 Command Codes with variable bits, as defined by datasheet Table 34 and Table 35
#define LTC6804_COMMAND_CODE_WRCFG(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_WRCFG, address)) // Write Configuration Register Group
#define LTC6804_COMMAND_CODE_RDCFG(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDCFG, address)) // Read Configuration Register Group
#define LTC6804_COMMAND_CODE_RDCVA(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDCVA, address)) // Read Cell Voltage Register Group A
#define LTC6804_COMMAND_CODE_RDCVB(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDCVB, address)) // Read Cell Voltage Register Group B
#define LTC6804_COMMAND_CODE_RDCVC(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDCVC, address)) // Read Cell Voltage Register Group C
#define LTC6804_COMMAND_CODE_RDCVD(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDCVD, address)) // Read Cell Voltage Register Group D
#define LTC6804_COMMAND_CODE_RDAUXA(address)       \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDAUXA, address)) // Read Auxiliary Register Group A
#define LTC6804_COMMAND_CODE_RDAUXB(address)       \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDAUXB, address)) // Read Auxiliary Register Group B
#define LTC6804_COMMAND_CODE_RDSTATA(address)      \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDSTATA, address)) // Read Status Register Group A
#define LTC6804_COMMAND_CODE_RDSTATB(address)      \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDSTATB, address)) // Read Status Register Group B
#define LTC6804_COMMAND_CODE_ADCV(address, md, dcp, ch)         \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_ADCV, address) + ((int16)(md & 0x3) << 7) + ((dcp & 0x1) << 4) + ((ch & 0x7) << 0)) // Start Cell Voltage ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_ADOW(address, md, pup, dcp, ch))   \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_ADOW, address) + ((int16)(md & 0x3) << 7) + ((pup & 0x1) << 4) + ((dcp & 0x1) << 4) + ((ch & 0x7) << 0)) // Start Open Wire ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_CVST(address, md, st)              \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_CVST, address) + ((int16)(md & 0x3) << 7) + ((st & 0x3) << 5)) // Start Self-Test Cell Voltage Conversion and Poll Status
#define LTC6804_COMMAND_CODE_ADAX(address, md, chg)             \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_ADAX, address) + ((int16)(md & 0x3) << 7) + ((chg & 0x7) << 0)) // Start GPIOs ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_AXST(address, md, st)              \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_AXST, address) + ((int16)(md & 0x3) << 7) + ((st & 0x3) << 5)) // Start Self-Test GPIOs Conversion and Poll Status
#define LTC6804_COMMAND_CODE_ADSTAT(address, md, chst)          \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_ADSTAT, address) + ((int16)(md & 0x3) << 7) + ((chst & 0x7) << 0)) // Start Status group ADC Conversion and Poll Status
#define LTC6804_COMMAND_CODE_STATST(address, md, st)            \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_STATST, address) + ((int16)(md & 0x3) << 7) + ((st & 0x3) << 5)) // Start Self-Test Status group Conversion and Poll Status
#define LTC6804_COMMAND_CODE_ADCVAX(address, md, dcp)           \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_ADCVAX, address) + ((int16)(md & 0x3) << 7) + ((dcp & 0x1) << 4)) // Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
#define LTC6804_COMMAND_CODE_CLRCELL(address)      \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_CLRCELL, address)) // Clear Cell Voltage Register Group
#define LTC6804_COMMAND_CODE_CLRAUX(address)       \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_CLRAUX, address)) // Clear Auxiliary Register Group
#define LTC6804_COMMAND_CODE_CLRSTAT(address)      \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_CLRSTAT, address)) // Clear Status Register Group
#define LTC6804_COMMAND_CODE_PLADC(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_PLADC, address)) // Poll ADC Conversion Status
#define LTC6804_COMMAND_CODE_DIAGN(address)        \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_DIAGN, address)) // Diagnose MUX and Poll Status
#define LTC6804_COMMAND_CODE_WRCOMM(address)       \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_WRCOMM, address)) // Write COMM Register Group
#define LTC6804_COMMAND_CODE_RDCOMM(address)       \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_RDCOMM, address)) // Read COMM Register Group
#define LTC6804_COMMAND_CODE_STCOMM(address)       \
        (LTC6804_COMMAND_CODE_ADDRESS(LTC6804_COMMAND_CODE_BASE_STCOMM, address)) // Start I2C/SPI Communication

// Configuration Register Group specified by datasheet Table 46: memory Bit Descriptions
// todo - some of this stuff could be dependent upon ADC bits, Number of GPIO, and Number of cells
#define LTC6804_CFGR0_ADCOPT(adcopt)    (adcopt << 0)               // ADC Mode Option Bit
#define LTC6804_CFGR0_ADCOPT_MASK       LTC6804_CFGR0_ADCOPT(1)     // ADCOPT:  0 -> Selects Modes 27kHz, 7kHz or 26Hz with MD[1:0] Bits in ADC Conversion Commands. 1 -> Selects Modes 14kHz, 3kHz or 2kHz with MD[1:0] Bits in ADC Conversion Commands.
#define LTC6804_CFGR0_SWTRD_MASK        0x02                        // SWTEN Pin Status (Read Only): 1 -> SWTEN Pin at Logic 1 0 -> SWTEN Pin at Logic 0
#define LTC6804_CFGR0_REFON(ref_on)     ((ref_on ? 1 : 0) << 2)     // Reference Powered Up
#define LTC6804_CFGR0_REFON_MASK        LTC6804_CFGR0_REFON(1)      // 1 -> Reference Remains Powered Up Until Watchdog Timeout 0 -> Reference Shuts Down after Conversions
#define LTC6804_CFGR0_GPIOx(gpiox)      ((gpiox & 0x1F) << 3)       // GPIOx Pin Control
#define LTC6804_CFGR0_GPIOx_MASK        LTC6804_CFGR0_GPIOx(0x1F)   // Write: 0 -> GPIOx Pin Pull-Down ON; 1-> GPIOx Pin Pull-Down OFF Read: 0 -> GPIOx Pin at Logic 0; 1 -> GPIOx Pin at Logic 1
#define LTC6804_CFGR1_VUV(vuv)          ((vuv & 0xFFF) << 0)        // Undervoltage Comparison Voltage*
#define LTC6804_CFGR1_VUV_MASK          LTC6804_CFGR1_VUV(0xFFF)    // Comparison voltage = (VUV + 1) • 16 • 100µV Default:  VUV  =  0x000
#define LTC6804_CFGR2_VOV(vov)          ((vov & 0xFFF) << 4)        // Overvoltage Comparison Voltage*
#define LTC6804_CFGR2_VOV_MASK          LTC6804_CFGR2_VOV(0xFFF)    // Comparison voltage = VOV • 16 • 100µV Default:  VOV  =  0x000
#define LTC6804_CFGR4_DCCx(dccx)        ((dccx & 0xFFF) << 0)       // Discharge Cell x
#define LTC6804_CFGR4_DCCx_MASK         LTC6804_CFGR4_DCCx(0xFFF)   // "x = 1 to 12    1 -> Turn ON Shorting Switch for Cell x 0 -> Turn OFF Shorting Switch for Cell x (Default)"
#define LTC6804_CFGR5_DCTO(dcto)        ((dcto & 0xF) << 4)         // Discharge  Time Out Value
#define LTC6804_CFGR5_DCTO_MASK         LTC6804_CFGR5_DCTO(0xF)     // todo - each value has a different code, with read and write being different.  Enum really necessary?

// Bit Definitions for adc options are specified by datasheet Table 46.
#define LTC6804_ADCOPT_0                    0x0         // 0 -> Selects Modes 27kHz, 7kHz or 26Hz with MD[1:0] Bits in ADC Conversion Commands.
#define LTC6804_ADCOPT_1                    0x1         // 1 -> Selects Modes 14kHz, 3kHz or 2kHz with MD[1:0] Bits in ADC Conversion Commands.

// Bit Definitions for command codes are specified by datasheet Table 35.
#define LTC6804_MD_MODE_FAST                0x1         // ADC Mode: 27kHz (ADCOPT = 0), 14kHz (ADCOPT = 1)
#define LTC6804_MD_MODE_NORMAL              0x2         // ADC Mode: 7kHz  (ADCOPT = 0), 3kHz (ADCOPT = 1)
#define LTC6804_MD_MODE_FILTERED            0x3         // ADC Mode: 26Hz (ADCOPT = 0), 2kHz (ADCOPT = 1)

// Bit Definition for discharge permission is specified by datasheet Table 46.
#define LTC6804_DCP_DISCHARGE_NOT_PERMITTED 0           // Discharge Not Permitted
#define LTC6804_DCP_DISCHARGE_PERMITTED     1           // Discharge Permitted

// Bit Definitions for COMM Register Group specified by datasheet Table 46: memory Bit Descriptions
#define LTC6804_ICOM_I2C_WRITE_START                            0x6
#define LTC6804_ICOM_I2C_WRITE_STOP                             0x1
#define LTC6804_ICOM_I2C_WRITE_BLANK                            0x0
#define LTC6804_ICOM_I2C_WRITE_NO_TRANSMIT                      0x7
#define LTC6804_ICOM_I2C_READ_START                             0x6
#define LTC6804_ICOM_I2C_READ_STOP                              0x1
#define LTC6804_ICOM_I2C_READ_SDA_LOW                           0x0
#define LTC6804_ICOM_I2C_READ_SDA_HIGH                          0x7
#define LTC6804_FCOM_WRITE_I2C_ACK                              0x0
#define LTC6804_FCOM_WRITE_I2C_NACK                             0x8
#define LTC6804_FCOM_WRITE_I2C_NACK_STOP                        0x9
#define LTC6804_FCOM_READ_I2C_ACK_FROM_MASTER                   0x0
#define LTC6804_FCOM_READ_I2C_ACK_FROM_SLAVE                    0x7
#define LTC6804_FCOM_READ_I2C_NACK_FROM_SLAVE                   0xF
#define LTC6804_FCOM_READ_I2C_ACK_FROM_SLAVE_STOP_FROM_MASTER   0x1
#define LTC6804_FCOM_READ_I2C_NACK_FROM_SLAVE_STOP_FROM_MASTER  0x9

#define LTC6804_ICOM_SPI_WRITE_CSB_LOW                          0x8
#define LTC6804_ICOM_SPI_WRITE_CSB_HIGH                         0x9
#define LTC6804_ICOM_SPI_WRITE_NO_TRANSMIT                      0xF
#define LTC6804_ICOM_SPI_READ                                   0x7
#define LTC6804_FCOM_SPI_WRITE_CSB_LOW                          0x8
#define LTC6804_FCOM_SPI_WRITE_CSB_HIGH                         0x9
#define LTC6804_FCOM_SPI_READ                                   0xF

// COMM register data bytes must be set to 0xFF when reading.
#define LTC6804_COMM_READ_DUMMY                                 0xFF

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
