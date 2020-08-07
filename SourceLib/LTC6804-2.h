/*
 Linear Technology DC2100A Demonstration Board.
 API Header File for LTC6804-2 Multicell Battery Monitors.
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
 $Revision: 693 $
 $Date: 2014-09-08 13:52:04 -0400 (Mon, 08 Sep 2014) $

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
    API Header File for LTC6804-2 Multicell Battery Monitors.
*/

#ifndef __LTC6804_H__
#define __LTC6804_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
//#include "DC2100A.h" // #NeededNow???

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! @name LTC6804-2 Driver Properties
//! @{
//!
#define LTC6804_MAX_BOARDS                  16                  //!< The maximum number of addresses available to the LTC6804-2
#define LTC6804_BROADCAST                   LTC6804_MAX_BOARDS  //!< Code for application code to indicate an LTC6804 command is to be broadcast to all boards
#define LTC6804_NUM_CELLV_ADC               12                  //!< Number of cell voltage ADC measurements
#define LTC6804_NUM_GPIO                    5                   //!< Number of GPIO pins available on LTC6804
#define LTC6804_NUM_AUX_ADC                 6                   //!< Number of auxiliary ADC measurements
#define LTC6804_ADC_CLEAR                   0xFFFF              //!< ADC Value returned when results are cleared, but not retaken.
//! @}

//! @name Cell Selections
//! @{
//! Cell selection for ADC Conversion from datasheet table 35.
typedef enum {
    LTC6804_CH_ALL = 0,                 //!< All Cells
    LTC6804_CH_CELL_1_AND_7 = 1,        //!< Cell 1 and Cell 7
    LTC6804_CH_CELL_2_AND_8 = 2,        //!< Cell 2 and Cell 8
    LTC6804_CH_CELL_3_AND_9 = 3,        //!< Cell 3 and Cell 9
    LTC6804_CH_CELL_4_AND_10 = 4,       //!< Cell 4 and Cell 10
    LTC6804_CH_CELL_5_AND_11 = 5,       //!< Cell 5 and Cell 11
    LTC6804_CH_CELL_6_AND_12 = 6,       //!< Cell 6 and Cell 12
    LTC6804_CH_CELL_NUM
} LTC6804_CH_CELL_TYPE;
//! @}

//! @name GPIO Selections
//! @{
//! GPIO for ADC Conversion from datasheet table 35.
typedef enum {
    LTC6804_CHG_GPIO_ALL = 0,       //!< GPIO 1-5, 2nd Ref
    LTC6804_CHG_GPIO1 = 1,          //!< GPIO 1
    LTC6804_CHG_GPIO2 = 2,          //!< GPIO 2
    LTC6804_CHG_GPIO3 = 3,          //!< GPIO 3
    LTC6804_CHG_GPIO4 = 4,          //!< GPIO 4
    LTC6804_CHG_GPIO5 = 5,          //!< GPIO 5
    LTC6804_CHG_GPIO_2ND_REF = 6,   //!< 2nd Reference
    LTC6804_CHG_GPIO_NUM
} LTC6804_CHG_GPIO_TYPE;
//! @}

//! @name Status Group Selections
//! @{
//! Status Group Selections from datasheet table 35.
typedef enum {
    LTC6804_CHST_ALL = 0,           //!< SOC, ITMP, VA, VD
    LTC6804_CHST_SOC = 1,           //!< SOC
    LTC6804_CHST_ITMP = 2,          //!< ITMP
    LTC6804_CHST_VA = 3,            //!< VA
    LTC6804_CHST_VD = 4,            //!< VD
    LTC6804_CHST_NUM
} LTC6804_CHST_TYPE;
//! @}

//! @name Conversion Modes
//! @{
//! Conversion Modes Available in the LTC6804 from datasheet table 35.
typedef enum {
    LTC6804_CONVERSION_27KHZ_MODE = 0,  //!< 27kHz conversion mode
    LTC6804_CONVERSION_14KHZ_MODE = 1,  //!< 14kHz conversion mode
    LTC6804_CONVERSION_7KHZ_MODE = 2,   //!< 7kHz conversion mode
    LTC6804_CONVERSION_3KHZ_MODE = 3,   //!< 3kHz conversion mode
    LTC6804_CONVERSION_2KHZ_MODE = 4,   //!< 2kHz conversion mode
    LTC6804_CONVERSION_26HZ_MODE = 5,   //!< 26Hz conversion mode
    LTC6804_CONVERSION_NUM_MODES
} LTC6804_CONVERSION_MODE_T;
//! @}

//! @name Conversion Delays
//! @{
//! Reference wakeup time from datasheet Electrical Characteristics page 7.
//! Time for ADC results for one channel to be ready, from Table 6 (t1C).
//! Time for ADC results for all channels to be ready, from Table 5 (t6C).
#define LTC6804_TREFUP                       4400       //!< in us, max reference wakeup time.
#define LTC6804_CONVERSION_27KHZ_DELAY       201        //!< in us, delay between sampling and reading ADC in 27kHz conversion mode
#define LTC6804_CONVERSION_14KHZ_DELAY       230        //!< in us, delay between sampling and reading ADC in 14kHz conversion mode
#define LTC6804_CONVERSION_7KHZ_DELAY        405        //!< in us, delay between sampling and reading ADC in 7kHz conversion mode
#define LTC6804_CONVERSION_3KHZ_DELAY        501        //!< in us, delay between sampling and reading ADC in 3kHz conversion mode
#define LTC6804_CONVERSION_2KHZ_DELAY        754        //!< in us, delay between sampling and reading ADC in 2kHz conversion mode
#define LTC6804_CONVERSION_26HZ_DELAY        34         //!< in ms, delay between sampling and reading ADC in 26Hz conversion mode
#define LTC6804_CONVERSIONS_ALL_27KHZ_DELAY  11130      //!< in us, delay between sampling and reading ADC in 27kHz conversion mode
#define LTC6804_CONVERSIONS_ALL_14KHZ_DELAY  1288       //!< in us, delay between sampling and reading ADC in 14kHz conversion mode
#define LTC6804_CONVERSIONS_ALL_7KHZ_DELAY   2335       //!< in us, delay between sampling and reading ADC in 7kHz conversion mode
#define LTC6804_CONVERSIONS_ALL_3KHZ_DELAY   3033       //!< in us, delay between sampling and reading ADC in 3kHz conversion mode
#define LTC6804_CONVERSIONS_ALL_2KHZ_DELAY   4430       //!< in us, delay between sampling and reading ADC in 2kHz conversion mode
#define LTC6804_CONVERSIONS_ALL_26HZ_DELAY   202        //!< in ms, delay between sampling and reading ADC in 26Hz conversion mode
//! @}

//! @name Voltage Resolution
//! @{
//! Voltage resolutions from Table 46 (t6C)
#define LTC6804_VOLTAGE_RESOLUTION           100        //!< uV per bit (CxV), cell voltage resolution.
#define LTC6804_UVOV_RESOLUTION              (16*100)   //!< uV per bit (VUV and VOV), under-voltage and over-voltage resolution.
#define LTC6804_SOC_RESOLUTION               (20*100)   //!< uV per bit (SOC), sum-of-cells resolution.
#define LTC6804_V_SUPPLY_VOLTAGE             11000      //!< mV per bit, Minimum supply voltage from datasheet page 7.
//! @}

//! @name Communication Protocol Constants
//! @{
//! Communication Protocol Message Sizes, not including PEC, as specified by datasheet Table 26.
//! Baud Rate from from datasheet page 8
#define LTC6804_COMMAND_SIZE                 2              //!< bytes per command
#define LTC6804_REGISTER_GROUP_SIZE          6              //!< bytes per register group
#define LTC6804_PEC_SIZE                     sizeof(int16)  //!< 15 bit PEC, requires int16 data type
#define LTC6804_ADC_SIZE                     sizeof(int16)  //!< 16 bit ADC results
#define LTC6804_BAUD_RATE                    1000           //!< in kHz, Max input SPI Frequency (1/tCLK from datasheet)
#define LTC6804_COMMAND_TIME         (1LL * (LTC6804_COMMAND_SIZE + LTC6804_PEC_SIZE) * \
                                      BITS_PER_BYTE * US_PER_MS / LTC6804_BAUD_RATE)  //!< minimum time necessary to send a command in us
#define LTC6804_REGISTER_GROUP_TIME  (1LL * (LTC6804_COMMAND_SIZE + LTC6804_REGISTER_GROUP_SIZE + 2 * LTC6804_PEC_SIZE) * \
                                      BITS_PER_BYTE * US_PER_MS / LTC6804_BAUD_RATE)  //!< minimum time necessary to perform a register group read or write in us
//! @}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//! Initializes the LTC6804-2 code module.
//! @return void
void LTC6804_Init(void);

//! Gets the LTC6804 revision.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_Revision_Get(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                             unsigned int8* revision          //!< Pointer where to return revision.
                             );

//! Sets the LTC6804 GPIO Pull Downs.
//! Write: 0 -> GPIOx Pin Pull-Down ON; 1-> GPIOx Pin Pull-Down OFF
//! @return void
void LTC6804_GPIO_Set(int16 board_num,                   //!< The logical address for the PCB containing this LTC6804-2 IC.
                      int8 gpio_bitmap                  //!< bitmap for GPIO pull-downs, bit0 = GPIO 0
                      );

//! Gets the LTC6804 ADC Reference status, where 1 = ON and 0 = OFF.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_Refon_Get(int16 board_num,                     //!< The logical address for the PCB containing this LTC6804-2 IC.
                          unsigned int8* refon                         //!< location where to return the ADC Reference status.
                          );


//! Turns the LTC6804 ADC Reference on and off.
//! @return void
void LTC6804_Refon_Set(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                       BOOLEAN refon                    //!< TRUE to turn on the reference, FALSE to turn it off.
                       );

//! Sets the LTC6804 discharger pin levels and timeout values.
//! @return void
void LTC6804_Dischargers_Set(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                             int16 discharge_bitmap,          //!< bitmap for discharger, bit0 = cell 0.
                             int16 timeout_value              //!< timeout value for dischargers.
                             );

//! Sets the LTC6804 under-voltage and over-voltage thresholds in LTC6804_UVOV_RESOLUTION units.
//! @return void
void LTC6804_UVOV_Thresholds_Set(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                                 int16 vuv_value,                 //!< Under-voltage value.
                                 int16 vov_value                  //!< Over-voltage value.
                                 );

//! Gets the LTC6804 under-voltage and over-voltage thresholds in LTC6804_UVOV_RESOLUTION units.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_UVOV_Thresholds_Get(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                                 unsigned int16* vuv_value,       //!< Pointer where to return under-voltage value.
                                 unsigned int16* vov_value        //!< Pointer where to return over-voltage value.
                                 );

//! Gets the LTC6804 flags indicating under-voltage and over-voltage conditions are present.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_UVOV_Flags_Get(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                               int16* vuv_flags,                //!< Pointer where to return under-voltage flags in a bitmap with bit 0 = cell 0.
                               int16* vov_flags                 //!< Pointer where to return over-voltage flags in a bitmap with bit 0 = cell 0.
                               );

//! Clears the LTC6804 Cell Voltage ADC registers.  This is useful to detect if the conversion was started properly when the results are read.
//! @return void
void LTC6804_Cell_ADC_Clear(int16 board_num                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                            );

//! Starts the LTC6804 Cell Voltage ADC conversion at the specified conversion mode.
//! Note function always permits discharge.
//! @return void
void LTC6804_Cell_ADC_Start(int16 board_num,                     //!< The logical address for the PCB containing this LTC6804-2 IC.
                            LTC6804_CONVERSION_MODE_T mode,     //!< The mode to use for ADC conversion.
                            LTC6804_CH_CELL_TYPE cell_select,   //!< The cells to convert.
                            BOOLEAN discharge_permitted         //!< True if discharge is to be permitted during this cell voltage conversion.
                            );

//! Reads the LTC6804 Cell Voltage ADC conversion results.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_Cell_ADC_Read(int16 board_num,                       //!< The logical address for the PCB containing this LTC6804-2 IC.
                              LTC6804_CH_CELL_TYPE cell_select,     //!< The cells that were converted.
                              unsigned int16* adc_value_ptr         //!< Pointer where up to LTC6804_NUM_CELLV_ADC cell voltages will be returned.
                              );

//! Clears the LTC6804 GPIO ADC registers.  This is useful to detect if the conversion was started properly when the results are read.
//! @return void
void LTC6804_GPIO_ADC_Clear(int16 board_num                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                           );

//! Starts the specified LTC6804 GPIO ADC conversion at the specified conversion mode.
//! @return void
void LTC6804_GPIO_ADC_Start(int16 board_num,                         //!< The logical address for the PCB containing this LTC6804-2 IC.
                            LTC6804_CONVERSION_MODE_T mode,         //!< The mode to use for ADC conversion.
                            LTC6804_CHG_GPIO_TYPE gpio_select       //!< The gpio to convert from LTC6804_CHG GPIO Selections.
                            );

//! Reads the specified LTC6804 GPIO ADC conversion results.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_GPIO_ADC_Read(int16 board_num,                       //!< The logical address for the PCB containing this LTC6804-2 IC.
                              LTC6804_CHG_GPIO_TYPE gpio_select,    //!< The gpio that was converted.
                              int16* adc_value_ptr                  //!< Pointer where up to LTC6804_NUM_GPIO GPIO ADC results will be returned.
                              );

//! Writes a string of bytes to the LTC6804 I2C port implemented on its GPIO pins.
//! @return void
void LTC6804_I2C_Write(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                       BOOLEAN start,                   //!< TRUE if this write would be started by an I2C Start Condition.
                       BOOLEAN stop,                    //!< TRUE if this write would be ended with an I2C Stop Condition.
                       int8* data_ptr,                  //!< Pointer to the data to write to the I2C Bus.
                       int16 num_bytes,                 //!< The number of bytes to write to the I2C Bus.
                       int16 baud_khz                   //!< The baud rate at which the I2C Bus should be clocked.
                       );

//! Writes one byte, and then reads a string of bytes to the LTC6804 I2C port implemented on its GPIO pins.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_I2C_Read(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                      BOOLEAN start,                   //!< TRUE if this read would be started by an I2C Start Condition.
                      BOOLEAN stop,                    //!< TRUE if this read would be ended with an I2C Stop Condition.
                      int8* data_ptr,                  //!< Pointer to a byte to first write to the I2C Bus, and where to store the data read from the I2C Bus.
                      int16 num_bytes,                 //!< The number of bytes to read to the I2C Bus.
                      int16 baud_khz                   //!< The baud rate at which the I2C Bus should be clocked.
                      );

//! Writes a string of bytes to the LTC6804 SPI port implemented on its GPIO pins.
//! @return void
void LTC6804_SPI_Write(int16 board_num,                 //!< The logical address for the PCB containing this LTC6804-2 IC.
                      BOOLEAN start,                   //!< TRUE if the CS should be raised at the start of SPI communication.
                      BOOLEAN stop,                    //!< TRUE if the CS should be lowered at the end of SPI communication.
                      int8* data_ptr,         //!< Pointer to the data to write to the SPI Bus. 
                      int16 num_bytes,                 //!< The number of bytes to write to the SPI Bus.
                      int16 baud_khz                   //!< The baud rate at which the SPI Bus should be clocked.
                      );

//! Writes one byte, and then reads a string of bytes to the LTC6804 SPI port implemented on its GPIO pins.
//! @return TRUE if the LTC6804 communication was successful.
BOOLEAN LTC6804_SPI_Read(int16 board_num,                  //!< The logical address for the PCB containing this LTC6804-2 IC.
                      BOOLEAN start,                   //!< TRUE if the CS should be raised at the start of SPI communication.
                      BOOLEAN stop,                    //!< TRUE if the CS should be lowered at the end of SPI communication.
                      int8* data_ptr,                  //!< Pointer to a byte to first write to the SPI Bus, and where to store the data read from the SPI Bus. 
                      int16 num_bytes,                 //!< The number of bytes to write to the SPI Bus.
                      int16 baud_khz                   //!< The baud rate at which the SPI Bus should be clocked.
                      );

//! Calculates the LTC6804 CRC over a string of signed bytes as per datasheet figure 22.
//! @return the calculated CRC
unsigned int16 LTC6804_PEC_Calc(int8 *data,           //!< Pointer to the data to over which to calculate the CRC.
                                int length            //!< The number of bytes over which to calculate the CRC.
                                );

//! Calculates the LTC6804 CRC over a string of unsigned bytes as per datasheet figure 22.
//! @return the calculated CRC
unsigned int16 LTC6804_PEC_Calc(unsigned int8 *data,           //!< Pointer to the data to over which to calculate the CRC. // #Changed - Created a function overload and Changed "data" from char to unsigned int8 or unsigned char
                                int length            //!< The number of bytes over which to calculate the CRC.
                                );

#endif
