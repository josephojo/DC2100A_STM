/*
 Linear Technology DC2100A Demonstration Board.
 Driver File for LTC6804-2 Multicell Battery Monitors.
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
 $Revision: 699 $
 $Date: 2014-09-09 16:02:09 -0400 (Tue, 09 Sep 2014) $

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

//! @defgroup LTC6804-2 LTC6804-2 Multicell Battery Monitors.

/*! @file
    @ingroup LTC6804-2
    Driver File for LTC6804-2 Multicell Battery Monitors.
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "LTC6804-2.h"
#include "LTC6804-2_Config.h"
#include "LTC6804-2_Registers.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Timing parameters for state transitions defined by datasheet Figure 1.
// Electrical Characteristics from datasheet pages 7 and 8 contain worst case values.
#define LTC6804_TWAKE                                           300     // us, max value
#define LTC6804_TSLEEP                                          1800    // ms, min value
#define LTC6804_TREADY                                          10      // us, max value
#define LTC6804_TIDLE                                           4300    // us, min value
#define LTC6804_TDWELL                                          1       // us, min value of 240ns in datasheet is outside resolution of uP timer.

#define LTC6804_NUM_CELLV_ADC_PER_REGISTER_GROUP                (LTC6804_REGISTER_GROUP_SIZE / LTC6804_ADC_SIZE) // Number of cell voltage ADC measurements per register group
#define LTC6804_NUM_REGISTER_GROUP_READS_FOR_ALL_CELLV          (LTC6804_NUM_CELLV_ADC / LTC6804_NUM_CELLV_ADC_PER_REGISTER_GROUP) // Number of register groups that must be read to get all cell voltage ADC measurements
#define LTC6804_NUM_REGISTER_GROUP_READS_FOR_TWO_CELLV          (2) // Number of register groups that must be read to get a pair of cell voltage ADC measurements
#define LTC6804_NUM_AUX_ADC_PER_REGISTER_GROUP                  (LTC6804_REGISTER_GROUP_SIZE / LTC6804_ADC_SIZE)

#define LTC6804_NUM_COMM_BYTES_PER_REGISTER_GROUP               3       // Number of I2C/SPI bytes contained in the COMM register group

// Relationship between input SPI clocks and I2C/SPI output clocks, as per page 32 of datasheet.
#define LTC6804_SPI_CLOCK_CYCLES_PER_STCOMM_BYTE                24
#define LTC6804_SPI_BYTES_PER_STCOMM_BYTE                       (LTC6804_SPI_CLOCK_CYCLES_PER_STCOMM_BYTE/BITS_PER_BYTE)

// input clocks, when used to drive the I2C/SPI output clocks, as per table 19 of datasheet
#define LTC6804_BAUD_RATE_DIVISOR                               2       // tCLK/(SCL Clock Frequency) in table 19 of datasheet

DigitalOut csPin(D10, 1);


// ! Configures CS pin used to communicate with LTC6804-2.  Note that code module directly controls CS for wakeup signals.
#define LT6804_CONFIG_CS                    csPin // #Changed - Placed in here since it was causing a lot of include issues ( multiple definition of `csPin')

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Table used perform PEC calculation defined by datasheet figure 22.
const unsigned int16 ltc6804_pec_seed_value = 16;
const unsigned int16 ltc6804_pec_table[256] =
{   0x0000, 0xc599, 0xceab, 0x0b32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
    0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
    0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
    0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
    0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
    0x2544, 0x02be, 0xc727, 0xcc15, 0x098c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
    0x3d6e, 0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x07c2, 0xc25b, 0xc969, 0x0cf0, 0xdf0d,
    0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
    0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
    0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
    0x4a88, 0x8f11, 0x057c, 0xc0e5, 0xcbd7, 0xe4e,  0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
    0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
    0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
    0x85e9, 0x0f84, 0xca1d, 0xc12f, 0x04b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
    0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
    0x2ac0, 0x0d3a, 0xc8a3, 0xc391, 0x0608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
    0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
    0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
    0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
    0x2fbc, 0x0846, 0xcddf, 0xc6ed, 0x0374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0x0af8, 0xcf61, 0xc453,
    0x01ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
    0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
    0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

unsigned int8 ltc6804_gpio_pulldown;        // Since the GPIOx values in the CFGR register group read differently than what was written, the only way
                                            // to read/modify/write other values in this register are to have local storage for the written values.

int8 ltc6804_adcopt;                        // When starting an ADC conversion at a sample frequency, the adcopt must be correctly set.  Local storage allows driver
                                            // to quickly detect whether the adcopt bit needs to be changed in the CFG register without reading a register group unnecessarily.

unsigned int32 ltc6804_wakeup_timestamp;    // in LTC6804_CONFIG_TIMER_RESOLUTION, timestamp for the last time that the LTC6804 system received a wakeup signal.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void ltc6804_wakeup(void);
void ltc6804_get_adcopt_and_md(int8 conversion_mode, int8* adcopt_ptr, int8* md_ptr);
void ltc6804_adc_opt_set(int16 board_num, int8 adcopt);  // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
void ltc6804_command_code_send(int16 command_code, BOOLEAN reg_group_command);
void ltc6804_register_group_write(unsigned int8* register_group); // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
BOOLEAN ltc6804_register_group_read(unsigned int8* register_group); // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
void ltc6804_cfgr_modify(int16 board_num, int8* register_mask_ptr, int8* register_value_ptr); // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
void ltc6804_clock_out(int16 bytes_to_send, int16 baud_khz);
inline unsigned int16 ltc6804_pec_lookup(char data, unsigned int16 remainder);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes the LTC6804-2 code module.
void LTC6804_Init(void)
{
    // init I/O pulldowns to be all off
    ltc6804_gpio_pulldown = ((1 << LTC6804_NUM_GPIO) - 1);

    // init ADCOPT to nonsensical value so that it will always be set the first time.
    ltc6804_adcopt = 0xFF;

    // Initialize wakeup timestamp to ensure that SLEEP is assumed for first communication with LTC6804
    ltc6804_wakeup_timestamp = LTC6804_CONFIG_TIMER - LTC6804_TSLEEP;

    return;
}

// Read/Modify/Write the Config register such that the GPIOs are at the values specified in the gpio_value_bitmap.
void LTC6804_GPIO_Set(int16 board_num, int8 gpio_bitmap) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    struct {
        int8 mask[LTC6804_REGISTER_GROUP_SIZE];
        int8 value[LTC6804_REGISTER_GROUP_SIZE];
    } register_modify;

    ltc6804_gpio_pulldown = gpio_bitmap;    // Remember this written value, as all other read/modify/write operations on the
                                            // CFGR register will need to write this value instead of the read value.

    // Start with clear mask and value, and then set up gpio mask/value
    memset(&register_modify.mask[1], 0, sizeof(register_modify)-1);
    register_modify.mask[0] = LTC6804_CFGR0_GPIOx_MASK;
    register_modify.value[0] = LTC6804_CFGR0_GPIOx(ltc6804_gpio_pulldown);

    ltc6804_cfgr_modify(board_num, register_modify.mask, register_modify.value);

    return;
}

// Read/Modify/Write the Config to turn the Reference on and off.
void LTC6804_Refon_Set(int16 board_num, BOOLEAN refon) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    struct {
        int8 mask[LTC6804_REGISTER_GROUP_SIZE];
        int8 value[LTC6804_REGISTER_GROUP_SIZE];
    } register_modify;

    // Start with clear mask and value, and then set up gpio mask/value
    memset(&register_modify.mask[1], 0, sizeof(register_modify)-1);
    register_modify.mask[0] = LTC6804_CFGR0_GPIOx_MASK;
    register_modify.value[0] = LTC6804_CFGR0_GPIOx(ltc6804_gpio_pulldown);

    // Create Mask to set Ref On
    register_modify.mask[0] |= LTC6804_CFGR0_REFON_MASK;

    // Set Value for Ref
    register_modify.value[0] |= LTC6804_CFGR0_REFON(refon);

    ltc6804_cfgr_modify(board_num, register_modify.mask, register_modify.value);

    return;
}

// Read/Modify/Write the Config register such that the undervoltage and overvoltage values are those specified.
void LTC6804_UVOV_Thresholds_Set(int16 board_num, int16 vuv_value, int16 vov_value) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    struct {
        int8 mask[LTC6804_REGISTER_GROUP_SIZE];
        int8 value[LTC6804_REGISTER_GROUP_SIZE];
    } register_modify;

    // Start with clear mask and value, and then set up gpio mask/value
    memset(&register_modify.mask[1], 0, sizeof(register_modify)-1);
    register_modify.mask[0] = LTC6804_CFGR0_GPIOx_MASK;
    register_modify.value[0] = LTC6804_CFGR0_GPIOx(ltc6804_gpio_pulldown);

    // Create Mask to set VUV and VOV
    *((int16*)(register_modify.mask+1)) |= LTC6804_CFGR1_VUV_MASK;
    *((int16*)(register_modify.mask+2)) |= LTC6804_CFGR2_VOV_MASK;

    // Set Value for VUV and VOV
    *((int16*)(register_modify.value+1)) |= LTC6804_CFGR1_VUV(vuv_value);
    *((int16*)(register_modify.value+2)) |= LTC6804_CFGR2_VOV(vov_value);

    ltc6804_cfgr_modify(board_num, register_modify.mask, register_modify.value);

    return;
}

// Read/Modify/Write the Config register such that the discharger and discharge timeout values are those specified.
void LTC6804_Dischargers_Set(int16 board_num, int16 discharge_bitmap, int16 timeout_value)  // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    struct {
        int8 mask[LTC6804_REGISTER_GROUP_SIZE];
        int8 value[LTC6804_REGISTER_GROUP_SIZE];
    } register_modify;

    // Start with clear mask and value, and then set up gpio mask/value
    memset(&register_modify.mask[1], 0, sizeof(register_modify)-1);
    register_modify.mask[0] = LTC6804_CFGR0_GPIOx_MASK;
    register_modify.value[0] = LTC6804_CFGR0_GPIOx(ltc6804_gpio_pulldown);

    // Create Mask to set discharger and discharge timeout values
    *((int16*)(register_modify.mask+4)) |= LTC6804_CFGR4_DCCx_MASK;
    register_modify.mask[5] |= LTC6804_CFGR5_DCTO_MASK;

    // Set Value for discharger and discharge timeout values
    *((int16*)(register_modify.value+4)) |= LTC6804_CFGR4_DCCx(discharge_bitmap);
    register_modify.value[5] |= LTC6804_CFGR5_DCTO(timeout_value);

    ltc6804_cfgr_modify(board_num, register_modify.mask, register_modify.value);

    return;
}

// Gets the LTC6804 under-voltage and over-voltage thresholds in LTC6804_UVOV_RESOLUTION units.
BOOLEAN LTC6804_UVOV_Thresholds_Get(int16 board_num, unsigned int16* vuv_value, unsigned int16* vov_value)  // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 cfgr[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Configuration Register Group B + PEC

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    command_code = LTC6804_COMMAND_CODE_RDCFG(address);

    // Send the command code
    ltc6804_command_code_send(command_code, TRUE);

    // Read the Status Register
    if(ltc6804_register_group_read(cfgr) == TRUE)
    {
        *vuv_value = *((int16*)(cfgr+1)) & LTC6804_CFGR1_VUV_MASK;
        *vov_value = (*((int16*)(cfgr+2)) & LTC6804_CFGR2_VOV_MASK) >> 4;
    }
    else
    {
        LTC6804_CONFIG_ERROR_CRC(board_num, command_code, cfgr, sizeof(cfgr));
        success = FALSE;
    }

    return success;
}

// Gets the LTC6804 flags indicating under-voltage and over-voltage conditions are present.
BOOLEAN LTC6804_UVOV_Flags_Get(int16 board_num, int16* vuv_flags, int16* vov_flags)  // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 stbr[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Status Register Group B + PEC
    int16 byte_num;
    int8 bit_num;
    int8 bit_mask_in;
    int16 bit_mask_out;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    command_code = LTC6804_COMMAND_CODE_RDSTATB(address);

    // Send the command code
    ltc6804_command_code_send(command_code, TRUE);

    // Read the Status Register
    if(ltc6804_register_group_read(stbr) == TRUE)
    {
        // Initialize so that uvov flags will be separated into uv and ov by loop below
        bit_mask_out = 0x0001;
        *vuv_flags = 0;
        *vov_flags = 0;

        // Interpret data in register group according to datasheet Table 44
        for (byte_num = 2; byte_num < 5; byte_num++)
        {
            bit_mask_in = 0x01;

            for (bit_num = 0; bit_num < 8; bit_num += 2)
            {
                if(stbr[byte_num] & bit_mask_in) *vuv_flags |= bit_mask_out;
                bit_mask_in <<= 1;

                if(stbr[byte_num] & bit_mask_in) *vov_flags |= bit_mask_out;
                bit_mask_in <<= 1;

                bit_mask_out <<= 1;
            }
        }
    }
    else
    {
        LTC6804_CONFIG_ERROR_CRC(board_num, command_code, stbr, sizeof(stbr));
        success = FALSE;
    }

    return success;
}

// Gets the LTC6804 revision.
BOOLEAN LTC6804_Revision_Get(int16 board_num, unsigned int8* revision)  // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 stbr[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Status Register Group B + PEC

    // Get the board address from the board number
    //address = 16; 
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num); // #ComeBack - Can't deal with system_address_table right now

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    command_code = LTC6804_COMMAND_CODE_RDSTATB(address);

    // Send the command code
    ltc6804_command_code_send(command_code, TRUE);

    // Read the Status Register
    if(ltc6804_register_group_read(stbr) == TRUE)
    {
        *revision = stbr[5] >> 4;
    }
    else
    {
        LTC6804_CONFIG_ERROR_CRC(board_num, command_code, stbr, sizeof(stbr)); // #ComeBack - This catches the error
        success = FALSE;
    }

    return success;
}

// Gets the LTC6804 ADC Reference status, where 1 = ON and 0 = OFF.
BOOLEAN LTC6804_Refon_Get(int16 board_num, unsigned int8* refon) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 cfgr[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Configuration Register Group B + PEC

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    command_code = LTC6804_COMMAND_CODE_RDCFG(address);

    // Send the command code
    ltc6804_command_code_send(command_code, TRUE);

    // Read the Status Register
    if(ltc6804_register_group_read(cfgr) == TRUE)
    {
        if (cfgr[0] & LTC6804_CFGR0_REFON_MASK)
        {
            *refon = TRUE;
        }
        else
        {
            *refon = FALSE;
        }
    }
    else
    {
        LTC6804_CONFIG_ERROR_CRC(board_num, command_code, cfgr, sizeof(cfgr));
        success = FALSE;
    }

    return success;
}


// Clears the LTC6804 Cell Voltage ADC registers.  This is useful to detect if the conversion was started properly when the results are read.
void LTC6804_Cell_ADC_Clear(int16 board_num) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    int8 address;
    unsigned int16 command_code;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    // Build the command code
    command_code = LTC6804_COMMAND_CODE_CLRCELL(address);

    // Send the command code
    ltc6804_command_code_send(command_code, FALSE);

    return;
}

// Starts the LTC6804 Cell Voltage ADC conversion at the specified conversion mode.
void LTC6804_Cell_ADC_Start(int16 board_num, LTC6804_CONVERSION_MODE_T mode, LTC6804_CH_CELL_TYPE cell_select, BOOLEAN discharge_permitted) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    int8 address;
    unsigned int16 command_code;
    int8 adcopt;
    int8 md;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Get adcopt and md values to achieve the desired sample rate.
    ltc6804_get_adcopt_and_md(mode, &adcopt, &md);

    // Set adcopt in the cfg register, if necessary
    if( ltc6804_adcopt != adcopt)
    {
        ltc6804_adc_opt_set(board_num, adcopt);
    }
    else
    {
        // Wakeup 6804 in case it has entered SLEEP or IDLE.
        ltc6804_wakeup();
    }

    // Build the command code to start ADC conversion
    command_code = LTC6804_COMMAND_CODE_ADCV(address, md, (discharge_permitted ? LTC6804_DCP_DISCHARGE_PERMITTED : LTC6804_DCP_DISCHARGE_NOT_PERMITTED), cell_select);

    // Send the command code
    ltc6804_command_code_send(command_code, FALSE);

    return;
}

// Reads the LTC6804 Cell Voltage ADC conversion results.
BOOLEAN LTC6804_Cell_ADC_Read(int16 board_num, LTC6804_CH_CELL_TYPE cell_select, unsigned int16* adc_value_ptr) // #Changed - Changed board_num input from int8 to  int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 reg_count;
    unsigned int8 reg_index;
    unsigned int8 reg_inc;
    unsigned int8 byte_index;
    unsigned int8 cv_r[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Cell Voltage Register Groups + PEC
    int16 byte_num;

    // Determine how many cell ADC results were requested and where that data is located in register groups
    if(LTC6804_CH_ALL == cell_select)
    {
        reg_count = LTC6804_NUM_REGISTER_GROUP_READS_FOR_ALL_CELLV;
        reg_index = 0;
        reg_inc = (LTC6804_COMMAND_CODE_BASE_RDCVB - LTC6804_COMMAND_CODE_BASE_RDCVA);
        byte_index = 0;
    }
    else
    {
        reg_count = LTC6804_NUM_REGISTER_GROUP_READS_FOR_TWO_CELLV;
        reg_index = (cell_select - 1) / LTC6804_NUM_CELLV_ADC_PER_REGISTER_GROUP;
        reg_inc = (LTC6804_COMMAND_CODE_BASE_RDCVC - LTC6804_COMMAND_CODE_BASE_RDCVA);
        byte_index = ((cell_select - 1) % LTC6804_NUM_CELLV_ADC_PER_REGISTER_GROUP) * LTC6804_ADC_SIZE;
    }

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    command_code = LTC6804_COMMAND_CODE_RDCVA(address) + reg_index * (LTC6804_COMMAND_CODE_BASE_RDCVB - LTC6804_COMMAND_CODE_BASE_RDCVA);

    do
    {
        // Send the command code
        ltc6804_command_code_send(command_code, TRUE);

        // Read the ADC results
        if(ltc6804_register_group_read(cv_r) == TRUE)
        {
            // Interpret data in register group according to datasheet Table 37, Table 38, Table 39, and Table 40
            if(LTC6804_CH_ALL == cell_select)
            {
                for (byte_num = 0; byte_num < LTC6804_REGISTER_GROUP_SIZE; byte_num += LTC6804_ADC_SIZE)
                {
                    *adc_value_ptr++ = ((int16) cv_r[byte_num + 1] << 8) + cv_r[byte_num];
                }
            }
            else
            {
                *adc_value_ptr++ = ((int16) cv_r[byte_index + 1] << 8) + cv_r[byte_index];
            }

            // Increment command code to request next register group
            command_code += reg_inc;

            // Count the register groups that have been requested
            reg_count--;
        }
        else
        {
            LTC6804_CONFIG_ERROR_CRC(board_num, command_code, cv_r, sizeof(cv_r));
            reg_count = 0; // Bail out of command after first bad read.
            success = FALSE;
        }

    } while(reg_count);

    return success;
}

// Clears the LTC6804 GPIO ADC registers.  This is useful to detect if the conversion was started properly when the results are read.
void LTC6804_GPIO_ADC_Clear(int16 board_num)  // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    int8 address;
    unsigned int16 command_code;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    // Build the command code
    command_code = LTC6804_COMMAND_CODE_CLRAUX(address);

    // Send the command code
    ltc6804_command_code_send(command_code, FALSE);

    return;
}

// Starts the specified LTC6804 GPIO ADC conversion at the specified conversion mode.
void LTC6804_GPIO_ADC_Start(int16 board_num, LTC6804_CONVERSION_MODE_T mode, LTC6804_CHG_GPIO_TYPE gpio_select) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    int8 address;
    unsigned int16 command_code;
    int8 adcopt;
    int8 md;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Get adcopt and md values to achieve the desired sample rate.
    ltc6804_get_adcopt_and_md(mode, &adcopt, &md);

    // Set adcopt in the cfg register, if necessary
    if( ltc6804_adcopt != adcopt)
    {
        ltc6804_adc_opt_set(board_num, adcopt);
    }
    else
    {
        // Wakeup 6804 in case it has entered SLEEP or IDLE.
        ltc6804_wakeup();
    }

    // Build the command code
    command_code = LTC6804_COMMAND_CODE_ADAX(address, md, gpio_select);

    // Send the command code
    ltc6804_command_code_send(command_code, FALSE);

    return;
}

// Reads the specified LTC6804 GPIO ADC conversion results.
BOOLEAN LTC6804_GPIO_ADC_Read(int16 board_num, LTC6804_CHG_GPIO_TYPE gpio_select, int16* adc_value_ptr) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 gpio_index;
    unsigned int8 gpio_count;
    unsigned int8 avar[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Auxiliary Register Group + PEC
    int16 byte_num;

    // Determine how many gpio ADC results were requested and where that data is located in register group
    if(LTC6804_CHG_GPIO_ALL == gpio_select)
    {
        gpio_count = LTC6804_NUM_AUX_ADC;
        gpio_index = 0;
    }
    else
    {
        gpio_count = 1;
        gpio_index = gpio_select - LTC6804_CHG_GPIO1;
    }

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    do
    {
        // Determine if this result is in Auxiliary Register Group A or B
        if(gpio_index < LTC6804_NUM_AUX_ADC_PER_REGISTER_GROUP)
        {
            // Build the command code
            command_code = LTC6804_COMMAND_CODE_RDAUXA(address);
        }
        else
        {
            // Build the command code
            command_code = LTC6804_COMMAND_CODE_RDAUXB(address);
        }

        // Send the command code
        ltc6804_command_code_send(command_code, TRUE);

        // Read the ADC results
        if(ltc6804_register_group_read(avar) == TRUE)
        {
            // Interpret data in register group according to datasheet Table 41 and Table 42
            byte_num = (gpio_index % LTC6804_NUM_AUX_ADC_PER_REGISTER_GROUP) * LTC6804_ADC_SIZE;
            while((byte_num < LTC6804_REGISTER_GROUP_SIZE) && (gpio_count > 0))
            {
                *adc_value_ptr++ = ((int16) avar[byte_num + 1] << 8) + avar[byte_num];
                gpio_count--;
                byte_num += LTC6804_ADC_SIZE;
            }
        }
        else
        {
            LTC6804_CONFIG_ERROR_CRC(board_num, command_code, avar, sizeof(avar));
            gpio_count = 0;             // Bail out of command after first bad read.
            success = FALSE;
        }

    } while(gpio_count);

    return success;
}

// Writes a string of bytes to the LTC6804 I2C port implemented on its GPIO pins.
void LTC6804_I2C_Write(int16 board_num, BOOLEAN start, BOOLEAN stop, int8* data_ptr, int16 num_bytes, int16 baud_khz) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    int8 address;
    unsigned int16 command_code;
    unsigned int8 comm[LTC6804_REGISTER_GROUP_SIZE];
    unsigned int8* comm_ptr; // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
    int8 bytes_to_send;
    int16 byte_num;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    while (num_bytes)
    {
        // Send the WRCOMM command code
        command_code = LTC6804_COMMAND_CODE_WRCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Build COMM Register Group
        bytes_to_send = 0;
        comm_ptr = comm;

        // First I2C byte might be a start or a stop, and will always transmit something (else we wouldn't be in this loop)
        *comm_ptr++ = ((start ? LTC6804_ICOM_I2C_WRITE_START : LTC6804_ICOM_I2C_WRITE_BLANK) << 4) + UPPER_NIBBLE(*data_ptr);
        *comm_ptr++ = (LOWER_NIBBLE(*data_ptr) << 4) + ((num_bytes == 1) && stop ? LTC6804_FCOM_WRITE_I2C_NACK_STOP : LTC6804_FCOM_WRITE_I2C_NACK);

        start = FALSE;
        data_ptr++;
        num_bytes--;
        bytes_to_send++;

        // Second and Third I2C byte might be a stop or no transmit.  Note that 1st byte is special case, and handled above this loop
        for(byte_num = 1; byte_num < LTC6804_NUM_COMM_BYTES_PER_REGISTER_GROUP; byte_num++)
        {
            *comm_ptr++ = ((num_bytes ? LTC6804_ICOM_I2C_WRITE_BLANK : LTC6804_ICOM_I2C_WRITE_NO_TRANSMIT) << 4) + UPPER_NIBBLE(*data_ptr);
            *comm_ptr++ = (LOWER_NIBBLE(*data_ptr) << 4) + ((num_bytes == 1) && stop ? LTC6804_FCOM_WRITE_I2C_NACK_STOP : LTC6804_FCOM_WRITE_I2C_NACK);
            if(num_bytes)
            {
                data_ptr++;
                num_bytes--;
                bytes_to_send++;
            }
        }

        // Write COMM Register Group
        ltc6804_register_group_write(comm);

        // Send the STRCOMM Command
        command_code = LTC6804_COMMAND_CODE_STCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        //Send the number clocks to send the COMM Register Group data to the I2C
        ltc6804_clock_out(bytes_to_send, baud_khz);
    }
}

// Writes one byte, and then reads a string of bytes to the LTC6804 I2C port implemented on its GPIO pins.
BOOLEAN LTC6804_I2C_Read(int16 board_num, BOOLEAN start, BOOLEAN stop, int8* data_ptr, int16 num_bytes, int16 baud_khz) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 comm[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Comm Register Group + PEC
    unsigned int8* comm_ptr; // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
    int8 bytes_to_send;
    int16 byte_num;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    while (num_bytes)
    {
        // Send the WRCOMM command code
        command_code = LTC6804_COMMAND_CODE_WRCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Build COMM Register Group
        bytes_to_send = 0;
        comm_ptr = comm;

        // First I2C byte might be a start or a stop, and will always transmit something (else we wouldn't be in this loop)
        if(start)
        {
            *comm_ptr++ = (LTC6804_ICOM_I2C_WRITE_START << 4) + UPPER_NIBBLE(*data_ptr);
            *comm_ptr++ = (LOWER_NIBBLE(*data_ptr) << 4) + LTC6804_FCOM_WRITE_I2C_NACK;
        }
        else
        {
            *comm_ptr++ = ((LTC6804_ICOM_I2C_WRITE_BLANK) << 4) + UPPER_NIBBLE(LTC6804_COMM_READ_DUMMY);
            *comm_ptr++ = (LOWER_NIBBLE(LTC6804_COMM_READ_DUMMY) << 4) + ((num_bytes == 1) && stop ? LTC6804_FCOM_WRITE_I2C_NACK_STOP : LTC6804_FCOM_WRITE_I2C_ACK);
        }
        num_bytes--;
        bytes_to_send++;

        // Second and Third I2C byte might be a stop or no transmit.  Note that 1st byte is special case, and handled above this loop
        for(byte_num = 1; byte_num < LTC6804_NUM_COMM_BYTES_PER_REGISTER_GROUP; byte_num++)
        {
            *comm_ptr++ = ((num_bytes ? LTC6804_ICOM_I2C_WRITE_BLANK : LTC6804_ICOM_I2C_WRITE_NO_TRANSMIT) << 4) + UPPER_NIBBLE(LTC6804_COMM_READ_DUMMY);
            *comm_ptr++ = (LOWER_NIBBLE(LTC6804_COMM_READ_DUMMY) << 4) + ((num_bytes == 1) && stop ? LTC6804_FCOM_WRITE_I2C_NACK_STOP : LTC6804_FCOM_WRITE_I2C_ACK);
            if(num_bytes)
            {
                num_bytes--;
                bytes_to_send++;
            }
        }

        // Write COMM Register Group
        ltc6804_register_group_write(comm);

        // Send the STRCOMM Command
        command_code = LTC6804_COMMAND_CODE_STCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        //Send the number clocks to send the COMM Register Group data to the I2C
        ltc6804_clock_out(bytes_to_send, baud_khz);

        // Send the RDCOMM command code
        command_code = LTC6804_COMMAND_CODE_RDCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Read COMM register group and verify pec
        if(ltc6804_register_group_read(comm) == TRUE)
        {
            // Interpret data in register group according to datasheet Table 45
            comm_ptr = comm;

            while(bytes_to_send--)
            {
                *data_ptr = (((*comm_ptr) << 4) & 0xF0) + ((*(comm_ptr + 1)) >> 4);
                if(start)  // ignore the first byte, which is a readback of the i2c address sent after the start bit
                {
                    start = FALSE;
                }
                else
                {
                    data_ptr++;
                }
                comm_ptr += 2;
            }
        }
        else
        {
            LTC6804_CONFIG_ERROR_CRC(board_num, command_code, comm, sizeof(comm));
            num_bytes = 0;  // Bail out of command after first bad read.
            success = FALSE;
        }
    }

    return success;
}

// Writes a string of bytes to the LTC6804 SPI port implemented on its GPIO pins.
void LTC6804_SPI_Write(int16 board_num, BOOLEAN start, BOOLEAN stop, int8* data_ptr, int16 num_bytes, int16 baud_khz) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler 
{
    int8 address;
    unsigned int16 command_code;
    unsigned int8 comm[LTC6804_REGISTER_GROUP_SIZE];
    unsigned int8* comm_ptr; // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
    int8 bytes_to_send;
    int16 byte_num;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    while (num_bytes)
    {
        // Send the WRCOMM command code
        command_code = LTC6804_COMMAND_CODE_WRCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Build COMM Register Group
        bytes_to_send = 0;
        comm_ptr = comm;

        for(byte_num = 0; byte_num < LTC6804_NUM_COMM_BYTES_PER_REGISTER_GROUP; byte_num++)
        {
            *comm_ptr++ = ((num_bytes ? LTC6804_ICOM_SPI_WRITE_CSB_LOW : LTC6804_ICOM_SPI_WRITE_NO_TRANSMIT) << 4) + UPPER_NIBBLE(*data_ptr);
            *comm_ptr++ = (LOWER_NIBBLE(*data_ptr) << 4) + ((num_bytes <= 1) && stop ? LTC6804_FCOM_SPI_WRITE_CSB_HIGH : LTC6804_FCOM_SPI_WRITE_CSB_LOW);
            if(num_bytes)
            {
                data_ptr++;
                num_bytes--;
                bytes_to_send++;
            }
        }

        // Send COMM Register Group
        ltc6804_register_group_write(comm);

        // Send the STRCOMM Command
        command_code = LTC6804_COMMAND_CODE_STCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        //Send the number clocks to send the COMM Register Group data to the SPI
        ltc6804_clock_out(bytes_to_send, baud_khz);
    }

}

// Writes one byte, and then reads a string of bytes to the LTC6804 SPI port implemented on its GPIO pins.
BOOLEAN LTC6804_SPI_Read(int16 board_num, BOOLEAN start, BOOLEAN stop, int8* data_ptr, int16 num_bytes, int16 baud_khz) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler 
{
    BOOLEAN success = TRUE;
    int8 address;
    unsigned int16 command_code;
    unsigned int8 comm[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Comm Register Group + PEC
    unsigned int8* comm_ptr; // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
    int8 bytes_to_send;
    int16 byte_num;

    // Get the board address from the board number
    address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_num);

    // Wakeup 6804 in case it has entered SLEEP or IDLE.
    ltc6804_wakeup();

    while (num_bytes)
    {
        // Send the WRCOMM command code
        command_code = LTC6804_COMMAND_CODE_WRCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Build COMM Register Group
        bytes_to_send = 0;
        comm_ptr = comm;

        // First SPI byte might be a start or a stop, and will always be something (else we wouldn't be in this loop)
        if(start)
        {
            *comm_ptr++ = (LTC6804_ICOM_SPI_WRITE_CSB_LOW << 4) + UPPER_NIBBLE(*data_ptr);
            *comm_ptr++ = (LOWER_NIBBLE(*data_ptr) << 4) + LTC6804_FCOM_SPI_WRITE_CSB_LOW;
        }
        else
        {
            *comm_ptr++ = (LTC6804_ICOM_SPI_WRITE_CSB_LOW << 4) + UPPER_NIBBLE(LTC6804_COMM_READ_DUMMY);
            *comm_ptr++ = (LOWER_NIBBLE(LTC6804_COMM_READ_DUMMY) << 4) + ((num_bytes <= 1) && stop ? LTC6804_FCOM_SPI_WRITE_CSB_HIGH : LTC6804_FCOM_SPI_WRITE_CSB_LOW);
        }
        num_bytes--;
        bytes_to_send++;

        for(byte_num = 1; byte_num < LTC6804_NUM_COMM_BYTES_PER_REGISTER_GROUP; byte_num++)
        {
            *comm_ptr++ = (LTC6804_ICOM_SPI_WRITE_CSB_LOW << 4) + UPPER_NIBBLE(LTC6804_COMM_READ_DUMMY);
            *comm_ptr++ = (LOWER_NIBBLE(LTC6804_COMM_READ_DUMMY) << 4) + ((num_bytes <= 1) && stop ? LTC6804_FCOM_SPI_WRITE_CSB_HIGH : LTC6804_FCOM_SPI_WRITE_CSB_LOW);
            if(num_bytes)
            {
                num_bytes--;
                bytes_to_send++;
            }
        }

        // Write COMM Register Group
        ltc6804_register_group_write(comm);

        // Send the STRCOMM Command
        command_code = LTC6804_COMMAND_CODE_STCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        //Send the number clocks to send the COMM Register Group data to the I2C
        ltc6804_clock_out(bytes_to_send, baud_khz);

        // Send the RDCOMM command code
        command_code = LTC6804_COMMAND_CODE_RDCOMM(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Read COMM register group and verify pec
        if(ltc6804_register_group_read(comm) == TRUE)
        {
            // Interpret data in register group according to datasheet Table 45
            comm_ptr = comm;

            while(bytes_to_send--)
            {
                *data_ptr = (((*comm_ptr) << 4) & 0xF0) + ((*(comm_ptr + 1)) >> 4);
                if(start)  // ignore the first byte, which is a readback of the i2c address sent after the start bit
                {
                    start = FALSE;
                }
                else
                {
                    data_ptr++;
                }
                comm_ptr += 2;
            }
        }
        else
        {
            LTC6804_CONFIG_ERROR_CRC(board_num, command_code, comm, sizeof(comm));
            num_bytes = 0;  // Bail out of command after first bad read.
            success = FALSE;
        }
    }
    return success;
}

//! Calculates the LTC6804 CRC over a string of bytes as per datasheet figure 22.
unsigned int16 LTC6804_PEC_Calc(int8* data, int length) 
{
    unsigned int16 remainder;

    remainder = ltc6804_pec_seed_value;

    for (int i = 0; i < length; i++)
    {
        remainder = ltc6804_pec_lookup(data[i], remainder);
    }

    return (remainder * 2); //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2

}

//! Calculates the LTC6804 CRC over a string of bytes as per datasheet figure 22. // #Changed - Created a function overload and Changed "data" from char to unsigned int8 or unsigned char
unsigned int16 LTC6804_PEC_Calc(unsigned int8* data, int length) 
{
    unsigned int16 remainder;

    remainder = ltc6804_pec_seed_value;

    for (int i = 0; i < length; i++)
    {
        remainder = ltc6804_pec_lookup(data[i], remainder);
    }

    return (remainder * 2); //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


// Wakes up LTC6804 by lowering and raising CS as per datasheet Figure 21.
void ltc6804_wakeup(void)
{
    unsigned int32 wakeup_timestamp_new;

    // Check timestamp to determine if a short or long delay is required after wakeup signal.
    wakeup_timestamp_new = LTC6804_CONFIG_TIMER;

    LT6804_CONFIG_CS = 0; // Equivalent to LT6804_CONFIG_CS.write(0);
    LTC6804_CONFIG_DELAY_US(LTC6804_TDWELL);
    LT6804_CONFIG_CS = 1; // Equivalent to LT6804_CONFIG_CS.write(1);

    if((wakeup_timestamp_new - ltc6804_wakeup_timestamp) < LTC6804_TSLEEP)
    {
        // If wakeup signal sent less than LTC6804_TSLEEP time ago, then a short delay of LTC6804_TREADY is required.
        LTC6804_CONFIG_DELAY_US(LTC6804_TREADY);
    }
    else
    {
        // If wakeup signal sent more than LTC6804_TSLEEP time ago, then a long delay of LTC6804_TWAKE is required.
        LTC6804_CONFIG_DELAY_US(LTC6804_TWAKE);
    }

    ltc6804_wakeup_timestamp = wakeup_timestamp_new;

    return;
}

// Returns adcopt and md values to achieve the desired sample rate.
void ltc6804_get_adcopt_and_md(int8 conversion_mode, int8* adcopt_ptr, int8* md_ptr)
{
    // Select adcopt for this sample rate.
    switch(conversion_mode)
    {
        case LTC6804_CONVERSION_27KHZ_MODE:
            *adcopt_ptr = LTC6804_ADCOPT_0;
            *md_ptr = LTC6804_MD_MODE_FAST;
            break;
        case LTC6804_CONVERSION_14KHZ_MODE:
            *adcopt_ptr = LTC6804_ADCOPT_1;
            *md_ptr = LTC6804_MD_MODE_FAST;
            break;
        case LTC6804_CONVERSION_7KHZ_MODE:
            *adcopt_ptr = LTC6804_ADCOPT_0;
            *md_ptr = LTC6804_MD_MODE_NORMAL;
            break;
        case LTC6804_CONVERSION_3KHZ_MODE:
            *adcopt_ptr = LTC6804_ADCOPT_1;
            *md_ptr = LTC6804_MD_MODE_NORMAL;
            break;
        case LTC6804_CONVERSION_2KHZ_MODE:
            *adcopt_ptr = LTC6804_ADCOPT_1;
            *md_ptr = LTC6804_MD_MODE_FILTERED;
            break;
        case LTC6804_CONVERSION_26HZ_MODE:
        default:
            *adcopt_ptr = LTC6804_ADCOPT_0;
            *md_ptr = LTC6804_MD_MODE_FILTERED;
             break;
    }

    return;
}

// Read/Modify/Write the Cfg Register to set the ADC option bit.
void ltc6804_adc_opt_set(int16 board_num, int8 adcopt) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    struct
    {
        int8 mask[LTC6804_REGISTER_GROUP_SIZE];
        int8 value[LTC6804_REGISTER_GROUP_SIZE];
    } register_modify;

    // Start with clear mask and value, and then set up gpio mask/value
    memset(&register_modify.mask[1], 0, sizeof(register_modify) - 1);
    register_modify.mask[0] = LTC6804_CFGR0_GPIOx_MASK;
    register_modify.value[0] = LTC6804_CFGR0_GPIOx(ltc6804_gpio_pulldown);

    // Create Mask to set ADC Option
    register_modify.mask[0] |= LTC6804_CFGR0_ADCOPT_MASK;

    // Set Value for ADC Option
    register_modify.value[0] |= LTC6804_CFGR0_ADCOPT(adcopt);

    ltc6804_cfgr_modify(board_num, register_modify.mask, register_modify.value);

    // Remember last value set for ADC Option
    ltc6804_adcopt = adcopt;

    return;
}

// Writes a command to the LTC6804
void ltc6804_command_code_send(int16 command_code, BOOLEAN reg_group_command)
{
    unsigned int8 writebyte[LTC6804_COMMAND_SIZE + LTC6804_PEC_SIZE];
    unsigned int16 pec;

    // Pull CS low to start write
    LT6804_CONFIG_CS = 0; // Equivalent to LT6804_CONFIG_CS.write(0);

    // Build the Command Code and Send
    writebyte[0] = UPPER_BYTE(command_code);
    writebyte[1] = LOWER_BYTE(command_code);
    LTC6804_CONFIG_SPI_BUFFER_SEND_START(writebyte, LTC6804_COMMAND_SIZE);

    // Calculate PEC and Send
    pec = LTC6804_PEC_Calc(writebyte, LTC6804_COMMAND_SIZE);
    writebyte[LTC6804_COMMAND_SIZE + 0] = UPPER_BYTE(pec);
    writebyte[LTC6804_COMMAND_SIZE + 1] = LOWER_BYTE(pec);
    LTC6804_CONFIG_SPI_BUFFER_SEND_START(&writebyte[LTC6804_COMMAND_SIZE], LTC6804_PEC_SIZE);

    // If no register group to follow, release CS to end write
    if (reg_group_command == FALSE)
    {
        // while(LTC6804_CONFIG_SPI_BUFFER_DONE() == FALSE);  // Wait for SPI transmission to be done before releasing CS // #NeededNow??? - Commenting this out in the meantime

        LT6804_CONFIG_CS = 1; // Equivalent to LT6804_CONFIG_CS.write(1);   // End the communication
    }

    return;
}

// Writes a register group to the LTC6804
void ltc6804_register_group_write(unsigned int8* register_group) // #Changed - Changed input from int8* to unsigned int8* due to conflict with function caller
{
    unsigned int8 writebyte[LTC6804_PEC_SIZE];
    unsigned int16 pec;

    // Send register group bytes
    LTC6804_CONFIG_SPI_BUFFER_SEND_START(register_group, LTC6804_REGISTER_GROUP_SIZE);

    // Calculate PEC and Send
    pec = LTC6804_PEC_Calc(register_group, LTC6804_REGISTER_GROUP_SIZE);
    writebyte[0] = UPPER_BYTE(pec);
    writebyte[1] = LOWER_BYTE(pec);
    LTC6804_CONFIG_SPI_BUFFER_SEND_START(writebyte, LTC6804_PEC_SIZE);

    // while(LTC6804_CONFIG_SPI_BUFFER_DONE() == FALSE);  // Wait for SPI transmission to be done before releasing CS // #NeededNow??? - Commenting this out in the meantime

    LT6804_CONFIG_CS = 1; // Equivalent to LT6804_CONFIG_CS.write(1);   // End the communication

    return;
}

// Reads a register group from the LTC6804 and verifies the PEC
BOOLEAN ltc6804_register_group_read(unsigned int8* register_group) // #Changed - Changef input from int8* to unsigned int8* due to conflict with function caller
{
    int16 byte_num; // #Temp - Changed to int16 from int8
    unsigned int16 pec_calc;

    // Start to receive the register group bytes from the SPI.
    LTC6804_CONFIG_SPI_BUFFER_RECEIVE_START(register_group, (LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE));

    // Initialize PEC calculation
    pec_calc =  ltc6804_pec_seed_value;

    // Read register group bytes one at a time, so that the PEC can be calculated as they're being received.
    for (byte_num = 0; byte_num < LTC6804_REGISTER_GROUP_SIZE; byte_num++)
    {
        // while(LTC6804_CONFIG_SPI_BUFFER_RECEIVE_BYTES_AVAILABLE(&register_group[byte_num]) == 0);  // Wait for a byte to be received // #NeededNow??? - Commenting this out in the meantime
        pec_calc =  ltc6804_pec_lookup(register_group[byte_num], pec_calc);         // Calculate PEC for this byte  #Temp
    }

    // Complete PEC calculation
    pec_calc <<=  1;      //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2

    // // Wait for PEC bytes to be received
    // while(LTC6804_CONFIG_SPI_BUFFER_RECEIVE_BYTES_AVAILABLE(&register_group[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE - 1]) == 0); // #NeededNow??? - Commenting this out in the meantime

    // End the communication
    LT6804_CONFIG_CS = 1;

    // Verify PEC and return result
    if((UPPER_BYTE(pec_calc) != register_group[LTC6804_REGISTER_GROUP_SIZE]) ||
       (LOWER_BYTE(pec_calc) != register_group[LTC6804_REGISTER_GROUP_SIZE + 1]))
    {
        return FALSE;
    }
    return TRUE;
}


// Clocks a number of bytes out of the i2c/spi at a set output baud rate, and then returns the baud rate to that for the LTC6804
void ltc6804_clock_out(int16 bytes_to_send, int16 baud_khz)
{
    unsigned int8 dummy;
    int16 byte_num;

    // while(LTC6804_CONFIG_SPI_BUFFER_DONE() == FALSE);  // Wait for SPI transmission to be done before changing the baud rate // #NeededNow??? - Commenting this out in the meantime

    // Set baud rate to input value for LTC6804, that results in the desired output baud rate.
    LTC6804_CONFIG_SPI_SET_BAUD( MIN(baud_khz * LTC6804_BAUD_RATE_DIVISOR, LTC6804_BAUD_RATE) );

    dummy = 0xFF;   // Keep transmit lines high as you provide clock for STCOMM command.

    //Send the number of clocks needed to send bytes (3 * 8 * number of bytes) in COMM register group
    for (byte_num = 0; byte_num < LTC6804_SPI_BYTES_PER_STCOMM_BYTE * bytes_to_send; byte_num++)
    {
        LTC6804_CONFIG_SPI_BUFFER_SEND_START(&dummy, sizeof(dummy));
    }

    // while(LTC6804_CONFIG_SPI_BUFFER_DONE() == FALSE);  // Wait for SPI transmission to be done before releasing CS // #NeededNow??? - Commenting this out in the meantime

    // End the communication
    LT6804_CONFIG_CS = 1;

    // Return baud rate to max allowed by the LTC6804
    LTC6804_CONFIG_SPI_SET_BAUD(LTC6804_BAUD_RATE);

    return;
}

// Reads, Modifies, and Writes bits in the Configuration Register Group.
// If a broadcast is desired, then this code actually implemented as some of the bits in the Configuration Register Group are unique to each board.
void ltc6804_cfgr_modify(int16 board_num, int8* register_mask_ptr, int8* register_value_ptr) // #Changed - Changed board_num input from int8 to int16 due to warning from compiler
{
    int8 address;
    unsigned int16 command_code;
    unsigned int8 cfgr[LTC6804_REGISTER_GROUP_SIZE + LTC6804_PEC_SIZE];    // storage for the Configuration Register Group + PEC
    int16 byte_num;
    int16 board_loop_num;
    int8 board_limit_num;

    // Since broadcast reads can't be performed in the 6804-2, this must be handled with a loop.
    if(board_num == LTC6804_BROADCAST)
    {
        board_loop_num = 0;
        board_limit_num = LTC6804_CONFIG_NUM_BOARDS;
    }
    else
    {
        board_loop_num = board_num;
        board_limit_num = board_num;
    }

    do
    {
        // Get the board address from the board number
        address = LTC6804_CONFIG_GET_BOARD_ADDRESS(board_loop_num);

        // Wakeup 6804 in case it has entered SLEEP or IDLE.
        ltc6804_wakeup();

        // Send the RDCFG command code.
        command_code = LTC6804_COMMAND_CODE_RDCFG(address);
        ltc6804_command_code_send(command_code, TRUE);

        // Read the Configuration Register Group
        // If PEC is correct for read data, modify masked bits and write back
        if(ltc6804_register_group_read(cfgr) == TRUE)
        {

            // Modify the Configuration Register Group, with only the masked bits modified
            for (byte_num = 0; byte_num < LTC6804_REGISTER_GROUP_SIZE; byte_num++)
            {
                cfgr[byte_num] &= ~(*(register_mask_ptr + byte_num));
                cfgr[byte_num] |= (*(register_value_ptr + byte_num));
            }

            // Send the WRCFG command code
            command_code = LTC6804_COMMAND_CODE_WRCFG(address);
            ltc6804_command_code_send(command_code, TRUE);

            // Send Configuration Register Group Write
            ltc6804_register_group_write(cfgr);
        }
        else
        {
            LTC6804_CONFIG_ERROR_CRC(board_loop_num, command_code, cfgr, sizeof(cfgr));
            board_loop_num = board_limit_num = 0;  // Bail out of command after first bad read.
        }

        board_loop_num++;
    } while (board_loop_num < board_limit_num);

    return;
}

// calculates the pec for one byte, and returns the intermediate calculation
inline unsigned int16 ltc6804_pec_lookup(char data, unsigned int16 remainder)
{
    unsigned int8 addr;

    addr = ((remainder >> 7) ^ data) & 0xff;    //calculate PEC table address
    remainder = (remainder << 8) ^ ltc6804_pec_table[addr];             //get value from CRC15Table;

    return remainder;
}
