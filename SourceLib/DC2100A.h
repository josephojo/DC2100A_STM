#ifndef __DC2100A_H__
#define __DC2100A_H__

/* Temporary DC2100A.h dependency definitions */

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Definitions specific to the DC2100A board
#define APP_FW_REV_MAJOR            0x01
#define APP_FW_REV_MINOR            0x02
#define APP_FW_REV_SUB_MINOR        0x04
#define APP_FW_REV_BETA             0x00
#define APP_FW_STRING_FORMAT        "xx.yy.zzbaa"     // xx = major revision, yy = minor revision, zz = sub-revision, aa = beta number, . = '.', b = 'b'
#define APP_FW_STRING_SIZE          (sizeof(APP_FW_STRING_FORMAT) - 1)
#define APP_FW_STRING_DEFAULT       "N/A        "

// IDSTRING that the board sends back to the host
#define DC2100A_IDSTRING            "DC2100A-A,LTC3300-1 demonstration board"
#define DC2100A_IDSTRING_SIZE       (sizeof(DC2100A_IDSTRING) - 1)
#define DC2100A_COMPANY_STRING      "Linear Technology Inc."
#define DC2100A_COMPANY_STRING_SIZE (sizeof(DC2100A_COMPANY_STRING) - 1)

#define DC2100A_MODEL_NUM_DEFAULT   "DC2100A-?"
#define DC2100A_MODEL_NUM_SIZE      (sizeof(DC2100A_MODEL_NUM_DEFAULT) - 1)  // The size of the model number string (non-unicode), no null terminator
#define DC2100A_CAP_DEMO_DEFAULT    '?'
#define DC2100A_CAP_DEMO_SIZE       1                                        // The size of the cap demo character
#define DC2100A_SERIAL_NUM_DEFAULT  "None             "
#define DC2100A_SERIAL_NUM_SIZE     (sizeof(DC2100A_SERIAL_NUM_DEFAULT) - 1) // The size of the serial number string (non-unicode), no null terminator

// DC2100A HW definition
#define DC2100A_OSCILLATOR_FREQUENCY    48000000                             // Oscillator frequency for DC2100A board
#define DC2100A_INSTRUCTION_CLOCK       (DC2100A_OSCILLATOR_FREQUENCY / 4)   // Instruction frequency derived from oscillator
#define DC2100A_MAX_BOARDS              8           // The maximum number of DC2100A boards that can be stacked together into one system.
                                                    // Note: the number of boards that can be stacked, is limited by the voltage rating on transformer T15.
#define DC2100A_NUM_CELLS               12          // The number of cells on one DC2100A board
#define DC2100A_NUM_MUXES               2           // The number of LTC1380 Muxes on a DC2100A board
#define DC2100A_NUM_TEMPS               12          // The number of thermistor inputs on one DC2100A board
#define DC2100A_NUM_LTC3300             2           // The number of LTC3300 Balancers on a DC2100A board
#define DC2100A_NUCLEO_BOARD_NUM        0           // It makes a lot of things simpler if the board with the PIC always has the same address.

extern bool useUSBTerminator;


/* ******************************************* */

#endif // __DC2100A_H__
