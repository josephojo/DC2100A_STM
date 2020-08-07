

//#define __RUNMAIN__
#define __RUNTEST__

#ifdef __RUNTEST__

/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */


#include "mbed.h"
#include "platform/mbed_thread.h"
#include "SourceLib/NUCLEO_SPI.h"
#include "SourceLib/LTC6804-2_Config.h"
#include "SourceLib/LTC6804-2.h"
#include "SourceLib/Voltage.h"
#include "SourceLib/Temperature.h"
#include "SourceLib/Balancer.h"


int main()
{
    //unsigned int8 rxCmd[4]; // Revision Group Cmd

    //unsigned int8 rxData[64];
    int16 board_num = 0;

    NUCLEO_SPI_Init(1000, MODE3);
    NUCLEO_Timer_Init();
    LTC6804_Init();

    Eeprom_Init();
    System_Init();

    // Run these twice to move the system state from init to awake and set the OV and UV thresholds
    System_Status_Task();
    System_Status_Task();

    int cells = 12;
    int cell_num = 0;

    //Eeprom_Cap_Save_Defaults(board_num, 0);
    printf("CAP Calibration factors: WithOUT Mfg Key\n");
    for (cell_num = 0; cell_num < cells; cell_num++)
    {
        printf("Calibration Cap[%d] = %d\n", cell_num + 1, Eeprom_Cap_Values[board_num].cap[cell_num]);
    }

    //Eeprom_Current_Save_Defaults(board_num, 0); 
    printf("Current Calibration factors: WithOUT Mfg Key\n");
    for (cell_num = 0; cell_num < cells; cell_num++)
    {
        printf("Charge Calibration Current[%d] = %d\n", cell_num + 1, Eeprom_Current_Values[board_num].current[cell_num].charge);
        printf("Discharge Calibration Current[%d] = %d\n\n", cell_num + 1, Eeprom_Current_Values[board_num].current[cell_num].discharge);
    }

   /* Eeprom_Current_Load(board_num, EEPROM_MFG_KEY);
    printf("Current Calibration factors: With Mfg Key\n");
    for (cell_num = 0; cell_num < cells; cell_num++)
    {
        printf("Charge Calibration Current[%d] = %d\n", cell_num + 1, Eeprom_Current_Values[board_num].current[cell_num].charge);
        printf("Discharge Calibration Current[%d] = %d\n\n", cell_num + 1, Eeprom_Current_Values[board_num].current[cell_num].discharge);
    }*/



#define BALANCER_ALGORITHM_NUM_BOARDS       1       // Note - the Balancer_Set(BALANCER_DELTA_Q_TYPE* charge_target_ptr) function is limited to this many boards.
    signed int32 signed_temp;
    struct
    {
        int16 charge_current;           // in mA
        int16 discharge_current;        // in mA
        signed int32 primary_charge;    // in mAs
        signed int32 total_charge;      // in mAs
    } cell[BALANCER_ALGORITHM_NUM_BOARDS][DC2100A_NUM_CELLS];

    // Initialize variables before iteration.
    // Balance currents are calibrated values from EEPROM..
    // Initial guess at primary charge is the value passed into the function. // # Copied from Balancer_Set(Q)
    {
        int16 base_charge_current;
        int16 base_discharge_current;

        // Pick the base current, depending upon the model
        if ((System_Model[board_num] == 'A') || (System_Model[board_num] == 'B'))
        {
            base_charge_current = BALANCER_AB_CURRENT_CHARGE_6CELL;
            base_discharge_current = BALANCER_AB_CURRENT_DISCHARGE_6CELL;
        }
        else // if((System_Model[board_num] == 'C') || (System_Model[board_num] == 'D'))
        {
            base_charge_current = BALANCER_CD_CURRENT_CHARGE_6CELL;
            base_discharge_current = BALANCER_CD_CURRENT_DISCHARGE_6CELL;
        }

        for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num = cell_num+8)    //fill w/ voltages and capacitances for test case
        {
            // Store and scale the charge current
            signed_temp = base_charge_current; 
            printf("BASE Charge Current[%d] = %d\n", cell_num + 1, base_charge_current);
            
            signed_temp *= Eeprom_Current_Values[board_num].current[cell_num].charge;
            printf("BASE x Calibration Charge Current[%d] = %d\n", cell_num + 1, signed_temp);

            signed_temp = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_CURRENT_SCALE_FACTOR_SHIFT);
            printf("SIGNED_RIGHT_SHIFT_WITH_ROUND Charge Current[%d] = %d\n", cell_num + 1, signed_temp);

            cell[board_num][cell_num].charge_current = base_charge_current + signed_temp;
            printf("Cell Charge Current[%d] = %d\n\n", cell_num + 1, cell[board_num][cell_num].charge_current);


            // Store and scale the discharge current
            signed_temp = base_discharge_current;
            signed_temp *= Eeprom_Current_Values[board_num].current[cell_num].discharge;
            signed_temp = SIGNED_RIGHT_SHIFT_WITH_ROUND(signed_temp, BALANCER_CURRENT_SCALE_FACTOR_SHIFT);
            cell[board_num][cell_num].discharge_current = base_discharge_current + signed_temp;

            //printf("Cell Charge Current[%d] = %d\n", cell_num + 1, cell[board_num][cell_num].charge_current);
            //printf("Cell Discharge Current[%d] = %d\n\n", cell_num + 1, cell[board_num][cell_num].discharge_current);

            //// Scale to time resolution used by balancer algorithm.
            //charge_target_ptr[cell_num] <<= BALANCER_TIME_RESOLUTION_SHIFT;

            //// Start with the primary charge moved equal to the total charge requested to be moved.
            //cell[board_num][cell_num].primary_charge = charge_target_ptr[cell_num];
        }
    }

    return 0;
}




#else

#include "mbed.h"
#include "mbed_events.h"
#include "BufferedSerial.h"
#include "platform/mbed_thread.h"
#include "SourceLib/NUCLEO_SPI.h"
#include "SourceLib/LTC6804-2_Config.h"
#include "SourceLib/LTC6804-2.h"
#include "SourceLib/LTC3300-1.h"
#include "SourceLib/Voltage.h"
#include "SourceLib/Temperature.h"
#include "SourceLib/USB_Parser.h"
#include "SourceLib/Balancer.h"
 // #include "SourceLib/System.h"

// Include Semaphore for preserving variables as they are shared with the USB module i.e. voltage and temperature
// Semaphore 

InterruptIn EMERGENCY_STOP_BTN(BUTTON1);

// creates a queue with the default size
EventQueue mainQueue;
EventQueue USBQueue;
EventQueue CalcQueue;
Thread USBThread;
Thread CalcThread;

bool useUSBTerminator = true; // Ensures that the Outgoing data is terminated with a newline (\n)


Mutex incomingData_Mutex;
ConditionVariable incomingData_CV(incomingData_Mutex);

// Serial and LED objects are initialized in "USB_Parser.cpp"

// For Debugging timing
Timer testTimer;
long voltTime = 0, statTime = 0, detTime = 0, errTime = 0, printTime = 0, balTime = 0;
int voltCount = 0, statCount=0, detCount=0, errCount=0, printCount=0, balCount = 0;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define RUN_TIME -1  // In ms
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

 // These bits allow tasks to be skipped while data is being shipped out USB.
 // This allows all of the data samples sent out USB to be from the same timestamp, at the expense of the regular sample period. 0 = False, 1 = True
struct {
    int voltage : 1;
    int temperature : 1;
} dc2100a_task_skip;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Define the tasks for the CCS RTOS
// todo - Fill in more realistic estimates for the worst case execution time
void Task_Balancer(void);                       // Controls the balancers in the DC2100A system
void Task_Voltage(void);                        // Monitors the voltages in the DC2100A system
void Task_Pack_Current(void);                   // Monitors the charger, discharger, and pack current in the DC2100A system
void Task_Temperature(void);                    // Monitors the temperatures in the DC2100A system
void Task_Get_USB(void);                        // Receives bytes from the USB bulk endpoint.  Must be its own task to receive bytes while Task_Parser is pending.
void Task_Parser(void);                         // Parses bytes received from the USB bulk endpoints for commands, and sends the responses to the commands
void Task_Status(void);                         // Indicates the status of the DC2100A system
void Task_Detect(void);                         // Detect new boards in the DC2100A system
void Task_Error(void);                          // Detect errors in the DC2100A system

void Task_TestPrint(void);

void StopBalancer_handler(void);

void threadedDispatch()
{
    USBQueue.dispatch(RUN_TIME);
}

int main()
{
    testTimer.start();

    // Might want to start a watchdog timer HERE to reset the mcu if it does not initialize properly.

     /* ################### Module Initilizations ################## */

    NUCLEO_SPI_Init(LTC6804_BAUD_RATE, MODE3); // Init SPI for the LTC6820 clock polarity/phase, and the 6804 baud rate.
    NUCLEO_Timer_Init(); 

    // Init the Error handler first, so that HW issues upon startup can be recorded
    Error_Init();

    // Initialize the Hardware Modules
    LTC6804_Init();                                 // Init after SPI, as LTC6804 commincation is through SPI
    LTC3300_Init();                                 // Init after LTC6804, as LTC3300 communication passes through the LTC6804

    // Initialize the Higher Level Functions Necessary to Identify System Via USB
    Eeprom_Init();                                  // Init after LTC6804, as EEPROM communication passes through the LTC6804
    System_Init();                                  // Init after EEPROM, as system config info is contained in the EEPROM
    //USB_Init();                                     // Init after EEPROM and System, as USB enumeration strings depend upon system config info contained in the EEPROM

    // Initialize the Other Higher Level Functions
    Voltage_Init();                                 // Init after LTC6804
    Temperature_Init();                             // Init after LTC6804
    Balancer_Init();                                // Init after LTC6804 and LTC3300
    //Pack_Current_Init();                            // Init after EEPROM and System
    USB_Parser_Init();                              // Initialize Last

    //serial.set_dma_usage_tx(DMA_USAGE_ALLOCATED);

     /* ################### Interrupts ################## */
    // The 'fall' handler will not execute in IRQ context but of the shared queue (actually the main thread)
    EventQueue* queue = mbed_event_queue();
    EMERGENCY_STOP_BTN.fall(queue->event(StopBalancer_handler));


    /* ################### RTOS Section ################## */

    // events are simple callbacks, call every specified amount of milliseconds
    mainQueue.call_every(BALANCER_TASK_RATE,        Task_Balancer);
    mainQueue.call_every(VOLTAGE_TASK_RATE,         Task_Voltage);
    //mainQueue.call_every(PACK_CURRENT_TASK_RATE,    Task_Pack_Current);
    mainQueue.call_every(TEMPERATURE_TASK_RATE,     Task_Temperature);
    mainQueue.call_every(USB_TASK_RATE,             Task_Parser);
    mainQueue.call_every(STATUS_TASK_RATE,          Task_Status);
    mainQueue.call_every(DETECT_TASK_RATE,          Task_Detect);
    mainQueue.call_every(ERROR_TASK_RATE,           Task_Error);

    USBQueue.call_every(USB_TASK_RATE, Task_Get_USB);

    // Allow all tasks to run initially
    dc2100a_task_skip.temperature = 0;
    dc2100a_task_skip.voltage = 0;

    //serial.printf("\n\nBefore Dispatch\r\n");

    // the dispatch method executes events in their respective contexts
    USBThread.start(threadedDispatch); //This must be run first before the next line so that USBThread is not stuck unrun while the main thread runs
    mainQueue.dispatch(RUN_TIME);


    USBThread.join();

    // For Debugging timing
    /*Task_TestPrint();
    serial.printf("Printing (%d times) takes: %lu us\r\n",printCount, printTime/ printCount);
    serial.printf("Voltage (%d times) takes: %lu us\r\n",voltCount, voltTime/ voltCount);
    serial.printf("Error(%d times) takes: %lu us\r\n",errCount, errTime/ errCount);
    serial.printf("Status (%d times) takes: %lu us\r\n", statCount, statTime/ statCount);
    serial.printf("Detect (%d times)takes: %lu us\r\n",detCount, detTime/ detCount);
    serial.printf("Balance (%d times)takes: %lu us\r\n",balCount, balTime/ balCount);
    serial.printf("--------------------------------------------------------------\r\n\n");*/

    testTimer.stop();

}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Detects the user input through the User button on the Nucleo board for a shutdown of the Balancer
void StopBalancer_handler()
{
    LTC3300_Suspend(DC2100A_NUCLEO_BOARD_NUM);
    serial.printf("%c\n", USB_PARSER_EMERGENCY_STOP_COMMAND);
    mainQueue.break_dispatch();
    USBQueue.break_dispatch();
}


void Task_TestPrint(void)
{
    long tim1 = testTimer.read_us();

    SYSTEM_STATE_TYPE System_State = returnSysState();
    serial.printf("### Iter %d  ###\r\nSystem_Model = %c\r\n", printCount+1, System_Model[0]);
    switch (System_State)
    {
    case SYSTEM_STATE_OFF:
        serial.printf("System is in OFF State\r\n");
        break;
    case SYSTEM_STATE_NUCLEO_BOARD_INIT:
        serial.printf("System MCU is still in the init State\r\n");
        break;

    case SYSTEM_STATE_INIT:
        serial.printf("Board is in the init State\r\n");
        break;

    case SYSTEM_STATE_AWAKE:
        serial.printf("Board is Awake\r\n");
        break;
    }

    serial.printf("System_Num_Boards = %d\r\n\n", System_Num_Boards);

    for (int ii = 2; ii < 6; ii++)
    {
        serial.printf("Voltage[%d] = %d \r\n", ii, voltage_cell[0][ii]);
    }
    serial.printf("\r\n");
    printTime += testTimer.read_us() - tim1;
    printCount++;
}


// Detects DC2100A-D boards attached to DC2100A-C, manages the DC2100A system state, and indicates via LED.
void Task_Status(void)
{
    //long tim1 = testTimer.read_us();
    System_Status_Task();
    /*statTime += testTimer.read_us() - tim1;
    statCount++;*/
}

// Reads Cell Voltages and Monitors for UV and OV
void Task_Voltage(void)
{        
    //long tim1 = testTimer.read_us();
    if (returnSysState() == SYSTEM_STATE_AWAKE)
    {
        if (dc2100a_task_skip.voltage == 0)
        {
            Voltage_Monitor_Task();
        }
    }            
    /*voltTime += testTimer.read_us() - tim1;
    voltCount++;*/
}

// Monitors the charger, discharger, and pack current in the DC2100A system
void Task_Pack_Current(void)
{
    if (returnSysState() == SYSTEM_STATE_AWAKE)
    {
        //Pack_Current_Monitor_Task();  // #Changed - Commented out in the mean time
    }
}

// Reads Temperatures // #ComeBack - Temperature Code not implemented yet
void Task_Temperature(void)
{
    if (returnSysState() == SYSTEM_STATE_AWAKE)
    {
        //DEBUG3_OUT_PIN = 0;
        if (dc2100a_task_skip.temperature == 0)
        {
            Temperature_Monitor_Task();
        }
        //DEBUG3_OUT_PIN = 1;
    }
}

// Controls the Balancers
void Task_Balancer(void)
{
    //long tim1 = testTimer.read_us();
    if (returnSysState() == SYSTEM_STATE_AWAKE)
    {
        Balancer_Control_Task(); 
    }
    /*balTime += testTimer.read_us() - tim1;
    balCount++;*/
}

// Detect new boards as they wake up in the system.  Necessary when the PIC is powered up before the cells so that they can not all be detected at init time.
void Task_Detect(void)
{
    //long tim1 = testTimer.read_us();
    if (returnSysState() == SYSTEM_STATE_AWAKE)
    {
        System_Detect_Task();
    }
    //detTime += testTimer.read_us() - tim1;
    //detCount++;
}

// Detect errors in the DC2100A system and report through USB.
void Task_Error(void)
{
    //long tim1 = testTimer.read_us();
    Error_Monitor_Task();
    //errTime += testTimer.read_us() - tim1;
    //errCount++;
}

// Receives bytes from the USB bulk endpoint
void Task_Get_USB(void)
{
    incomingData_Mutex.lock();      // Locks this critical section that makes a modification to the usb_parser_transmit_queue struct so other threads can't access it until it's done
    USB_Parser_Check_Incoming();    // Receive bytes from the USB if they are available
    incomingData_CV.notify_one();   // Notify other blocked threads waiting for the condition variable that you are done with this critical section 
    incomingData_Mutex.unlock();    // Unlocks this critical section that makes a modification to the usb_parser_transmit_queue struct
}


// Note - this must be its own task, in order to use rtos await.
// Parse commands and invoke lower level routines.
void Task_Parser(void)
{

    //serial.printf("Inside Task Parser\r\n");
    // Prevent a new response from being started before the old one has been sent
    if (usb_parser_transmit_queue.Length != 0)
    {
    }
    // If async responses are pending, process them now
    else if (usb_parser_async_queue.Length != 0)
    {
        // Send asyncs if any are pending
        USB_Parser_Async_Response();
    }
    // Otherwise, process responses for the incoming USB
    else if (usb_parser_receive_queue.Length != 0)
    {
        //LED_COMM_PIN = 1;
        //DEBUG1_OUT_PIN = 0;
        //serial.printf("In Queue Length : %d!\r\n", usb_parser_receive_queue.Length);

        // Get the command to process.
        usb_parser_command = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);

        // Parse the command
        switch (usb_parser_command)
        {
        default:                                                /* By default anything not specified is a no-op */
            //printf(USB_Parser_Buffer_Put_Char, USB_PARSER_DEFAULT_STRING);
            serial.printf(USB_PARSER_DEFAULT_STRING);
            break;

        case USB_PARSER_HELLO_COMMAND:                          /*  Reply with Hello String.  Mostly useful for testing. */
            USB_Parser_Hello_Response();
            break;

        case USB_PARSER_IDSTRING_COMMAND:                       /*  Read controller ID and firmware rev, this supports legacy functions */
        case USB_PARSER_IDSTRING_COMMAND_2:
            USB_Parser_IDString_Response();
            break;

        case USB_PARSER_BOOT_MODE_COMMAND:                      /* Enter Bootload Mode */
            /*rtos_await(usb_parser_receive_queue.Length >= SYSTEM_BOOTLOAD_KEY_SIZE);
            USB_Parser_Bootload_Command();*/
            break;

        case USB_PARSER_ERROR_COMMAND:                          /* Read Error Data*/
            // Get the board number
            USB_Parser_Error_Data_Response();
            break;

        case USB_PARSER_SYSTEM_COMMAND:                         /* Read System Data*/
            USB_Parser_System_Data_Response();
            break;

        case USB_PARSER_MFG_COMMAND:                            /* Read/Write Board Manufacturing Data */
             //Get the read/write character and board number
            //rtos_await(usb_parser_receive_queue.Length >= 3); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables

            //Get the read/write character and board number
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 3) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }

            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            usb_parser_board_num = getUSBint8_ASCII();

            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            // If a write, handle the write
            if (usb_parser_subcommand == 'W')
            {
                // Get the mfg data
                //rtos_await(usb_parser_receive_queue.Length >= (DC2100A_MODEL_NUM_SIZE + DC2100A_CAP_DEMO_SIZE + DC2100A_SERIAL_NUM_SIZE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (DC2100A_MODEL_NUM_SIZE + DC2100A_CAP_DEMO_SIZE + DC2100A_SERIAL_NUM_SIZE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_Mfg_Data_Command(usb_parser_board_num);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex

            }
            // If a write, handle the write
            else if (usb_parser_subcommand == 'D')
            {
                //rtos_await(usb_parser_receive_queue.Length >= (EEPROM_RESET_KEY_SIZE + SYSTEM_RESET_KEY_SIZE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (EEPROM_RESET_KEY_SIZE + SYSTEM_RESET_KEY_SIZE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_Mfg_Data_Reset(usb_parser_board_num);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex

            }

            // Always send a response, whether it's a read or a write
            USB_Parser_Board_Mfg_Data_Response(usb_parser_board_num);

            break;

        case USB_PARSER_UVOV_COMMAND:                           /* Read Board Over-Voltage and Under-Voltage Conditions */
            // Get the board number
            //rtos_await(usb_parser_receive_queue.Length >= (sizeof(usb_parser_board_num) * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < (sizeof(usb_parser_board_num) * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_board_num = getUSBint8_ASCII();
            incomingData_Mutex.unlock(); // Unlock incoming data mutex

            USB_Parser_Board_Vov_Vuv_Response(usb_parser_board_num);
            break;

        case USB_PARSER_VOLTAGE_COMMAND:                        /* Read Board Voltage Data */
            dc2100a_task_skip.voltage = 1;
            for (usb_parser_board_num = 0; usb_parser_board_num < System_Num_Boards; usb_parser_board_num++)
            {
                USB_Parser_Board_Voltage_Data_Response(usb_parser_board_num);

                // Wait for USB driver to be ready for data, and then send out one board at time
                //rtos_await(usb_tbe(1));  // #Changed - Might not be needed since the equivalent of usb_tbe - serial.writeable is always true
                sendUSBmessage();
            }
            dc2100a_task_skip.voltage = 0;
            break;

        case USB_PARSER_TEMPERATURE_COMMAND:                    /* Read Board Temperature Data */
            dc2100a_task_skip.temperature = 1;
            for (usb_parser_board_num = 0; usb_parser_board_num < System_Num_Boards; usb_parser_board_num++)
            {
                USB_Parser_Board_Temperature_Data_Response(usb_parser_board_num);

                // Wait for USB driver to be ready for data, and then send out one board at time
                //rtos_await(usb_tbe(1));  // #Changed - Might not be needed since the equivalent of usb_tbe - serial.writeable is always true
                sendUSBmessage();
                sendUSBmessage();
            }
            dc2100a_task_skip.temperature = 0;
            break;

        case USB_PARSER_TEMP_ADC_COMMAND:                       /* Read Board Temperature Adc Values */
            // Get the board number
            //rtos_await(usb_parser_receive_queue.Length >= (sizeof(usb_parser_board_num) * ASCII_PER_BYTE));
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < (sizeof(usb_parser_board_num) * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_board_num = getUSBint8_ASCII();
            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            USB_Parser_Board_Temperature_Adc_Value_Response(usb_parser_board_num);
            break;

        case USB_PARSER_PACK_CURRENT_COMMAND:
            USB_Parser_Pack_Current_Data_Response();
            break;

        case USB_PARSER_LTC3300_COMMAND:                        /* LTC3300 Raw Write via LTC6804 */
            // Get the read/write character, board number, and number of bytes in command.
            //rtos_await(usb_parser_receive_queue.Length >= 4); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 4) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            usb_parser_board_num = getUSBint8_ASCII();
            usb_parser_num_bytes = ASCIItonybble(USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue));
            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            // If a write, handle the write
            if (usb_parser_subcommand == 'W')
            {
                // Wait for the bytes that need to be sent.
                //rtos_await(usb_parser_receive_queue.Length >= (usb_parser_num_bytes * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (usb_parser_num_bytes * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_LTC3300_Write_Command(usb_parser_board_num, usb_parser_num_bytes);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex

            }
            else
            {
                // Wait for the one bytes that needs to be sent, before receiving num bytes.
                //rtos_await(usb_parser_receive_queue.Length >= (1 * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (1 * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_LTC3300_Read_Response(usb_parser_board_num, usb_parser_num_bytes);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex
            }
            break;

        case USB_PARSER_TIMED_BALANCE_COMMAND:                  /* Board Timed Balance */
            // Get the usb_parser_subcommand and board number
            // Get the read/write character and board number
            //rtos_await(usb_parser_receive_queue.Length >= 3); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 3) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            usb_parser_board_num = getUSBint8_ASCII();
            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            // Write a balancing sequence for one board.
            if (usb_parser_subcommand == 'W')
            {
                //rtos_await(usb_parser_receive_queue.Length >= (sizeof(BALANCER_ACTIVE_STATE_TYPE) * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (sizeof(BALANCER_ACTIVE_STATE_TYPE) * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                Balancer_Set();
                USB_Parser_Board_Active_Balancer_Command(usb_parser_board_num);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex

            }
            // Begin the Balancing.
            else if (usb_parser_subcommand == 'B')
            {
                Balancer_Start();
            }
            // Suspend the Balancing.
            else if (usb_parser_subcommand == 'S')
            {
                Balancer_Suspend();
            }
            // End the Balancing.
            else if (usb_parser_subcommand == 'E')
            {
                Balancer_Stop();
            }

            // Always send a response, whether it's a read or a write
            USB_Parser_Board_Active_Balancer_Response(usb_parser_board_num);
            break;

        case USB_PARSER_PASSIVE_BALANCE_COMMAND:                /* Board Passive Balancers */
            //rtos_await(usb_parser_receive_queue.Length >= 3); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 3) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            usb_parser_board_num = getUSBint8_ASCII();
            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            // If a write, handle the write
            if (usb_parser_subcommand == 'W')
            {
                //rtos_await(usb_parser_receive_queue.Length >= (sizeof(BALANCER_PASSIVE_STATE_TYPE) * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (sizeof(BALANCER_PASSIVE_STATE_TYPE) * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_Passive_Balancer_Command(usb_parser_board_num);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex

            }

            // Always send a response, whether it's a read or a write
            USB_Parser_Board_Passive_Balancer_Response(usb_parser_board_num);
            break;

        case USB_PARSER_CELL_PRESENT_COMMAND:                   /* Board Cell Present */
            //rtos_await(usb_parser_receive_queue.Length >= 3); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 3) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            usb_parser_board_num = getUSBint8_ASCII();
            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            // If a write, handle the write
            if (usb_parser_subcommand == 'W')
            {
                //rtos_await(usb_parser_receive_queue.Length >= (sizeof(VOLTAGE_CELL_PRESENT_TYPE) * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < (sizeof(VOLTAGE_CELL_PRESENT_TYPE) * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_Cell_Present_Command(usb_parser_board_num);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex
            }

            // Always send a response, whether it's a read or a write
            USB_Parser_Board_Cell_Present_Response(usb_parser_board_num);
            break;

        case USB_PARSER_EEPROM_COMMAND:                         /* Read/Write/Default EEPROM */
            // Get the read/write/default character, board number, and EEPROM_data_id.
            //rtos_await(usb_parser_receive_queue.Length >= 4); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 4) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            usb_parser_board_num = getUSBint8_ASCII();
            usb_parser_item_num = ASCIItonybble(USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue));
            incomingData_Mutex.unlock(); // Unlock incoming data mutex


            if (usb_parser_item_num >= USB_PARSER_EEPROM_NUM_ITEMS)
            {
                break;
            }

            // If a writing/loading/defaulting, handle that before the response
            if ((usb_parser_subcommand == 'L') || (usb_parser_subcommand == 'l'))
            {
                USB_Parser_Board_EEPROM_Default_Load_Command(usb_parser_board_num, usb_parser_item_num, (usb_parser_subcommand == 'L') ? 0 : EEPROM_MFG_KEY);
            }
            else if ((usb_parser_subcommand == 'D') || (usb_parser_subcommand == 'd'))
            {
                USB_Parser_Board_EEPROM_Default_Save_Command(usb_parser_board_num, usb_parser_item_num, (usb_parser_subcommand == 'D') ? 0 : EEPROM_MFG_KEY);
            }
            else if ((usb_parser_subcommand == 'W') || (usb_parser_subcommand == 'w'))
            {
                usb_parser_num_bytes = usb_parser_eeprom_num_bytes[(int16)usb_parser_item_num];

                // Wait for the one bytes that needs to be sent, before receiving num bytes.
                //rtos_await(usb_parser_receive_queue.Length >= usb_parser_num_bytes); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
                incomingData_Mutex.lock(); // Lock incoming data mutex
                while (usb_parser_receive_queue.Length < usb_parser_num_bytes) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
                {
                    incomingData_CV.wait();
                }
                USB_Parser_Board_EEPROM_Write_Command(usb_parser_board_num, usb_parser_item_num, (usb_parser_subcommand == 'W') ? 0 : EEPROM_MFG_KEY);
                incomingData_Mutex.unlock(); // Unlock incoming data mutex
            }

            USB_Parser_Board_EEPROM_Read_Response(usb_parser_board_num, usb_parser_item_num);
            break;

        case USB_PARSER_UVOV_THRESHOLDS_COMMAND:                /* Over and Under Voltage Thresholds */
            //rtos_await(usb_parser_receive_queue.Length >= (2 * sizeof(int16) * ASCII_PER_BYTE)); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < (2 * sizeof(int16) * ASCII_PER_BYTE)) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }

            USB_Parser_System_UVOV_Command();
            USB_Parser_System_Data_Response();
            incomingData_Mutex.unlock(); // Unlock incoming data mutex
            break;

        //    // todo - rename this command // #NeededNow??? - Will we need this? #Changed - Commented this out since I don't think we need it
        //case USB_PARSER_CAP_DEMO_COMMAND:                       /* Charge/Discharge the Cap Board */
        //    //rtos_await(0 != usb_parser_receive_queue.Length); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
        //    incomingData_Mutex.lock(); // Lock incoming data mutex
        //    while (usb_parser_receive_queue.Length == 0) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
        //    {
        //        incomingData_CV.wait();
        //    }
        //    switch (USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue))
        //    {
        //    case 'C':   //start charging
        //        System_Cap_Demo.charging = 1;
        //        System_Cap_Demo.discharging = 0;
        //        USB_Parser_System_Data_Response();
        //        break;
        //    case 'N':   //suspend charging
        //        System_Cap_Demo.charging = 0;
        //        System_Cap_Demo.discharging = 0;
        //        USB_Parser_System_Data_Response();
        //        break;
        //    case 'D':   //start discharging
        //        System_Cap_Demo.charging = 0;
        //        System_Cap_Demo.discharging = 1;
        //        USB_Parser_System_Data_Response();
        //        break;
        //    case 'c':   //toggle charging
        //        if (Pack_Current_IO.output_enabled)
        //        {
        //            Pack_Current_IO.charging_output = 1 - Pack_Current_IO.charging_output;
        //        }
        //        else
        //        {
        //            Pack_Current_IO.charging_output = 0;
        //        }
        //        USB_Parser_Pack_Current_Data_Response();
        //        break;
        //    case 'd':   //toggle discharging
        //        if (Pack_Current_IO.output_enabled)
        //        {
        //            Pack_Current_IO.discharging_output = 1 - Pack_Current_IO.discharging_output;
        //        }
        //        else
        //        {
        //            Pack_Current_IO.discharging_output = 0;
        //        }
        //        USB_Parser_Pack_Current_Data_Response();
        //        break;
        //    }
        //    incomingData_Mutex.unlock(); // Unlock incoming data mutex
        //    break;

        case USB_PARSER_ALGORITHM_COMMAND:                  /* Timed Balance Incorporating Algorithm */
            // Get the usb_parser_subcommand and board number
            //rtos_await(usb_parser_receive_queue.Length >= 1); // #Changed - Needed an equivalent for rtos_await. Using mutexes and controlvariables
            incomingData_Mutex.lock(); // Lock incoming data mutex
            while (usb_parser_receive_queue.Length < 1) // While condition is not true, block current thread with condition variable wait func while another thread works on making the condition true
            {
                incomingData_CV.wait();
            }
            usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
            incomingData_Mutex.unlock(); // Unlock incoming data mutex

            usb_parser_board_num = DC2100A_NUCLEO_BOARD_NUM;   //  Balance algorithm only implemented for 1 board with 12 cells.

            // Write a balancing sequence for one board.
            if (usb_parser_subcommand == 'W')
            {
                USB_Parser_Balancer_Algorithm_Command(usb_parser_board_num);
            }

            // Always send a response, whether it's a read or a write
            USB_Parser_Board_Active_Balancer_Response(usb_parser_board_num);
            break;

        case USB_PARSER_EMERGENCY_STOP_COMMAND:                  /* EMERGENCY STOP COMMAND */
            if (Balancer_Is_Balancing() == true)
            {
                Balancer_Stop();
            }

            break;

        }  //end of outer switch statement

        TOGGLE(LED_COMM_PIN);

    } // end of if statement

    //LED_COMM_PIN = 0;
    //DEBUG1_OUT_PIN = 1;

    USB_Parser_Check_Outgoing();

}


#endif
