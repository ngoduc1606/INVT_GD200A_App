/**
@file
Arduino library for communicating with GD200 Inverter over RS/485 (via RTU protocol).
*/
/*

  INVT_GD200A_V1.h - Arduino library for communicating with OP320A&S HMI
  over RS232/485 (via RTU protocol).

  Library:: INVT_GD200A_V1

  This version sofware driver is dev for GD200 series of INVT inverter. Prequisites is
  P00.01 = 2;
  P00.06 = 8;
  P14.00 = 3; //MODBUS_INVT_ADDRESS 
  P14.01 = 3;
  P14.02 = 1;
  P14.03 = 0;
  p02.01 = 5.5;// Load Power
  P00.17 = 1 ;// if the load is low
  p06.04 = 5 ;
  _RW_ characteristics means the function is with read and write characteristics
  _R_ characteristic can only read other than write 
  _W_ characteristic can only write other than read
  Copyright:: 2021 nguyentrinhtuan1996@gmail.com
*/

#ifndef INVT_GD200A_h
#define INVT_GD200A_h

/* _____PROJECT INCLUDES____________________________________________________ */
#include "Arduino.h"
#include <ModbusRTU.h>
#include <SoftwareSerial.h>


/* _____GLOBAL VARIABLES_____________________________________________________ */
ModbusRTU mb;

/* _____DEFINETIONS__________________________________________________________ */

// the fault type instruction
enum Fault_Type_Instruction
{
  
  #define	FAULT1	1 
  #define	FAULT2	2
  #define	FAULT3	3
  #define	FAULT4	4
  #define	FAULT5	5
  #define	FAULT6	6
  #define	FAULT7	7
  #define	FAULT8	8
  #define	FAULT9	9
  #define	FAULT10	10
  #define	FAULT11	11
  #define	FAULT12	12
  #define	FAULT13	13
  #define	FAULT14	14
  #define	FAULT15	15
  #define	FAULT16	16
  #define	FAULT17	17
  #define	FAULT18	18
  #define	FAULT19	19
  #define	FAULT20	20
  #define	FAULT21	21
  #define	FAULT22	22
  #define	FAULT23	23
  #define	FAULT24	24
  #define	FAULT25	25
  #define	FAULT26	26
  #define	FAULT27	27
  #define	FAULT28	28
  #define	FAULT29	29
  #define	FAULT30	30
  #define	FAULT31	31
  #define	FAULT32	32
  #define	FAULT33	33
  #define	FAULT34	34
  #define	FAULT35	35
  #define	FAULT36	36
};

/**
 * the fault type instruction
 * These defines are fault instruction in char* and used
 */ 
enum Fault_Type_Instruction_Code
{
  #define FAULT0_Char  "[Done] - NOFAULT\n"
  #define FAULT1_Char  "[Error] - IGBT U phase protection (OUt1)\n"
  #define FAULT2_Char  "[Error] - IGBT V phase protection (OUt2)\n"
  #define FAULT3_Char  "[Error] - IGBT W phase protection (OUt3)\n"
  #define FAULT4_Char  "[Error] - Over current phase 1 (OC1)\n"
  #define FAULT5_Char  "[Error] - Over current phase 2 (OC2)\n"
  #define FAULT6_Char  "[Error] - Over current phase 3 (OC3)\n"
  #define FAULT7_Char  "[Error] - Over voltage phase 1 (OV1)\n"
  #define FAULT8_Char  "[Error] - Over voltage phase 2 (OV2)\n"
  #define FAULT9_Char  "[Error] - Over voltage phase 3 (OV3)\n"
  #define FAULT10_Char "[Error] - Under volatge (UV)\n"
  #define FAULT11_Char "[Error] - Motor overload (OL1)\n"
  #define FAULT12_Char "[Error] - The inverter overload (OL2)\n"
  #define FAULT13_Char "[Error] - Input side phase loss (SPI)\n"
  #define FAULT14_Char "[Error] - Output side phase loss (SPO)\n"
  #define FAULT15_Char "[Error] - Overheat of the rectifier module (OH1)\n"
  #define FAULT16_Char "[Error] - Overheat fault of the inverter module (OH2)\n"
  #define FAULT17_Char "[Error] - External fault (EF)\n"
  #define FAULT18_Char "[Error] - 485 communication fault (CE)\n"
  #define FAULT19_Char "[Error] - Current detection fault (ItE)\n"
  #define FAULT20_Char "[Error] - Motor antotune fault (tE)\n"
  #define FAULT21_Char "[Error] - EEPROM operation fault (EEP)\n"
  #define FAULT22_Char "[Error] - PID response offline fault (PIDE)\n"
  #define FAULT23_Char "[Error] - Braking unit fault (bCE)\n"
  #define FAULT24_Char "[Error] - Running time arrival (END)\n"
  #define FAULT25_Char "[Error] - Electrical overload (OL3)\n"
  #define FAULT26_Char "[Error] - Panel communication fault (PCE)\n"
  #define FAULT27_Char "[Error] - Parameter uploading fault (UPE)\n"
  #define FAULT28_Char "[Error] - Parameter downloading fault (DNE)\n"
  #define FAULT32_Char "[Error] - Grounding short circuit fault 1(ETH1)\n"
  #define FAULT33_Char "[Error] - Grounding short circuit fault 2(ETH2)\n"
  #define FAULT36_Char "[Error] - Undervoltage fault (LL)\n"
};
// direction of fan that is drived by inverter
#define FORWARD 1
#define REVERSE 0
// the INVT modbus register
// Communication control command
#define INVT_REGISTER_RW_CONTROL_COMMAND                0x2000
    #define INVT_VALUE_FORWARD_RUNNING    0x0001
    #define INVT_VALUE_REVERRE_RUNNING    0x0002
    #define INVT_VALUE_FORWARD_JOGGING    0x0003
    #define INVT_VALUE_REVERSE_JOGGING    0x0004
    #define INVT_VALUE_STOP               0x0005
    #define INVT_VALUE_COAST_TO_STOP      0x0006 // emergency stop
    #define INVT_VALUE_FAULT_RESET        0x0007
    #define INVT_VALUE_JOGGING_STOP       0x0008
// Communication setting frequency(0~Fmax(unit: 0.01Hz)) 
#define INVT_REGISTER_RW_SETTING_FREQUENCY              0x2001 // 0~Fmax(unit: 0.01Hz)
// PID reference, range(0~1000, 1000 corresponds to100.0% )
#define INVT_REGISTER_RW_PID_REFERENCE                  0x2002 
// PID feedback, range(0~1000, 1000 corresponds to100.0% )
#define INVT_REGISTER_RW_PID_FEEDBACK                   0x2003 
// Torque setting value (-3000~3000, 1000 corresponds to the 100.0% of the rated current of the motor)
#define INVT_REGISTER_RW_TORQUE_SETTING_VALUE           0x2004 
/**
 * The upper limit frequency setting during forward rotation(0~Fmax(unit: 0.01Hz))
 * #define MAX_FORWARD_ROTATION_LIMIT_FREQ                 5000
 */
#define INVT_REGISTER_RW_FORWARD_ROTATION_LIMIT_FREQ    0x2005 
/**
 * The upper limit frequency setting during reverse rotation(0~Fmax(unit: 0.01Hz))
 * // #define MAX_RESERVE_ROTATION_LIMIT_FREQ                 5000
 */  
#define INVT_REGISTER_RW_REVERSE_ROTATION_LIMIT_FREQ    0x2006 
// The upper limit torque of electromotion torque (0~3000, 1000 corresponds to the 100.0% of the rated current of the motor)
#define INVT_REGISTER_RW_LIMIT_TORQUE_ELECTROMOTION     0x2007 
// The upper limit torque of braking torque (0~3000, 1000 corresponds to the 100.0% of the rated current of the motor)
#define INVT_REGISTER_RW_LIMIT_TORQUE_BRAKING           0x2008 
/**
 * Special control command word
 * Bit0~1:
 * =00:motor 1 
 * =01:motor 2
 * =10:motor 3 
 * =11:motor 4
 * Bit2:
 * =1 torque control 
 * =0:speed control
 */
#define INVT_REGISTER_RW_SPECIAL_CONTROL_COMMAND_WORD   0x2009
// Virtual input terminal command , range: 0x000~0x1FF
#define INVT_REGISTER_RW_VIRTUAL_INPUT_TERMINAL_COMMAND 0x200A
// Virtual input terminal command , range: 0x000~0x0F
#define INVT_REGISTER_RW_VIRTUAL_INPUT_TERMINAL_COMMAND_ 0x200B
// Voltage setting value(special for V/F separation) (0~1000, 1000 corresponds to the 100.0% of the rated voltage of the motor)
#define INVT_REGISTER_RW_VOLTAGE_SETTING_VALUE          0x200C
// AO output setting 1(-1000~1000, 1000 corresponds to 100.0%)
#define INVT_REGISTER_RW_AO_OUTPUT_SETTING_1            0x200D
// AO output setting 2(-1000~1000, 1000 corresponds to 100.0%)
#define INVT_REGISTER_RW_AO_OUTPUT_SETTING_2            0x200E

/**
 * Switch 1 of the inverter
 * 0001H: forward running
 * 0002H: reverse running
 * 0003H: stop
 * 0004H: fault
 * 0005H: POFF state
 */ 
#define INVT_REGISTER_R_SW1                             0x2100
/**
 * Switch 2 of the inverter
 * Bit0: 
 * =0:bus voltage is not established
 * =1:bus voltage is established
 * Bi1~2:
 * =00:motor 1 
 * =01:motor 2
 * =10:motor 3 
 * =11:motor 4
 * Bit3: 
 * =0:asynchronous motor
 * =1:synchronous motor
 * Bit4:
 * =0:pre-alarm without overload
 * =1:overload pre-alarm
 * Bit5~ Bit6:
 * =00:keypad control
 * =01:terminal control
 * =10:commuincation control
 */
#define INVT_REGISTER_R_SW2                             0x2101
// Fault code of the inverter, See the fault type instruction
#define INVT_REGISTER_R_FAULT_CODE                      0x2102
/**
 * Identifying code of the inverter ( GD200A-----0x010C )
 * The code is consisted of 16 bit which is high 8 bits and low 8 bits. High 8 bits mean
the motor type series and low 8 bits mean the derived motor types of the series. For
example, 0110H means Goodrive200A vector inverters.
* Code high 8 bit: 01 GD
* Code low 8 bit:
* = 0x08 : GD35 vector inverters
* = 0x09 : GD35-H1 vector inverters 
* = 0x0A : GD300 vector inverters
* = 0x0B : GD100 simple vector invert
* = 0x0C : GD200A general inverter
* = 0x0D : GD10 mini inverters
*/ 
#define INVT_REGISTER_R_ID_CODE_OF_INVERTER             0x2103
// Operation frequency Range: 0.00Hz~P00.03
#define INVT_REGISTER_R_OPERATION_FREQUENCY             0x3000  
// Setting frequency. Range: 0.00Hz~P00.03
#define INVT_REGISTER_R_SETTING_FREQUENCY               0x3001
// Bus voltage. Range: 0~12000V. Unit 0.1V
#define INVT_REGISTER_R_BUS_VOLTAGE                     0x3002
// Output voltage. Range: 0~1200V. Unit V
#define INVT_REGISTER_R_OUTPUT_VOLTAGE                  0x3003
// Output current. Range: 0.0~5000.0A
#define INVT_REGISTER_R_OUTPUT_CURRENT                  0x3004
// Operation speed. Range: 0~65535RPM
#define INVT_REGISTER_R_OPERATION_SPEED                 0x3005
// Output power. Range: -300.0~300.0%
#define INVT_REGISTER_R_OUTPUT_POWER                    0x3006
// Output torque. Range: -32768 - 32768 Unit 0.1Nm
#define INVT_REGISTER_R_OUTPUT_TORQUE                   0x3007
// Close loop setting. Range: -100.0%~100.0%
#define INVT_REGISTER_R_CLOSE_LOOP_SETTING              0x3008
// Close loop feedback. Range: -100.0%~100.0%
#define INVT_REGISTER_R_CLOSE_LOOP_FEEDBACK             0x3009
// Input IO state. Range: 0000~00FF
#define INVT_REGISTER_R_INPUT_IO_STATE                  0x300A
// Output IO state. Range: 0000~00FF
#define INVT_REGISTER_R_OUTPUT_IO_STATE                 0x300B
// AI 1. Range: 0.00~10.00V
#define INVT_REGISTER_R_AI_1                            0x300C
// AI 2. Range: 0.00~10.00V
#define INVT_REGISTER_R_AI_2                            0x300D
// AI 3. Range: 0.00~10.00V
#define INVT_REGISTER_R_AI_3                            0x300E
// AI 4. Reserved
#define INVT_REGISTER_R_AI_4                            0x300F
// Read high speed pulse 1 input. Range: 0.00~50.00kHz
#define INVT_REGISTER_R_HIGH_SPEED_PULSE_1              0x3010
// Read high speed pulse 1 input. Reserved
#define INVT_REGISTER_R_HIGH_SPEED_PULSE_2              0x3011
// Read current step of the multi-step speed. Range: 0~15
#define INVT_REGISTER_R_CURRENT_STEP_MULTI_STEP_SPEED   0x3012
// External length. Range: 0~65535
#define INVT_REGISTER_R_EXTERNAL_LENGTH                 0x3013
// External counting value. Range: 0~65535
#define INVT_REGISTER_R_EXTERNAL_COUNTING_VALUE         0x3014
// Torque setting. Range: -32767~32768 x 0.1Nm
#define INVT_REGISTER_R_TORQUE_SETTING                  0x3015
// Inverter code
#define INVT_REGISTER_R_INVERTER_CODE                   0x3016
// Faulte code
#define INVT_REGISTER_R_FAULTE_CODE                     0x5000
/**
 * inverter buffer for register uint16_t INVT_Buffer_RW_Registers[16];
 */
enum INVT_Buffer_RW_Index
{
#define INVT_Buffer_RW_Index_Control_Command                  0
#define INVT_Buffer_RW_Index_Setting_Frequency                1
#define INVT_Buffer_RW_Index_PID_REFERENCE                    2
#define INVT_Buffer_RW_Index_PID_FEEDBACK                     3
#define INVT_Buffer_RW_Index_PID_TORQUE_SETTING_VALUE         4
#define INVT_Buffer_RW_Index_FORWARD_ROTATION_LIMIT_FREQ      5
#define INVT_Buffer_RW_Index_REVERSE_ROTATION_LIMIT_FREQ      6
#define INVT_Buffer_RW_Index_LIMIT_TORQUE_ELECTROMOTION       7
#define INVT_Buffer_RW_Index_LIMIT_TORQUE_BRAKING             8
#define INVT_Buffer_RW_Index_SPECIAL_CONTROL_COMMAND_WORD     9
#define INVT_Buffer_RW_Index_VIRTUAL_INPUT_TERMINAL_COMMAND   10
#define INVT_Buffer_RW_Index_VIRTUAL_INPUT_TERMINAL_COMMAND_  11
#define INVT_Buffer_RW_Index_VOLTAGE_SETTING_VALUE            12
#define INVT_Buffer_RW_Index_AO_OUTPUT_SETTING_1              13
#define INVT_Buffer_RW_Index_AO_OUTPUT_SETTING_2              14
};


/**
 * inverter buffer for register uint16_t INVT_Buffer_R_Registers_1[4];
 */
enum INVT_Buffer_R_Index_Registers_1
{
#define INVT_Buffer_R_Index_SW1                               0
#define INVT_Buffer_R_Index_SW2                               1
#define INVT_Buffer_R_Index_FAULT_CODE                        2
#define INVT_Buffer_R_Index_ID_CODE_OF_INVERTER               3
};
// uint16_t INVT_Buffer_R_Registers_2[23];
enum INVT_Buffer_R_Index_Registers_2
{
#define INVT_Buffer_R_Index_OPERATION_FREQUENCY               00  
#define INVT_Buffer_R_Index_SETTING_FREQUENCY                 01
#define INVT_Buffer_R_Index_BUS_VOLTAGE                       02
#define INVT_Buffer_R_Index_OUTPUT_VOLTAGE                    03
#define INVT_Buffer_R_Index_OUTPUT_CURRENT                    04
#define INVT_Buffer_R_Index_OPERATION_SPEED                   05
#define INVT_Buffer_R_Index_OUTPUT_POWER                      06
#define INVT_Buffer_R_Index_OUTPUT_TORQUE                     07
#define INVT_Buffer_R_Index_CLOSE_LOOP_SETTING                08
#define INVT_Buffer_R_Index_CLOSE_LOOP_FEEDBACK               09
#define INVT_Buffer_R_Index_INPUT_IO_STATE                    10
#define INVT_Buffer_R_Index_OUTPUT_IO_STATE                   11
#define INVT_Buffer_R_Index_AI_1                              12
#define INVT_Buffer_R_Index_AI_2                              13
#define INVT_Buffer_R_Index_AI_3                              14
#define INVT_Buffer_R_Index_AI_4                              15
// #define INVT_Buffer_R_Index_HIGH_SPEED_PULSE_1                16
// #define INVT_Buffer_R_Index_HIGH_SPEED_PULSE_2                17
// #define INVT_Buffer_R_Index_CURRENT_STEP_MULTI_STEP_SPEED     18
// #define INVT_Buffer_R_Index_EXTERNAL_LENGTH                   19
// #define INVT_Buffer_R_Index_EXTERNAL_COUNTING_VALUE           20
// #define INVT_Buffer_R_Index_TORQUE_SETTING                    21
// #define INVT_Buffer_R_Index_INVERTER_CODE                     22
};
/* _____CLASS DEFINETION_____________________________________________________ */
/**
 * this public class is compatitble with esp32. To interact with the GD200 INVT, 
 * this class need a serial port as a stream to transciever data with inverter.
 * Init by begin
*/
class INVT_GD200A 
{
public :
  uint8_t   ID_Modbus = 3;
  void      Begin(HardwareSerial &serialPort, int Baudrate, uint32_t Serial_config);
  // inforamtion
  uint8_t   Fault_Code(void);
  char*     Fault_Char(void);
  float     Frequency_Operating(void);
  float     Frequency_Setting(void);
  float     Speed_Operating(void);
  float     Voltage_Bus(void);
  float     Voltage_Output(void);
  float     Current_Output(void);
  float     Power_Output(void);
  float     Torque_Output(void);

protected:
 // command
  uint8_t   Set_Frequency(uint16_t Set_Frequency);
  uint8_t   Set_Frequency_Limit(uint16_t Forward_Limit, uint16_t Reverse_Limit);
  uint8_t   Run(bool Direction);
  uint8_t   Stop_Running(void);
  uint8_t   Jog(bool Direction);
  uint8_t   Stop_Jogging(void);
  uint8_t   Stop_Emergency(void);
  uint8_t   Reset_Fault(void);

  uint8_t  Get_Data_From_INVT(void);
  uint16_t INVT_Buffer_RW_Registers[15];
  uint16_t INVT_Buffer_R_Registers_1[4];
  uint16_t INVT_Buffer_R_Registers_2[16];
private :
  uint16_t INVT_Buffer_Fault_Code = 0;
};
/**
 * Initialize class object. 
 *
 * Assigns the INVT slave ID and serial port.
 * Call once class has been instantiated, typically within setup(). this funtion will * open a software serial port and init modbus at this port.*
 *
 * @param INVT_ID HMI slave ID (1..255)
 * @param &serialPort reference to serial port object (Serial, Serial1, ... Serial3)
 * @param Baudrate is th baudrate of serial port for (1200, 2400, 4800, 9600, 19200, 38400, 57600)
 * @param Serial_config is the construction of data frame that is transmit by serial 
 * @ingroup setup
*/
// void INVT_GD200A::Begin(HardwareSerial &serialPort, int Baudrate, uint32_t Serial_config = SERIAL_8N1)
// { 
//   Serial1.begin(Baudrate, Serial_config);
//   mb.begin(&serialPort);
//   mb.master();
// }

/**
 * Get read registers from inverter. 
 * This function will read all read register from form inverter and write it to INVT_Buffer.
 * Need to be call regularly to get data from 
 */

uint8_t INVT_GD200A::Get_Data_From_INVT(void)
{
    uint8_t result =  0;
  // INVT_Buffer_RW_Register
  // Read all the holding register form HMI 3 if there any fault.

    // result = mb.readHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND,&INVT_Buffer_RW_Registers[0]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_SETTING_FREQUENCY,&INVT_Buffer_RW_Registers[1]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_PID_REFERENCE,&INVT_Buffer_RW_Registers[2]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_PID_FEEDBACK,&INVT_Buffer_RW_Registers[3]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_TORQUE_SETTING_VALUE,&INVT_Buffer_RW_Registers[4]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_FORWARD_ROTATION_LIMIT_FREQ,&INVT_Buffer_RW_Registers[5]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_REVERSE_ROTATION_LIMIT_FREQ,&INVT_Buffer_RW_Registers[6]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_LIMIT_TORQUE_ELECTROMOTION,&INVT_Buffer_RW_Registers[7]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_LIMIT_TORQUE_BRAKING,&INVT_Buffer_RW_Registers[8]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_SPECIAL_CONTROL_COMMAND_WORD,&INVT_Buffer_RW_Registers[9]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_VIRTUAL_INPUT_TERMINAL_COMMAND,&INVT_Buffer_RW_Registers[10]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_VIRTUAL_INPUT_TERMINAL_COMMAND_,&INVT_Buffer_RW_Registers[11]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_VOLTAGE_SETTING_VALUE,&INVT_Buffer_RW_Registers[12]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_AO_OUTPUT_SETTING_1,&INVT_Buffer_RW_Registers[13]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_RW_AO_OUTPUT_SETTING_2,&INVT_Buffer_RW_Registers[14]);


  // INVT_Buffer_R_Registers_1
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_SW1,&INVT_Buffer_R_Registers_1[0]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_SW2,&INVT_Buffer_R_Registers_1[1]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_FAULT_CODE,&INVT_Buffer_R_Registers_1[2]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_ID_CODE_OF_INVERTER,&INVT_Buffer_R_Registers_1[3]);


  // INVT_Buffer_R_Registers_2
    result = mb.readHreg(ID_Modbus,INVT_REGISTER_R_OPERATION_FREQUENCY,&INVT_Buffer_R_Registers_2[0]);
    delay(500);
    result = mb.readHreg(ID_Modbus,INVT_REGISTER_R_SETTING_FREQUENCY,&INVT_Buffer_R_Registers_2[1]);
    delay(500);
    result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_BUS_VOLTAGE,&INVT_Buffer_R_Registers_2[2]);
    delay(500);
    result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_OUTPUT_VOLTAGE,&INVT_Buffer_R_Registers_2[3]);
    delay(500);
    result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_OUTPUT_CURRENT,&INVT_Buffer_R_Registers_2[4]);
    delay(500);
    result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_OPERATION_SPEED,&INVT_Buffer_R_Registers_2[5]);
    delay(500);
    result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_OUTPUT_POWER,&INVT_Buffer_R_Registers_2[6]);
    delay(500);
    result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_OUTPUT_TORQUE,&INVT_Buffer_R_Registers_2[7]);
    delay(500);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_CLOSE_LOOP_SETTING,&INVT_Buffer_R_Registers_2[8]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_CLOSE_LOOP_FEEDBACK,&INVT_Buffer_R_Registers_2[9]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_INPUT_IO_STATE,&INVT_Buffer_R_Registers_2[10]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_OUTPUT_IO_STATE,&INVT_Buffer_R_Registers_2[11]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_AI_1,&INVT_Buffer_R_Registers_2[12]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_AI_2,&INVT_Buffer_R_Registers_2[13]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_AI_3,&INVT_Buffer_R_Registers_2[14]);
    // result =mb.readHreg(ID_Modbus,INVT_REGISTER_R_AI_4,&INVT_Buffer_R_Registers_2[15]);
    delay(500);
    result = mb.readHreg(ID_Modbus,INVT_REGISTER_R_FAULTE_CODE,&INVT_Buffer_Fault_Code);
    return result;
  
}

/**
 * Run the inverter fellow the direction and the frequency that have been set.
 * @param Direction : FORWARD or REVERSE
 */
uint8_t INVT_GD200A::Run(bool Direction)
{
   uint8_t result =  0;
  if (Direction == FORWARD)
    result = mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_FORWARD_RUNNING);
  else
    result =mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_REVERRE_RUNNING);
    return result;
}
/**
 * Stop running.
 * This will stop the inverter 
 */
uint8_t INVT_GD200A::Stop_Running(void)
{
    uint8_t result =  0;
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_STOP);
    return result;
}
/**
 * Jogging 
 * @param Direction  FORWARD or REVERSE
 */
uint8_t INVT_GD200A::Jog(bool Direction)

{
  uint8_t result =  0;
  if (Direction == FORWARD)
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_FORWARD_JOGGING);
  else
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_REVERSE_JOGGING);
  return result;
}
/**
 * Stop jogging
 */
uint8_t INVT_GD200A::Stop_Jogging()
{
    uint8_t result =  0;
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_JOGGING_STOP);
    return result;
}
/**
 * Emergency stop. Just use in emergency case
 */
uint8_t INVT_GD200A::Stop_Emergency(void)
{
    uint8_t result =  0;
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_COAST_TO_STOP);
    return result;
}
/**
 * reset fault.
 * This function will erease the fault history in inverter. Should be call after an electrical fault
 */
uint8_t INVT_GD200A::Reset_Fault(void)
{
    uint8_t result =  0;
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_CONTROL_COMMAND, INVT_VALUE_FAULT_RESET);
    return result;
}
/**
 * Set frequency for inverter.
 * this frequency will be send to inverter and udpate to operate inverter. To
 * ensure if the defined user frequency, user can check by Frequency_Setting();
 * There are 2 Fmaxs for 2 rotation direction: forward and reverse rotation frequency
 * Range: 0~Fmax
 * Unit: 0.01Hz
 * @param Set_Frequency the frequency will be set for inverter
 */
uint8_t INVT_GD200A::Set_Frequency(uint16_t Set_Frequency)
{
  uint8_t result =  0;
  mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_SETTING_FREQUENCY, Set_Frequency);
  return result;  
}

/**
 * Set limits for forward and reverse rotation frequency. The setting frequency must be less than limit
 * @param Forward_Limit the upper forward rotation frequency (0.00Hz)
 * @param Reverse_Limit the upper reverse rotation frequency (0.00Hz)
 */
uint8_t INVT_GD200A::Set_Frequency_Limit(uint16_t Forward_Limit, uint16_t Reverse_Limit)
{
    uint8_t result =  0;
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_FORWARD_ROTATION_LIMIT_FREQ, Forward_Limit);
    mb.writeHreg(ID_Modbus,INVT_REGISTER_RW_REVERSE_ROTATION_LIMIT_FREQ, Reverse_Limit);
    return result;
}
/**
 * read the code of inverter fault 
 */
uint8_t INVT_GD200A::Fault_Code()
{
  return INVT_Buffer_Fault_Code;
}
/**
 * return the fault by char*
 */
char* INVT_GD200A::Fault_Char(void)
{
  switch (INVT_Buffer_Fault_Code)
  {
  case FAULT1:
    return FAULT1_Char;
    break;
  case FAULT2:
    return FAULT2_Char;
    break;
  case FAULT3:
    return FAULT3_Char;
    break;
  case FAULT4:
    return FAULT4_Char;
    break;
  case FAULT5:
    return FAULT5_Char;
    break;
  case FAULT6:
    return FAULT6_Char;
    break;
  case FAULT7:
    return FAULT7_Char;
    break;
  case FAULT8:
    return FAULT8_Char;
    break;
  case FAULT9:
    return FAULT9_Char;
    break;
  case FAULT10:
    return FAULT10_Char;
    break;
  case FAULT11:
    return FAULT11_Char;
    break;
  case FAULT12:
    return FAULT12_Char;
    break;
  case FAULT13:
    return FAULT13_Char;
    break;
  case FAULT14:
    return FAULT14_Char;
    break;
  case FAULT15:
    return FAULT15_Char;
    break;
  case FAULT16:
    return FAULT16_Char;
    break;
  case FAULT17:
    return FAULT17_Char;
    break;
  case FAULT18:
    return FAULT18_Char;
    break;
  case FAULT19:
    return FAULT19_Char;
    break;
  case FAULT20:
    return FAULT20_Char;
    break;
  case FAULT21:
    return FAULT21_Char;
    break;
  case FAULT22:
    return FAULT22_Char;
    break;
  case FAULT23:
    return FAULT23_Char;
    break;
  case FAULT24:
    return FAULT24_Char;
    break;
  case FAULT25:
    return FAULT25_Char;
    break;
  case FAULT26:
    return FAULT26_Char;
    break;
  case FAULT27:
    return FAULT27_Char;
    break;
  case FAULT28:
    return FAULT28_Char;
    break;
  case FAULT32:
    return FAULT32_Char;
    break;
  case FAULT33:
    return FAULT33_Char;
    break;
  case FAULT36:
    return FAULT36_Char;
    break;

  default:
    return FAULT0_Char;
    break;
  }
}
/**
 * return the operating frequency
 * Unit: Hz
 */
float INVT_GD200A::Frequency_Operating(void)
{
  return (float)INVT_Buffer_R_Registers_2[0] / 100;
}
/**
 * return the setting frequency
 * Unit: Hz
 */
float INVT_GD200A::Frequency_Setting(void)
{
  return (float)INVT_Buffer_R_Registers_2[1] / 100;
}
/**
 * Bus voltage. 
 * Range: 0~1200V
 * Unit: V
 */
float INVT_GD200A::Voltage_Bus(void)
{
  return (float)INVT_Buffer_R_Registers_2[2]/10;
}
/**
 * output voltage
 * Range: 0~1200V
 * Unit: V
 */
float INVT_GD200A::Voltage_Output(void)
{
  return INVT_Buffer_R_Registers_2[3];
}
/**
 * Output current
 * Range: 0.0~5000.0A
 * Unit; A
 */
float INVT_GD200A::Current_Output(void)
{
  return (float)INVT_Buffer_R_Registers_2[4]/10;
}
/**
 * output power
 * Range: -300.0~300.0%
 * Unit: %
 */
float  INVT_GD200A::Power_Output(void)
{
  return (float)INVT_Buffer_R_Registers_2[6]/10;
}

/**
 * operation speed
 * Range: 0-65535 
 * Unit: RPM
 */
float INVT_GD200A::Speed_Operating()
{
  return (float)INVT_Buffer_R_Registers_2[5];
}

/**
 * output torque
 * Range: 0.0 - 6553.5
 * Unit: N.m
 */
float INVT_GD200A::Torque_Output()
{
  return (float)((int16_t)INVT_Buffer_R_Registers_2[7])/10;
}
#endif