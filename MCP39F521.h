/*
  MCP39F521.h - Library to be used with the MCP39F521,
  single-phase power-monitoring IC designed for
  real-time measurement of input power for AC/DC power
  supplies, providing power and energy values.
  Created by Jorge Santos, February 2, 2018.
*/
#ifndef MCP39F521_h
#define MCP39F521_h

#include "Arduino.h"
#include "Wire.h"

#define HEADER 0xA5
#define MAX_BYTE_WRITE 32
#define MAX_BYTE_READ 32

/*MCP39F521 INSTRUCTION SET*/
#define REGISTER_READ 0x4E
#define REGISTER_WRITE 0x4D
#define SET_ADDRESS_POINTER 0x41
#define SAVE_REGISTERS_FLASH 0x53
#define PAGE_READ_EEPROM 0x42
#define PAGE_WRITE_EEPROM 0x50
#define BULK_ERASE_EEPROM 0x4F
#define AUTO_CAL_GAIN 0x5A
#define AUTO_CAL_REACTIVE_GAIN 0x7A
#define AUTO_CAL_FREQUENCY 0x76

/*Output Registers*/
#define INSTRUCTION_POINTER 0x0000 //Address pointer for read or write commands
#define SYSTEM_STATUS 0x0002 //System Status Register
#define SYSTEM_VERSION 0x0004 //System version date code information for MCP39F521, set at Microchip factory format YMDD
#define VOLTAGE_RMS 0x0006 //RMS Voltage output
#define LINE_FREQUENCY 0x0008 //Line Frequency output
#define ANALOG_INPUT_VOLTAGE 0x000A //Output of the 10-bit SAR ADC
#define POWER_FACTOR 0x000C //Power Factor output
#define CURRENT_RMS 0x000E //RMS Current output
#define ACTIVE_POWER 0x0012 //Active Power output
#define REACTIVE_POWER 0x0016 //Reactive Power output
#define APPARENT_POWER 0x001A //Apparent Power output
#define IMPORT_ACTIVE_ENERGY_COUNTER 0x001E //Accumulator for Active Energy, Import
#define EXPORT_ACTIVE_ENERGY_COUNTER 0x0026 //Accumulator for Active Energy, Export
#define IMPORT_REACTIVE_ENERGY_COUNTER 0x002E //Accumulator for Reactive Energy, Import
#define EXPORT_REACTIVE_ENERGY_COUNTER 0x0036 //Accumulator for Reactive Energy, Export
#define MINIMUM_RECORD_1 0x003E //Minimum Value of the Output Quantity Address in Min / Max Pointer 1 Register
#define MINIMUM_RECORD_2 0x0042 //Minimum Value of the Output Quantity Address in Min / Max Pointer 2 Register
#define MAXIMUM_RECORD_1 0x004E //Maximum Value of the Output Quantity Address in Min / Max Pointer 1 Register
#define MAXIMUM_RECORD_2 0x0052 //Maximum Value of the Output Quantity Address in Min / Max Pointer 2 Register

/*Calibration Registers*/
#define CALIBRATION_REGISTER_DELIMITER 0x005E //8.8 R / W u16 May be used to initiate loading of the default calibration coefficients at start - up
#define GAIN_CURRENT_RMS 0x0060 //8.3 R / W u16 Gain Calibration Factor for RMS Current
#define GAIN_VOLTAGE_RMS 0x0062 //8.3 R / W u16 Gain Calibration Factor for RMS Voltage
#define GAIN_ACTIVE_POWER 0x0064 //8.3 R / W u16 Gain Calibration Factor for Active Power
#define GAIN_REACTIVE_POWER 0x0066 //8.3 R / W u16 Gain Calibration Factor for Reactive Power
#define OFFSET_CURRENT_RMS 0x0068 //8.5.1 R / W s32 Offset Calibration Factor for RMS Current
#define OFFSET_ACTIVE_POWER 0x006C //8.5.1 R / W s32 Offset Calibration Factor for Active Power
#define OFFSET_REACTIVE_POWER 0x0070 //8.5.1 R / W s32 Offset Calibration Factor for Reactive Power
#define DC_OFFSET_CURRENT 0x0074 //8.5.2 R / W s16 Offset Calibration Factor for DC Current
#define PHASE_COMPENSATION 0x0076 //8.5 R / W s16 Phase Compensation
#define APPARENT_POWER_DIVISOR 0x0078 //5.7 R / W u16 Number of Digits for apparent power divisor to match IRMS and VRMS resolution

/*Design Configuration Registers*/
#define SYSTEM_CONFIGURATION 0x007A
#define EVENT_CONFIGURATION 0x007E  //7.0 R / W b16 Settings for the Event pin
#define RANGE 0x0082  //6.6 R / W b32 Scaling factor for Outputs
#define CALIBRATION_CURRENT 0x0086  //8.3.1 R / W u32 Target Current to be used during single - point calibration
#define CALIBRATION_VOLTAGE 0x008A  //8.3.1 R / W u16 Target Voltage to be used during single - point calibration
#define CALIBRATION_POWER_ACTIVE 0x008C  //8.3.1 R / W u32 Target Active Power to be used during single - point calibration
#define CALIBRATION_POWER_REACTIVE 0x0090  //8.3.1 R / W u32 Target Reactive Power to be used during single - point calibration
#define LINE_FREQUENCY_REFERENCE 0x0094  //8.6.1 R / W u16 Reference Value for the nominal line frequency
#define ACCUMULATION_INTERVAL_PARAMETER 0x009E  //5.2 R / W u16 N for 2N number of line cycles to be used during a single computation cycle
#define VOLTAGE_SAG_LIMIT 0x00A0  //7.2 R / W u16 RMS Voltage threshold at which an event flag is recorded
#define VOLTAGE_SURGE_LIMIT 0x00A2  //7.2 R / W u16 RMS Voltage threshold at which an event flag is recorded
#define OVER_CURRENT_LIMIT 0x00A4  //7.2 R / W u32 RMS Current threshold at which an event flag is recorded
#define OVER_POWER_LIMIT 0x00A8  //7.2 R / W u32 Active Power Limit at which an event flag is recorded

/*Temperature Compensation Registers*/
#define TEMPERATURE_COMPENSATION_FOR_FREQUENCY 0x00C6  //8.7 R / W u16 Correction factor for compensating the line frequency indication over temperature
#define TEMPERATURE_COMPENSATION_FOR_CURRENT 0x00C8  //8.7 R / W u16 Correction factor for compensating the Current RMS indication over temperature
#define TEMPERATURE_COMPENSATION_FOR_POWER 0x00CA  //8.7 R / W u16 Correction factor for compensating the active power indication over temperature
#define AMBIENT_TEMPERATURE_REFERENCE_VOLTAGE 0x00CC  //8.7 R / W u16 Register for storing the reference temperature during calibration

/*Control Registers for Peripherals*/
#define MIN_MAX_POINTER1 0x00D4  //5.12 R / W u16 Address Pointer for Min / Max 1 Outputs
#define MIN_MAX_POINTER2 0x00D6  //5.12 R / W u16 Address Pointer for Min / Max 2 Outputs
#define ENERGY_CONTROL 0x00DC  //5.6 R / W u16 Input register for reset / start of EnergyAccumulation
#define NO_LOAD_THRESHOLD 0x00E0  //5.6.1 R / W u16 No - Load Threshold for Energy Counting

/*comands for events*/
#define ENABLE 1
#define DESABLE 2
#define LATCH_ON 3
#define LATCH_OFF 4
#define CLEAR_ON 5
#define CLEAR_OFF 6
#define TEST_ON 7
#define TEST_OFF 8

/*types of events*/
#define OVER_POWER 1
#define OVER_CURRENT 2
#define VOLTAGE_SURGE 3
#define VOLTAGE_SAG 4

class MCP39F521
{
public:
  MCP39F521(void);
  bool begin(byte devAddress, TwoWire &wirePort = Wire);
  /*Funções de leitura de output*/
  int read_SystemStatus();
  String read_SystemVersion();
  float read_VRMS();
  float read_IRMS();
  float read_PowerFactor();
  float read_Freq();
  float read_Active();
  float read_Reactive();
  float read_Apparent();
  long long read_Imp_Act_Ener();
  long long read_Exp_Act_Ener();
  long long read_Imp_React_Ener();
  long long read_Exp_React_Ener();
  /*Funções de configuração do MCP39F521*/
  void set_PGAGain( byte channel, byte gain);
  byte read_PGAGain( byte channel);
  int read_Gain_VRMS();
  int read_Gain_IRMS();
  int read_Gain_ActiveP();
  int read_Gain_ReactiveP();
  void set_Gain_VRMS( unsigned int volt_calib);
  void set_Gain_IRMS( unsigned int amp_calib);
  void set_Gain_ActiveP( unsigned int actP_calib);
  void set_Gain_ReactiveP( unsigned int reactP_calib);
  long read_OffsetCurrent();
  long read_OffsetActive();
  long read_OffsetReactive();
  void set_OffsetCurrent( long offset_I);
  void set_OffsetActive( long offset_P);
  void set_OffsetReactive( long offset_Q);
  boolean read_Range( byte *r_power, byte *r_current, byte *r_voltage);
  boolean set_V_Range( byte r_voltage);
  boolean set_I_Range( byte r_current);
  boolean set_P_Range( byte r_power);
  int read_Phase_Comp();
  void set_Phase_Comp( int phase_comp);
  int read_ApparentPowerDivisor();
  int read_EnergyAcum_Control();
  void set_EnergyAcum_Control( byte EA_state);
  void save_registers_flash();
  //void on_event_call( void(*function)(int),  int pin);
  //unsigned int ler_status_evento();
  unsigned int ler_registo_evento();
  int evento();
  void set_event_over_pow_limit(unsigned int limit);
  void set_event_over_curr_limit(unsigned int limit);
  void set_event_vsurge_limit(unsigned int limit);
  void set_event_vsag_limit(unsigned int limit);
  void set_event_over_pow(unsigned int config);
  void set_event_over_curr(unsigned int config);
  void event_vsurge(unsigned int config);
  void event_vsag(unsigned int config);
  void set_event_manu(unsigned int config);
  void set_event_config_all(unsigned int registo);


private:
  int _devAddress;
  TwoWire *_i2cPort;
  /*Implementação dos comandos principais do MCP39F521*/
  boolean read_register(byte devAddress, unsigned int register_to_read, byte n_bytes_read, byte data[]);
  void set_pointer(unsigned int address);
  byte write_register(byte devAddress, unsigned int register_to_write, unsigned int bytes_to_write, byte data[]);
  /*funções de apoio*/
  byte calc_checksum(byte terms[], int n_terms);
  void write_Configuration(byte devAddress, long config);
  long read_Configuration(byte devAddress);
  /*Interrupção por evento*/
  typedef void(*_call)(int);
  _call _function_event;
};
#endif
