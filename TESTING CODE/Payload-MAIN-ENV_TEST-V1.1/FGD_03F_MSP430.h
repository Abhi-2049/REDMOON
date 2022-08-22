/*
* HEADER FILE 
* - 
*/

#ifndef FGD_03F_MSP430_h
#define FGD_03F_MSP430_h


//-------------------------------------- LIBRARIES -------------------------------------------------------------------------------

  //#include "Arduino.h"
  #include "SPI.h"

  #include <SPI.h>
  #include <driverlib.h>


//--------------------------------------  MACROS  -------------------------------------------------------------------------------
  
  // CHOOSE SENSITIVITY & ISR & WINDOW
  #define HIGHSENS                                                                                    // $$ Change based on HISENS/LOWSENS
  // #define LOWSENS
  #define FGD_ISR true                                                                                // $$ Change to disable ISR for NIRQ on FGDOS
  //#define FGD_ISR false
  #define WINDOW_PULSES 4096                                                                          // $$ Change based on WINDOW setting and implement in FGDOS init
  //#define WINDOW_PULSES 8192
  //#define WINDOW_PULSES 16384
  // #define WINDOW_PULSES 32768

  
  
  // LaunchPad pins                     // MCU PIN Number

  // pins for slave selection and SPI
  #define SS1 12                        // P1.4      
  #define SS2 11                        // P1.3  
  
  #define NSTBY_1 35                    // P3.3
  #define NSTBY_2 36                    // P4.7
  // Window clock signal pins
  #define WCK_1 8                       // P3.4
  
  #define NIRQ_1 19                     // P1.2
  #define NIRQ_2 18                     // P3.0

  // Useful masks
  // WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
  // frequency mask to select the correct bits from the frequency registers
  #define WR 0x40
  #define RD 0x80
  #define FREQ_MASK 0x3FFFF

  // Register definitions
  #define x0_TEMP 0x00
  #define x1_RECHARGE_COUNT 0x01
  #define x9_TARGET 0x09
  #define xA_THRESHOLD 0x0A
  #define xB_RECHARGE_WINDOW 0x0B
  #define xC_CHARGE_SENS 0x0C
  #define xD_RECHARGE_REF 0x0D
  #define xE_NIRQ_ENGATE 0x0E

  // sensor constant definitions (these depend on settings in Arduino and sensor!)
  #define CK_FREQ 32768.0f // depends on the settings of the PWM                                                             // 31250/32768 $$ Change based on WCK source
//  #define CK_FREQ 31250.0f
  #define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES) // one should be defined as float, otherwise rounding errors

  // register overall valid settings
  // nirq setting to push-pull (nirqoc bit 0) or open collector (1), measurement window to count clocks (engate bit 0)
  // edirt bit to 1
  #define xE_settings 0x06
  // set the reference oscillator and window measurement amount of pulses settings
  // bits (8:4) for recharging (enable autorech, internal pump at VB, rech by pump, 0) and (3:2) for window (bit counting from lsb to msb)
  // windows set to 11 (4096 pulses)(00=32768 ck pulses per window)
  // TDIV (bit 0) to 1 for more precise frequency range, ONLY FOR LOWEST 2 AMOUNTS OF PULSES USED 
  // bitshift is used for threshold and target register comparison, depends on TDIV (if 1, then 10 shifts, if 0 then 13)
  // Cx default, 4x to disable auto recharge (called active and passive but is confusing, I know...)
  #if WINDOW_PULSES == 4096                                                                                                     
    // TDIV to 1
    #define xB_settings_a 0xCD
    #define xB_settings_p 0x4D
    #define BITSHIFT 1024 // is 10 bit shifts
  #elif WINDOW_PULSES == 8192
    // TDIV to 1
    #define xB_settings_a 0xC9
    #define xB_settings_p 0x49
    #define BITSHIFT 1024
  #elif WINDOW_PULSES == 16384
    // TDIV to determine
    #define xB_settings_a 0xC5
    #define xB_settings_p 0x45
    #define BITSHIFT ????
  #elif WINDOW_PULSES == 32768
    // TDIV to 0
    #define xB_settings_a 0xC0
    #define xB_settings_p 0x40
    #define BITSHIFT 8192
  #endif

  #ifdef HIGHSENS
    #define SENS "high"
    // these values are not exact! Since only 5 bits are used (depends on TDIV)
    #define THRESHOLD_FREQ 50000                                                                                              // $$ Change based on target and threshold freq desired for HISENS
    #define TARGET_FREQ 90000
    // manual recharge off and sesitivity to high
    // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
    #define xC_settings 0x79
    // disconnect recharging system before setting targets, set pump level (for shortest recharge time bit 3:0 to 111)
    // the recharge voltage of 100 (16.5 V, ...4) seems to be good, otherwise to fast and overshoots in recharging
    // set reference close to target, according to sensitivity  (bit 3 to 1 for lower values)
    #define xD_settings_off 0x04
    // allow recharges again
    #define xD_settings_on 0x44
    // for enabling manual recharges
    #define xC_settings_manual 0xF9
    #define xD_settings_manual 0x64
  #endif

  #ifdef LOWSENS
    #define SENS "low"
    // these values are not exact! Since only 5 bits are used (depends on TDIV)
    #define THRESHOLD_FREQ 140000                                                                                               // $$ Change based on target and threshold freq desired for LOWSENS
    #define TARGET_FREQ 180000
    // manual recharge off and sesitivity to low
    // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
    #define xC_settings 0x7C
    // disconnect recharging system before setting targets, set pump level (for shortest recharge time bit 3:0 to 111)
    // the recharge voltage of 100 (16.5V) seems to be good, otherwise to fast and overshoots in recharging
    // set reference close to target, according to sensitivity  (bit 3 to 1 for lower values)
    #define xD_settings_off 0x04
    // allow recharges again
    #define xD_settings_on 0x44
    // for enabling manual recharges
    #define xC_settings_manual 0xFC
    #define xD_settings_manual 0x64
  #endif

  //-------------------------------------- FUNCTIONS DECLARATIONS -------------------------------------------------------------------------------
  
  unsigned int read_reg(byte sensor, byte reg);
  void write_reg(byte sensor, byte reg, byte data);
  void collect_freq(byte sensor,long unsigned int *sens_freq, long unsigned int *ref_freq,float window_factor);
  void collect_data(byte sensor,int *temperature, byte*recharge_count, unsigned long int *sens_freq, unsigned long int *ref_freq, float window_factor);
  void fgdos_init(byte sensor);
  void fgdos_init_variable(byte sensor,String sens,String state);
  void collect_range(byte sensor,unsigned long int *target_freq, unsigned long int *threshold_freq);
  void discharge_enable(byte sensor);
  void discharge_disable(byte sensor);
  void recharge_enable(byte sensor);
  void recharge_disable(byte sensor);
  void auto_recharge_enable(byte sensor);
  void auto_recharge_disable(byte sensor);
  void print_freq(byte freq_reg[3],unsigned long int freq_value,char f_type);
  void print_meas_full(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor);
  void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte reg_recharge_counter, byte sensor);
  void float_to_bytes(float val,byte* bytes_array);
  long int convert_to_pulses(int window);
  String convert_to_sens(int sens);
  char get_command();
  void set_command(char command, unsigned long int *sens_freq_1, unsigned long int *sens_freq_2, unsigned long int *target_freq,
                  unsigned long int *threshold_freq, unsigned long int *ref_freq_1, unsigned long int *ref_freq_2,
                  int *temperature_1, int *temperature_2, byte *recharge_count_1, byte *recharge_count_2, bool flag_isr);
  void collect_data_SS1();
  void collect_data_SS2();
  void read_all_registers(byte sensor);

//----------------------------------------- EXTENDED FUNCTIONS----------------------------------------------

  // WCK Function definitions
  void Clock_set_readout();
  void Clock_reset();
  // RS485 Transceiver Function definitions
  void Enable_RS485_send();
  void Disable_RS485_send();



  
#endif
