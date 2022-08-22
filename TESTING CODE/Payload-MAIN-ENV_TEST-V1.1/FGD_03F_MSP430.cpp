/*
#include "Arduino.h"
#include <SPI.h>
#include "FGD_03F.h"
*/

/*
 * SOURCE FILE
 */

#include <SPI.h>
#include <driverlib.h>
#include "FGD_03F_MSP430.h"


bool flag_print = true;



////////////////////////////////////// FUNCTIONS ////////////////////////////////////////////////////



//------------------------------------- SETUP -------------------------------------
//--------------------------------------------------------------------------







//------------------------------------- SETTINGS & DATA GATHERING -------------------------------------
//--------------------------------------------------------------------------
/* 
 * Fuction to read a register from the sensor.  
 * first send the address, then read what is being send back
 * apparently the automatic register incrementation (necessary to read more than one register with a single request) 
 * doesn't work properly. Reads have been limited to one reg at a time. (note from CERN)
 */
unsigned int read_reg(byte sensor, byte reg){
  byte data = 0;
  byte adr = reg|RD; 
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  //delayMicroseconds(10);
  data = SPI.transfer(0x00); 
  digitalWrite(sensor, HIGH);
  return data;
}
/* 
 * write to a register 
 */
void write_reg(byte sensor, byte reg, byte data){
  byte adr = reg|WR;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  SPI.transfer(data);
  digitalWrite(sensor, HIGH);
}

void read_all_registers(byte sensor){
  Serial.print("Read all registers sensor ");
  Serial.println(sensor);
  Serial.println("0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14");
  int registers[21] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14};
  for(int i=0;i<21;i++){
    Serial.print(read_reg(sensor,registers[i]));
    Serial.print(",");
  }
  Serial.println();
}

/* 
 * read the frequency registers and reconstruct their values
 * frequency = registers / window pulses amount * ck frequency
 * also stores them in a data array before reconstructing the value in decimal
 * millis timeout disabled because of conflict when used in ISRs
 */
void collect_freq(byte sensor,long unsigned int *sens_freq, long unsigned int *ref_freq,float window_factor){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  //unsigned int time_start=millis(); // to prevent getting stuck
  int x = 1000;// increase amount of time to wait if serial.prints are used! Otherwise set to 1
  
  // first check to see if no recharge is going on
  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(sensor,0x08);
      freq_reg[1] = read_reg(sensor,0x07);
      freq_reg[2] = read_reg(sensor,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) << 8 | freq_reg[2] ) & FREQ_MASK );
      have_sens_freq = true;
      //print_freq(freq_reg,*sens_freq,'S');
      *sens_freq = *sens_freq * window_factor;
    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(sensor,0x05);
      freq_reg[1] = read_reg(sensor,0x04);
      freq_reg[2] = read_reg(sensor,0x03);
      *ref_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) << 8 | freq_reg[2] ) & FREQ_MASK );
      have_ref_freq = true;
      //print_freq(freq_reg,*ref_freq,'R');
      *ref_freq = *ref_freq * window_factor;
    }
    //if ((millis()-time_start>(4*1/window_factor*x) && !have_ref_freq && !have_sens_freq)){
    //  Serial.println("reading failed, timeout");}
  }
}

void collect_data(byte sensor,int *temperature, byte*recharge_count, unsigned long int *sens_freq, unsigned long int *ref_freq, float window_factor){
  //unsigned long int sens_freq = 0, ref_freq = 0;
  //int temperature, recharge_count;
  collect_freq(sensor,sens_freq,ref_freq, window_factor);
  *temperature = read_reg(sensor,x0_TEMP);
  *recharge_count = read_reg(sensor,x1_RECHARGE_COUNT); // only 7 LSBs are used for counting
  if (*recharge_count == 0x7F){
    Serial.println("reset recharge counter");
    write_reg(sensor,x1_RECHARGE_COUNT,0x00);
  }
  print_meas_short(*temperature,*sens_freq,*ref_freq,*recharge_count,sensor-10);
}

void fgdos_init(byte sensor){
  Serial.print("SENSOR "); Serial.println(sensor-10);
  write_reg(sensor,xB_RECHARGE_WINDOW,xB_settings_a); 
  Serial.print("window_factor "); Serial.println(WINDOW_FACTOR);
  write_reg(sensor,xC_CHARGE_SENS,xC_settings);
  Serial.print("sensitivity: "); Serial.println(SENS);
  write_reg(sensor,xE_NIRQ_ENGATE,xE_settings);
  write_reg(sensor,xD_RECHARGE_REF,xD_settings_off);
  // wait for reference to stabilise, eg 4 measurement windows (= 4*32768 pulses at 31.25 kHz = 4194 millisecs) (only stated in FGD_02F)
  __delay_cycles(4/WINDOW_FACTOR*1000);
  // set target and threshold registers by selecting 5 MSBs (/8192) and set them to approprirate registers
  // do not forget to apply window factor!
  // TDIV is set to 1 for higher resolution, so do /1024 instead, watch out though, you are not really using 8 bits to compare,
  // so you are not covering full range
  // dividing by 8192 is bitshifting by 13 ;) casting to int = floor()
  byte reg_target = floor(TARGET_FREQ/WINDOW_FACTOR/BITSHIFT);
  write_reg(sensor,x9_TARGET,reg_target);
  Serial.print("target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println(((unsigned long)reg_target*BITSHIFT)*WINDOW_FACTOR);
  byte reg_threshold = floor(THRESHOLD_FREQ/WINDOW_FACTOR/BITSHIFT);
  write_reg(sensor,xA_THRESHOLD,reg_threshold);
  Serial.print("threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println(((unsigned long)reg_threshold*BITSHIFT)*WINDOW_FACTOR);
  write_reg(sensor,xD_RECHARGE_REF,xD_settings_on);
  Serial.println();
  // wait for registers to update ( 2 measurement windows +10% recommended by Sealicon )
  __delay_cycles(2.1/WINDOW_FACTOR*1000);
}

void fgdos_init_variable(byte sensor,String sens,String state){
  unsigned long int threshold_freq, target_freq;
  byte xb_settings, xc_settings, xd_settings_on, xd_settings_off;
  if (sens == "high"){
    threshold_freq =  50000;
    target_freq =  90000;
    xc_settings = 0x79;
    xd_settings_off = 0x04;
    xd_settings_on = 0x44;
  } else if (sens == "low"){
    threshold_freq =  140000;
    target_freq =  180000;
    xc_settings = 0x7C;
    xd_settings_off = 0x04;
    xd_settings_on = 0x44;
  }
  if (state == "active"){
    xb_settings = xB_settings_a;
  } else if (state == "passive"){
    xb_settings = xB_settings_p;
  }
  Serial.print("SENSOR "); Serial.println(sensor-10);
  write_reg(sensor,xB_RECHARGE_WINDOW,xb_settings); 
  Serial.print("AUTO RECHARGE "); Serial.println(state);
  Serial.print("window_factor "); Serial.println(WINDOW_FACTOR);
  write_reg(sensor,xC_CHARGE_SENS,xc_settings);
  Serial.print("sensitivity: "); Serial.println(sens);
  write_reg(sensor,xE_NIRQ_ENGATE,xE_settings);
  write_reg(sensor,xD_RECHARGE_REF,xd_settings_off);
  __delay_cycles(4/WINDOW_FACTOR*1000);
  byte reg_target = floor(target_freq/WINDOW_FACTOR/1024);
  write_reg(sensor,x9_TARGET,reg_target);
  Serial.print("target ("); Serial.print(target_freq); Serial.print(") , "); Serial.println((reg_target << 10)*WINDOW_FACTOR);
  byte reg_threshold = floor(threshold_freq/WINDOW_FACTOR/1024);
  write_reg(sensor,xA_THRESHOLD,reg_threshold);
  Serial.print("threshold ("); Serial.print(threshold_freq); Serial.print(") , "); Serial.println((reg_threshold << 10)*WINDOW_FACTOR);
  write_reg(sensor,xD_RECHARGE_REF,xd_settings_on);
  Serial.println();
  // wait for registers to update
  __delay_cycles(2.1/WINDOW_FACTOR*1000);
}

void collect_range(byte sensor,unsigned long int *target_freq, unsigned long int *threshold_freq){
  // assume TDIV = 1! otherwise multiply by 8192 or bitshift by 13
  *target_freq = (read_reg(sensor,x9_TARGET)<<10)*WINDOW_FACTOR;
  *threshold_freq = read_reg(sensor,xA_THRESHOLD)*1024*WINDOW_FACTOR;
}

void discharge_enable(byte sensor){
  // charge pump to max, epwr to 0 and ech to 1 (normally already so)
  write_reg(sensor,0x0D,0x47); //47
  // set endch to 1 and evbch to 0 (normally so) on top of regular settings
  write_reg(sensor,0x0B,0xDD); //DD
  // pin FCH to 1 to start, is not mentioned in datasheet
  write_reg(sensor,0x0C,0xF9);
  // PINS!
}
void discharge_disable(byte sensor){
  write_reg(sensor,0x0B,0xCD); //CD
  write_reg(sensor,0x0D,0x44);
  write_reg(sensor,0x0C,0x79);
  // PINS!
}
void recharge_enable(byte sensor){
  // charge pump to 16.5 V, epwr to 1 and ech to 1 (last two should be ok by default)
  write_reg(sensor,0x0D,xD_settings_manual);
  // FCH to 1 (F) to start and to 0 (7) to stop
  write_reg(sensor,0x0C,xC_settings_manual);
  // PINS!
}
void recharge_disable(byte sensor){
  write_reg(sensor,0x0D,xD_settings_on);
  write_reg(sensor,0x0C,xC_settings);
  // PINS!
}

void auto_recharge_enable(byte sensor){
  write_reg(sensor,xB_RECHARGE_WINDOW,xB_settings_a);
}
void auto_recharge_disable(byte sensor){
  write_reg(sensor,xB_RECHARGE_WINDOW,xB_settings_p);
}

//------------------------------------- PRINTS -------------------------------------
//--------------------------------------------------------------------------
void print_freq(byte freq_reg[3],unsigned long int freq_value,char f_type){
  switch (f_type){
    case 'S':
      Serial.print("SENSOR  frequency registers (0x05,0x04,0x03): ");
      break;
    case 'R':
      Serial.print("REFERENCE frequency registers (0x05,0x04,0x03): ");
      break;
    default:
      Serial.print("error in frequency type");
      break;
  }
  Serial.print(freq_reg[0],HEX); Serial.print(" | ");
  Serial.print(freq_reg[1],HEX); Serial.print(" | ");
  Serial.print(freq_reg[2],HEX); Serial.println(" | ");
  Serial.print("Frequency reconstructed HEX "); Serial.print(freq_value,HEX);
  Serial.print(" | DEC "); Serial.println(freq_value);
}

void print_meas_full(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor){
  Serial.print("SENSOR "); Serial.println(sensor-10);
  Serial.print("\t temperature (sensor offset = ?): "); Serial.println(temperature);
  Serial.print("\t sensor and reference frequencies : ");
  Serial.print(sens_freq); Serial.print(" | ");
  Serial.print(ref_freq); Serial.println(" | ");
  }
  
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte reg_recharge_counter, byte sensor){
  flag_print ? Serial.println("Sensor, Temp , F_sens , F_ref, Rech_Count"),flag_print=false:0;
  Serial.print(sensor);Serial.print(" , ");
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.print(ref_freq);Serial.print(" , ");
  Serial.println(reg_recharge_counter,BIN);
  }

//------------------------------------- CONVERSIONS ETC -------------------------------------
//--------------------------------------------------------------------------
void float_to_bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

long int convert_to_pulses(int window){
  switch (window){
    case 0x0C:
      return 4096;
    case 0x08:
      return 8192;
    case 0x04:
      return 16384;
    case 0x00:
      return 32768;
      }
}
String convert_to_sens(int sens){
  switch (sens){
    case 0x04:
      return "LOW";
    case 0x01:
      return "HIGH";
  }
}

//------------------------------------- COMMAND LINES -------------------------------------
//--------------------------------------------------------------------------
char get_command(){
  String command_str;
  while(Serial.available()){
      command_str = Serial.readString();
    }
    return command_str[0];
}

void set_command(char command, unsigned long int *sens_freq_1, unsigned long int *sens_freq_2, unsigned long int *target_freq,
                  unsigned long int *threshold_freq, unsigned long int *ref_freq_1, unsigned long int *ref_freq_2,
                  int *temperature_1, int *temperature_2, byte *recharge_count_1, byte *recharge_count_2, bool flag_isr)
  {
  // flags to remember on or off states
  static bool flag_discharge_1 = true, flag_discharge_2 = true, flag_active_1 = true, flag_on_1 = true, flag_on_2 = true,
              flag_active_2 = true, flag_recharge_1 = true, flag_recharge_2 = true;
  //deactivate interrupt before to prevent collision
  if (flag_isr){                                                                                                                  // $$ check if this is needed
    detachInterrupt(digitalPinToInterrupt(NIRQ_1));
    detachInterrupt(digitalPinToInterrupt(NIRQ_2));
    pinMode(NIRQ_1,INPUT);
    pinMode(NIRQ_2,INPUT);
  }
  switch (command){
    case '1':
      collect_data(SS1, temperature_1, recharge_count_2, sens_freq_1, ref_freq_1, WINDOW_FACTOR);
      break;
    case '2':
      collect_data(SS2, temperature_2, recharge_count_2, sens_freq_2, ref_freq_2, WINDOW_FACTOR);
      break;     
    case 't':
      // T for target and threshold
      collect_range(SS1,target_freq,threshold_freq);
      Serial.print("TARGET 1: "); Serial.print(*target_freq); 
      Serial.print(", THRESHOLD 1: "); Serial.println(*threshold_freq);
      break; 
    case 'T':
      collect_range(SS2,target_freq,threshold_freq);
      Serial.print("TARGET 2: "); Serial.print(*target_freq); 
      Serial.print(", THRESHOLD 2: "); Serial.println(*threshold_freq);
      break;    
    case 'o':
      if (!flag_on_1){
        digitalWrite(NSTBY_1,HIGH);
        Serial.println("ON 1");
        flag_on_1 = true;
      } else {
        digitalWrite(NSTBY_1,LOW);
        Serial.println("STANDBY 1");
        flag_on_1 = false;
      }
      break;
    case 'O':
        if (!flag_on_2){
          digitalWrite(NSTBY_2,HIGH);
          Serial.println("ON 2");
          flag_on_2 = true;
        } else {
          digitalWrite(NSTBY_2,LOW);
          Serial.println("STANDBY 2");
          flag_on_2 = false;
        }
      break;     
//    case 'a':
//      if (!flag_active_1){
//        digitalWrite(PASSIVE,LOW);
//        Serial.println("ACTIVE 1");
//        flag_active_1 = true;
//      } else {
//        digitalWrite(PASSIVE,HIGH);
//        Serial.println("PASSIVE 1");
//        flag_active_1 = false;
//      }
//      break;
//    case 'A':
//      if (!flag_active_2){
//        digitalWrite(PASSIVE,LOW);
//        Serial.println("ACTIVE 2");
//        flag_active_2 = true;
//      } else {
//        digitalWrite(PASSIVE,HIGH);
//        Serial.println("PASSIVE 2");
//        flag_active_2 = false;
//      }
//      break;
    case 'r':
      if (flag_recharge_1){
        recharge_enable(SS1);
        Serial.println("RECHARGING STARTED 1");
        flag_recharge_1 = false;  
      } else {
        recharge_disable(SS1);
        Serial.println("RECHARGING STOPPED 1");
        flag_recharge_1 = true;
      }
      break;
    case 'R':
      if (flag_recharge_2){
        recharge_enable(SS2);
        Serial.println("RECHARGING STARTED 2");
        flag_recharge_2 = false;  
      } else {
        recharge_disable(SS2);
        Serial.println("RECHARGING STOPPED 2");
        flag_recharge_2 = true;
      }
      break;
    case 'd':
      if (flag_discharge_1){
        discharge_enable(SS1);
        Serial.println("discharging... 1");
        flag_discharge_1 = false;  
      } else {
        discharge_disable(SS1);
        Serial.println("DISCHARGING STOPPED 1");
        flag_discharge_1 = true;
      }
      break;
    case 'D':
      if (flag_discharge_2){
        discharge_enable(SS2);
        Serial.println("discharging... 2");
        flag_discharge_2 = false;  
      } else {
        discharge_disable(SS2);
        Serial.println("DISCHARGING STOPPED 2");
        flag_discharge_2 = true;
      }
      break;
    case 'i':
      // all settings reset
      Serial.print("RESET: ");
      fgdos_init(SS1);
      break;
    case 'I':
      // all settings reset
      Serial.print("RESET: ");
      fgdos_init(SS2);
      break;     
    case 'l':
      fgdos_init_variable(SS1,"low","active");
      break;       
    case 'L':
      fgdos_init_variable(SS2,"low","active");
      break;
    case 'h':
      fgdos_init_variable(SS1,"high","active");
      break;
    case 'H':
      fgdos_init_variable(SS2,"high","active");
      break;
    default:
      break;
  }
//  if (flag_isr){                                                                                      // $$ check if this is needed
//    if (flag_on_1){
//      pinMode(NIRQ_1, INPUT_PULLUP);
//      attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);
//    }
//    if (flag_on_2){                                                                                   // $$ check if this is needed
//      pinMode(NIRQ_2, INPUT_PULLUP);
//      attachInterrupt(digitalPinToInterrupt(NIRQ_2), collect_data_SS2, LOW);
//    }
//  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////




//---------------------------------- CLOCK (WCK) FUNCTIONS ---------------------------------------------

void Clock_set_readout(){
  // Set DCO frequency to 1 MHz
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
  // Set SMCLK = DCO with frequency divider of 32, should give an SMCLK of 31250 Hz
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);
}


void Clock_reset(){
  // Set DCO and SMCLK back to original value
  // Set DCO to 8 MHz
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
  // Set SMCLK = DCO with frequency divider of 1, should give an SMCLK of 8 MHz
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  
}



//---------------------------------- DATABUS FUNCTIONS ---------------------------------------------

void Enable_RS485_send(){
  // setting RS485 pins ~RE and DE high to start transmission over RS485 Bus. Uses P2.6 on MCU                                
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
  //delay(100);
  __delay_cycles(100000);

  //Serial.println("RE and DE RS485 pins are set HIGH");
}



void Disable_RS485_send(){
  // setting RS485 pins ~RE and DE high to start transmission over RS485 Bus. Uses P2.6 on MCU                                
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
  //delay(100);
  __delay_cycles(100000);

  //Serial.println("RE and DE RS485 pins are set LOW");
}
