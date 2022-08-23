//AS:  From Github - Simple Arduino test - FGDOS_Arduino/FGDOS/FGDOS_read_test/FGDOS_read_test.ino 
// Added fgdos_init() and write_reg() functions , and the pin and register defintions in the begininning

// 22/03 AS: Some problems spotted while reading out with Demo board

/* TO DO 
 *    - use window measurement cycles as a timeout for the frequency write function?
 * 
*/
//--------------------------------------------------------------------
#include <SPI.h>
//#include <FGD_03F.h>   // for debug
//--------------------------------------------------------------------
// Micro pins
// pins for slave selection
//#define SS1 7 //9 
//#define SS2 8
//#define PWM_PIN 9

// for debug

  #define HIGHSENS
  #define SENS "high"
  // #define LOWSENS

  #define THRESHOLD_FREQ 30000                                                  // 50000
  #define TARGET_FREQ 50000                                                     // 90000
  #define xC_settings 0x79
  #define xD_settings_off 0x04
  #define xD_settings_on 0x44
  
  #define FGD_ISR true
  //#define FGD_ISR false
  #define WINDOW_PULSES 4096
  #define xE_settings 0x06
  #define xB_settings_a 0xCD
  #define xB_settings_p 0x4D
  #define BITSHIFT 1024 // is 10 bit shifts

  #define SS1 7 
  #define SS2 8
  #define PWM_PIN 9
  #define PASSIVE 6
  #define NSTBY_1 4
  #define NSTBY_2 5
  #define NIRQ_1 2
  #define NIRQ_2 3


  #define x0_TEMP 0x00
  #define x1_RECHARGE_COUNT 0x01
  #define x9_TARGET 0x09
  #define xA_THRESHOLD 0x0A
  #define xB_RECHARGE_WINDOW 0x0B
  #define xC_CHARGE_SENS 0x0C
  #define xD_RECHARGE_REF 0x0D
  #define xE_NIRQ_ENGATE 0x0E

  #define CK_FREQ 31250.0f // depends on the settings of the PWM
  #define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES)

// debug definitions end 


//--------------------------------------------------------------------
// Useful masks
// WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF
//--------------------------------------------------------------------
// Function definitions
unsigned int read_reg(byte sensor, byte reg);                               
bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq);    
void fgdos_init(byte sensor); // for debug
void write_reg(byte sensor, byte reg, byte data); // for debug

//--------------------------------------------------------------------
// variables
long int sens_freq ;
long int ref_freq ;
//--------------------------------------------------------------------
// Main setup
void setup() {
  Serial.begin(9600); // 9600
  while(!Serial);
  
  // SPI setup
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0)); // 32000
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);
   
   /* 
   * --------------- PWM GENERATION ------------------------
   * Set the registers so the PWM is generated at the desired frequency. For more details see either
   * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
   * Atmega32p datasheet
   * Current mode is: Fast PWM Mode with OCRA top
   * _BV(x) shifts bits x times to the left and sets this one to 1
   */
   
  /* 
  pinMode(PWM_PIN, OUTPUT); 
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR2A = 63; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR2B = 31; // duty cycle = OCR2B+1 / OCR2A+1
  */
  pinMode(PWM_PIN, OUTPUT); // Set PWM PIN on Arduino
  TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1

  fgdos_init(SS2); // for debug
 
}


//--------------------------------------------------------------------
// Main loop
void loop() {

   
  
  // read the temperature and frequency registers
  Serial.print("Temperature (sensor 1445 offset = 8): "); Serial.println(read_reg(SS2,0x00),HEX);
  delay(1000);

  //  Checking register values - for debugging

  Serial.println(read_reg(SS2,0x00),BIN);
  Serial.println(read_reg(SS2,0x01),BIN);
  Serial.println(read_reg(SS2,0x02),BIN);
  Serial.println(read_reg(SS2,0x03), BIN);     //
  Serial.println(read_reg(SS2,0x04), BIN);
  Serial.println(read_reg(SS2,0x05), BIN);
  Serial.println(read_reg(SS2,0x06), BIN);
  Serial.println(read_reg(SS2,0x07), BIN);
  Serial.println(read_reg(SS2,0x08), BIN);
  Serial.println(read_reg(SS2,0x09), BIN);
  Serial.println(read_reg(SS2,0x0A), BIN);
//  Serial.println(read_reg(SS2,0x0B), BIN);
//  Serial.println(read_reg(SS2,0x0C), BIN);
//  Serial.println(read_reg(SS2,0x0D), BIN);
//  Serial.println(read_reg(SS2,0x0E), BIN);
//  Serial.println(read_reg(SS2,0x10), BIN);
//  Serial.println(read_reg(SS2,0x11), BIN);
//  Serial.println(read_reg(SS2,0x13), BIN);
//  Serial.println(read_reg(SS2,0x14), BIN);

  
  //Serial.println(read_reg(SS2,0x06));           //
  //Serial.println(read_reg(SS2,0x01));            //


  // end debug
  
  if(!collect_freq(SS2,&sens_freq,&ref_freq)){   
    Serial.println("reading failed, timeout");
  }
  Serial.print("Sensor and Reference Frequencies : ");
  Serial.print(sens_freq); Serial.print(" | ");
  Serial.print(ref_freq); Serial.println(" | ");
  
  delay(5000);
}


//--------------------------------------------------------------------
// FUNCTIONS

/* 
 * Fuction to read a registry from the sensor.  
 * first send the address, then read what is being send back
 * apparently the automatic register incrementation (necessary to read more than one register with a single request) 
 * doesn't work properly. Reads have been limited to one reg at a time. (note from CERN)
 */
                                                  //for debug
unsigned int read_reg(byte sensor, byte reg){
  byte data = 0;
  byte adr = reg|RD;
  
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  delayMicroseconds(10);
  data = SPI.transfer(0x00); 
  digitalWrite(sensor, HIGH);

  return data;
}

bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck

  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(SS2,0x08);
      freq_reg[1] = read_reg(SS2,0x07);
      freq_reg[2] = read_reg(SS2,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = ((long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK;
      have_sens_freq = true;
    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(SS2,0x05);
      freq_reg[1] = read_reg(SS2,0x04);
      freq_reg[2] = read_reg(SS2,0x03);
      *ref_freq = ((long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK;
      have_ref_freq = true;
    }
    if (millis()-time_start>1000){
      return false;
    }
  }

//  Serial.print("Reference frequency registers (0x05,0x04,0x03): "); 
//  Serial.print(freq_reg[0],BIN); Serial.print(" | ");
//  Serial.print(freq_reg[1],BIN); Serial.print(" | ");
//  Serial.print(freq_reg[2],BIN); Serial.println(" | ");
//  Serial.print("Frequency reconstructed: "); Serial.println(*ref_freq,BIN); Serial.print(" | "); Serial.println(*ref_freq);

  return true;
}


// for debug

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



void fgdos_init(byte sensor) {
  Serial.print("SENSOR "); Serial.println(sensor-6);
  write_reg(sensor,xB_RECHARGE_WINDOW,xB_settings_a); 
  Serial.print("window_factor "); Serial.println(WINDOW_FACTOR);
  write_reg(sensor,xC_CHARGE_SENS,xC_settings);
  Serial.print("sensitivity: "); Serial.println(SENS);
  write_reg(sensor,xE_NIRQ_ENGATE,xE_settings);
  write_reg(sensor,xD_RECHARGE_REF,xD_settings_off);
  // wait for reference to stabilise, eg 4 measurement windows (= 4*32768 pulses at 31.25 kHz = 4194 millisecs) (only stated in FGD_02F)
  delay(4/WINDOW_FACTOR*1000);                                                                                 
  // set target and threshold registers by selecting 5 MSBs (/8192) and set them to approprirate registers
  // do not forget to apply window factor!
  // TDIV is set to 1 for higher resilution, so do /1024 instead, watch out though, you are not really using 8 bits to compare,
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
  delay(2.1/WINDOW_FACTOR*1000);
}  // for debug
