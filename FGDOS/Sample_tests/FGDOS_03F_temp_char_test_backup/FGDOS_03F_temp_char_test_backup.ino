// Originally: FGDOS_03F_test
// 12/04/22: Adapted into script for temperature characterization test for FGD_03F

/* Test Setup:
 *  FGD_03F breakout and breadboard from radiation tests, with MOSFET connections removed
 *  FGD_03F QFN package
 *  FGD_03F DIL package
 *  Arduino Una R3
 *  
 *  SAM V71 with INA219 sensor breakouts
 * 
 *  E Type thermocouple with MCP9600 breakout
 *  connected with Arduino over i2c, supplied with 5V from Breadboard power supply
 */



//-----------------------------------------------------------------------------------------------------------------------



//---------------------------------------------- MCP9600 libraries, definitions & variables ----------------------------------------------------

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS (0x67)

Adafruit_MCP9600 mcp;

//variables
float tcouple_temp_hot, tcouple_temp_cold ;          // hot and cold junction temperatures, without offset
int tcouple_adc ;






//---------------------------------------------- FGDOS libraries, definitions & variables -------------------------------------------------------

#include <SPI.h>

// Micro pins
// pins for slave selection
#define SS1 7 // 9
#define SS2 8 // 

#define NSTBY_1 4
#define NSTBY_2 5
#define PWM_PIN 9


// Useful masks
// WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF

// Register definitions
#define TEMP 0x00
#define RECHARGE_COUNT 0x01
#define TARGET 0x09
#define THRESHOLD 0x0A
#define RECHARGE_WINDOW 0x0B                                                                         
#define CHARGE_SENS 0x0C
#define RECHARGE_REF 0x0D                                                                          
#define NIRQ_ENGATE 0x0E                                                                        // NIRQ pin not used for now


// handy constant definitions (these depend on settings in Arduino and sensor!)
#define THRESHOLD_FREQ 50000 // these values are not exact! Since only 8 MSBs are used          
#define TARGET_FREQ 90000                                                                       
#define CK_FREQ 31250.0f // depends on the settings of the PWM                                  //31250
#define WINDOW_PULSES 4096 // depends on window register settings!                              //8192.0f
#define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES)

// Function definitions
unsigned int read_reg(byte sensor, byte reg);
void write_reg(byte sensor, byte reg, byte data);
bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq);
void fgdos_init(byte sensor);
void wait(int microsecs);
void print_freq(byte freq_reg[3],unsigned long int freq_value,char f_type);
void print_meas_full(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor);
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, int recharge_count, byte sensor, float tcouple_temp_hot, float tcouple_temp_cold, int tcouple_adc);

// variables
unsigned long int sens_freq = 0;
unsigned long int ref_freq = 0;
int temperature, recharge_count;
byte i = 0, j = 0;
bool flag = true;





//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP



void setup() {
  Serial.begin(250000);                                                                           // reduce from 250000 for MCP9600 ?
  while(!Serial);

  // ------------------------------ MCP9600 setup ---------------------------------------------

  Serial.println("MCP9600 Thermocouple Amplifier Setup");

    /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
    if (! mcp.begin(I2C_ADDRESS)) {
        Serial.println("Sensor not found. Check wiring!");
        while (1);
    }

  Serial.println("Found MCP9600!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);                          // Set ADC Resolution here
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_E);                                  // Change thermocouple type here
  Serial.print("Thermocouple type set to ");
  switch (mcp.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp.setFilterCoefficient(3);                                              // filter coefficient
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp.getFilterCoefficient());

  mcp.setAlertTemperature(1, 30);                                           // Alert temperature
  Serial.print("Alert #1 temperature set to ");
  Serial.println(mcp.getAlertTemperature(1));
  mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp.enable(true);

  Serial.println(F("------------------------------"));




  

  //--------------------------------FGDOS setup -------------------------------------------------

  //Serial.println("FGDOS FGD_03F Dual Sensor Setup - Temperature Characterization");
   /* 
   * --------------- PWM GENERATION ------------------------
   * Set the registers so the PWM is generated at the desired frequency. For more details see either
   * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
   * Atmega32p datasheet
   * Current mode is: Fast PWM Mode with OCRA top
   * _BV(x) shifts bits x times to the left and sets this one to 1
   */
                                                               
  pinMode(PWM_PIN, OUTPUT); // Set PWM PIN on Arduino
  TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1

  // Set NSTBY pins to high to prevent garbage output 
  pinMode(NSTBY_1,OUTPUT);
  digitalWrite(NSTBY_1,HIGH);
  pinMode(NSTBY_2,OUTPUT);
  digitalWrite(NSTBY_2,HIGH);
  
  // SPI setup
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  //pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);

                                                                                          // wait 
  // --------------- SENSOR SETUP ------------------------
  
  // first wait for the PWM to settle, this is necessary as it is used for window!!!!
  delay(1000);

  // set up the sensors basic properties, see function for more details
  fgdos_init(SS1);
  fgdos_init(SS2); 
}






//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {


  //---------------------------------------- MCP9600 loop code ---------------------------------

  /*
  Serial.print("Hot Junction: "); Serial.println(mcp.readThermocouple());
  Serial.print("Cold Junction: "); Serial.println(mcp.readAmbient());
  Serial.print("ADC: "); Serial.print(mcp.readADC() * 2); Serial.println(" uV");
  */

  tcouple_temp_hot = mcp.readThermocouple();
  tcouple_temp_cold = mcp.readAmbient() ;          // hot and cold junction temperatures, without offset
  tcouple_adc = (mcp.readADC() * 2) ;
  

  //---------------------------------------- FGDOS loop code -----------------------------------

  // Sensor 2
  // read the temperature, recharge count and frequency registers
  temperature = read_reg(SS2,TEMP);                                                       // temperature = read_reg(SS2,TEMP) - 87; offset can be set here
  recharge_count = read_reg(SS2,RECHARGE_COUNT);
  recharge_count = recharge_count & 0x0F;
  
  // collect freq from sensor 1 and print along with thermocouple measurements
  collect_freq(SS1,&sens_freq,&ref_freq);
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,1,tcouple_temp_hot, tcouple_temp_cold, tcouple_adc);

  // collect freq from sensor 2 and print along with thermocouple measurements
  collect_freq(SS2,&sens_freq,&ref_freq);
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,2,tcouple_temp_hot, tcouple_temp_cold, tcouple_adc);

  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  delay(2/WINDOW_FACTOR*1000);                                                             
  // extra delay to prevent too much output
  delay(1000);                                                                                
}









//-------------------------------------- FUNCTION DEFINITIONS -------------------------------------------------------
// 

/* 
 * Fuction to read a registry from the sensor.  
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

// write to a register
void write_reg(byte sensor, byte reg, byte data){
  byte adr = reg|WR;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  SPI.transfer(data);
  digitalWrite(sensor, HIGH);
}

// read the frequency registers and reconstruct their values
// frequency = registers / window pulses amount * ck frequency
bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck
  int x = 1000;// increase amount of time to wait if serial.prints are used! Otherwise set to 1
  
  // first check to see if no recharge is going on
  if((read_reg(sensor,RECHARGE_COUNT) & 0x80) == 0x80){                                           // RCHEV bit = 1 when recharge is in progress
    Serial.println("recharge in progress... BREAK"); 
    return false;
  }
  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(sensor,0x08);
      freq_reg[1] = read_reg(sensor,0x07);
      freq_reg[2] = read_reg(sensor,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK );
      have_sens_freq = true;
      // print_freq(freq_reg,*sens_freq,'S');                                                     // to stop printing registers
      *sens_freq = *sens_freq * WINDOW_FACTOR;
    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(sensor,0x05);
      freq_reg[1] = read_reg(sensor,0x04);
      freq_reg[2] = read_reg(sensor,0x03);
      *ref_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK );
      have_ref_freq = true;
      //print_freq(freq_reg,*ref_freq,'R');
      *ref_freq = *ref_freq * WINDOW_FACTOR;
    }
    if ((millis()-time_start>(4*1/WINDOW_FACTOR*x) && !have_ref_freq && !have_sens_freq)){
      Serial.println("reading failed, timeout");
      return false;
    }
  }
  return true;
}

void fgdos_init(byte sensor){
  Serial.print("SENSOR "); Serial.println(abs(sensor-6));   
  // set the reference oscillator and window measurement amount of pulses settings
  // bits (6:4) for ref and (3:2) for window (bit counting from lsb to msb)
  // reference set to 100 (= ??) and windows set to 10 (8192 pulses)(00=32768 ck pulses per window)
  write_reg(sensor,RECHARGE_WINDOW,0xCD);                                                                          // write_reg(sensor,RECHARGE_WINDOW,0x48);
  Serial.print("window_factor "); Serial.println(WINDOW_FACTOR);
  // manual recharge off and sesitivity to low
  // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
  write_reg(sensor,CHARGE_SENS,0x79);
  Serial.println("sensitivity high");
  // no new data avialable signal via nirq pin (mnrev bit 6), nirq setting to push-pull (nirqoc bit 1), 
  // measurement window to count clocks (engate bit 0)
  write_reg(sensor,NIRQ_ENGATE,0x06);
  // disconnect recharging system before setting targets
  write_reg(sensor,RECHARGE_REF,0x04);
  // wait for reference to stabilise, eg 4 measurement windows (= 4*32768 pulses at 31.25 kHz = 4194 millisecs)
  delay(4/WINDOW_FACTOR*1000); // 4194                                                                             // wait(4/WINDOW_FACTOR*1000);
  // read reference register ,select 8 MSBs and set them to target register
  // or just pick 50 kHz advised from Sealicon
  //unsigned long int target = (((read_reg(sensor,0x05) << 8 | read_reg(sensor,0x04) ) <<8 | read_reg(sensor,0x03) ) & FREQ_MASK );
  //byte reg_target = (target & 0x3FC00) >> 10;
  byte reg_target = floor(TARGET_FREQ/WINDOW_FACTOR/1024); // dividing by 1023 is bitshifting by 10 ;)             //byte reg_target = round(TARGET_FREQ/WINDOW_FACTOR/1023);
  write_reg(sensor,TARGET,reg_target);
  Serial.print("target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println((reg_target << 10)*WINDOW_FACTOR);
  //Serial.print("-registry "); Serial.println(reg_target,BIN);
  // set threshold to 30 kHz equivalent (0x1D = 29, when converted to 8 MSBs of sens_freq (18 bit) = 29696)
  // do not forget to apply window factor! (lower alternative: 0x08 = 8192)
  byte reg_threshold = floor(THRESHOLD_FREQ/WINDOW_FACTOR/1024);                                                   //byte reg_threshold = round(THRESHOLD_FREQ/WINDOW_FACTOR/1023);
  write_reg(sensor,THRESHOLD,reg_threshold);
  Serial.print("threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println((reg_threshold << 10)*WINDOW_FACTOR);
  //Serial.print("-registry "); Serial.println(reg_threshold, BIN);
  // enable interrupt upon new data (for now no interrupt pin is actually being used
  //write_reg(sensor,NIRQ_ENGATE,0x30);                                                                            // 
  // also enable automatic recharging again
  write_reg(sensor,RECHARGE_REF,0x44);
  Serial.println();
  delay(1000);
}

void wait(int millisecs){
  long int t1 = millis();
  while(millis()-t1<millisecs);
}

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
  Serial.print("SENSOR "); Serial.println(sensor);
  Serial.print("\t temperature (sensor 1445 offset = 87): "); Serial.println(temperature);
  Serial.print("\t sensor and reference frequencies : ");
  Serial.print(sens_freq); Serial.print(" | ");
  Serial.print(ref_freq); Serial.println(" | ");
  }
  
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, int recharge_count, byte sensor, float tcouple_temp_hot, float tcouple_temp_cold, int tcouple_adc){
  flag ? Serial.println("Sensor, Temp , F_sens , F_ref, Rech_Count, Thermocouple_hot, Thermocouple_cold, Thermocouple_ADC"),flag=false:0;
  Serial.print(sensor);Serial.print(" , ");
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.print(ref_freq);Serial.print(" , ");
  Serial.print(recharge_count);Serial.print(" , ");
  Serial.print(tcouple_temp_hot);Serial.print(" , ");
  Serial.print(tcouple_temp_cold);Serial.print(" , ");
  Serial.println(tcouple_adc);
  
  }
