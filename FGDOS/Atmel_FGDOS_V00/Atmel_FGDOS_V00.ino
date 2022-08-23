/*
 * Added I2C libraries, ISRs and communication
 *  - 11 bytes are send upoon send request
 *  - the collect_freq function has been changed to store freq registers in the I2C data to send array
 *  - upon I2C receive request the register and data are stored and written to FGDOS
 */
//-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
#include <Wire.h>
// header file where libraries, constants, masks, pins, etc are defined
#include "definitions.h"

// variables
unsigned long int sens_freq = 0;
unsigned long int ref_freq = 0;
unsigned long int data32;
int temperature, recharge_count, data16;
byte i = 0, reg, data8;
bool flag = true, flag_receive = false, flag_request = false;
byte I2C_in[3], I2C_out[BYTES_SEND-1];

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  Serial.begin(9600);
  while(!Serial);

   /* 
   * --------------- PWM GENERATION ------------------------
   * Set the registers so the PWM is generated at the desired frequency. For more details see either
   * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
   * Atmega32p datasheet
   * Current mode is: Fast PWM Mode with OCRA top
   * _BV(x) shifts bits x times to the left and sets this one to 1
   */
  pinMode(3, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR2A = 63; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR2B = 31; // duty cycle = OCR2B+1 / OCR2A+1
  
  // --------------- SPI SETUP ------------------------
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);

  // --------------- I2C SETUP ------------------------
  Wire.begin(0x25); // set slave address
  //Wire.setClock(1000000);
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent);

  // --------------- SENSOR SETUP ------------------------
  // set up the sensors basic properties, see function for more details
  // first wait for the PWM to settle, this is necessary as it is used for window!!!!
  fgdos_init(SS2);
  //fgdos_init(SS1); // for sensor 1 some miso and mosi lines are still connected...
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  if (flag_receive == true){
    reg = I2C_in[0];
    switch(i){
    case 2:
      data8 = I2C_in[1];
      write_reg(SS2,reg,data8);
      Serial.print("I2C RECEIVE | REG(HEX) : "); Serial.print(reg,HEX); Serial.print(" | DATA(BIN) : "); Serial.println(data8,BIN);
      break;
    case 3:
      data16 = I2C_in[1] << 8 | I2C_in[2];
      break;
    case 5:
      data32 = ((I2C_in[1] << 8 | I2C_in[2]) << 16) | (I2C_in[3] << 8 | I2C_in[4]);
    default:
      Serial.println("data type not recognised...");
    }
    flag_receive = false;
  }
  if (flag_request == true){
    Serial.print("I2C REQUEST | DATA SEND : ");
    print_meas_short(temperature,sens_freq,ref_freq,recharge_count,2);
    flag_request = false;
  }
  
//  Sensor 2
//  read the temperature, recharge count and frequency registers
//  temperature = read_reg(SS2,TEMP) - 87;
//  recharge_count = read_reg(SS2,RECHARGE_COUNT);
//  recharge_count = recharge_count & 0x0F;
//  collect_freq(SS2,&sens_freq,&ref_freq);
//  print_meas_full(temperature,sens_freq,ref_freq,2);

  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  //wait(2/WINDOW_FACTOR*1000);
  // extra delay to prevent too much output
  //delay(1000);
}
