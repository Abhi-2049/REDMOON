//-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
// header file where libraries, constants, masks, pins, etc are defined
#include "definitions.h"

// variables
unsigned long int sens_freq = 0;
unsigned long int ref_freq = 0;
unsigned long int data32;
int temperature, recharge_count, data16;
byte i = 0, reg, data8;
bool flag = true, flag_receive = false, flag_request = false;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  Serial.begin(9600); // higher for faster usart transfer, be careful not to lose data when transferring too fast   // 250000
  while(!Serial);

  // pin to enable/disble passive mode via MOSFET
  // pinMode(PASSIVE,OUTPUT);
  // digitalWrite(PASSIVE,LOW);

   /* 
   * --------------- PWM GENERATION ------------------------
   * Set the registers so the PWM is generated at the desired frequency. For more details see either
   * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
   * Atmega32p datasheet
   * Current mode is: Fast PWM Mode with OCRA top
   * _BV(x) shifts bits x times to the left and sets this one to 1
   */
   /* for pin 3
  pinMode(PWM_PIN, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR2A = 63; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR2B = 31; // duty cycle = OCR2B+1 / OCR2A+1
  */
     /* 
   * Pin 9 is used now, which uses timer 1. Timer 2 is needed for pin 3 (see Atmel_FGDOS code)
   * Refrain from using timer 0, it is used for Arduino functions
   */
  pinMode(PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1
  
  // --------------- SPI SETUP ------------------------
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));                      
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);

  // wait for everything to stabilise
  delay(100);

  // --------------- SENSOR SETUP ------------------------
  // set up the sensors basic properties, see function for more details
  // first wait for the PWM to settle, this is necessary as it is used for window!!!!
  fgdos_init(SS2);
  //fgdos_init(SS1); // for sensor 1 miso and mosi lines are still connected...
  
  
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  // SS2
  temperature = read_reg(SS2,TEMP);
  recharge_count = read_reg(SS2,RECHARGE_COUNT);
  collect_freq(SS2,&sens_freq,&ref_freq);
  
//  Sensor 2
//  read the temperature, recharge count and frequency registers
//  temperature = read_reg(SS2,TEMP) - 87;
//  recharge_count = read_reg(SS2,RECHARGE_COUNT);
//  recharge_count = recharge_count & 0x0F;
//  collect_freq(SS2,&sens_freq,&ref_freq);
//  print_meas_full(temperature,sens_freq,ref_freq,2);

  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  wait(2/WINDOW_FACTOR*1000);

  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,2);
}
