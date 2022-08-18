// RADIATION PAYLOAD FIRMWARE V1.1
// By: A. Shanbhag


/*
 * 
 * 17/07: Script created for Perfomance testing of the Radiation Payload MAIN BOARD V1 under irradiation
 *        
 *        
 *        USEFUL RESOURCES:
 *        User Guide for MSP430FR59xx devices - http://www.ti.com/lit/pdf/slau367
 *        MSP430FR5969 Datasheet for pin functions and register descriptions - https://www.ti.com/lit/ds/symlink/msp430fr5969.pdf?HQS=dis-mous-null-mousermode-dsf-pf-null-wwe&ts=1642351946587&ref_url=https%253A%252F%252Fnl.mouser.com%252F 
 *        MSP430 driver lib library User Guide - https://dev.ti.com/tirex/explore/node?node=AANMz9MCP3TctaSCJuEa0Q__IOGqZri__LATEST 
 *        
 *        
 * 
*/

//------------------------------------- LIBRARIES & MACROS -------------------------------------------------
#include <SPI.h>
#include <driverlib.h> 
#include "FGD_03F_MSP430.h"                                                                             


//------------------------------------- FUNCTION DEFINITIONS -------------------------------------------------
// Function definitions



//------------------------------------------ VARIABLES -------------------------------------------------
unsigned long int payload_on_time = 0;

unsigned long int sens_freq_1 = 0;
unsigned long int sens_freq_2 = 0;

unsigned long int ref_freq_1 = 0;
unsigned long int ref_freq_2 = 0;

unsigned long int target_freq = 0;
unsigned long int threshold_freq = 0;

int temperature_1 = 0;
int temperature_2 = 0;

int recharge_count_1 = 0;
int recharge_count_2 = 0;


char command;
bool flag_isr = FGD_ISR;                                                                          // $$ check if this is needed

byte i = 0, j = 0;                                                                                // $$ check if this is needed
bool flag = true;                                                                                 // $$ check if this is needed


//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {

  // setting RS485 pins ~RE and DE low to start listening only over RS485 Bus. Uses P2.6 on MCU                                
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
  
  //setting FRAM CS pin high to prevent output. Uses P3.6 on MCU
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);

  // setting NSTBY pins high using driverlib GPIO function
  // Internal FGDOS NSTBY pins
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3);
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);

  // WCK signal generation - Page 92 - https://dev.ti.com/tirex/explore/node?node=AANMz9MCP3TctaSCJuEa0Q__IOGqZri__LATEST 
  pinMode(WCK_1, OUTPUT);  // set pin direction to O/P                                                            // Try P3DIR |= 0x10 ; if this does not work
  P3SEL1 |= 0x10 ;         //Set PxSEL registers of P3. P3SEL1.4 is set to 1 to select SMCLK as the pin function



  // Start serial communication 
  Serial.begin(9600);                                                                           //250000
  while(!Serial);

  
  // SPI setup
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));                                                         
  SPI.begin();

  // Set Internal FGDOS CS pins to output mode and high
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);

  __delay_cycles(1000000);                                                                                                  //$$






  
  
  // --------------- SENSOR SETUP ------------------------
  // set up the sensors basic properties, see function for more details
  // first wait for the WCK to settle, this is necessary as it is used for window!!!!
  fgdos_init(SS1);
  __delay_cycles(1000000);  
  fgdos_init(SS2);
  __delay_cycles(1000000);  
                                                                                                 //$$

/*
 * // ISR upon new data, interrupt request data ready, active low
  // Arduino pullup used and sensor set to open collector (can only pull low)
  if (flag_isr){
    pinMode(NIRQ_1, INPUT_PULLUP); 
    pinMode(NIRQ_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);                                                    // $$ change this to extended
    attachInterrupt(digitalPinToInterrupt(NIRQ_2), collect_data_SS2, LOW);                                                    // $$ change this to extended
  }

  // Read all registers of the sensors so they can be double checked later if needed
  read_all_registers(SS1);
  read_all_registers(SS2);
 * 
 * 
 */


                                                                                                 
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  //update Payload ON Time
  payload_on_time = millis();

  // Set SMCLK to WCK = 31250 Hz
  Clock_set_readout();

  __delay_cycles(1000000);                                                                 // to let the SMCLK settle


  
  // Sensor 1
  
  // read the temperature, recharge count and frequency registers
  temperature_1 = read_reg(SS1,x0_TEMP);                                                       // temperature = read_reg(SS2,TEMP) - 87; the temperature read is witout offset
  recharge_count_1 = read_reg(SS1,x1_RECHARGE_COUNT);
  recharge_count_1 = recharge_count_1 & 0x0F;

  // collect freq from sensor 1 
  collect_freq(SS1,&sens_freq_1,&ref_freq_1, WINDOW_FACTOR);

  
  // Sensor 2
  
  // read the temperature, recharge count and frequency registers
  temperature_2 = read_reg(SS2,x0_TEMP);                                                       
  recharge_count_2 = read_reg(SS2,x1_RECHARGE_COUNT);
  recharge_count_2 = recharge_count_2 & 0x0F;

  // collect freq from sensor 2 
  collect_freq(SS2,&sens_freq_2,&ref_freq_2, WINDOW_FACTOR);


   // reset SMCLK to 8 MHz
   Clock_reset();
   //delay(10);
   __delay_cycles(100000);


  //Print Sensor 1 & 2 measurments to Serial Monitor
  print_meas_short(temperature_1,sens_freq_1,ref_freq_1,recharge_count_1,1);
  print_meas_short(temperature_2,sens_freq_2,ref_freq_2,recharge_count_2,2);

//  Serial.print("Payload ON Time: ");Serial.println(payload_on_time);

   
   __delay_cycles(1000000);                                                       // to let the SMCLK settle
  
}




//-----------------------------------------------------------------------------------------------------------------------
// FUNCTIONS
