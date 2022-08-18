/* Arduino script for the Payload Performance Test under irradaition
 * 
 * - ina219 - i2c
 * - Si5351 osc - i2c
 * - Arduino PWM (as backup WCK)
 * - relays
 *    - OSC
 *    - 5V supply
 *    - SBW - TDIO, TCK, RX, TX
 *    - Arduino PWM WCK
 *    
 * - serial commands for relay ops
 * - print ina219 readings and relay states over serial
 * 
 */


////////////////////// LIBRARIES, HEADER FILES & MACROS ////////////////////

#include <Adafruit_SI5351.h>                            // OSC library
Adafruit_SI5351 clockgen = Adafruit_SI5351();           // OSC inititalization

//----------------------------- INA219 libraries & Macros -------------------------------------------

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

//----------------------------- Relay libraries & Macros ----------------------------------------------



// Relay control pins
#define RELAY_OSC A0                                     // NO                   // $$ Check pin numbers !!!
                                              
#define RELAY_PAYLOAD_SUPPLY A1                          // NC
#define RELAY_SBW A2                                // NC
//#define RELAY_SBW_TCK A3                                 // NC
//#define RELAY_SBW_RX 6                                  // NC
//#define RELAY_SBW_TX 7                                  // NC

#define RELAY_PWM_PIN A3                                 // NO      


//---------------------------------------------------------------------------

// WCK signal pin from Arduino
#define PWM_PIN 9






/////////////////////////////// FUNCTION DEFINITIONS & VARIABLES ////////////////////////////////




//------------------------------- INA219 --------------------------------------------

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

//-------------------------------- RELAYS -------------------------------------------

//int relay_number = 0;                                                             // to control relay by pin number

// flags to remember relay ON or OFF states                                       // false indicates unactuated state
bool osc = false;                                                                 // true indicates actuated state - true on an NO connection implies the connection is closed
bool payload_supply = false;
bool sbw = false;
//bool sbw_tck = false;
//bool sbw_rx = false;
//bool sbw_tx = false;
bool pwm_wck = false;




//-------------------------------- COMMANDS ------------------------------------------

char command ;

char get_command();
void set_command(char command);





/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{

////////////////////////// RELAY SETUP ////////////////////////////////////////

  // setting relay control pins to O/P mode
  pinMode(RELAY_OSC, OUTPUT);
  pinMode(RELAY_PAYLOAD_SUPPLY, OUTPUT);
  pinMode(RELAY_SBW, OUTPUT);
//  pinMode(RELAY_SBW_TCK, OUTPUT);
//  pinMode(RELAY_SBW_RX, OUTPUT);
//  pinMode(RELAY_SBW_TX, OUTPUT);
  pinMode(RELAY_PWM_PIN, OUTPUT);

  // setting all relay control pins to HIGH to prevent accidental relay actuation on start-up
  digitalWrite(RELAY_OSC, HIGH);
  digitalWrite(RELAY_PAYLOAD_SUPPLY, HIGH);
  digitalWrite(RELAY_SBW, HIGH);
//  digitalWrite(RELAY_SBW_TCK, HIGH);
//  digitalWrite(RELAY_SBW_RX, HIGH);
//  digitalWrite(RELAY_SBW_TX, HIGH);
  digitalWrite(RELAY_PWM_PIN, HIGH);





/////////////////////////// INA219 POWER MONITOR SETUP ///////////////////////////////////////
  
  
  uint32_t currentFrequency;
    
  Serial.println("Payload Performance Test: Irradiation");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();

  Serial.println("Measuring Radiation Payload PCB voltage and current with INA219 ...");



/////////////////////////// OSC SETUP ///////////////////////////////////////
  
  Serial.begin(9600);
  Serial.println("Si5351 Clockgen Setup"); Serial.println("");

  /* Initialise the sensor */
  if (clockgen.begin() != ERROR_NONE)
  {
    /* There was a problem detecting the IC ... check your connections */
    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("OK!");

  /* INTEGER ONLY MODE --> most accurate output */
  /* Setup PLLA to integer only mode @ 900MHz (must be 600..900MHz) */
  /* Set Multisynth 0 to 112.5MHz using integer only mode (div by 4/6/8) */
  /* 25MHz * 36 = 900 MHz, then 900 MHz / 8 = 112.5 MHz */
  Serial.println("Set PLLA to 900MHz");
  clockgen.setupPLLInt(SI5351_PLL_A, 36);
  Serial.println("Set Output #0 to 112.5MHz");
  clockgen.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8);

  /* FRACTIONAL MODE --> More flexible but introduce clock jitter */
  /* Setup PLLB to fractional mode @616.66667MHz (XTAL * 24 + 2/3) */
  /* Setup Multisynth 1 to 13.55311MHz (PLLB/45.5) */
  clockgen.setupPLL(SI5351_PLL_B, 16, 77722, 100000);
  Serial.println("Set Output #1 to 838.8608 kHz");
  clockgen.setupMultisynth(1, SI5351_PLL_B, 500, 0, 1);

  /* Multisynth 2 is not yet used and won't be enabled, but can be */
  /* Use PLLB @ 616.66667MHz, then divide by 900 -> 685.185 KHz */
  /* then divide by 64 for 10.706 KHz */
  /* configured using either PLL in either integer or fractional mode */

  Serial.println("Set Output #2 to 32.768 KHz");
  clockgen.setupMultisynth(2, SI5351_PLL_B, 100, 0, 1);
  clockgen.setupRdiv(2, SI5351_R_DIV_128);

  /* Enable the clocks */
  clockgen.enableOutputs(true);


////////////////////////////// PWM WCK SETUP///////////////////////////////////////////////////

  // WCK signal generation on Pin 9 of Arduino - 31250 kHz - to be used as backup
  pinMode(PWM_PIN, OUTPUT); // Set PWM PIN on Arduino
  TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1

  Serial.println("31250 kHz on Arduino Pin 9");



//----------------------------- DATA FIELDS ----------------------------------------------------


  Serial.print("Bus Voltage (V), ");
  Serial.print("Shunt Voltage (mV), "); 
  Serial.print("Load Voltage (V), "); 
  Serial.print("Current (mA), "); 
  Serial.print("Power (mW), ");
  Serial.print("RELAY_OSC , ");
  Serial.print("RELAY_PAYLOAD_SUPPLY, ");
  Serial.print("RELAY_SBW, ");
//  Serial.print("RELAY_SBW_TCK, ");
//  Serial.print("RELAY_SBW_RX, ");
//  Serial.print("RELAY_SBW_TX, ");
  Serial.println("RELAY_PWM_PIN ");
  Serial.println(" ");
  
  
  // delay to let everything settle before readout
  delay(1000);
  
}






/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{


//----------------------- INA219 ----------------------------------------------------

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);




//----------------------- RELAYS ----------------------------------------------------






//----------------------- SERIAL PRINT ----------------------------------------------------
  

  Serial.print(busvoltage); Serial.print(" , ");
  Serial.print(shuntvoltage); Serial.print(" , ");
  Serial.print(loadvoltage); Serial.print(" , ");
  Serial.print(current_mA); Serial.print(" , ");
  Serial.print(power_mW); Serial.print(" , ");
  Serial.print(osc, DEC); Serial.print(" , ");
  Serial.print(payload_supply, DEC); Serial.print(" , ");
  Serial.print(sbw, DEC); Serial.print(" , ");
//  Serial.print(sbw_tck, DEC); Serial.print(" , ");
//  Serial.print(sbw_rx, DEC); Serial.print(" , ");
//  Serial.print(sbw_tx, DEC); Serial.print(" , ");
  Serial.println(pwm_wck, DEC);

  

  // Command receiveal and execution

  command = get_command();
  set_command(command);

  delay(250);


  
}


/**************************************************************************/
/*
    FUNCTION DEFINITIONS
*/
/**************************************************************************/



char get_command(){
  String command_str;
  while(Serial.available()){
      command_str = Serial.readString();
    }
    return command_str[0];                                                          
  
}



void set_command(char command){

  switch (command){
      case '3':
        digitalWrite(RELAY_OSC, HIGH);                                                  // relay is in default state when the control pin is HIGH
        osc = false;
        break;
      case '#':
        digitalWrite(RELAY_OSC, LOW);                                                  // relay is thrown when the control pin is LOW
        osc = true;
        break;     
      case '4':
        digitalWrite(RELAY_PAYLOAD_SUPPLY, HIGH);                                                  
        payload_supply = false;
        break;
      case '$':
        digitalWrite(RELAY_PAYLOAD_SUPPLY, LOW);                                                  
        payload_supply = true;
        break;
      case '5':
        digitalWrite(RELAY_SBW, HIGH);                                                  
        sbw = false;
        break;
      case '%':
        digitalWrite(RELAY_SBW, LOW);                                                  
        sbw = true;
        break;    
//      case '5':
//        digitalWrite(RELAY_SBW_TCK, HIGH);                                                  
//        sbw_tck = false;
//        break;
//      case '%':
//        digitalWrite(RELAY_SBW_TCK, LOW);                                                  
//        sbw_tck = true;
//        break;
//      case '6':
//        digitalWrite(RELAY_SBW_RX, HIGH);                                                  
//        sbw_rx = false;
//        break;
//      case '^':
//        digitalWrite(RELAY_SBW_RX, LOW);                                                  
//        sbw_rx = true;
//        break;
//      case '7':
//        digitalWrite(RELAY_SBW_TX, HIGH);                                                  
//        sbw_tx = false;
//        break;
//      case '&':
//        digitalWrite(RELAY_SBW_TX, LOW);                                                  
//        sbw_tx = true;
//        break;
      case '6':
        digitalWrite(RELAY_PWM_PIN, HIGH);                                                  
        pwm_wck = false;
        break;
      case '^':
        digitalWrite(RELAY_PWM_PIN, LOW);                                                  
        pwm_wck = true;
        break;
   
      default:
        //Serial.println("Command not recognized. Please try again.");
        break;
    }

  
}
