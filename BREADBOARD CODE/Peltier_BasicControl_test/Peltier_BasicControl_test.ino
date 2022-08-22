/* Script written to test basic control of a peltier module wih an arduino in open loop and closed loop configuration 
 * 
 * Control profiles:
 *  - Open loop
 *    - manual increase /decrease by serial commands
 *    - constant input / linear
 *    - linear 
 *    - sawtooth
 *    
 *   - Closed loop
 *    - constant temperature
 *    - staircase / stepped
 * 
 * 
 * Test using thermocouple-MCP9600 affixed to peltier for feedback based control
 */


//-------------------------------- MCP9600 - THERMOCOUPLE READOUT ------------------------------------------

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS (0x67)

Adafruit_MCP9600 mcp;



//----------------------------------------------------------------------------------------------------------






#define PELTIER_PIN 6                                // peltier input pin: The N-Channel MOSFET Gate is connected to this pin


//----------------------------------- VARIABLES -----------------------------------------------------

unsigned long int ON_time = 0;                       // for timing open loop control inputs
unsigned int step_time = 20;                          // amount of time to maintain current temperture / peltier input,  in [s]                               %^ MODIFY
unsigned int start_time = 0; 
unsigned int current_time = 0;

int pcb_temp = 0;                                    // temperature measured on the PCB - $$ REMEMBER to use proper offset
int temp_error = 0;                                  // temperature error variable for closed loop control
int temp_error_integral = 0;                         // temperature error variable for closed loop control
int temp_error_derivative = 0;
int temp_error_prev = 0;



int peltier_low_temp = 30;                            // temperature lower limit                                                                               %^ MODIFY
int peltier_high_temp = 50;                           // temperature upper limit                                                                               %^ MODIFY
int peltier_const_temp = 35;                          // constant temperature to be maintained                                                                 %^ MODIFY
int peltier_step_temp = 5;                           // temperature step height to be achieved,        range: [0,10]                                          %^  MODIFY
int target_temp = 0;                                 // the initial target to be achieved by the stepped profile
int peltier_linear_input = 40;                        // for passing heating input in linear profile,   range: [0,99]                                         %^  MODIFY


int power = 0;                                       // Power level from 0 to 99
int peltier_level = map(power, 0, 99, 0, 255);       // value mapped from power variable to [0 to 255]
int peltier_input = 0;                               // value from 0 to 255 that actually controls the MOSFET







//------------------------------ PELTIER CONTROL FUNCTIONS ----------------------------------------------------------


// flags to invoke functions                                            
bool flag_peltier_sawtooth_OL = false;                                                // false indicates command not received                                                                 
bool flag_peltier_constant_temp = false;
bool flag_peltier_stepped = false;
bool flag_peltier_sawtooth_CL = false;




// Open Loop (OL) control functions
void peltier_manual_inc();                                                            // increase the peltier input by 1 'unit'
void peltier_manual_dec();                                                            // reduce the peltier input by 1 'unit'
void peltier_linear(int peltier_linear_input);                                        // maintain a constant peltier input 
void peltier_sawtooth_OL(unsigned int step_time, int peltier_linear_input);           // maintain a constant input for 'step time' them remove it
void peltier_manual_min();                                                            // reduce the peltier input to the lowest setting (OFF) in 1 jump
void peltier_manual_max();                                                            // increase the peltier input to the highest setting in 1 jump



// Closed Loop (CL) control functions                                                 
void peltier_constant_temp(int peltier_const_temp);                                   // some combination of P, I and D control                                   
void peltier_stepped(int step_time, int peltier_step_temp, int peltier_high_temp);    // above with wait for step time, them increase by step_temp; repeat until upper limit is reached
void peltier_sawtooth_CL(int peltier_low_temp, int peltier_high_temp);                // sawtooth between upper and lower limit


//-------------------------------- COMMANDS ------------------------------------------

char command ;

char get_command();
void set_command(char command);



///////////////////////////////////////////////////////////////////////////////////////////

void setup(){

  Serial.begin(9600);

  


  //-------------------------------- MCP9600 - THERMOCOUPLE READOUT ------------------------------------------

  while (!Serial) {
      delay(10);
    }
    Serial.println("MCP9600 HW test");

    /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
    if (! mcp.begin(I2C_ADDRESS)) {
        Serial.println("Sensor not found. Check wiring!");
        while (1);
    }

  Serial.println("Found MCP9600!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);                          // Set ADC Resolution here - Set to 18 bits
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_E);                                  // Change thermocouple type here - Set to E Type
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

  mcp.enable(true);

  Serial.println("MCP9600 Configured Successfully");
  Serial.println(" ");






  //---------------------------------- PELTIER CONTROL ------------------------------------------------
  

  
  
  // Arduino used for controlling peltier via the n-MOSFET
  // setting peltier control pin to O/P mode and voltage to zero to prevent heat exchange
  pinMode(PELTIER_PIN, OUTPUT);
  analogWrite(PELTIER_PIN, 0);
  



}





void loop(){

  //-------------------------------- MCP9600 - THERMOCOUPLE READOUT ------------------------------------------

  pcb_temp = mcp.readThermocouple();
  Serial.print("Power="); 
  Serial.print(power);Serial.print(",");
  Serial.print(" PLevel=");
  Serial.print(peltier_input);Serial.print(",");
  Serial.print("Hot Junction: "); Serial.print(pcb_temp); Serial.print(",");                               //Serial.print("Hot Junction: "); Serial.println(mcp.readThermocouple());
  Serial.print("Cold Junction: "); Serial.print(mcp.readAmbient());Serial.print(",");
  Serial.print("ADC: "); Serial.print(mcp.readADC() * 2); Serial.println(" uV");




  //--------------------------------- PELTIER CONTROL -------------------------------------------------------

  ON_time = millis();    // time (in ms) for which the Arduino has been ON                                  

  // update PCB temperature
  // pcb_temp = mcp.readThermocouple();                                                                  // $$ complete the expression

/*
  Serial.print("Power=");
  Serial.print(power);
  Serial.print(" PLevel=");
  Serial.println(peltier_input);                            
*/

  // Command receiveal and execution

  command = get_command();
  set_command(command);

  
  // call repeating functions based on flag status

  if(flag_peltier_sawtooth_OL){
    peltier_sawtooth_OL(step_time, peltier_linear_input);
  }

  if(flag_peltier_constant_temp){
    peltier_constant_temp(peltier_const_temp);
  }

  if(flag_peltier_stepped){
    peltier_stepped(step_time, peltier_step_temp, peltier_high_temp);
  }

  if(flag_peltier_sawtooth_CL){
    peltier_sawtooth_CL(peltier_low_temp, peltier_high_temp);
  }


  delay(100);

}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------- OPEN LOOP CONTROL FUNCTIONS--------------------------------------------------------------------------

void peltier_manual_inc(){                                                              

   
  power += 5;
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);

  peltier_input = peltier_level;
  
  analogWrite(PELTIER_PIN, peltier_input);
  
}


void peltier_manual_dec(){                                                              

  power -= 5;
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);

  peltier_input = peltier_level;
  
  analogWrite(PELTIER_PIN, peltier_input);
  
}



void peltier_linear(int peltier_linear_input){

  power = peltier_linear_input;
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);

  peltier_input = peltier_level;
  
  analogWrite(PELTIER_PIN, peltier_input);

  
}



void peltier_sawtooth_OL(unsigned int step_time, int peltier_linear_input){

  power = peltier_linear_input;
  current_time = millis();
  
  if((current_time - start_time)>(step_time*1000)){     // if the step time is reached, turn OFF the peltier
    power = 0;
    current_time = 0;
    flag_peltier_sawtooth_OL = false;                   // set the flag false so that the function will not be called automatically again                                       
  }

  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);
  peltier_input = peltier_level;
  analogWrite(PELTIER_PIN, peltier_input);
  
}


void peltier_manual_min(){
  
  // reset flags to force override on other functions
  flag_peltier_sawtooth_OL = false;
  flag_peltier_constant_temp = false;
  flag_peltier_stepped = false;
  flag_peltier_sawtooth_CL = false;  
  
  power = 0;
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);
  peltier_input = peltier_level;
  analogWrite(PELTIER_PIN, peltier_input);
  
}

void peltier_manual_max(){

  // reset flags to force override on other functions
  flag_peltier_sawtooth_OL = false;
  flag_peltier_constant_temp = false;
  flag_peltier_stepped = false;
  flag_peltier_sawtooth_CL = false;

  
  power = 99;
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);
  peltier_input = peltier_level;
  analogWrite(PELTIER_PIN, peltier_input);
}








//-------------------------- CLOSED LOOP CONTROL FUNCTIONS--------------------------------------------------------------------------


void peltier_constant_temp(int peltier_const_temp){

  //power = 0;
  
  int k_p_gain = 1;                                                                                                   // $$ tune the proportional gain
  int k_i_gain = 1;                                                                                                   // $$ tune the integral gain
  int k_d_gain = 1; 
  temp_error = peltier_const_temp - pcb_temp ;
  temp_error_integral = temp_error_integral + temp_error;
  temp_error_derivative = temp_error - temp_error_prev;
  temp_error_prev = temp_error;
  
  if(abs(temp_error)>2){              // if the absolute value of temperature difference is greater than 5 deg C

    power = power + (temp_error*k_p_gain)+ (temp_error_derivative*k_d_gain);// + (temp_error_integral*k_i_gain) ;  // PID Control 
                                       
  }

  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);
  peltier_input = peltier_level;
  analogWrite(PELTIER_PIN, peltier_input);
                                    
}




void peltier_stepped(int step_time, int peltier_step_temp, int peltier_high_temp){

  //power = 0;
  current_time = millis();
  

  if(pcb_temp > peltier_high_temp){                     // if the highest step threshold has been surpassed, maintain that temperature
    peltier_const_temp = peltier_high_temp;
    flag_peltier_constant_temp = true;                  // use the closed loop constant temperature function by setting the flag high
  }
  
  else if((current_time - start_time)<(step_time*1000)){     // if the step time has not elapsed, keep the temperature constant
    peltier_const_temp = (target_temp-peltier_step_temp);    // current target = next target - step
    flag_peltier_constant_temp = true;                                 
  }

  else if(pcb_temp > (target_temp+2)){
    flag_peltier_constant_temp = false;
    target_temp = target_temp + peltier_step_temp ;    // update the next target temperature
    start_time = current_time ;                        // update the start time for the new step
    power = 0;
  }
                                           
  else{
    flag_peltier_constant_temp = false;
    power = peltier_linear_input;
         
  }
  
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);
  peltier_input = peltier_level;
  analogWrite(PELTIER_PIN, peltier_input);

}




void peltier_sawtooth_CL(int peltier_low_temp, int peltier_high_temp){

  //power = 0;
  
  if(pcb_temp > peltier_high_temp){              // if the temperature is higher than desired, turn OFF the peltier for natural cooling/heating
    power = 0;                          
  }
  if(pcb_temp < peltier_low_temp){
    power = peltier_linear_input;               // the heating/cooling happens at a predetermined rate
  }
  
  
  if(power > 99) power = 99;
  if(power < 0) power = 0;
  
  peltier_level = map(power, 0, 99, 0, 255);
  peltier_input = peltier_level;
  analogWrite(PELTIER_PIN, peltier_input);
  
}



//-------------------------- COMMANDS & EXECUTION--------------------------------------------------------------------------
char get_command(){
  String command_str;
  while(Serial.available()){
      command_str = Serial.readString();
    }
    return command_str[0];                                                          
  
}



void set_command(char command){

  switch (command){
      case 'q':
        peltier_manual_inc();                                                  // 
        break;
      case 'w':
        peltier_manual_dec();                                                  // 
        break;
      case 'Q':
        peltier_linear(peltier_linear_input);                                  // 
        break;
      case 'W':
        flag_peltier_sawtooth_OL = true;                                       // set flag true so that the function may be called in the main loop
        start_time = millis();                                          // mark the time when the function was called
        break;
      case 'e':
        peltier_manual_min();                                                  // 
        break;
      case 'E':
        peltier_manual_max();                                                  // 
        break;
     
      case 'z':
        flag_peltier_constant_temp = true;                                                  // 
        break;
      case 'x':
        flag_peltier_stepped = true;                                                  // 
        start_time = millis();
        target_temp = pcb_temp + peltier_step_temp;
        break; 
      case 'c':
        flag_peltier_sawtooth_CL = true;                                                  // 
        break;              
      
      default:
        //Serial.println("Command not recognized. Please try again.");
        break;
    }
  
}
