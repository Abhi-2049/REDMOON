/*
 * Arduino script for testing the Daughterboard PCBs under combined radiation and thermal input; FGDOS characterisation
 * 
 * 16/08: PC_FGDOS_03F_CR adapted for use  
 * 
 * 
 * - ina219
 * - TMP100 
 * - peltier control
 * - FGDOS 
 * - relays
 * 
 * - serial prints
 *    - FGDOS readout (alternating 1 & 2)
 *    - ina219 readout
 *    - TMP100 readout 
 *    - peltier gate voltage readout
 *    - relay states
 * 
 * - more serial commands
 *    - relay
 *        - FGDOS passive
 *        - peltier relay ON/OFF
 *        - 5V power ON/OFF
 *        - FGDOS SPI ON/OFF
 *    - peltier control
 *        - step wise increase / decrease gate voltage       
 *        - linear profile
 *        - constant temperature
 *        - sawtooth profile
 *        - step profile
 *        - TMP100 based feedback control
 *        
 *  
 */




////////////////////////////////////// LIBRARIES AND MACROS ///////////////////////////////////////////

//------------------------------------------ FGDOS libraries, macros, variables --------------------------------------------------------------

// extra needed libraries
#include <SPI.h>
#include "FGD_03F_extended.h"

// variables
unsigned long int sens_freq_1 = 0, sens_freq_2 = 0, target_freq = 0, threshold_freq = 0;
unsigned long int ref_freq_1 = 0, ref_freq_2 = 0;
char command;
int temperature_1, temperature_2;
byte recharge_count_1, recharge_count_2;
bool flag_isr = FGD_ISR;

bool flag_print_extended = true;                                                 // for print_meas_short_extended()


//------------------------------------------ INA219 libraries, macros, variables --------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;


//------------------------------------------ TMP100 libraries, macros, variables --------------------------------------------------------------

#include <Temperature_LM75_Derived.h>

// The Generic_LM75 class will provide 9-bit (±0.5°C) temperature for any
// LM75-derived sensor. More specific classes may provide better resolution.
TI_TMP100 Daughterboard_TMP100;                                                                                 // $$ object name changed

//int pcb_temp = 0;     // declared with peltier variables


//------------------------------------------ Peltier libraries, macros, variables --------------------------------------------------------------

#define PELTIER_PIN A3                                        // peltier input pin: The N-Channel MOSFET Gate is connected to this pin

unsigned long int ON_time = 0;                       // for timing open loop control inputs
unsigned int step_time = 20;                          // amount of time to maintain current temperture / peltier input,  in [s]                       %^ MODIFY
unsigned int start_time = 0; 
unsigned int current_time = 0;

int pcb_temp = 0;                                    // temperature measured on the PCB - $$ REMEMBER to use proper offset
int temp_error = 0;                                  // temperature error variable for closed loop control
int temp_error_integral = 0;                         // temperature error variable for closed loop control
int temp_error_derivative = 0;
int temp_error_prev = 0;



int peltier_low_temp = 30;                            // temperature lower limit                                                                      %^ MODIFY
int peltier_high_temp = 60;                           // temperature upper limit                                                                      %^ MODIFY
int peltier_const_temp = 35;                          // constant temperature to be maintained                                                        %^ MODIFY
int peltier_step_temp = 5;                           // temperature step height to be achieved,        range: [0,10]                                  %^ MODIFY
int target_temp = 0;                                 // the initial target to be achieved by the stepped profile
int peltier_linear_input = 40;                        // for passing heating input in linear profile,   range: [0,99]                                 %^ MODIFY


int power = 0;                                       // Power level from 0 to 99
int peltier_level = map(power, 0, 99, 0, 255);       // value mapped from power variable to [0 to 255]
int peltier_input = 0;   


//------------------------------------------ Relay libraries, macros, variables --------------------------------------------------------------

// Relay control pins
#define RELAY_PELTIER A0                                     // NC                   // $$ Check pin numbers !!!
                                              
#define RELAY_DAUGHTERBOARD_SUPPLY A1                        // NC
#define RELAY_FGDOS_SPI A2                                   // NC  

// flags to remember relay ON or OFF states                                       // false indicates unactuated state
bool relay_peltier = false;                                                       // true indicates actuated state - true on an NO connection implies the connection is closed
bool relay_daughterboard_supply = false;
bool relay_fgdos_spi = false;

//------------------------------------------ FGDOS FUNCTION Declarations --------------------------------------------------------------

void set_command_extended(char command, unsigned long int *sens_freq_1, unsigned long int *sens_freq_2, unsigned long int *target_freq,
                  unsigned long int *threshold_freq, unsigned long int *ref_freq_1, unsigned long int *ref_freq_2,
                  int *temperature_1, int *temperature_2, byte *recharge_count_1, byte *recharge_count_2, bool flag_isr);



void collect_data_extended(byte sensor,int *temperature, byte*recharge_count, unsigned long int *sens_freq, unsigned long int *ref_freq, float window_factor);

void print_meas_short_extended(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte reg_recharge_counter, byte sensor);



//------------------------------------------ PELTIER FUNCTION Declarations --------------------------------------------------------------

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







///////////////////////////////////////////////////////////////////////////////////////////////////////

// MAIN SETUP
void setup() {

  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);

  // --------------- INA219 SETUP -----------------------------------------------------------

  uint32_t currentFrequency;                                                                                  // $$   what is this used for ???
  
  // Initialize the INA219.
  if (! ina219.begin()) {                                                                                    // Comment this to ignore the INA219 setup check
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();



  // --------------- TMP100 SETUP -----------------------------------------------------------



  // --------------- PELTIER SETUP -----------------------------------------------------------
  // Arduino used for controlling peltier via the n-MOSFET
  // setting peltier control pin to O/P mode and voltage to zero to prevent heat exchange
  pinMode(PELTIER_PIN, OUTPUT);
  analogWrite(PELTIER_PIN, 0);



  // --------------- RELAY SETUP -----------------------------------------------------------

  // setting relay control pins to O/P mode
  pinMode(RELAY_PELTIER, OUTPUT);
  pinMode(RELAY_DAUGHTERBOARD_SUPPLY, OUTPUT);
  pinMode(RELAY_FGDOS_SPI, OUTPUT);

  // setting all relay control pins to HIGH to prevent accidental relay actuation on start-up
  digitalWrite(RELAY_PELTIER, HIGH);
  digitalWrite(RELAY_DAUGHTERBOARD_SUPPLY, HIGH);
  digitalWrite(RELAY_FGDOS_SPI, HIGH);


  // --------------- FGDOS SETUP -----------------------------------------------------------  

  standby_passive_pins_setup();
  pwm_setup();
  spi_setup();
  // wait for everything to settle (MOSFET, PWM, Serial)
  wait(1000);
  
  // --------------- SENSOR SETUP -----------------------------------------------------------
  // set up the sensors basic properties, see function for more details
  // a wait in included in the init function, so no additional wait for new data necessary
  Serial.println("---------------------");
  fgdos_init(SS2);
  fgdos_init(SS1);

  // ISR upon new data, interrupt request data ready, active low
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

}




///////////////////////////////////////////////////////////////////////////////////////////////////////





// MAIN LOOP
void loop() {

  // -------------------------------INA219 ----------------------------------------------------------- 

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  // -------------------------------TMP100 ----------------------------------------------------------- 

  pcb_temp = Daughterboard_TMP100.readTemperatureC();                                                                               



  // ------------------------------- PELTIER ---------------------------------------------------------

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


  // ------------------------------- RELAYS ----------------------------------------------------------- 




  // ------------------------------- FGDOS ----------------------------------------------------------- 

 

  // Collect data in a loop when ISR is not used
  // NIRQ does not work during recharges, collect data via loop during recharges
  if (!flag_isr){
    collect_data_extended(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);                               // $$ change this to extended
    collect_data_extended(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);                               // $$ change this to extended
    //wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
    delay(2.1/WINDOW_FACTOR*1000);
  } else if (((recharge_count_1 & 0x80) == 0x80) || ((recharge_count_2 & 0x80) == 0x80)){
    if ((recharge_count_1 & 0x80) == 0x80){collect_data_extended(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);}   // $$ change this to extended
    if ((recharge_count_2 & 0x80) == 0x80){collect_data_extended(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);}   // $$ change this to extended
    delay(2.1/WINDOW_FACTOR*1000);
    Serial.println("measurement loop running");
  }

  // ------------------------------- COMMAND - RECEIVE & EXECUTE ----------------------------------------------------------- 
  
  // Option to change sensor settings
  command = get_command();
  set_command_extended(command, &sens_freq_1, &sens_freq_2, &target_freq, &threshold_freq, &ref_freq_1, &ref_freq_2,                        //  $$ function call modified to extended version
      &temperature_1, &temperature_2, &recharge_count_1, &recharge_count_2, flag_isr);

  
}









////////////////////////////////////////////// ISR //////////////////////////////////////////////////////////////////////////////

void collect_data_SS1(){                                                                                                          // $$ change this to extended
  collect_data_extended(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);
}
void collect_data_SS2(){                                                                                                          // $$ change this to extended
  collect_data_extended(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
}





////////////////////////////////////// EXTRA FUNCTION DEFINITIONS///////////////////////////////////////////////////////////////

void set_command_extended(char command, unsigned long int *sens_freq_1, unsigned long int *sens_freq_2, unsigned long int *target_freq,
                  unsigned long int *threshold_freq, unsigned long int *ref_freq_1, unsigned long int *ref_freq_2,
                  int *temperature_1, int *temperature_2, byte *recharge_count_1, byte *recharge_count_2, bool flag_isr)
  {
  // flags to remember on or off states
  static bool flag_discharge_1 = true, flag_discharge_2 = true, flag_active_1 = true, flag_on_1 = true, flag_on_2 = true,
              flag_active_2 = true, flag_recharge_1 = true, flag_recharge_2 = true;
  //deactivate interrupt before to prevent collision
  if (flag_isr){
    detachInterrupt(digitalPinToInterrupt(NIRQ_1));
    detachInterrupt(digitalPinToInterrupt(NIRQ_2));
    pinMode(NIRQ_1,INPUT);
    pinMode(NIRQ_2,INPUT);
  }
  switch (command){
    case '1':
      collect_data_extended(SS1, temperature_1, recharge_count_2, sens_freq_1, ref_freq_1, WINDOW_FACTOR);                           // $$ change this to extended
      break;
    case '2':
      collect_data_extended(SS2, temperature_2, recharge_count_2, sens_freq_2, ref_freq_2, WINDOW_FACTOR);                           // $$ change this to extended
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
    case 'a':
      if (!flag_active_1){
        digitalWrite(PASSIVE,LOW);                                                                            // $$ change wth relay - power + spi
        digitalWrite(RELAY_DAUGHTERBOARD_SUPPLY,HIGH);
        digitalWrite(RELAY_FGDOS_SPI,HIGH);
        relay_daughterboard_supply = false;
        relay_fgdos_spi = false;
        Serial.println("ACTIVE 1");
        flag_active_1 = true;
      } else {
        digitalWrite(PASSIVE,HIGH);
        digitalWrite(RELAY_DAUGHTERBOARD_SUPPLY,LOW);
        digitalWrite(RELAY_FGDOS_SPI,LOW);
        relay_daughterboard_supply = true;
        relay_fgdos_spi = true;
        Serial.println("PASSIVE 1");
        flag_active_1 = false;
      }
      break;
    case 'A':
      if (!flag_active_2){
        digitalWrite(PASSIVE,LOW);
        Serial.println("ACTIVE 2");
        flag_active_2 = true;
      } else {
        digitalWrite(PASSIVE,HIGH);
        Serial.println("PASSIVE 2");
        flag_active_2 = false;
      }
      break;
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
    
    // Relay control commands
    case '3':
        digitalWrite(RELAY_PELTIER, HIGH);                                                  // relay is in default state when the control pin is HIGH
        relay_peltier = false;
        break;
    case '#':
      digitalWrite(RELAY_PELTIER, LOW);                                                  // relay is thrown when the control pin is LOW
      relay_peltier = true;
      break;     
    case '4':
      digitalWrite(RELAY_DAUGHTERBOARD_SUPPLY, HIGH);                                                  
      relay_daughterboard_supply = false;
      break;
    case '$':
      digitalWrite(RELAY_DAUGHTERBOARD_SUPPLY, LOW);                                                  
      relay_daughterboard_supply = true;
      break;
    case '5':
      digitalWrite(RELAY_FGDOS_SPI, HIGH);                                                  
      relay_fgdos_spi = false;
      break;
    case '%':
      digitalWrite(RELAY_FGDOS_SPI, LOW);                                                  
      relay_fgdos_spi = true;
      break;

    // PELTIER control commands
    // Open loop
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

    // closed loop
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
      break;
  }
  if (flag_isr){
    if (flag_on_1){
      pinMode(NIRQ_1, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);                                                // $$ change this to extended
    }
    if (flag_on_2){
      pinMode(NIRQ_2, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(NIRQ_2), collect_data_SS2, LOW);                                                // $$ change this to extended
    }
  }
}



void collect_data_extended(byte sensor,int *temperature, byte*recharge_count, unsigned long int *sens_freq, unsigned long int *ref_freq, float window_factor){
  
  collect_freq(sensor,sens_freq,ref_freq, window_factor);
  *temperature = read_reg(sensor,x0_TEMP);
  *recharge_count = read_reg(sensor,x1_RECHARGE_COUNT); // only 7 LSBs are used for counting
  if (*recharge_count == 0x7F){
    Serial.println("reset recharge counter");
    write_reg(sensor,x1_RECHARGE_COUNT,0x00);
  }
  print_meas_short_extended(*temperature,*sens_freq,*ref_freq,*recharge_count,sensor-6);
}


void print_meas_short_extended(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte reg_recharge_counter, byte sensor){
  flag_print_extended ? Serial.println("Sensor, Temp , F_sens , F_ref, Rech_Count, Shunt Voltage(V), Bus Voltage(V), Load Voltage(V), Current(mA), Power(mW), PCB_Temp(C), R_peltier, R_DB, R_SPI, P_Power, P_Input"),flag_print_extended=false:0;
  Serial.print(sensor);Serial.print(" , ");
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.print(ref_freq);Serial.print(" , ");
  Serial.print(reg_recharge_counter,BIN);Serial.print(" , ");
  Serial.print(shuntvoltage);Serial.print(" , ");
  Serial.print(busvoltage);Serial.print(" , ");
  Serial.print(loadvoltage);Serial.print(" , ");
  Serial.print(current_mA);Serial.print(" , ");
  Serial.print(power_mW);Serial.print(" , ");
  Serial.print(pcb_temp);Serial.print(" , ");
  Serial.print(relay_peltier,DEC);Serial.print(" , ");
  Serial.print(relay_daughterboard_supply,DEC);Serial.print(" , ");
  Serial.print(relay_fgdos_spi,DEC);Serial.print(" , ");
  Serial.print(power);Serial.print(" , ");
  Serial.println(peltier_input);
}




//-------------------------- PELTIER OPEN LOOP CONTROL FUNCTIONS--------------------------------------------------------------------------

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








//-------------------------- PELTIER CLOSED LOOP CONTROL FUNCTIONS--------------------------------------------------------------------------


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
