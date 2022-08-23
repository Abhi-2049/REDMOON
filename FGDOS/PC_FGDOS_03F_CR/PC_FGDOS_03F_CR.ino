///-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
#include <FGD_03F.h>

// variables
unsigned long int sens_freq_1 = 0, sens_freq_2 = 0, target_freq = 0, threshold_freq = 0;
unsigned long int ref_freq_1 = 0, ref_freq_2 = 0;
char command;
int temperature_1, temperature_2;
byte recharge_count_1, recharge_count_2;
bool flag_isr = FGD_ISR;
//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);

  standby_passive_pins_setup();
  pwm_setup();
  spi_setup();
  // wait for everything to settle (MOSFET, PWM, Serial)
  wait(1000);
  
  // --------------- SENSOR SETUP ------------------------
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
    attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);
    attachInterrupt(digitalPinToInterrupt(NIRQ_2), collect_data_SS2, LOW);
  }

  // Read all registers of the sensors so they can be double checked later if needed
  read_all_registers(SS1);
  read_all_registers(SS2);
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  // Collect data in a loop when ISR is not used
  // NIRQ does not work during recharges, collect data via loop during recharges
  if (!flag_isr){
    collect_data(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);
    collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
    //wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
    delay(2.1/WINDOW_FACTOR*1000);
  } else if (((recharge_count_1 & 0x80) == 0x80) || ((recharge_count_2 & 0x80) == 0x80)){
    if ((recharge_count_1 & 0x80) == 0x80){collect_data(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);}
    if ((recharge_count_2 & 0x80) == 0x80){collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);}
    delay(2.1/WINDOW_FACTOR*1000);
    Serial.println("measurement loop running");
  }
  
  // Option to change sensor settings
  command = get_command();
  set_command(command, &sens_freq_1, &sens_freq_2, &target_freq, &threshold_freq, &ref_freq_1, &ref_freq_2,
      &temperature_1, &temperature_2, &recharge_count_1, &recharge_count_2, flag_isr);
  
}

//------------------------------------- ISR -------------------------------------

void collect_data_SS1(){
  collect_data(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);
}
void collect_data_SS2(){
  collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
}
