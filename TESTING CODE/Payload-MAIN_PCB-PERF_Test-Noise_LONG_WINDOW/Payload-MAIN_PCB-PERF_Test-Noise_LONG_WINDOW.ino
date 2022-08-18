// Originally: Payload_DummyPCB_Test


/*
 * 13/06: Setting up script to send Data from both sensors over serial, cleaning up some unnecessary code
 * 03/08: Prep for External OSC and Longer Window Noise tests
 * 04/08: Code commented for long WINDOW of 32768 pulses with WCK provided by Ext OSC at 32768 kHz
 * 
 * 
*/

//-----------------------------------------------------------------------------------------------------------------------
#include <SPI.h>
#include <driverlib.h>                                                                              

// Micro pins                         // MCU PIN Number

// pins for slave selection
#define SS1 12                        // P1.4      
#define SS2 11                        // P1.3      

// Standby pins - don't leave floating!!
#define NSTBY_1 35                    // P3.3
#define NSTBY_2 36                    // P4.7

// Window clock signal pin
#define WCK_1 8                       // P3.4
//float clock_check = 0;                        // to check clock signal value


// Useful masks
// WR and RD are combined with the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF

// FGDOS Register definitions
#define TEMP 0x00
#define RECHARGE_COUNT 0x01
#define TARGET 0x09
#define THRESHOLD 0x0A
#define RECHARGE_WINDOW 0x0B                                                                         
#define CHARGE_SENS 0x0C
#define RECHARGE_REF 0x0D                                                                           
#define NIRQ_ENGATE 0x0E                                                                            // NIRQ pin not used for now


// handy constant definitions (these depend on settings in Arduino and sensor!)
#define THRESHOLD_FREQ 50000 // these values are not exact! Since only 8 MSBs are used          
#define TARGET_FREQ 90000                                                                       
//#define CK_FREQ 31250.0f // depends on the settings of the PWM                                     //31250
#define CK_FREQ 32768.0f // supplied by external OSC                                               //$$ For lower noise / more precise WCK
//#define WINDOW_PULSES 4096 // depends on window register settings!                                 //8192.0f
#define WINDOW_PULSES 32768 // depends on window register settings!                              // $$ For longest WINDOW
#define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES)

// Function definitions
unsigned int read_reg(byte sensor, byte reg);
void write_reg(byte sensor, byte reg, byte data);
bool collect_freq(byte sensor, unsigned long int *sens_freq, unsigned long int *ref_freq);
void fgdos_init(byte sensor);
void wait(int microsecs);
void print_freq(byte freq_reg[3],unsigned long int freq_value,char f_type);
void print_meas_full(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor);
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, int recharge_count, byte sensor);

// New Funtion definitions
void Clock_set_readout();
void Clock_reset();

void Enable_RS485_send();
void Disable_RS485_send();

void Read_RS485_DataBus();
void Send_RS485_DataBus();

void crc_check_16();




// variables
unsigned long int payload_on_time = 0;

unsigned long int sens_freq_1 = 0;
unsigned long int sens_freq_2 = 0;

unsigned long int ref_freq_1 = 0;
unsigned long int ref_freq_2 = 0;

int temperature_1 = 0;
int temperature_2 = 0;

int recharge_count_1 = 0;
int recharge_count_2 = 0;


byte i = 0, j = 0;
bool flag = true;






// Command and Telemetry variables
#define START_BYTE 0x3A
#define STOP_BYTE 0x0A
#define RAD_PAYLOAD_ADDR 0x00AA

//for command ID
int command_id = 0;

// for receiving data
const int message_size = 16;              // number of bytes to be read
byte message_buffer[16];                  // $$ change buffer size once protocol is decided                                       $$ using Hyperion protocol for now
bool message_received = false;

//for sending data
const int data_message_size = 16;        // number of bytes to be sent
byte data_buffer[16];                    // $$ change buffer size once protocol is decided

// for CRC check
byte crc_check_payload[2];





//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {

  // setting RS485 pins ~RE and DE low to start listening only over RS485 Bus. Uses P2.6 on MCU                                $$ for RS485 transceiver test on Dummy PCB
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
  //setting FRAM CS pin high to prevent output. Uses P3.6 on MCU
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);


// $$
  // WCK signal generation - Page 92 - https://dev.ti.com/tirex/explore/node?node=AANMz9MCP3TctaSCJuEa0Q__IOGqZri__LATEST 


  pinMode(WCK_1, OUTPUT);                                                                                                   // Try P3DIR |= 0x10 ; if this does not work
  P3SEL1 |= 0x10 ;   //Set 3.4 to 1 to select SMCLK 
  



  
  Serial.begin(9600);                                                                           //250000
  while(!Serial);


  // Set NSTBY pins to high 
/*
  pinMode(NSTBY_1, OUTPUT);
  digitalWrite(NSTBY_1, HIGH);
  pinMode(NSTBY_2, OUTPUT);
  digitalWrite(NSTBY_2, HIGH);
*/

  //Comment the 4 lines below if using the LaunchPad and Breadboard B Setup                                                                 $$
  // setting NSTBY pins high using driverlib GPIO function
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3);
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);


//  Serial.println("NSTBY pins are set HIGH");

  Serial.println("NSTBY pins are set HIGH");

  



  
  // SPI setup
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));                                                         
  SPI.begin();

  // Set CS pins to output mode and high
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

  //delay(1000);
  __delay_cycles(1000000);                                                                                                  //$$
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  //update Payload ON Time
  payload_on_time = millis();

  // Set SMCLK to WCK = 31250 Hz
  //Clock_set_readout();                                                                                                   // $$ trying with EXT OSC

  __delay_cycles(1000000);

  //delay(1000);
  //Clock_reset();
  //delay(1000);
  //__delay_cycles(1000000);

  
  // Sensor 1
  
  // read the temperature, recharge count and frequency registers
  temperature_1 = read_reg(SS1,TEMP);                                                       // temperature = read_reg(SS2,TEMP) - 87;
  recharge_count_1 = read_reg(SS1,RECHARGE_COUNT);
  recharge_count_1 = recharge_count_1 & 0x0F;

  //__delay_cycles(1000000);                                                                                                                    //    $$

  // collect freq from sensor 1 
  collect_freq(SS1,&sens_freq_1,&ref_freq_1);
  
  //__delay_cycles(1000000);                                                                                                                    //    $$
  
  // Sensor 2
  
  // read the temperature, recharge count and frequency registers
  temperature_2 = read_reg(SS2,TEMP);                                                       
  recharge_count_2 = read_reg(SS2,RECHARGE_COUNT);
  recharge_count_2 = recharge_count_2 & 0x0F;

  // collect freq from sensor 2 
  collect_freq(SS2,&sens_freq_2,&ref_freq_2);

   // reset SMCLK to 8 MHz
   //Clock_reset();                                                                                                                             // $$ trying with EXT OSC
   
   __delay_cycles(100000);


  //Print Sensor 1 & 2 measurments to Serial Monitor
  print_meas_short(temperature_1,sens_freq_1,ref_freq_1,recharge_count_1,1);
  print_meas_short(temperature_2,sens_freq_2,ref_freq_2,recharge_count_2,2);
//  Serial.print("Payload ON Time: ");Serial.println(payload_on_time);


   __delay_cycles(1000000);

/*
  // Once sensor has been read and data has been stored in variables, read the Data Bus to see if rover is sending commands
  Read_RS485_DataBus();

  message_received = true;                                                                      // $$ FOR DEBUG
  command_id = 1;                                                                               // $$ FOR DEBUG
  
  // if a command was received, send data over the Data Bus
  if(message_received){
    Send_RS485_DataBus();
  }
*/
  
}




//-----------------------------------------------------------------------------------------------------------------------
// FUNCTIONS

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
bool collect_freq(byte sensor, unsigned long int *sens_freq, unsigned long int *ref_freq){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck
  int x = 1000;// increase amount of time to wait if serial.prints are used! Otherwise set to 1
  
  // first check to see if no recharge is going on
//  if((read_reg(sensor,RECHARGE_COUNT) & 0x80) == 0x80){                                           // RCHEV bit = 1 when recharge is in progress         $$
//    Serial.println("recharge in progress... BREAK");                                                                                                    $$
//    return false;                                                                                                                                       $$
//  }
  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(sensor,0x08);
      freq_reg[1] = read_reg(sensor,0x07);
      freq_reg[2] = read_reg(sensor,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK );
      have_sens_freq = true;
      //print_freq(freq_reg,*sens_freq,'S');
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
  Serial.print("-SENSOR "); Serial.println(abs(sensor-8));   
  // set the reference oscillator and window measurement amount of pulses settings
  // bits (6:4) for ref and (3:2) for window (bit counting from lsb to msb)
  // reference set to 100 (= ??) and windows set to 10 (8192 pulses)(00=32768 ck pulses per window)
  //write_reg(sensor,RECHARGE_WINDOW,0xCD);                                                                          // write_reg(sensor,RECHARGE_WINDOW,0x48);
  // comment the line above and uncomment the line below to get a WINDOW of 32768 pulses
  write_reg(sensor,RECHARGE_WINDOW,0xC0);                                                                                                    // $$ Longest WINDOW for low noise
  Serial.print("-window_factor "); Serial.println(WINDOW_FACTOR);
  // manual recharge off and sesitivity to low
  // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
  write_reg(sensor,CHARGE_SENS,0x79);
  Serial.println("-sensitivity high");
  // no new data avialable signal via nirq pin (mnrev bit 6), nirq setting to push-pull (nirqoc bit 1), 
  // measurement window to count clocks (engate bit 0)
  write_reg(sensor,NIRQ_ENGATE,0x06);
  // disconnect recharging system before setting targets
  write_reg(sensor,RECHARGE_REF,0x04);
  // wait for reference to stabilise, eg 4 measurement windows (= 4*32768 pulses at 31.25 kHz = 4194 millisecs)
  //delay(4/WINDOW_FACTOR*1000); // 4194                                                                             // wait(4/WINDOW_FACTOR*1000);
  __delay_cycles(4/WINDOW_FACTOR*1000000);
  // read reference register ,select 8 MSBs and set them to target register
  // or just pick 50 kHz advised from Sealicon
  //unsigned long int target = (((read_reg(sensor,0x05) << 8 | read_reg(sensor,0x04) ) <<8 | read_reg(sensor,0x03) ) & FREQ_MASK );
  //byte reg_target = (target & 0x3FC00) >> 10;
  //byte reg_target = floor(TARGET_FREQ/WINDOW_FACTOR/1024); // dividing by 1023 is bitshifting by 10 ;)             //byte reg_target = round(TARGET_FREQ/WINDOW_FACTOR/1023);
  // comment the line above and uncomment the line below to get a WINDOW of 32768 pulses
  byte reg_target = floor(TARGET_FREQ/WINDOW_FACTOR/8192); // dividing by 8192 is bitshifting by 13                                         // $$ Longest WINDOW for low noise
  write_reg(sensor,TARGET,reg_target);
  //Serial.print("-target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println((reg_target << 10)*WINDOW_FACTOR);
  // comment the line above and uncomment the line below to get a WINDOW of 32768 pulses
  Serial.print("target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println(((unsigned long)reg_target*8192)*WINDOW_FACTOR);             // $$ Longest WINDOW for low noise
  
  //Serial.print("-registry "); Serial.println(reg_target,BIN);
  // set threshold to 30 kHz equivalent (0x1D = 29, when converted to 8 MSBs of sens_freq (18 bit) = 29696)
  // do not forget to apply window factor! (lower alternative: 0x08 = 8192)
  //byte reg_threshold = floor(THRESHOLD_FREQ/WINDOW_FACTOR/1024);                                                   //byte reg_threshold = round(THRESHOLD_FREQ/WINDOW_FACTOR/1023);
  // comment the line above and uncomment the line below to get a WINDOW of 32768 pulses
  byte reg_threshold = floor(THRESHOLD_FREQ/WINDOW_FACTOR/8192);                                                                              // $$ Longest WINDOW for low noise
  write_reg(sensor,THRESHOLD,reg_threshold);
  //Serial.print("-threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println((reg_threshold << 10)*WINDOW_FACTOR);
  // comment the line above and uncomment the line below to get a WINDOW of 32768 pulses
  Serial.print("threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println(((unsigned long)reg_threshold*8192)*WINDOW_FACTOR);      // $$ Longest WINDOW for low noise
  
  //Serial.print("-registry "); Serial.println(reg_threshold, BIN);
  // enable interrupt upon new data (for now no interrupt pin is actually being used
  //write_reg(sensor,NIRQ_ENGATE,0x30);                                                                            // 
  // also enable automatic recharging again
  //write_reg(sensor,RECHARGE_REF,0x44);                                                                           //  $$ FOR DEBUG 
  Serial.println();
  //delay(1000);
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
  
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, int recharge_count, byte sensor){
  flag ? Serial.println("S, T , Fs , Fr, Rc"),flag=false:0;
  Serial.print(sensor);Serial.print(" , ");
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.print(ref_freq);Serial.print(" , ");
  Serial.println(recharge_count);
  }






// New functions

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

  //Serial.println("RE and DE RS485 pins are set HIGH");
}




void Read_RS485_DataBus(){
  // the data bus is read only if the message received flag is false i.e. the message was not read
  // and if the sensor readout has happened at least once in the last 5 seconds i.e. a 5s timeout on waiting for the message is implemented 
  while((!message_received) && ((millis()- payload_on_time) < 5000)){                                                                                   //$$
    Serial.println("I'm here 1!");                                                                              //  $$ FOR DEBUG
    
    // read bytes of data over the bus
    //Serial.readBytes(message_buffer,message_size);



    Serial.readBytesUntil('\n',message_buffer,message_size);

    // print what was read over the DataBus                                                                   //  $$ FOR DEBUG
    Serial.print("Number of bytes received by Payload: ");Serial.println(Serial.available());
    for (int k=0;k<message_size;k++){                                                                         
      Serial.print(message_buffer[k],HEX);
    }
    Serial.println("");



    // check if message is addressed to the Payload, 0xAA used as the Payload address for now                 $$
    if ((message_buffer[0] == START_BYTE) && (message_buffer[15] == STOP_BYTE)) {

      // check if message is addressed to the Payload, 0x00AA used as the Payload address for now                 $$
      if (( message_buffer[1] << 8 | message_buffer[2] )== RAD_PAYLOAD_ADDR){
        
        // read command type or id and set the command_id variable accordingly
        // only reading one byte now since more than 256 commands are not expected in the testing software
        command_id = int(message_buffer[4]);

        Serial.print("Command ID: ");Serial.println(command_id);
        
        // set the message received flag to true to break out of while loop and stop reading over the data bus
        message_received = true;
      }
      
    }
    
  }
  
}


void Send_RS485_DataBus(){

  Serial.println("I'm here 2!");                                                                               // $$ FOR DEBUG
  
  // construct data buffer to be sent according to the received command
  data_buffer[0] =  START_BYTE;     // Start Byte
  data_buffer[1] =  0x00;           // Address Byte - Use OBC / PPU Address ??                                $$
  data_buffer[2] =  0x22;           // Address Byte - Use OBC / PPU Address ??                                $$
  data_buffer[3] =  0x00;           // command byte
  data_buffer[4] =  0x00;           // command byte
  data_buffer[5] =  0x00;           // 8 data bytes from here
  data_buffer[6] =  0x00;
  data_buffer[7] =  0x00;
  data_buffer[8] =  0x00;
  data_buffer[9] =  0x00;
  data_buffer[10] = 0x00;
  data_buffer[11] = 0x00;
  data_buffer[12] = 0x00;
  data_buffer[13] = 0x00;           // = crc_check_payload[0];
  data_buffer[14] = 0x00;           // = crc_check_payload[1];
  data_buffer[15] = STOP_BYTE;      // Stop Byte
  
//  command_id = 1;                                                                                          // $$ FOR DEBUG

  // Set Data byte(s) for return based on the command  
  switch(command_id){
    case 1:                                                                                                // $$ add byte conversions and relevant variables here
      data_buffer[12] =  (payload_on_time) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (payload_on_time >> 8) & 0xFF; 
      data_buffer[10] =  (payload_on_time >> 8) & 0xFF; 
      data_buffer[9]  =  (payload_on_time >> 8) & 0xFF; 
      break;
    case 2:
      data_buffer[12] =  (sens_freq_1) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (sens_freq_1 >> 8) & 0xFF; 
      data_buffer[10] =  (sens_freq_1 >> 8) & 0xFF; 
      data_buffer[9]  =  (sens_freq_1 >> 8) & 0xFF;       
      break;
    case 3:
      data_buffer[12] =  (ref_freq_1) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (ref_freq_1 >> 8) & 0xFF; 
      data_buffer[10] =  (ref_freq_1 >> 8) & 0xFF; 
      data_buffer[9]  =  (ref_freq_1 >> 8) & 0xFF;
      break;
    case 4:
      data_buffer[12] =  (sens_freq_2) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (sens_freq_2 >> 8) & 0xFF; 
      data_buffer[10] =  (sens_freq_2 >> 8) & 0xFF; 
      data_buffer[9]  =  (sens_freq_2 >> 8) & 0xFF;
      break;
    case 5:
      data_buffer[12] =  (ref_freq_2) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (ref_freq_2 >> 8) & 0xFF; 
      data_buffer[10] =  (ref_freq_2 >> 8) & 0xFF; 
      data_buffer[9]  =  (ref_freq_2 >> 8) & 0xFF;
      break;
    case 6:
      //data_buffer[2] = lowByte(temperature_1);
      data_buffer[12] =  (temperature_1) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (temperature_1 >> 8) & 0xFF;
      break;
    case 7:
      //data_buffer[2] = lowByte(temperature_2);
      data_buffer[12] =  (temperature_2) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (temperature_2 >> 8) & 0xFF;
      break;
    case 8:
      //data_buffer[2] = lowByte(recharge_count_1);
      data_buffer[12] =  (recharge_count_1) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (recharge_count_1 >> 8) & 0xFF;
      break;
    case 9:
      //data_buffer[2] = lowByte(recharge_count_2);
      data_buffer[12] =  (recharge_count_2) & 0xFF;                                                         // little endian - bit shifting and masking
      data_buffer[11] =  (recharge_count_2 >> 8) & 0xFF;
      break;
    default:
//      data_buffer[2] = 0xFF;
      Serial.println("No command selected");                                                              //    $$
      break;
  }
  
  // Set RS485 transceiver pins to send data over data bus
  Enable_RS485_send();
  

  // send the data_buffer over UART
  Serial.write(data_buffer,data_message_size);

  // Set RS485 transceiver pins back to listening over data bus
  Disable_RS485_send();

  // print what was sent to the DataBus                                                                 //  $$ FOR DEBUG
  for (int k=0;k<data_message_size;k++){
    Serial.print(data_buffer[k],HEX);
  }
  Serial.println("");
  

  // set the message received flag to false for reading next message over data bus
  message_received = false;

  // set command_id back to 0 for the next command
  command_id = 0;
  
}


void crc_check_16(){

  Serial.println("This function performs CRC checks");


}
