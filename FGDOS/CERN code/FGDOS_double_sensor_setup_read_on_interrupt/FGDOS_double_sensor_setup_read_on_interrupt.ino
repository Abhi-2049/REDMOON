//----------------------------------------------------------
/* FGD-03F (datasheet rev. A0.7)
 *  
 * General notes on the setup:
 * This firmware was written for a setup reading two TC974_Z sensors in two DIL16 separate packages. Prototype PCB "TC974_z_GSI - Copy"
 *  
 * General notes on configuration:
 *  In order to configure the sensor you need to write the registers 9, A, B, C, D, E.
 *  - 0x09 -> [000 | target frequency (4:0)] these are oly the 5 MSB. TDIV defines the number of LSBs (0 for 13 LSBs, 1 for 10)
 *  - 0x0A -> [000 | threshold frequency (4:0)] same bits as target's
 *  - 0x0B -> [enable automatic recharge | internal charge pump at vb | not recharge by internal charge pump | 0 | window (1:0) | 0 | TDIV]
 *  - 0x0C -> [force charge in manual | 1111 | SENS(2:0) (001 HS, 100 LS)]
 *  - 0x0D -> [0 | enable recharge | enable charge pump** | 0 | E2V (ref frequency config, 0 high, 1 low) | set(2:0) charge pump out voltage]
 *  - 0x0E -> [000000 | interrupt pin config (0 push pull, 1 open collector) | ENGATE (0 for internal clk freq, 1 for external gating clk sig)]
 * 
 * ** not described on datasheet. Need to pull this up when manually recharging.
 */
//--------------------------------------------------------------------
#include <SPI.h>

#define TARGET        7000.0
#define THRESHOLD     0000.0
//--------------------------------------------------------------------
// Micro pins
#define CE0 15 
#define CE1 16
#define NIRQ0 11
#define NIRQ1 10
//--------------------------------------------------------------------
// Useful masks
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF
#define WINDOW_MASK 0xC
//--------------------------------------------------------------------
// Initial register configuration
//#define Reg9 0x00
//#define RegA 0x00
#define RegB 0x4C  // 040, 044, 048, 04C for tdiv=0 // 041, 045, 049, 04D for tdiv=1
#define RegC 0x79//79//ls 7C
#define RegD 0x00
#define RegE 0x00
//--------------------------------------------------------------------
// Additional handy definitions
#if RegB&0x01
# define factor 1024      // Factor depending on TDIV. With TDIV=0 we get a higher resolution for target and threshold (respectively 13 or 10 LSBs -> 8192 or 1024) 
#else                     // Used in the conversion function from target and threshold to their config values.
# define factor 8192
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//---------------function declarations------------------------------------

long int collect_and_send_data(byte sensor);
void write_reg(byte sensor, byte reg, byte data);
unsigned int read_reg(byte sensor, byte reg);
bool wait_and_collect_freq(byte sensor, long int* sens_freq, long int* ref_freq);
void init(byte sensor);
int int_to_tt(float frequency);
void print_data_and_registers(byte sensor,long int sens_freq, long int ref_freq);
void activate_sensor(byte sensor);
long int sens_read(byte sensor);
long int rch_read(byte sensor);
void rch_config(byte sensor, String init_param);

//---------------------SETUP---------------------------------------------
void setup() {
  Serial.begin(9600);
  while(!Serial);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  pinMode(CE0, OUTPUT); //chip enable, active low
  pinMode(CE1, OUTPUT);
  pinMode(NIRQ0, INPUT); //interrupt request data ready, active low
  pinMode(NIRQ1, INPUT);
  digitalWrite(CE1, HIGH);
  digitalWrite(CE0, HIGH);

  attachInterrupt(digitalPinToInterrupt(NIRQ0), read_sensor, LOW);
}
//---------------------------------------------------------------------------------------------------------------------------------
//              Global variables

long int freq;
int threshold = THRESHOLD;
int target = TARGET;
//float val[]={0.0,3.3};
int counter = 0;
bool first_time=true;
unsigned long time_last = millis();
int window_factor = 0;


//------------------LOOP-------------------------------------------------------------------------------------------------------------
// if only one sensor is needed, cooment out the second init and activate_sensor lines. Don't need to do anything else.
void loop() {
    if (first_time == true){
      switch (RegB&WINDOW_MASK) {   // window factor definition. Move this somewhere else if you want the window to be reconfigurable.
        case 0x00:
          window_factor = 1;
        break;
        case 0x04:
          window_factor = 2;
        break;
        case 0x08:
          window_factor = 4;
        break;
        case 0x0C:
          window_factor = 8;
        break;
      }
      init(CE0);
      //init(CE1);
      first_time=false;
      delay(100);
    }
    else{
      activate_sensor(CE0);
      //activate_sensor(CE1);
      delay(100);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                            Functions.
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Initialization function. It initializes all of the configuration registers (recharge safe mode, registers as defined in the definitions section of the code) The recharge registers will be reconfigured by the recharge functions. Leave the config order as it is.
void init(byte sensor){
  write_reg(sensor, 0x0D, 0x04); //Disable recharge till after boundaries are set
  write_reg(sensor, 0x09, int_to_tt(TARGET));
  write_reg(sensor, 0x0A, int_to_tt(THRESHOLD));
  write_reg(sensor, 0x0B, RegB);
  write_reg(sensor, 0x0C, RegC);
  write_reg(sensor, 0x0D, RegD);
  write_reg(sensor, 0x0E, RegE);
}
//----------------------------------------------------------------------------------------------------------------

// int_to_tt converts the integer frequency declared above into its five most significant bits. Used for threshold and target frequencies.
int int_to_tt(float frequency){ 
   int temp=((frequency)/(float(factor)));//*float(window_factor)));
    return temp;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// This function activates the sensor and reads it. It also checks wether the sensor's in the linear range, and calls the recharge functions if necessary.
void activate_sensor(byte sensor){
  freq = collect_and_send_data(sensor);
  if(freq<THRESHOLD){
    Serial.println("RECHARGE");
    rch_config(sensor,"start");   // Comment out to disable recharge
    freq = rch_read(sensor);
    Serial.print("freq: ");
    Serial.println(freq);
    while(freq<TARGET || (read_reg(sensor,0x01)&0x80)==1){
      freq = rch_read(sensor);
      Serial.print("FREQ: ");
      Serial.println(freq);
      delay(25);
    }
    Serial.println("RECHARGE_DONE");
    rch_config(sensor,"stop");
   }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// wait_and_collect_frequency, quite tautologically, waits for the flags of the sensor and ref frequencies to be valid, and then collects , stitches, masks and stores the data.
bool wait_and_collect_freq(byte sensor, long int* sens_freq, long int* ref_freq){
  bool have_sens = false;
  bool have_ref = false;
  byte byte1; // in register order. These serve as temporary storage for the three frequency registers
  byte byte2;
  byte byte3;
  unsigned int time_start=millis();
  while(!have_sens || !have_ref){                                                         
    if(!have_sens){
      if(read_reg(sensor, 0x08) & 0x08){
//        *sens_freq = sens_read(sensor);   // alternative version to try and fix the reading.
        
        byte2 = read_reg(sensor, 0x07);
        byte1 = read_reg(sensor, 0x06);
        byte3 = read_reg(sensor, 0x08);
        *sens_freq = ((((byte3 << 8) | ((byte2 <<4)) | byte1)) & FREQ_MASK) * window_factor; // should  still multiply by the window factor. keep it.
        // debug prints
        //Serial.println(byte1, BIN);
        //Serial.println(byte2, BIN);
        //Serial.println(byte3, BIN);
        //Serial.println(*sens_freq, BIN);
        have_sens = true;
      }
    }
    if(!have_ref){
      if(read_reg(sensor, 0x05) & 0x08){
        byte1 = read_reg(sensor, 0x03);
        byte2 = read_reg(sensor, 0x04);
        byte3 = read_reg(sensor, 0x05);
        *ref_freq = ((((((byte3 << 8) | byte2) <<8) | byte1)) & FREQ_MASK) * window_factor;
        have_ref = true;
      }
    }
    if(millis()-time_start>(4*window_factor*1000)){
      return false;
    }
  }
  return true;
}


//---------------------------------------------------------------------------------------------------------------------

// These two function write and read, respectively, one register at a time through the SPI.
void write_reg(byte sensor, byte reg, byte data){
  byte adr = reg|WR;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  SPI.transfer(data);
  digitalWrite(sensor, HIGH);
  Serial.println("write |");
}

unsigned int read_reg(byte sensor, byte reg){
  unsigned long data=0;
  byte adr = reg|RD;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  data |= SPI.transfer(0x00);  // apparently the automatic register incrementation (necessary to read more than one register with a single request) doesn't work properly. Reads have been limited to one reg at a time.
  //delay(100);
  digitalWrite(sensor, HIGH);
  delay(100);
  return data;
}

//-----------------------------------------------------------------------------------------------------------------------
// this is used by the activate_sensor fuction. It calls the wait_and_collect_freq function and prints its output through serial by calling  print_data_and_registers.
long int collect_and_send_data(byte sensor){
    long int sens_freq = 0, ref_freq = 0;
    if(wait_and_collect_freq(sensor, &sens_freq, &ref_freq)){
      print_data_and_registers(sensor, sens_freq, ref_freq);
    }else{
      Serial.println("Timeout Occurred for: "+String(sensor));
    }
    return sens_freq;
}

//--------------------------------------------------------------------------------------------------------------------------

// print_data_and_registers sends the collected data to the PC through serial. You might need to modify it to store stuff with a python script.
void print_data_and_registers(byte sensor,long int sens_freq, long int ref_freq){
  int printable_registers[] = {0x00,0x0B,0x0C,0x0D,0x0E};
  int reg_count = sizeof(printable_registers)/sizeof(printable_registers[0]);
  Serial.print("S");
  Serial.println(sensor);
  Serial.print("Sensor Frequency:  ");
  Serial.println(sens_freq);
  Serial.print("Reference Frequency:  ");
  Serial.println(ref_freq);
  Serial.println(read_reg(sensor,0x09)*factor);
  Serial.println(read_reg(sensor,0x0A)*factor);
  Serial.println((read_reg(sensor,0x01)&0x7F));
  for(int i = 0;i < reg_count;i++){
    Serial.println(read_reg(sensor, printable_registers[i]),BIN);
  }

// debugging section of this function. Not supported by python script for storing. Comment from here on when done with debugging.
//  Serial.print("Target:  ");
//  Serial.println(read_reg(sensor,0x09)*factor);
//  Serial.print("Threshold:  ");
//  Serial.println(read_reg(sensor,0x0A)*factor);
//  Serial.print("Recharge register:  ");
//  Serial.println(read_reg(sensor,0x01)&0x7F);
////  Serial.print("Config regs temp, b, c, d, e:  ");
////  for(int i = 0;i < reg_count;i++){
////    Serial.println(read_reg(sensor, printable_registers[i], 1),BIN);
////  }
//  Serial.print("Temperature:  ");
//  Serial.println((read_reg(sensor,0x00))-70);
////  Serial.print("Temperature bin:  ");
////  Serial.println(read_reg(sensor,0x00), BIN);
//  Serial.print("Window Factor:  ");
//  Serial.println(window_factor);
//   Serial.print("Factor:  ");
//  Serial.println(factor);
////  Serial.print("Reference Frequency bin:  ");
////  Serial.println(ref_freq, BIN);
//
//  Serial.print("register B :  ");
//  Serial.println(read_reg(sensor,0x0B), BIN);
//  Serial.print("register C :  ");
//  Serial.println(read_reg(sensor,0x0C), BIN);
//  Serial.print("register D :  ");
//  Serial.println(read_reg(sensor,0x0D), BIN);
//  Serial.print("register E :  ");
//  Serial.println(read_reg(sensor,0x0E), BIN);
}

//----------------------------------------------------------------------------------------------------------------------------------------
//****************************************************************************************************************************************
// -------------RECHARGE FUNCTIONS--------------------------------------------------------------------------------------------------------

// All of the recharge bits should only be activated in here, in order to prevent unwanted rehcarges and counter-intuitive running of the code.

// The rch_config funcion configures the recharge of the floating gate.
void rch_config(byte sensor, String init_param){
  if(init_param == "start"){
      //write_reg(sensor, 0x0B, 0xCD);
      write_reg(sensor, 0x0C, 0x79);
      write_reg(sensor, 0x0D, 0x04); //Disable recharge till after boundaries are set
      write_reg(sensor, 0x0E, 0x00); 
      target = int_to_tt(TARGET); // you'll need to modify these two lines to make it recharge more gradually. At the moment only modifying the global scope target and threshold. Have a look at how it used to be in IHopeItWorks.
      threshold = int_to_tt(THRESHOLD);
      write_reg(sensor, 0x09, target);
      write_reg(sensor, 0x0A, threshold);
      delay(300);
      write_reg(sensor, 0x0D, 0x67); // these two registers force  the manual recharge.
      write_reg(sensor, 0x0C, 0xF9); 
  }
  else if(init_param == "stop"){
      write_reg(sensor, 0x0B, RegB);
      write_reg(sensor, 0x0C, RegC);
      write_reg(sensor, 0x0D, RegD); //Disable recharge till after boundaries are set
      write_reg(sensor, 0x0E, RegE); 
      target = int_to_tt(TARGET);
      threshold = int_to_tt(THRESHOLD);
      write_reg(sensor, 0x09, target);
      write_reg(sensor, 0x0A, threshold);
      delay(2200);
  }  
}


long int rch_read(byte sensor){
  long int frequency = 0;
  if(read_reg(sensor, 0x08) & 0x08){
        byte byte1 = read_reg(sensor, 0x06);
        byte byte2 = read_reg(sensor, 0x07);
        byte byte3 = read_reg(sensor, 0x08);
        frequency = ((((byte3 << 8) | ((byte2 <<4)) | byte1)) & FREQ_MASK) * window_factor;
      }
  return frequency;
}

//---------------------------------------------------------------------------------------------------------------------------------------
//***************************************************************************************************************************************
//                 Debugging functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//debug function to try and solve the window_factor issue
long int sens_read(byte sensor){
  long int frequency = 0;
  if(read_reg(sensor, 0x08) & 0x08){
        byte adr = 0x08|RD;
        //Serial.println(adr);
        digitalWrite(sensor, LOW);
        SPI.transfer(adr);
        byte byte1 = SPI.transfer(0x00);
        byte byte2 = SPI.transfer(0x00);
        byte byte3 = SPI.transfer(0x00);
        
        //for(int i = 0;i < 3; i++){
            //frequency |= SPI.transfer(0x00) << i*8 ; // insert print here for debugging  3.
            //Serial.println(data, BIN);
        //}
        digitalWrite(sensor, HIGH);
        //frequency = frequency&FREQ_MASK;
        frequency = ((((byte3 << 8) | ((byte2 <<4)) | byte1)) & FREQ_MASK) * window_factor;
        return frequency;
  }
}


// Interrupt driven funtion. It reads the sensor data. Still trying to solve the debug issue
void read_sensor() {
  long int sens_freq = 0, ref_freq = 0;
  int sensor = CE0;
    if(read_reg(sensor, 0x08) & 0x08){
//        *sens_freq = sens_read(sensor);   // alternative version to try and fix the reading.
        byte byte2 = read_reg(sensor, 0x07);
        byte byte1 = read_reg(sensor, 0x06);
        byte byte3 = read_reg(sensor, 0x08);
        sens_freq = ((((((byte3 << 8) | byte2) <<8) | byte1)) & FREQ_MASK) * window_factor;
    }
    if(read_reg(sensor, 0x05) & 0x08){
        byte byte1 = read_reg(sensor, 0x03);
        byte byte2 = read_reg(sensor, 0x04);
        byte byte3 = read_reg(sensor, 0x05);
        ref_freq = ((((((byte3 << 8) | byte2) <<8) | byte1)) & FREQ_MASK) * window_factor;
    }
    print_data_and_registers(sensor, sens_freq, ref_freq); 
}
