/*
 * NOTE: no need to define variables, etc here
 * Arduino (alphabetically) compiles and includes what was first compiled into the next ones
 * see bottom for I2C ISR functions
 */
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

/* 
 * write to a register 
 */
void write_reg(byte sensor, byte reg, byte data){
  byte adr = reg|WR;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  SPI.transfer(data);
  digitalWrite(sensor, HIGH);
}

/* 
 * read the frequency registers and reconstruct their values
 * frequency = registers / window pulses amount * ck frequency
 * also stores them in the I2C data array
 */
bool collect_freq(byte sensor, unsigned long int *sens_freq, unsigned long int *ref_freq){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck
  int x = 1000;// increase amount of time to wait if serial.prints are used! Otherwise set to 1
  
  // first check to see if no recharge is going on
  if((read_reg(sensor,RECHARGE_COUNT) & 0x80) == 0x80){
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
      *sens_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) << 8 | freq_reg[2] ) & FREQ_MASK );
      have_sens_freq = true;
      //print_freq(freq_reg,*sens_freq,'S');
      *sens_freq = *sens_freq * WINDOW_FACTOR;

    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(sensor,0x05);
      freq_reg[1] = read_reg(sensor,0x04);
      freq_reg[2] = read_reg(sensor,0x03);
      *ref_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) << 8 | freq_reg[2] ) & FREQ_MASK );
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

void collect_data_SS1(){
  //unsigned long int sens_freq = 0, ref_freq = 0;
  //int temperature, recharge_count;
  collect_freq(SS1,&sens_freq,&ref_freq);
  temperature = read_reg(SS1,TEMP);
  recharge_count = read_reg(SS1,RECHARGE_COUNT); // only 4 LSBs are used for counting
  if ((recharge_count & 0x8F) == 0x0F){
    Serial.println("reset counter");
    write_reg(SS1,RECHARGE_COUNT,0x00);
  }
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,1);
}
void collect_data_SS2(){
  //unsigned long int sens_freq = 0, ref_freq = 0;
  //int temperature, recharge_count;
  collect_freq(SS2,&sens_freq,&ref_freq);
  temperature = read_reg(SS2,TEMP);
  recharge_count = read_reg(SS2,RECHARGE_COUNT); // only 4 LSBs are used for counting
  if ((recharge_count & 0x8F) == 0x0F){
    Serial.println("reset counter");
    write_reg(SS2,RECHARGE_COUNT,0x00);
  }
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,2);
}

void fgdos_init(byte sensor){
  Serial.print("SENSOR "); Serial.println(abs(sensor-10));
  // set the reference oscillator and window measurement amount of pulses settings
  // bits (6:4) for ref and (3:2) for window (bit counting from lsb to msb)
  // reference set to 100 (= ??) and windows set to 10 (8192 pulses)(00=32768 ck pulses per window)
  write_reg(sensor,REF_WINDOW,0x48);
  Serial.print("window_factor "); Serial.println(WINDOW_FACTOR);
  // manual recharge off and sesitivity to low
  // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
  write_reg(sensor,CHARGE_SENS,0x79);
  Serial.println("sensitivity high");
  // no new data avialable signal via nirq pin (mnrev bit 6), nirq setting to push-pull (nirqoc bit 1), 
  // measurement window to count clocks (engate bit 0)
  write_reg(sensor,NIRQ_ENGATE,0x30);
  // disconnect recharging system before setting targets
  write_reg(sensor,RECHARGE,0x00);
  // wait for reference to stabilise, eg 4 measurement windows (= 4*32768 pulses at 31.25 kHz = 4194 millisecs)
  wait(4/WINDOW_FACTOR*1000); // 4194
  // read reference register ,select 8 MSBs and set them to target register
  // or just pick 50 kHz advised from Sealicon
  //unsigned long int target = (((read_reg(sensor,0x05) << 8 | read_reg(sensor,0x04) ) <<8 | read_reg(sensor,0x03) ) & FREQ_MASK );
  //byte reg_target = (target & 0x3FC00) >> 10;
  byte reg_target = round(TARGET_FREQ/WINDOW_FACTOR/1023); // dividing by 1023 is bitshifting by 10 ;)
  write_reg(sensor,TARGET,reg_target);
  Serial.print("target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println((reg_target << 10)*WINDOW_FACTOR);
  //Serial.print("-registry "); Serial.println(reg_target,BIN);
  // set threshold to 30 kHz equivalent (0x1D = 29, when converted to 8 MSBs of sens_freq (18 bit) = 29696)
  // do not forget to apply window factor! (lower alternative: 0x08 = 8192)
  byte reg_threshold = round(THRESHOLD_FREQ/WINDOW_FACTOR/1023);
  write_reg(sensor,THRESHOLD,reg_threshold);
  Serial.print("threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println((reg_threshold << 10)*WINDOW_FACTOR);
  //Serial.print("-registry "); Serial.println(reg_threshold, BIN);
  // enable interrupt upon new data (for now no interrupt pin is actually being used
  write_reg(sensor,NIRQ_ENGATE,0x30);
  // also enable automatic recharging again
  write_reg(sensor,RECHARGE,0x41);
  Serial.println();
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
  
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte reg_recharge_counter, byte sensor){
  flag ? Serial.println("Sensor, Temp , F_sens , F_ref, Rech_Count"),flag=false:0;
  Serial.print(sensor);Serial.print(" , ");
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.print(ref_freq);Serial.print(" , ");
  Serial.println(reg_recharge_counter,BIN);
  }

void float_to_bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

void wait(int millisecs){
  long int t1 = millis();
  while(millis()-t1<millisecs);
}
