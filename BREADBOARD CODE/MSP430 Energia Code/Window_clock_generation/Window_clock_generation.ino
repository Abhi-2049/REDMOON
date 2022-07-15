#include <driverlib.h>



// Window clock signal pin
#define WCK_1 8                       // P3.4
float clock_check_WCK = 0;                  // to check clock signal value
float clock_check_default = 0;
int count = 0;

// for debugging
//#define LED RED_LED


void setup() {
  // put your setup code here, to run once:

  //pinMode(LED,OUTPUT);
  //digitalWrite(LED,HIGH);
  
  pinMode(WCK_1, OUTPUT);                                                                                                   // Try P3DIR |= 0x10 ; if this does not work
  P3SEL1 |= 0x10 ;   //Set 3.4 to 1 to select SMCLK 
  
  
  // Set DCO frequency to 1 MHz
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
  
  // Set SMCLK = DCO with frequency divider of 32, should give an SMCLK of 31250 Hz
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);                                                   
  //CS_enableClockRequest(CS_SMCLK);
  clock_check_WCK = CS_getSMCLK ();

//  // trying to reset eUSCI to get the serial output to work, code from demo example
//  UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
//  UCA0CTLW0 |= UCSSEL__ACLK;               // CLK = SMCLK
//
//  UCA0BR0 = 52;                             // 8000000/16/9600
//  UCA0BR1 = 0x00;
//  UCA0MCTLW |= UCOS16 | UCBRF_1;
//  UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI

  
  
  //trying to light up the green LED (LED2)
  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
  GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);


  // Set SMCLK back to original value
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  clock_check_default = CS_getSMCLK ();
  

  Serial.begin(9600);
  while(!Serial);

  Serial.println("Serial connection established!");



  // Set DCO frequency to 1 MHz
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
  // Set SMCLK = DCO with frequency divider of 32, should give an SMCLK of 31250 Hz
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);
  


  
}

void loop() {
  // put your main code here, to run repeatedly: 
  //Serial.println("What's happening?");

  //delay(1000);

  // Set DCO and SMCLK back to original value
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  clock_check_default = CS_getSMCLK ();

  
  __delay_cycles(250000);
  GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
  Serial.println("Clock values:");
  Serial.println(clock_check_WCK);
  Serial.println(clock_check_default);
  Serial.println(count);

  __delay_cycles(250000);

  // Set DCO frequency to 1 MHz
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
  // Set SMCLK = DCO with frequency divider of 32, should give an SMCLK of 31250 Hz
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);

  count++;

  __delay_cycles(250000);
  
}
