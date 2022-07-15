#include <driverlib.h>



// Window clock signal pin
#define WCK_1 8                       // P3.4
//int clock_check = 0;                  // to check clock signal value

// for debugging
//#define LED RED_LED


void setup2() {
  // put your setup code here, to run once:

  //pinMode(LED,OUTPUT);
  //digitalWrite(LED,HIGH);
  
  pinMode(WCK_1, OUTPUT);                                                                                                   // Try P3DIR |= 0x10 ; if this does not work
  P3SEL1 |= 0x10 ;   //Set 3.4 to 1 to select SMCLK 
  
  
  // Set DCO frequency to 1 MHz
  CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_0);
  
  // Set SMCLK = DCO with frequency divider of 32, should give an SMCLK of 31250 Hz
  CS_clockSignalInit(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_32);
  CS_enableClockRequest(CS_SMCLK);
  //clock_check = CS_getSMCLK ();

  //digitalWrite(LED,LOW);
}

void loop2() {
  // put your main code here, to run repeatedly: 
  Serial.println("What's happening?");
  
}
