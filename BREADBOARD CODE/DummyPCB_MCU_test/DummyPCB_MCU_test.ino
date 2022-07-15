// Simple program to test flashing code to the Dummy PCB using the LaunchPad
#include<driverlib.h>

# define NSTBY_1 35  // P3.3
# define NSTBY_2 36  // P4.7

int count = 0;

void setup() {
  // put your setup code here, to run once:

  
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  Serial.println("This is the Payload Dummy PCB!");
  Serial.println(" ");

  // Setting NSTBY pins high
/*
  pinMode(NSTBY_1, OUTPUT);
  digitalWrite(NSTBY_1, HIGH);
  pinMode(NSTBY_2, OUTPUT);
  digitalWrite(NSTBY_2, HIGH);
*/
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3);
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
  GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);


  Serial.println("NSTBY pins are set HIGH");

  // setting RS485 pins ~RE and DE high to start transmission over RS485 Bus. Uses P2.6 on MCU                                $$ for RS485 transceiver test on Dummy PCB
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
  delay(1000);

  Serial.println("RE and DE RS485 pins are set HIGH");
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  
  Serial.println(count);
  count++;
  Serial.println("The MCU is communicating over serial");
  delay(1000);

  
}
