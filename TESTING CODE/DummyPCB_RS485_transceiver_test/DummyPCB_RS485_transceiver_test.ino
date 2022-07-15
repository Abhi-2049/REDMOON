// Sketch to test UART tramsmission and RS485 transceiver operation on Dummy PCB, using an Arduino (and logic analyzer)
// Arduino reads over the serial pins and prints the result to the Serial Monitor

//Added WCK signal generation too for Payload PCB test





// Declare pin for WCK signal generation
#define PWM_PIN 9

String data_rcvd;
//char data_rcvd[]="Arduino";

void setup() {

    // WCK signal geenration on Pin 9 of Arduino
    pinMode(PWM_PIN, OUTPUT); // Set PWM PIN on Arduino
    TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
    OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
    OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1

  
    Serial.begin(9600);       // initialize UART with baud rate of 9600 bps
    Serial.println("Serial Communication Test");
}

void loop() {
  while(Serial.available()) {
    data_rcvd = Serial.readStringUntil('\n');   // read from serial buffer until the newline character and save to data_rcvd

    //if (data_rcvd == '1') digitalWrite(13, HIGH); // switch LED On
    //if (data_rcvd == '0') digitalWrite(13, LOW);  // switch LED Off
    Serial.println(data_rcvd);
  }

  //Serial.println("No data received over Serial");

  //Serial.write('1'); // send the char '1' to serial
  //delay(1000);
  //Serial.write('0'); // send the char '0' to serial
  //delay(1000);
  
}
