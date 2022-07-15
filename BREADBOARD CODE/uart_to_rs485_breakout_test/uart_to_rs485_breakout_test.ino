// Sample sketch to test UART tramsmission and RS485 transceiver breakout operation, using an Arduino and logic analyzer

void setup() {
    Serial.begin(9600);       // initialize UART with baud rate of 9600 bps
}

void loop() {
  if (Serial.available()) {
    char data_rcvd = Serial.read();   // read one byte from serial buffer and save to data_rcvd

    if (data_rcvd == '1') digitalWrite(13, HIGH); // switch LED On
    if (data_rcvd == '0') digitalWrite(13, LOW);  // switch LED Off
  }

  Serial.write('1'); // send the char '1' to serial
  delay(1000);
  Serial.write('0'); // send the char '0' to serial
  delay(1000);
  
}
