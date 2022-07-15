// Adapted from: DummyPCB_RS485_transceiver_test

// Sketch to test communication over data bus, commands and message sending on Dummy PCB, using an Arduino (and logic analyzer)



// Declare pin for WCK signal generation
//#define PWM_PIN 9

// GPIO pin to enable/disable listening/sending data over RS485
#define RS485_RE_PIN 10

unsigned long int ON_Time = 0;

//String data_rcvd;
//char data_rcvd[]="Arduino";

// Command and Telemetry variables
#define START_BYTE 0x3A
#define STOP_BYTE 0x0A
#define RAD_PAYLOAD_ADDR 0x00AA

byte message_buffer_to_send[16];
const int message_size = 16;              // number of bytes to be read

byte data_buffer_to_receive[16];
const int data_message_size = 16;        // number of bytes to be read




void setup() {

  // Set RE/DE driving GPIO pin to Output mode
  pinMode(RS485_RE_PIN, OUTPUT);
  // Set RE/DE pin low to prevent Data Bus pollution
  digitalWrite(RS485_RE_PIN, LOW);


/*
    // WCK signal geenration on Pin 9 of Arduino
    pinMode(PWM_PIN, OUTPUT); // Set PWM PIN on Arduino
    TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
    OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
    OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1

*/  
    Serial.begin(9600);       // initialize UART with baud rate of 9600 bps
    Serial.println("Data Bus Communication Test");
}

void loop() {                                                        
    
    
  //data_rcvd = Serial.readStringUntil('\n');   // read from serial buffer until the newline character and save to data_rcvd

    ON_Time = millis();
    
    
    // Construct message buffer to be sent
    message_buffer_to_send[0] =  START_BYTE;     // Start Byte - 0x3A
    message_buffer_to_send[1] =  0x00;           // Address Byte - Use OBC / PPU Address ??                                $$
    message_buffer_to_send[2] =  0xAA;           // Address Byte - Use Payload Address-0x00AA for now                      $$
    message_buffer_to_send[3] =  0x00;           // command byte
    message_buffer_to_send[4] =  0x06;           // command byte - 1 for payload ON Time
    message_buffer_to_send[5] =  0x00;           // 8 data bytes from here, unused since there are no arguments for commands
    message_buffer_to_send[6] =  0x00;
    message_buffer_to_send[7] =  0x00;
    message_buffer_to_send[8] =  0x00;
    message_buffer_to_send[9] =  0x00;
    message_buffer_to_send[10] = 0x00;
    message_buffer_to_send[11] = 0x00;
    message_buffer_to_send[12] = 0x00;
    message_buffer_to_send[13] = 0x00;           // = crc_check_payload[0];
    message_buffer_to_send[14] = 0x00;           // = crc_check_payload[1];
    message_buffer_to_send[15] = STOP_BYTE;      // Stop Byte - 0x0A

/*
    // driveRS485 RE/DE pins high to enable data transmission
    digitalWrite(RS485_RE_PIN, HIGH);
    
    // Send message buffer
    Serial.write(message_buffer_to_send,message_size);

    // drive RS485 RE/DE pins low to start listening
    digitalWrite(RS485_RE_PIN, LOW);
    
    // print what was sent
    for (int i = 0; i < message_size; i++) {
      Serial.print(message_buffer_to_send[i],HEX);
    }
    Serial.println();

*/    
    

    // wait before reading from the serial port
    delay(1000);

     
     // read x bytes of data over the bus
    Serial.readBytes(data_buffer_to_receive,data_message_size);

    //Serial.readBytesUntil('\n',message_buffer,message_size);

    
    // print what was recieved
    Serial.print("Number of bytes received by Arduino: ");Serial.println(Serial.available());
    for (int i = 0; i < data_message_size; i++) {
      Serial.print(data_buffer_to_receive[i],HEX);
    }
    Serial.println();
    Serial.print("Data Bus Communications ON for (ms): ");Serial.println(ON_Time);
    
    //Longer delay before sending commands again, 
    delay(4000);


  //Serial.println("No data received over Serial");

  //Serial.write('1'); // send the char '1' to serial
  //delay(1000);
  //Serial.write('0'); // send the char '0' to serial
  //delay(1000);
  
}
