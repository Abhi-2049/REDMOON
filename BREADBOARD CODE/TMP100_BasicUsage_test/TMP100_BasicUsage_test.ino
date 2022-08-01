/*
Read the temperature from an LM75-derived temperature sensor, and display it
in Celcius every 250ms. Any LM75-derived temperature should work.
*/

#include <Temperature_LM75_Derived.h>

// The Generic_LM75 class will provide 9-bit (±0.5°C) temperature for any
// LM75-derived sensor. More specific classes may provide better resolution.
TI_TMP100 temperature;

void setup() {
  while(!Serial) {}
  
  Serial.begin(9600);
  Serial.println("TMP100 Sensor Readout Test");

  Wire.begin();
}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(temperature.readTemperatureC());
  Serial.println(" C");

  delay(1000);
}
