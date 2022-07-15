// Test script to measure payload PCB power consumption and temperature during standalone functional tests
/*
 * Test Setup:
 * 
 * Radiation Payload MAIN PCB
 * MSP430 LaunchPad
 * Arduino Uno R3
 * Breadboard Supply / Lab Power Supply
 * INA219
 * MCP9600 with E-Type thermocouple
 * 
 */

//---------------------------------------------- INA219 libraries, definitions & variables ----------------------------------------------------


#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

uint32_t currentFrequency;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;


//---------------------------------------------- MCP9600 libraries, definitions & variables ----------------------------------------------------

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS (0x67)

Adafruit_MCP9600 mcp;

//variables
float tcouple_temp_hot, tcouple_temp_cold ;          // hot and cold junction temperatures, without offset
int tcouple_adc ;


//----------------------------------------------------------------------------------------------------------------------------------------------






void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) {delay(1);}

  Serial.println("Payload Performance Test: Power Consumption & Temperature Monitor Calibration");

  // ------------------------------ INA219 setup ---------------------------------------------
  

  Serial.println("INA219 Sensor Setup");
    
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();

  
  // ------------------------------ MCP9600 setup ---------------------------------------------

  Serial.println("MCP9600 Thermocouple Amplifier Setup");

    /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
    if (! mcp.begin(I2C_ADDRESS)) {
        Serial.println("Sensor not found. Check wiring!");
        while (1);
    }

  Serial.println("Found MCP9600!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);                          // Set ADC Resolution here
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_E);                                  // Change thermocouple type here
  Serial.print("Thermocouple type set to ");
  switch (mcp.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp.setFilterCoefficient(3);                                              // filter coefficient
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp.getFilterCoefficient());

  mcp.setAlertTemperature(1, 30);                                           // Alert temperature - not used in this test
  Serial.print("Alert #1 temperature set to ");
  Serial.println(mcp.getAlertTemperature(1));
  mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp.enable(true);

  Serial.println(F("------------------------------"));
  Serial.println("Bus Voltage(V), Shunt Voltage(mV), Load Voltage(V), Current(mA), Power(mW), Hot Junction(C), Cold Junction(C), ADC(uV)");


  //--------------------------------------------------------------------------------

}

void loop(void) 
{


  //---------------------------------------- INA219 loop code ---------------------------------
  
//  float shuntvoltage = 0;
//  float busvoltage = 0;
//  float current_mA = 0;
//  float loadvoltage = 0;
//  float power_mW = 0;



  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  /*
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
  */

  //---------------------------------------- MCP9600 loop code ---------------------------------

  /*
  Serial.print("Hot Junction: "); Serial.println(mcp.readThermocouple());
  Serial.print("Cold Junction: "); Serial.println(mcp.readAmbient());
  Serial.print("ADC: "); Serial.print(mcp.readADC() * 2); Serial.println(" uV");
  */

  tcouple_temp_hot = mcp.readThermocouple();
  tcouple_temp_cold = mcp.readAmbient() ;          // hot and cold junction temperatures, without offset
  tcouple_adc = (mcp.readADC() * 2) ;
  

  //------------------------------------------ Print Measured Values --------------------------------

  Serial.print(busvoltage);Serial.print(" , ");
  Serial.print(shuntvoltage);Serial.print(" , ");
  Serial.print(loadvoltage);Serial.print(" , ");
  Serial.print(current_mA);Serial.print(" , ");
  Serial.print(power_mW);Serial.print(" , ");
  Serial.print(tcouple_temp_hot);Serial.print(" , ");
  Serial.print(tcouple_temp_cold);Serial.print(" , ");
  Serial.println(tcouple_adc);



  delay(1000);
}
