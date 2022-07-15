// AS, 29/04: example sketch modified to check compatibility of driverlib with other Energia functions



/******************************************************************************
*  GPIO - Blink LED
*
*  Description: This example demonstrates how to set a GPIO pin as an output
*  pin and toggle the output of the pin
*
*                                    MSP430
*                              -----------------
*                             |                 |
*                             |             P1.0|-->LED
*                             |                 |
*                             |                 |
*                             |                 |
*
* Tested On: MSP430FR5969
* Author: Zack Lalanne
******************************************************************************/
#include <driverlib.h>

void setup()
{
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    pinMode(25,OUTPUT);
    digitalWrite(25,HIGH);

    Serial.begin(9600);
    
    
}

void loop()
{
    // Wait for a second
    //digitalWrite(25,HIGH);
    Serial.println("Is this real life?");
    delay(1000);
    digitalWrite(25,LOW);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    delay(1000);
    digitalWrite(25,HIGH);
}
