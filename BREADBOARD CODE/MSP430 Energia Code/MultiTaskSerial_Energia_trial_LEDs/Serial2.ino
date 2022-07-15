#define LED GREEN_LED

void setup2()
{
  //Serial.begin(115200);
  // initialize the digital pin as an output.
  pinMode(LED, OUTPUT);  
}

void loop2()
{
  //Serial.print("Task 2 millis: ");
  //Serial.println(millis());
  delay(500);
  digitalWrite(LED, HIGH);
}
