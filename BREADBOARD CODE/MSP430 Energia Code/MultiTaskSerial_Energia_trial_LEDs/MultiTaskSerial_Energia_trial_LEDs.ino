#define LED_R RED_LED
#define LED_G 26
void setup()
{
  //Serial.begin(115200);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT); 
}

void loop()
{
  //Serial.print("Task 1 millis: ");
  //Serial.println(millis());
  digitalWrite(LED_G, HIGH);
  delay(1000);
  digitalWrite(LED_R, HIGH);
}
