
int peltier = 3; //The N-Channel MOSFET is on digital pin 3
int power = 0; //Power level fro 0 to 99%
int peltier_level = map(power, 0, 99, 0, 255); //This is a value from 0 to 255 that actually controls the MOSFET

void setup(){
Serial.begin(9600);

//pinMode(peltier, OUTPUT);
}

void loop(){
char option;

if(Serial.available() > 0)
{
option = Serial.read();
if(option == 'a') 
power += 5;
else if(option == 'z')
power -= 5;

if(power > 99) power = 99;
if(power < 0) power = 0;

peltier_level = map(power, 0, 99, 0, 255);
}

Serial.print("Power=");
Serial.print(power);
Serial.print(" PLevel=");
Serial.println(peltier_level);

analogWrite(peltier, peltier_level); //Write this new value out to the port

}
