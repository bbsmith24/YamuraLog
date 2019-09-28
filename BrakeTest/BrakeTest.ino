/*
  DigitalReadSerial

  Reads a digital input on pin 2, prints the result to the Serial Monitor

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/DigitalReadSerial
*/

// digital pin 2 has a pushbutton attached to it. Give it a name:
#define SIGNAL 10
#define LED 2

// the setup routine runs once when you press reset:
void setup() {
  pinMode(SIGNAL, INPUT);
  pinMode(LED, OUTPUT);
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }
}

// the loop routine runs over and over again forever:
void loop() 
{
  // read the input pin:
  if(digitalRead(SIGNAL) == HIGH)
  {
     digitalWrite(LED, HIGH);
  }
  else
  {
     digitalWrite(LED, LOW);
  }
}
