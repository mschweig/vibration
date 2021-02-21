#include <Arduino.h>

int analogPin = A1; 
int val = 0;  // variable to store the value read

void setup() {
  Serial.begin(115200);           //  setup serial
  analogReadResolution(12);       // set ADC to 12 bit
}

void loop() {
  val = analogRead(analogPin);  // read the input pin
  Serial.println(val*(3.3/4096));          // debug value
}