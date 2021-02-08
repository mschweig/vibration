#include <Arduino.h>

int analogPin = A3; // Pin, der gelesen werden soll: Pin A3
int val = 0; // Variable, die den gelesenen Wert speichert

void setup() {
  Serial.begin(9600); // Setup der seriellen Verbindung
  Serial.println("Setup completed");
}

void loop() {
  val = analogRead(analogPin); // Pin einlesen
  Serial.println(val); // Wert ausgeben
}