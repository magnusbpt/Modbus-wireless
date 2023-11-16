// Clignotement de LED du Raspberry Pi Pico
// https://tutoduino.fr/
// Copyleft 2020
   
#include "Arduino.h"
  
void setup() {
  Serial.begin(9600);
  // Declare la broche sur laquelle la LED est  
  // reliee comme une sortie
  pinMode(LED_BUILTIN, OUTPUT);
}
   
void loop() {
  Serial.println("LED ON");
  // Passer le sortie à l'état HAUT pour allumer la LED
  digitalWrite(LED_BUILTIN, HIGH);
     
  // Attendre 1 seconde, pendant ce temps la LED reste allumee
  delay(1000);
     
  Serial.println("LED OFF");
  // Passer le sortie à l'état BAS pour eteindre la LED
  digitalWrite(LED_BUILTIN, LOW);    
   
  // Attendre 1 seconde, pendant ce temps la LED reste donc éteinte
  delay(1000);
}