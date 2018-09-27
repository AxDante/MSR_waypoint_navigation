#include <IRremote.h>

IRsend irsend;

void setup() {
  Serial.begin(9600);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
}

void loop() {
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  irsend.sendSony(0xA91A1111, 12);
  delay(300);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  irsend.sendSony(0xA91A2222, 12);
  delay(300);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  irsend.sendSony(0xA91A3333, 12);  
  delay(300); 
}
