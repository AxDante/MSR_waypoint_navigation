#include <IRremote.h>

IRsend irsend;

void setup() {
  Serial.begin(9600);
}

void loop() {
irsend.sendSony(0xA91A9472, 12);
Serial.print("ss");
    delay(300);
}
