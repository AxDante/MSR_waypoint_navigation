#include <IRremote.h>
unsigned int rawCodes[] = {5,5,1,5,0,2,0,1,5};
IRsend irsend;

void setup()
{
  Serial.begin(9600);
}

void loop() {
  //if (Serial.read() != -1) {
    for (int i = 0; i < 100; i++) {
      Serial.print("Start sending ...");
      irsend.sendNEC(0x57E34AB5, 32);
      delay(300);
    //}
  }
}
