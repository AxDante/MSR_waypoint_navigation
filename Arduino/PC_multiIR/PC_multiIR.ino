#include <IRremote.h>

// To support more than 5 receivers, remember to change the define
// IR_PARAMS_MAX in IRremoteInt.h as well.
#define RECEIVERS 2

IRrecv *irrecvs[RECEIVERS];

decode_results results;

void setup()
{
  Serial.begin(9600);

  irrecvs[0] = new IRrecv(12); // Receiver #0: pin 2
  irrecvs[1] = new IRrecv(13); // Receiver #1: pin 3

  for (int i = 0; i < RECEIVERS; i++)
    irrecvs[i]->enableIRIn();
}

void loop() {
  for (int i = 0; i < 2; i++)
  {
    if (irrecvs[i]->decode(&results))
    {
      Serial.print("Receiver #");
      Serial.print(i);
      Serial.print(":");
      Serial.println(results.value, HEX);
      irrecvs[i]->resume();
    }
  }
}
