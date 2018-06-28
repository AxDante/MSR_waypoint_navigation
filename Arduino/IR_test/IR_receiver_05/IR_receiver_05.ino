#include <IRremote.h>

const int Pin_1 = 12;
const int Pin_2 = 13;

IRrecv irrecv1(Pin_1);
IRrecv irrecv2(Pin_2);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv1.enableIRIn(); // Start the receiver
  irrecv1.blink13(true);
  irrecv2.enableIRIn(); // Start the receiver
  irrecv2.blink13(true);
}

void loop() {
  if (irrecv1.decode(&results)) {
    if (results.decode_type == NEC) {
      Serial.print("IR01-NEC: ");
    } else if (results.decode_type == SONY) {
      Serial.print("IR01-SONY: ");
    } else if (results.decode_type == RC5) {
      Serial.print("IR01-RC5: ");
    } else if (results.decode_type == RC6) {
      Serial.print("IR01-RC6: ");
    } else if (results.decode_type == UNKNOWN) {
      Serial.print("IR01-UNKNOWN: ");
    }
    Serial.println(results.value, HEX);
    irrecv1.resume(); // Receive the next value
  }
  if (irrecv2.decode(&results)) {
    if (results.decode_type == NEC) {
      Serial.print("IR02-NEC: ");
    } else if (results.decode_type == SONY) {
      Serial.print("IR02-SONY: ");
    } else if (results.decode_type == RC5) {
      Serial.print("IR02-RC5: ");
    } else if (results.decode_type == RC6) {
      Serial.print("IR02-RC6: ");
    } else if (results.decode_type == UNKNOWN) {
      Serial.print("IR02-UNKNOWN: ");
    }
    Serial.println(results.value, HEX);
    irrecv2.resume(); // Receive the next value
  }
}
