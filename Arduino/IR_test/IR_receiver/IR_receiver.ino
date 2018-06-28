void setup() {
  Serial.begin(9600);  /* Define baud rate for serial communication */
}

void loop() {
if(Serial.available())  /* If data is available on serial port */
  {
    Serial.print(char(Serial.read()));  /* Print character received on to the serial monitor */
  }
}
