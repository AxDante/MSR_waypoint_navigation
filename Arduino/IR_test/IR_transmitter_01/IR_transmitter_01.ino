void setup() {
  Serial.begin(9600);  /* Define baud rate for serial communication */
}

void loop() {
  int count;
  for(count = 0; count<100; count++)
  {
    Serial.println(count);
    delay(100);
  }
}
