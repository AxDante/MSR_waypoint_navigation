void setup() {
  Serial.begin(9600);

}
char inChar;
void loop() {
  if (Serial.available())
  {
    inChar = Serial.read();
    Serial.print(inChar);
  }
}
