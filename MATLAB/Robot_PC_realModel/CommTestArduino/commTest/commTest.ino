void setup() {
  Serial.begin(9600);
  Serial.print("Begin");
  pinMode(LED_BUILTIN, OUTPUT);
}
char inChar;
bool isInput;
void loop() {
  if (Serial.available())
  {
    inChar = Serial.read();
  }
  if (inChar != NULL){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(300);                       // wait for a second
  }
  Serial.print(inChar);
  delay(300);
}
