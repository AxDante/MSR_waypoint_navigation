byte incomingByte1;
void setup(){
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(9600);
  Serial.println("Ready");       
}
 void loop() {
  digitalWrite(LED_BUILTIN,LOW); //turn off LED
  delay(500);
  if (Serial.available() > 0) { 
   digitalWrite(LED_BUILTIN,HIGH); //flash LED everytime data is available
   delay(500);
   incomingByte1 = Serial.read(); //read incoming data
   Serial.println(incomingByte1); //print data
  }
}
