char inChar;
char lastChar;
void setup(){
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(9600);
  Serial.println("Ready");       
}
 void loop() {
  digitalWrite(LED_BUILTIN,LOW); //turn off LED
  if (Serial.available() > 0) { 
   digitalWrite(LED_BUILTIN,HIGH); //flash LED everytime data is available
   inChar = Serial.read(); //read incoming data
  }   
  if (inChar != lastChar){
     Serial.print(inChar); //print data
     lastChar = inChar;
   }
}
