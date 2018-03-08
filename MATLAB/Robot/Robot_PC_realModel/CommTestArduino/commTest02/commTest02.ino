char inChar;
char lastChar;
void setup(){
  pinMode(LED_BUILTIN,OUTPUT);
  Serial.begin(9600);
  Serial.print("R");       
}
 void loop() {
  if (Serial.available() > 0) { 
   inChar = Serial.read(); //read incoming data
  }   
  if (inChar != lastChar){
     Serial.print(inChar); //print data
     lastChar = inChar;
   }
}
