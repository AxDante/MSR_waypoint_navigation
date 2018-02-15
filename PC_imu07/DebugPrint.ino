void debugPrint(){
  Serial.println("======================================");
  Serial.println("Time: " + String(millis()/1000) + "s");
  Serial.println("Char Input: " + String(charInput));
  Serial.println("Current Shape: " + String(S_Shape[RobotForm-1]));
  //Serial.println("mx:" + String(magX) + "my:" + String(magY);
  Serial.println("Heading_raw: " + String(heading_raw) +  ", Heading_cal: " + String(heading_filtered));
  if (isRobotRotating){
    Serial.println("degToTurn:" + String(degreeToPos));
    Serial.println("target: " + String(rotateTargetHeading) + ", upperTol: " + String(rotateTargetUpper) +  ", lowerTol: " + String(rotateTargetLower));
    String stringDirection;
    switch(robotDir){
      case 1:
        stringDirection = "Clockwise";
        break;
      case 0:
        stringDirection = "Stopped";
        break;
      case -1:
        stringDirection = "CounterClockwise";
        break;
    }
    Serial.println("Robot is currently turning " + stringDirection + ".");
  
  }
}
