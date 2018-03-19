void debugPrint(){

  String stringRotateDirection;
  switch(robotDir){
    case 1:
      stringRotateDirection = "Clockwise";
      break;
    case 0:
      stringRotateDirection = "Stopped";
      break;
    case -1:
      stringRotateDirection = "CounterClockwise";
      break;
    }

  String stringLinearDirection;
  switch(charInput){
    case 'R':
      stringLinearDirection = "Right";
      break;
    case 'B':
      stringLinearDirection = "Backward";
      break;
    case 'L':
      stringLinearDirection = "Left";
      break;
    case 'F':
      stringLinearDirection = "Forward";
      break;
    default:
    break;
  }

  Serial.println("===========================================");
  Serial.println("Time: " + String(millis()/1000) + "s");
  Serial.println("Char Input: " + String(charInput));
  Serial.println("Current Shape: " + String(S_Shape[RobotForm-1]));
  //Serial.println(" (Debug) mx:" + String(magX) + "my:" + String(magY);
  Serial.println("Heading_raw: " + String(heading_raw) +  "deg,  Heading_cal: " + String(heading_filtered)+ " deg");
  if (isRobotRotation || isRobotRotationAdjust){
    Serial.println("degToTurn:" + String(degreeToPos));
    Serial.println("World Target Heading:" + String(worldRobotTargetHeading) + ", Self Target Heading: " + String(rotateTargetHeading) + ", upperTol: " + String(rotateTargetUpper) +  ", lowerTol: " + String(rotateTargetLower));

    Serial.println("Robot is currently turning " + stringRotateDirection + ".");
  
  }
  if (isRobotLinearMotion){
    Serial.print("Robot is currently moving " + String(stringLinearDirection) + ".");
    if (robotMode > 1){
      Serial.print("  " + String(millis()-timerLinearMotion) + " sec remaining.");
    }
    Serial.println();
  }

  Serial.println("(Debug) millis: " + String(millis())+ " , timerLinearMotion: " + String(timerLinearMotion));
  Serial.println("(Debug) Current DC Motor Power :" + String(DCMotorPower()));
}

void threadSerial()
{
  if(robotMode <= 3){
    if(Serial.available() > 0){
      charInput = Serial.read();  
    };
  }
  else if (robotMode == 4){

  }
}
