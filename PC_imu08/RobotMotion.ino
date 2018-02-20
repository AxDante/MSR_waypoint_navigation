void threadMotion(){
  // List of input characters:

  // Transformation: '1': straight, '2': square, '3': L-shape (4th), '4': L-shape (1st)
  //                 '5': Z-shape; '6': T-shape, '7': S-shape
  // Liner Motion: 'R': right, 'L': left, 'B': backward, 'F': forward
  // Rotation: 'r': right turn(clockwise), 'l': left turn(counterclockwise)
  // 'S': stop all DC motors
  // '-': previous motion completed, awaiting next char input
  // 'k': secure shape (activate servo motors)
  
  // Always stop first while receiving stop command
  if (charInput == 'S')
  {
      isRobotRotating = false;
      isFinishRotation = true;
      stopMotorDC();
  }

  if (isRobotLinearMotion && timerLinearMotion - millis() > linerMotionStopTime){
    isRobotLinearMotion = false;
    charInput = '-';
  }
 
  // Priotorise heading correction before performing any motion when headingCorrection is ON.
  if (headingCorrectionDuringMotion){
    if (reachTargetHeading(heading_filtered) == 1){
        for (char j = 0; j < 4; j ++){
          switch(R_LeftTurn[RobotForm-1][j]){
            case 'f':
              Forward(j,true);
              break;
            case 'b':
              Backward(j,true);
              break;
            case 'r':
              Right(j,true);
              break;
            case 'l':
              Left(j,true);
              break;
            case '0':
              break;
          }
        }
        secureShape(RobotForm);
      }
      else if (reachTargetHeading( heading_filtered) == -1){
        for (char j = 0; j < 4; j ++){
          switch(R_RightTurn[RobotForm-1][j]){
            case 'f':
              Forward(j,true);
              break;
            case 'b':
              Backward(j,true);
              break;
            case 'r':
              Right(j,true);
              break;
            case 'l':
              Left(j,true);
              break;
            case '0':
              break;
          }
        }
        secureShape(RobotForm);
      }
      else{
        CharInputMotion();
      }
  }

  // Normal input-based motion when headingCorrection is OFF.
  else{
    CharInputMotion();
  }
}
void CharInputMotion(){
   switch(charInput)
  {
    case '1':
    {
      straight();
      RobotForm = 1;
      break;
    }
    case '2':
    {
      square();
      RobotForm = 2;
      break;
    }
    case '3':
    {
      L_4th();
      RobotForm = 3;
      break;
    }
    case '4':
    {
      L_1st();
      RobotForm = 4;
      break;
    }
    case '5':
    {
      Z_1_4();
      RobotForm = 5;
      break;
    }
    case '6':
    {
      plus();
      RobotForm = 6;
      break;
    }
    case '7':
    {
      S_1_3();
      RobotForm = 7;
      break;
    }
  }
  if (charInput == 'F')
  {  
    isRobotLinearMotion = true;
    timerLinearMotion = millis();
    for (char j = 0; j < 4; j ++){
      switch(M_Forward[RobotForm-1][j]){
        case 'f':
          Forward(j,false);
          break;
        case 'b':
          Backward(j,false);
          break;
        case 'r':
          Right(j,false);
          break;
        case 'l':
          Left(j,false);
          break;
        case '0':
          break;
      }
    }
  }
  if (charInput == 'B')
  {  
    isRobotLinearMotion = true;
    timerLinearMotion = millis();
    for (char j = 0; j < 4; j ++){
      switch(M_Backward[RobotForm-1][j]){
        case 'f':
          Forward(j,false);
          break;
        case 'b':
          Backward(j,false);
          break;
        case 'r':
          Right(j,false);
          break;
        case 'l':
          Left(j,false);
          break;
        case '0':
          break;
      }
    }
  }
  if (charInput == 'R')
  {  
    isRobotLinearMotion = true;
    timerLinearMotion = millis();
    for (char j = 0; j < 4; j ++){
      switch(M_Right[RobotForm-1][j]){
        case 'f':
          Forward(j,false);
          break;
        case 'b':
          Backward(j,false);
          break;
        case 'r':
          Right(j,false);
          break;
        case 'l':
          Left(j,false);
          break;
        case '0':
          break;
      }
    }
  }
  if (charInput == 'L')
  {  
    isRobotLinearMotion = true;
    timerLinearMotion = millis();
    for (char j = 0; j < 4; j ++){
      switch(M_Left[RobotForm-1][j]){
        case 'f':
          Forward(j,false);
          break;
        case 'b':
          Backward(j,false);
          break;
        case 'r':
          Right(j,false);
          break;
        case 'l':
          Left(j,false);
          break;
        case '0':
          break;
      }
    }
  }
  if (charInput == 'r' || charInput == 'l'){
    if (!isRobotRotating && !isFinishRotation){
      isRobotRotating = true;
      rotateStartHeading = heading_filtered+0.01;

      if (charInput == 'r') robotDir = 1;
      if (charInput == 'l') robotDir = -1;
      
      rotateTargetHeading = rotateStartHeading + robotDir * 90;
      rotateTargetLower = rotateTargetHeading - angleTolerance;
      rotateTargetUpper = rotateTargetHeading + angleTolerance;

      if (rotateTargetHeading > 360) rotateTargetHeading -= 360;
      if (rotateTargetHeading < 360) rotateTargetHeading += 360;

      Serial.println("=====Start Rotation=====");
      Serial.println("Start Heading : " + String(rotateStartHeading));
      Serial.println("Target Heading : " + String(rotateTargetHeading));
    }
    else{
      if (reachTargetHeading(heading_filtered) == 1){
        for (char j = 0; j < 4; j ++){
          switch(R_LeftTurn[RobotForm-1][j]){
            case 'f':
              Forward(j,true);
              break;
            case 'b':
              Backward(j,true);
              break;
            case 'r':
              Right(j,true);
              break;
            case 'l':
              Left(j,true);
              break;
            case '0':
              break;
          }
        }
        secureShape(RobotForm);
      }
      else if (reachTargetHeading( heading_filtered) == -1){
        for (char j = 0; j < 4; j ++){
          switch(R_RightTurn[RobotForm-1][j]){
            case 'f':
              Forward(j,true);
              break;
            case 'b':
              Backward(j,true);
              break;
            case 'r':
              Right(j,true);
              break;
            case 'l':
              Left(j,true);
              break;
            case '0':
              break;
          }
        }
        secureShape(RobotForm);
      }
      else if (reachTargetHeading( heading_filtered) == 0){
         isRobotRotating = false;
         isFinishRotation = true;
         stopMotorDC();
         Serial.println("=====End Rotation=====");
      }
    }
  }
  if (charInput == 'k')
  {
    secureShape(RobotForm);
  }

  if (prevCharInput != charInput){
    isFinishRotation = false;
    prevCharInput = charInput;
  }
}

