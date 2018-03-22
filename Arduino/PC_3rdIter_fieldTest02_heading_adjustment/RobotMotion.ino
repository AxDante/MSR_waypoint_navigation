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
  if (charInput == 'S' || charInput == '1' ||charInput == '2' ||charInput == '3' ||charInput == '4' ||charInput == '5' ||charInput == '6' ||charInput == '7' )
  {
      isRobotRotation = false;
      isFinishRotation = true;
      stopMotorDC();
  }


 
  // Priotorise heading correction before performing any motion when headingCorrection is ON.
  if (headingCorrectionDuringMotion && !isRobotRotation){
    if (reachTargetHeading(heading_filtered) == 1){
        isRobotRotationAdjust = true;
        if (debugPrintActive) Serial.println("   --> Robot heading deviated, auto correcting now.(turning right)");
        
        for (char j = 0; j < 4; j ++){
          switch(R_LeftTurn[RobotForm-1][j]){
            case 'f':
              Forward(j);
              break;
            case 'b':
              Backward(j);
              break;
            case 'r':
              Right(j);
              break;
            case 'l':
              Left(j);
              break;
            case '0':
              break;
          }
        }
        secureShape(RobotForm);
      }
      else if (reachTargetHeading(heading_filtered) == -1){
        isRobotRotationAdjust = true;
        if (debugPrintActive) Serial.println("   --> Robot heading deviated, auto correcting now.(turning left)");
        
        for (char j = 0; j < 4; j ++){
          switch(R_RightTurn[RobotForm-1][j]){
            case 'f':
              Forward(j);
              break;
            case 'b':
              Backward(j);
              break;
            case 'r':
              Right(j);
              break;
            case 'l':
              Left(j);
              break;
            case '0':
              break;
          }
        }
        secureShape(RobotForm);
      }
      else{
        if (isRobotRotationAdjust){
          isRobotRotationAdjust = false;
          stopMotorDC();
        } else{
           CharInputMotion();
        }
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
    if (!isRobotLinearMotion){
      isRobotLinearMotion = true;
      timerLinearMotion = millis();
    }
    for (char j = 0; j < 4; j ++){
      switch(M_Forward[RobotForm-1][j]){
        case 'f':
          Forward(j);
          break;
        case 'b': 
          Backward(j);
          break;
        case 'r':
          Right(j);
          break;
        case 'l':
          Left(j);
          break;
        case '0':
          break;
      }
    }
    if (secureShapeLinearActive){
      if (counterSecureShape < counterSecureShapeMaxLinearCount){
        counterSecureShape += 1;
      }else{
        counterSecureShape = 0;
        secureShape(RobotForm);
      }
    }
  }
  if (charInput == 'B')
  {  
    if (!isRobotLinearMotion){
      isRobotLinearMotion = true;
      timerLinearMotion = millis();
    }
    for (char j = 0; j < 4; j ++){
      switch(M_Backward[RobotForm-1][j]){
        case 'f':
          Forward(j);
          break;
        case 'b':
          Backward(j);
          break;
        case 'r':
          Right(j);
          break;
        case 'l':
          Left(j);
          break;
        case '0':
          break;
      }
    }
    if (secureShapeLinearActive){
      if (counterSecureShape < counterSecureShapeMaxLinearCount){
        counterSecureShape += 1;
      }else{
        counterSecureShape = 0;
        secureShape(RobotForm);
      }
    }
  }
  if (charInput == 'R')
  {  
    if (!isRobotLinearMotion){
      isRobotLinearMotion = true;
      timerLinearMotion = millis();
    }
    for (char j = 0; j < 4; j ++){
      switch(M_Right[RobotForm-1][j]){
        case 'f':
          Forward(j);
          break;
        case 'b':
          Backward(j);
          break;
        case 'r':
          Right(j);
          break;
        case 'l':
          Left(j);
          break;
        case '0':
          break;
      }
    }
    if (secureShapeLinearActive){
      if (counterSecureShape < counterSecureShapeMaxLinearCount){
        counterSecureShape += 1;
      }else{
        counterSecureShape = 0;
        secureShape(RobotForm);
      }
    }
  }
  if (charInput == 'L')
  {  
    if (!isRobotLinearMotion){
      isRobotLinearMotion = true;
      timerLinearMotion = millis();
    }
    for (char j = 0; j < 4; j ++){
      switch(M_Left[RobotForm-1][j]){
        case 'f':
          Forward(j);
          break;
        case 'b':
          Backward(j);
          break;
        case 'r':
          Right(j);
          break;
        case 'l':
          Left(j);
          break;
        case '0':
          break;
      }
    }
    if (secureShapeLinearActive){
      if (counterSecureShape < counterSecureShapeMaxLinearCount){
        counterSecureShape += 1;
      }else{
        counterSecureShape = 0;
        secureShape(RobotForm);
      }
    }
  }
  if (charInput == 'r' || charInput == 'l'){
    if (!isRobotRotation && !isFinishRotation){
      isRobotRotation = true;
      rotateStartHeading = heading_filtered+0.01;

      if (charInput == 'r') robotDir = 1;
      if (charInput == 'l') robotDir = -1;
      
      if (initialPositionAsWorldFrame){
        rotateTargetHeading = rotateStartHeading + robotDir * 90;
      }else{
        worldRobotTargetHeading +=  robotDir * 90;
        rotateTargetHeading = worldRobotTargetHeading;
      }
      rotateTargetLower = rotateTargetHeading - angleTolerance;
      rotateTargetUpper = rotateTargetHeading + angleTolerance;

      if (rotateTargetHeading > 360) rotateTargetHeading -= 360;
      if (rotateTargetHeading < 360) rotateTargetHeading += 360;
      if (debugPrintActive){
        Serial.println("=====Start Rotation=====");
        Serial.println("Start Heading : " + String(rotateStartHeading));
        Serial.println("Target Heading : " + String(rotateTargetHeading));
      }
    }
    else{
      if (reachTargetHeading(heading_filtered) == 1){
        for (char j = 0; j < 4; j ++){
          switch(R_LeftTurn[RobotForm-1][j]){
            case 'f':
              Forward(j);
              break;
            case 'b':
              Backward(j);
              break;
            case 'r':
              Right(j);
              break;
            case 'l':
              Left(j);
              break;
            case '0':
              break;
          }
        }
        if (secureShapeActive){
          if (counterSecureShape < counterSecureShapeMaxCount){
            counterSecureShape += 1;
          }else{
            counterSecureShape = 0;
            secureShape(RobotForm);
          }
        }
      }
      else if (reachTargetHeading( heading_filtered) == -1){
        for (char j = 0; j < 4; j ++){
          switch(R_RightTurn[RobotForm-1][j]){
            case 'f':
              Forward(j);
              break;
            case 'b':
              Backward(j);
              break;
            case 'r':
              Right(j);
              break;
            case 'l':
              Left(j);
              break;
            case '0':
              break;
          }
        }
        if (secureShapeActive){
          if (counterSecureShape < counterSecureShapeMaxCount){
            counterSecureShape += 1;
          }else{
            counterSecureShape = 0;
            secureShape(RobotForm);
          }
        }
      }
      else if (reachTargetHeading(heading_filtered) == 0){
         isRobotRotation = false;
         isFinishRotation = true;
         stopMotorDC();
         if (debugPrintActive) Serial.println("=====End Rotation=====");
      }
    }
  }
  if (charInput == 'k')
  {
    if (secureShapeActive){
      if (counterSecureShape < counterSecureShapeMaxCount){
        counterSecureShape += 1;
      }else{
        counterSecureShape = 0;
        secureShape(RobotForm);
      }
    }
  }
  if (prevCharInput != charInput){
    isFinishRotation = false;
    prevCharInput = charInput;
  }
  //Serial.print("k");
  if (robotMode == 3 || robotMode == 4){
    if (isRobotLinearMotion &&  millis() - timerLinearMotion > linerMotionStopTime){
      isRobotLinearMotion = false;
      stopMotorDC();
    }
    //Serial.print("s");
    if (robotMode >= 4){
      //Serial.print("i");
      if (!isRobotLinearMotion && !isRobotRotation && !isRobotRotationAdjust){
        //Serial.print("t");
        charInput = stringStream[0];
        stringStream.remove(0, 1);
      }
    }
  }
}


