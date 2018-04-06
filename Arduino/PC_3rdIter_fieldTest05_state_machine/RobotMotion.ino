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

  /*
  Robot State:  0: Stop all motions(Auto adjust will not trigger); 
                1: Idle;
                2: Idle, angle adjusting;
                3: Linear Motion;
                4: Linear Motion, angle adjusting;
                5: Rotation
                6: Transformation;
                7: Transformation, angle adjusting;
                8: Linear Motion (Slight Adjust);
  */

  checkRobotState();
  stateMotion();
}

void checkRobotState(){
  
  if (charInput == 'S'){
    robotState = 0;
  }

  if (charInput == 's'){
    robotState = 1;
  }

  if (charInput == 'F' ||charInput == 'R' ||charInput == 'B' ||charInput == 'L'){
    if ( robotState == 0 || robotState == 1 || robotState == 3 || robotState == 8){
      robotState = 3;
    }
  }

  if (charInput == 'O' ||charInput == 'P'){
    if ( robotState == 0 || robotState == 1 || robotState == 3 || robotState == 8){
      robotState = 5;
      rotateStartHeading = heading_filtered;

      if (charInput == 'P') robotDir = 1;
      if (charInput == 'O') robotDir = -1;
      
      if (initialPositionAsWorldFrame){
        rotateTargetHeading = rotateStartHeading + robotDir * 90;
      } else {
        worldRobotTargetHeading +=  robotDir * 90;
        rotateTargetHeading = worldRobotTargetHeading;
      }

      if (rotateTargetHeading > 180) rotateTargetHeading -= 360;
      if (rotateTargetHeading < -180) rotateTargetHeading += 360;

      rotateTargetLower = rotateTargetHeading - angleTolerance;
      rotateTargetUpper = rotateTargetHeading + angleTolerance;

      if (rotateTargetLower > 180) rotateTargetLower -= 360;
      if (rotateTargetLower < -180) rotateTargetLower += 360;

      if (rotateTargetUpper > 180) rotateTargetUpper -= 360;
      if (rotateTargetUpper < -180) rotateTargetUpper += 360;

    }
  }

  if (charInput == '1' || charInput == '2' ||charInput == '3' ||charInput == '4' ||charInput == '5' ||charInput == '6' ||charInput == '7' 
      || charInput == '8'){
    robotState = 6;
  }    

  if (charInput == 'f' || charInput == 'r' ||charInput == 'b' ||charInput == 'l'){
    if ( robotState == 0 || robotState == 1 || robotState == 3 || robotState == 8){
      robotState = 8;
    }
  }
}


void stateMotion()
{

  rotateTargetHeading = worldRobotTargetHeading;   
  rotateTargetLower = rotateTargetHeading - angleTolerance;
  rotateTargetUpper = rotateTargetHeading + angleTolerance; 
  degreeToPos = rotateTargetHeading - heading_filtered;
  
  if (robotState == 0){
    stopMotorDC();
  }
  else if (robotState == 1){
    stopMotorDC();
    if (reachTargetHeading(heading_filtered) == 1){
      robotState = 2;
    }
    else if (reachTargetHeading(heading_filtered) == -1){
      robotState = 2;
    }
  }
  else if (robotState == 2){
    if (reachTargetHeading(heading_filtered) == 1){
      for (char j = 0; j < 4; j ++){
        switch(R_LeftTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
           case 'R':
            Right(j);
            break;
          case 'L':
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
    else if (reachTargetHeading(heading_filtered) == -1){
      for (char j = 0; j < 4; j ++){
        switch(R_RightTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
      robotState = 1;
      stopMotorDC();
    }
  }
  else if (robotState == 3){
    if (reachTargetHeading(heading_filtered) == 1){
      robotState = 4;
    }
    else if (reachTargetHeading(heading_filtered) == -1){
      robotState = 4;
    }
    else{
      if (charInput == 'F'){  
        for (char j = 0; j < 4; j ++){
          switch(M_Forward[RobotForm-1][j]){
            case 'F':
              Forward(j);
              break;
            case 'B': 
              Backward(j);
              break;
            case 'R':
              Right(j);
              break;
            case 'L':
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
        for (char j = 0; j < 4; j ++){
          switch(M_Backward[RobotForm-1][j]){
            case 'F':
              Forward(j);
              break;
            case 'B':
              Backward(j);
              break;
            case 'R':
              Right(j);
              break;
            case 'L':
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
        for (char j = 0; j < 4; j ++){
          switch(M_Right[RobotForm-1][j]){
            case 'F':
              Forward(j);
              break;
            case 'B':
              Backward(j);
              break;
            case 'R':
              Right(j);
              break;
            case 'L':
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
        for (char j = 0; j < 4; j ++){
          switch(M_Left[RobotForm-1][j]){
            case 'F':
              Forward(j);
              break;
            case 'B':
              Backward(j);
              break;
            case 'R':
              Right(j);
              break;
            case 'L':
              Left(j);
              break;
            case '0':
              break;
          }
        }
      }
    }
  }
  else if (robotState == 4){
    if (reachTargetHeading(heading_filtered) == 1){
      for (char j = 0; j < 4; j ++){
        switch(R_LeftTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
           case 'R':
            Right(j);
            break;
          case 'L':
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
    else if (reachTargetHeading(heading_filtered) == -1){
      for (char j = 0; j < 4; j ++){
        switch(R_RightTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
    else{
      robotState = 3;
      stopMotorDC();
    }
  }
  else if (robotState == 5){
    if (reachTargetHeading(heading_filtered) == 1){
      for (char j = 0; j < 4; j ++){
        switch(R_LeftTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
           case 'R':
            Right(j);
            break;
          case 'L':
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
    else if (reachTargetHeading(heading_filtered) == -1){
      for (char j = 0; j < 4; j ++){
        switch(R_RightTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
      robotState = 1;
      stopMotorDC();
    }
  }
  else if (robotState == 6){
    switch(charInput){
      case '1':
      {
        straight();
        RobotForm = 1;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '2':
      {
        square();
        RobotForm = 2;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '3':
      {
        L_4th();
        RobotForm = 3;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '4':
      {
        L_1st();
        RobotForm = 4;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '5':
      {
        Z_1_4();
        RobotForm = 5;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '6':
      {
        plus();
        RobotForm = 6;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '7':
      {
        S_1_3();
        RobotForm = 7;
        worldRobotTargetHeading = worldTargetHeading;
        break;
      }
      case '8':
      {
        straight();
        RobotForm = 1;
        worldRobotTargetHeading = worldTargetHeading - 90;
        break;
      }
    } 
    delay(1000);
    robotState = 7;
  }  
  else if (robotState == 7){
    if (reachTargetHeading(heading_filtered) == 1){
      for (char j = 0; j < 4; j ++){
        switch(R_LeftTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
           case 'R':
            Right(j);
            break;
          case 'L':
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
    else if (reachTargetHeading(heading_filtered) == -1){
      for (char j = 0; j < 4; j ++){
        switch(R_RightTurn[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
      robotState = 1;
      stopMotorDC();
    }
  }
  else if (robotState == 8){
    if (charInput == 'f'){  
      for (char j = 0; j < 4; j ++){
        switch(M_Forward[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B': 
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
    if (charInput == 'b')
    {  
      for (char j = 0; j < 4; j ++){
        switch(M_Backward[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
    if (charInput == 'r')
    {  
      for (char j = 0; j < 4; j ++){
        switch(M_Right[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
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
    if (charInput == 'l')
    {  
      for (char j = 0; j < 4; j ++){
        switch(M_Left[RobotForm-1][j]){
          case 'F':
            Forward(j);
            break;
          case 'B':
            Backward(j);
            break;
          case 'R':
            Right(j);
            break;
          case 'L':
            Left(j);
            break;
          case '0':
            break;
        }
      }
    }
  }
}

