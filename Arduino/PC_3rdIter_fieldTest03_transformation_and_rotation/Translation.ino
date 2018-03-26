// DC motor commands
void Forward(int i){
  int power = DCMotorPower();
  switch(i){
    case 0:
      digitalWrite(rp1, LOW);
      roboclaw.ForwardM1(address1,power);
      roboclaw.ForwardM2(address1,power);
      break;
    case 1:
      digitalWrite(rp2, LOW);
      roboclaw.ForwardM1(address2,power);
      roboclaw.ForwardM2(address2,power);
      break;
    case 2:
      digitalWrite(rp3, LOW);
      roboclaw.ForwardM1(address3,power);
      roboclaw.ForwardM2(address3,power);
      break;
    case 3:
      digitalWrite(rp4, LOW);
      roboclaw.ForwardM1(address4,power);
      roboclaw.ForwardM2(address4,power);
      break;
  }
}

void Backward(int i){
  int power = DCMotorPower();
  switch(i){
    case 0:
      digitalWrite(rp1, LOW);
      roboclaw.BackwardM1(address1,power);
      roboclaw.BackwardM2(address1,power);
      break;
    case 1:
      digitalWrite(rp2, LOW);
      roboclaw.BackwardM1(address2,power);
      roboclaw.BackwardM2(address2,power);
      break;
    case 2:
      digitalWrite(rp3, LOW);
      roboclaw.BackwardM1(address3,power);
      roboclaw.BackwardM2(address3,power);
      break;
    case 3:
      digitalWrite(rp4, LOW);
      roboclaw.BackwardM1(address4,power);
      roboclaw.BackwardM2(address4,power);
      break;
  }
}

void Right(int i){
  int power = DCMotorPower();
  switch(i){
    case 0:
      digitalWrite(rp1, HIGH);
      roboclaw.ForwardM1(address1,power);
      roboclaw.ForwardM2(address1,power);
      break;
    case 1:
      digitalWrite(rp2, HIGH);
      roboclaw.ForwardM1(address2,power);
      roboclaw.ForwardM2(address2,power);
      break;
    case 2:
      digitalWrite(rp3, HIGH);
      roboclaw.ForwardM1(address3,power);
      roboclaw.ForwardM2(address3,power);
      break;
    case 3:
      digitalWrite(rp4, HIGH);
      roboclaw.ForwardM1(address4,power);
      roboclaw.ForwardM2(address4,power);
      break;
  }
}

void Left(int i){
  int power = DCMotorPower();
  switch(i){
    case 0:
      digitalWrite(rp1, HIGH);
      roboclaw.BackwardM1(address1,power);
      roboclaw.BackwardM2(address1,power);
      break;
    case 1:
      digitalWrite(rp2, HIGH);
      roboclaw.BackwardM1(address2,power);
      roboclaw.BackwardM2(address2,power);
      break;
    case 2:
      digitalWrite(rp3, HIGH);
      roboclaw.BackwardM1(address3,power);
      roboclaw.BackwardM2(address3,power);
      break;
    case 3:
      digitalWrite(rp4, HIGH);
      roboclaw.BackwardM1(address4,power);
      roboclaw.BackwardM2(address4,power);
      break;
  }
}

void stopMotorDC()
{
  roboclaw.ForwardM1(address1,0);
  roboclaw.ForwardM2(address1,0);
  roboclaw.ForwardM1(address2,0);
  roboclaw.ForwardM2(address2,0);
  roboclaw.ForwardM1(address3,0);
  roboclaw.ForwardM2(address3,0);
  roboclaw.ForwardM1(address4,0);
  roboclaw.ForwardM2(address4,0);
}

int DCMotorPower(){
  if (isRobotRotation){
    return rotatePower;
  }else if (isRobotRotationAdjust){
    return rotateAdjustPower;
  }else{
    return linearPower;
  }
}


