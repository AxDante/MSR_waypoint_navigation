// Activates the servo motors while rotating or transforming.
void secureShape(int form){
  switch(form){
    case 1:
      straight();
      break;
    case 2:
      square();
      break;
    case 3:
      L_4th();
      break;
    case 4:
      L_1st();
      break;
    case 5:
      Z_1_4();
      break;
    case 6:
      plus();
      break;
    case 7:
      S_1_3();
      break;
    default:
      break;
  }
}

void straight()
{
  if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, -100, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, 98, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, 95, transformSpeed, LED_BLUE); //move motor with 300 speed 
  }
}
void square()
{
  if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, -101, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, -104, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, 93, transformSpeed, LED_BLUE); //move motor with 300 speed 
  }
}
void L_4th()
{
  if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, -101, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, 98, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, -99, transformSpeed, LED_BLUE); //move motor with 300 speed 
  }
}
void L_1st()
{
  if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, 93, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, 98, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, 93, transformSpeed, LED_BLUE); //move motor with 300 speed 
  }
}
void Z_1_4()
{
  if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, 92, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, 98, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, -98, transformSpeed, LED_BLUE); //move motor with 300 speed 
  }
}
void plus()
{
  if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, 3, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, -105, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, -98, transformSpeed, LED_BLUE); //move motor with 300 speed 
  }
}
void S_1_3()
{
   if (debugHerkulexMotorActive){
    Herkulex.moveOneAngle(p, 92, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(q, -10, transformSpeed, LED_BLUE); //move motor with 300 speed 
    Herkulex.moveOneAngle(r, 96, transformSpeed, LED_BLUE); //move motor with 300 speed   
  }
}

