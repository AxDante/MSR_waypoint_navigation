#include <Herkulex.h>
#include <SoftwareSerial.h>
#include <SparkFunMPU9250-DMP.h>
#include "RoboClaw.h"

RoboClaw roboclaw(&Serial2,10000);
MPU9250_DMP imu;

#define address1 0x80
#define address2 0x81
#define address3 0x82
#define address4 0x83

#define debugPrintActive true
#define debugMotorSetupActive true
#define debugMotorActive true
#define headingAutoCorrection false


int rotatePower = 50;

int rp1 = 25;
int rp2 = 24;
int rp3 = 26;
int rp4 = 27;
int p = 1;
int q = 2;
int r = 219;//motor ID - verify your ID !!!!

char* M_Forward[] = {"ffff", "ffbb", "fffb", "bfff", "bffb", "rfbf", "rfll"};
char* M_Backward[] = {"bbbb", "bbff", "bbbf", "fbbb", "fbbf", "lbfb", "fbrr"};
char* M_Right[] = {"rrrr", "rrll", "rrrl", "lrrr", "lrrl", "brlr", "lrff"};
char* M_Left[] = {"llll", "llrr", "lllr", "rlll", "rllr", "flrl", "rlbb"};

char* R_RightTurn[] = {"rrll", "rbrb", "rflf", "brbl", "brlf", "bbrf", "lbbl"};
char* R_LeftTurn[] = {"llrr", "lflf", "lbrb", "flfr", "flrb", "fflb", "rffr"};

char* S_Shape[] = {"Straight", "Square", "L-shape (4th)", "L-shape (1st)", "Z-shape", "T-shape", "S-shape"};

int RobotForm;
char charInput;
char prevCharInput;

int timerSerial;
int timerMotion;
int timerDebug;

float heading_raw;

float rad_to_deg = 180/3.141592654;
float Total_angle[3];

float magX ;
float magY ;
  
int imuTime;

bool isRobotRotating;
bool isFinishRotation;
bool isUpdatingNextValue = true;
float rotateStartHeading;
float rotateTargetHeading;
float degreeToPos;
float rotateTargetLower;
float rotateTargetUpper;
int angleTolerance = 5;
int robotDir = 0;

void setup()
{ 
 

if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }
  
  imu.setSensors(INV_XYZ_COMPASS);
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  
  RobotForm = 1;

  Serial.begin(9600);
  
  if (debugMotorSetupActive){
    roboclaw.begin(38400);
    
    if (debugPrintActive) Serial.println("Setup Begin.");
    delay(2000);  //a delay to have time for serial monitor opening
    Herkulex.beginSerial1(115200); //open serial port 1
    Herkulex.reboot(p); //reboot first motor
    Herkulex.reboot(q);
    Herkulex.reboot(r);
  
    delay(500);
    Herkulex.initialize(); //initialize motors
    delay(200);
  }
  
  pinMode(rp1, OUTPUT);
  pinMode(rp2, OUTPUT);
  pinMode(rp3, OUTPUT);
  pinMode(rp4, OUTPUT);

  timerSerial = millis();
  timerMotion = millis();
  timerDebug = millis();
  imuTime = millis();
}

void threadSerial()
{
  if(Serial.available() > 0){
    charInput = Serial.read();  
    Serial.print(charInput); 
  };
}

void threadMotion(){

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
      rotateStartHeading = Total_angle[2]+0.01;

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
      if (reachTargetArea(Total_angle[2]) == 1){
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
      else if (reachTargetArea( Total_angle[2]) == -1){
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
      else if (reachTargetArea( Total_angle[2]) == 0){
         isRobotRotating = false;
         isFinishRotation = true;
         Stop();
         Serial.println("=====End Rotation=====");
      }
    }
  }
  if (charInput == 'k')
  {
    secureShape(RobotForm);
  }
  if (charInput == 'S')
  {
      isRobotRotating = false;
      isFinishRotation = true;
      Stop();
  }
  if (prevCharInput != charInput){
    isFinishRotation = false;
    prevCharInput = charInput;
  }
}

void loop() {

  
  delay(500);
  
  if (millis() - timerSerial > 990){
    if(Serial.available() > 0){
  
      charInput = Serial.read();   
      //Serial.print(charInput);
      
    }

    timerSerial = millis();
    
  }
  
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_COMPASS);
    imuRead();
  }
  threadMotion();
  if (debugPrintActive){
    if (millis() - timerDebug > 1000){
      debugPrint();
      timerDebug = millis();
    }
  }
}

void Stop()
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
