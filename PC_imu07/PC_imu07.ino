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

//****************************************
/////////////  Definitions  //////////////
//****************************************
#define debugPrintActive true
#define debugMotorSetupActive true
#define debugMotorActive true
#define headingCorrectionDuringMotion false   // While active, the robot prioritise rotating to the ideal heading during motion

#define linearPower 64  // Power provided to DC motors during robot linear motion (F, B, R, L) (max: 128)
#define rotatePower 50  // Power provided to DC motors during robot rotation (r, l) (max: 128)

#define angleTolerance 7  // Angle tolerance for robot rotation (+- degree away from targeted heading)

#define timerSerialPeriod 990  // The time interval between two input readings from serial port (ms)
#define timerDebugPeriod 1000  // The time interval for printing debug information to the serial port (ms)

//****************************************
///////// Variables Declaration //////////
//****************************************

int RobotForm;
char charInput;
char prevCharInput;


// timers
int timerSerial;
int timerMotion;
int timerDebug;


// IMU Related
float magX ;
float magY ;

// rotation related
float rad_to_deg = 180/3.141592654;
float heading_raw;
float heading_filtered;
bool isRobotRotating;
bool isFinishRotation;
float rotateStartHeading;
float rotateTargetHeading;
float degreeToPos;
float rotateTargetLower;
float rotateTargetUpper;
int robotDir = 0;



void setup()
{ 

  Serial.begin(9600);
  RobotForm = 1;   // Default shape: straight

  // IMU Setup
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      if (debugPrintActive){
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
      }
      delay(5000);
    }
  }
  // Only enabling magnetometer readings from the IMU sensors
  imu.setSensors(INV_XYZ_COMPASS);
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  
  
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

  // timer setup
  timerSerial = millis();
  timerMotion = millis();
  timerDebug = millis();
}

void threadSerial()
{
  if(Serial.available() > 0){
    charInput = Serial.read();  
    Serial.print(charInput); 
  };
}

// Main Loop
void loop() {
  
  delay(500);
  
  if (millis() - timerSerial > timerSerialPeriod){
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
    if (millis() - timerDebug > timerDebugPeriod){
      debugPrint();
      timerDebug = millis();
    }
  }
}

