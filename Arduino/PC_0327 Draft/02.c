#include <Herkulex.h>
#include <SoftwareSerial.h>
#include <SparkFunMPU9250-DMP.h>
#include "RoboClaw.h"

RoboClaw roboclaw(&Serial2,10000);
MPU9250_DMP imu;

#define address1 0x83
#define address2 0x80
#define address3 0x81
#define address4 0x82

int rp1 = 25;
int rp2 = 24;
int rp3 = 26;
int rp4 = 27;

int p = 2;
int q = 1;
int r = 219;//motor ID - verify your ID !!!!/

char* M_Forward[] = {"FFFF", "FFBB", "FFFB", "BFFF", "BFFB", "RFBF", "RFBL"};
char* M_Backward[] = {"BBBB", "BBFF", "BBBF", "FBBB", "FBBF", "LBFB", "LBFR"};
char* M_Right[] = {"RRRR", "RRLL", "RRRL", "LRRR", "LRRL", "BRLR", "BRLF"};
char* M_Left[] = {"LLLL", "LLRR", "LLLR", "RLLL", "RLLR", "FLRL", "FLRB"};

char* R_LeftTurn[] = {"RRLL", "RBRB", "RRLL", "LRLL", "BRLF", "BBRF", "BBRL"};
char* R_RightTurn[] = {"LLRR", "LFLF", "LLRL", "RLRR", "FLRB", "FFLB", "FFLR"};

char* S_Shape[] = {"Straight", "Square", "L-shape (4th)", "L-shape (1st)", "Z-shape", "T-shape", "S-shape"};

String stringStream = "FRBLLF";    //test
int linerMotionStopTime = 5000;
float worldRobotTargetHeading = -42.0; //(for robotMode 3 or 4, which recognize its initial position)

//****************************************
/////////////  Definitions  //////////////
//****************************************
#define debugPrintActive true                 // Printing Debug information on Serial Port
#define debugMotorSetupActive true           


#define debugHerkulexSetupActive true
#define debugHerkulexMotorActive true

#define debugRoboclawSetupActive true
#define debugRoboclawMotorActive true



#define secureShapeActive true
#define secureShapeLinearActive true

#define headingCorrectionDuringMotion true   // While active, the robot prioritise rotating to the ideal heading during motion
#define imuActive true

#define robotMode 1
// ( 1: Read one input char from serial port every time, motion not stopping unless receieves next input)
// ( 2: Read one input char from serial port every time, motion automatically stops after meeting certain conditions) 
// ( 3: Read and move following string input (static, for debug use), motion automatically stops after meeting certain conditions)
// ( 4: 03/27 Trial Run:

#define initialPositionAsWorldFrame false     // When this is set true, the robot will recognize its initial heading as world coordinate 
                                              // and perform rotations based on it; while set false, the initial heading can be either given
                                              // in advance or be set through serial ports.

#define linearPower 43  // Power provided to DC motors during robot linear motion (F, B, R, L) (max: 128)
#define rotatePower 58  // Power provided to DC motors during robot rotation (r, l) (max: 128)
#define rotateAdjustPower 58 // Power provided to DC motors during robot heading self adjustment (max: 128)

#define angleTolerance 6.5  // Angle tolerance for robot rotation (+- degree away from targeted heading)

#define transformSpeed 1500

#define timerSerialThreadPeriod 990  // The time interval between two input readings from serial port (ms)
#define timerDebugThreadPeriod 1000  // The time interval for printing debug information to the serial port (ms)
#define counterSecureShapeMaxCount 10  // The time interval for the Herkulex motor to switch on to secure robot shapes (ms)
#define counterSecureShapeMaxLinearCount 20 
#define imuCalibrationTime 10000     // Time for the IMU to calibrate in the beginning (ms)
//****************************************
///////// Variables Declaration //////////
//****************************************

int RobotForm;
char charInput;
char prevCharInput;

int counterSecureShape;

// timers
unsigned long timerSerialThread;
unsigned long timerMotionThread;
unsigned long timerDebugThread;
unsigned long timerLinearMotion;

// IMU Related
float magX ;
float magY ;

// Robot motion boolean
bool isRobotRotation;
bool isFinishRotation;
bool isRobotLinearMotion;
bool isRobotRotationAdjust;
bool isStopped;

// Robot rotation related
float rad_to_deg = 180/3.141592654;
float heading_raw;                  // raw heading values in angle from imu
float heading_filtered;              
float rotateStartHeading;
float rotateTargetHeading;
float degreeToPos;
float rotateTargetLower;
float rotateTargetUpper;
int robotDir = 0;

void setup()
{ 
  Serial.begin(9600);
  RobotForm = 2;   // Default shape: square

  counterSecureShape = 0;
  
  // IMU Setup
  if(imuActive){
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
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  
    imu.setGyroFSR(2000); 
    imu.setAccelFSR(2); // Set accel to +/-2g
    imu.setLPF(5);
    imu.setSampleRate(10); // Set sample rate to 10Hz
    imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  }


  // Roboclaw Setup
  if (debugRoboclawSetupActive){
    if (debugPrintActive) Serial.println("Roboclaw Setup Begin.");
    roboclaw.begin(38400);
  }
  
  // Herkulex Setup
  if (debugHerkulexSetupActive){
    if (debugPrintActive) Serial.println("Herkulex Setup Begin.");
    delay(2000);  //a delay to have time for serial monitor opening
    Herkulex.beginSerial1(115200); //open serial port 1
    Herkulex.reboot(p); //reboot first motor
    Herkulex.reboot(q);
    Herkulex.reboot(r);
  
    delay(500);
    Herkulex.initialize(); //initialize motors
    delay(200);
  }

  // Robot Realy setup
  pinMode(rp1, OUTPUT);
  pinMode(rp2, OUTPUT);
  pinMode(rp3, OUTPUT);
  pinMode(rp4, OUTPUT);

  // timers setup
  timerSerialThread = millis();
  timerMotionThread = millis();
  timerDebugThread = millis();
  timerLinearMotion = millis();

  // Heading setup
  if (!initialPositionAsWorldFrame){
    rotateTargetHeading = worldRobotTargetHeading;   
    rotateTargetLower = rotateTargetHeading - angleTolerance;
    rotateTargetUpper = rotateTargetHeading + angleTolerance; 
  }
}

// Main Loop
void loop() {
  
  delay(50);
  
  if (millis() - timerSerialThread > timerSerialThreadPeriod){
    threadSerial();
    timerSerialThread = millis();
  }

  if(imuActive){
    if ( imu.dataReady() )
    {
      imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
      imuRead();
    }
  }
  
  if ( millis() > imuCalibrationTime){ 
    threadMotion();
  }
  
  if (debugPrintActive){
    if (millis() - timerDebugThread > timerDebugThreadPeriod){
      debugPrint();
      timerDebugThread = millis();
    }
  }
}

