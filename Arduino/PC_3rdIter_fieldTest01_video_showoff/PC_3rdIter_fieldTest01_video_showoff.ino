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
int p = 5;
int q = 3;
int r = 219;//motor ID - verify your ID !!!!

/*
char* M_Forward[] = {"ffff", "ffbb", "fffb", "bfff", "bffb", "rfbf", "rfll"};
char* M_Backward[] = {"bbbb", "bbff", "bbbf", "fbbb", "fbbf", "lbfb", "fbrr"};
char* M_Right[] = {"rrrr", "rrll", "rrrl", "lrrr", "lrrl", "brlr", "lrff"};
char* M_Left[] = {"llll", "llrr", "lllr", "rlll", "rllr", "flrl", "rlbb"};
*/

char* M_Forward[] = {"ffff", "ffbb", "fffb", "bfff", "bffb", "rfbf", "rfbl"};
char* M_Backward[] = {"bbbb", "bbff", "bbbf", "fbbb", "fbbf", "lbfb", "lbfr"};
char* M_Right[] = {"rrrr", "rrll", "rrrl", "lrrr", "lrrl", "brlr", "brlf"};
char* M_Left[] = {"llll", "llrr", "lllr", "rlll", "rllr", "flrl", "flrb"};

char* R_RightTurn[] = {"rrll", "rbrb", "rflf", "brbl", "brlf", "bbrf", "bbrl"};
char* R_LeftTurn[] = {"llrr", "lflf", "lbrb", "flfr", "flrb", "fflb", "fflr"};

char* S_Shape[] = {"Straight", "Square", "L-shape (4th)", "L-shape (1st)", "Z-shape", "T-shape", "S-shape"};

String stringStream = "FRBLLF";    //test
int linerMotionStopTime = 5000;
float worldRobotTargetHeading = 180.0; //(for robotMode 3 or 4, which recognize its initial position)

//****************************************
/////////////  Definitions  //////////////
//****************************************
#define debugPrintActive true                 // Printing Debug information on Serial Port
#define debugMotorSetupActive true           


#define debugHerkulexSetupActive false
#define debugHerkulexMotorActive false

#define debugRoboclawSetupActive false
#define debugRoboclawMotorActive false



#define secureShapeActive true
#define secureShapeLinearActive true

#define headingCorrectionDuringMotion false   // While active, the robot prioritise rotating to the ideal heading during motion
#define imuActive true

#define robotMode 1
// ( 1: Read one input char from serial port every time, motion not stopping unless receieves next input)
// ( 2: Read one input char from serial port every time, motion automatically stops after meeting certain conditions) 
// ( 3: Read and move following string input (static, for debug use), motion automatically stops after meeting certain conditions)
// ( 4: Read and move following string input, receives new char input which will be added to the end of string, motion automatically 
//      stops after meeting certain conditions)

#define initialPositionAsWorldFrame false     // When this is set true, the robot will recognize its initial heading as world coordinate 
                                              // and perform rotations based on it; while set false, the initial heading can be either given
                                              // in advance or be set through serial ports.

#define linearPower 55  // Power provided to DC motors during robot linear motion (F, B, R, L) (max: 128)
#define rotatePower 40  // Power provided to DC motors during robot rotation (r, l) (max: 128)
#define rotateAdjustPower 35 // Power provided to DC motors during robot heading self adjustment (max: 128)

#define angleTolerance 5  // Angle tolerance for robot rotation (+- degree away from targeted heading)

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
  
    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(2); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at 
    // +/- 4912 uT (micro-tesla's)
  
    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(5); // Set LPF corner frequency to 5Hz
  
    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(10); // Set sample rate to 10Hz
  
    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
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


