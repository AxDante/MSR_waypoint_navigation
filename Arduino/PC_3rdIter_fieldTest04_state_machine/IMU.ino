// IMU Data collected here (only magnetometer values).
void imuRead()
{  
   imu.update(UPDATE_COMPASS);
   magX = imu.calcMag(imu.mx) + 7;
   magY = imu.calcMag(imu.my) - 30;

  // Converting the compass readings to degrees.
  heading_raw = atan2(magY,magX)*rad_to_deg;

  // Adding a filter to the heading value.
  heading_filtered = 0.7*heading_filtered + 0.3*heading_raw ;

  if (magX == 7 && magY == -30){
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
    delay(1000);
    if (debugPrintActive) Serial.println("Resetting IMU");
  }
}

// Simple function for the robot to determine the heading is reached.
// TODO: Improve the preformance of these functions.
int reachTargetHeading( float currentAng){

    if (currentAng <  rotateTargetLower){
      return 1;
    }
    else if (currentAng >  rotateTargetUpper){
      return -1;
    }
    else{
      return 0;
    }
}

