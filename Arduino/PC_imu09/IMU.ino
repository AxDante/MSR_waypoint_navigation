// IMU Data collected here (only magnetometer values).
void imuRead()
{  
   magX = imu.calcMag(imu.mx);
   magY = imu.calcMag(imu.my);

  // Converting the compass readings to degrees.
  heading_raw = atan2(magY,magX)*rad_to_deg*2;

  // Adding a filter to the heading value.
  heading_filtered = 0.7*heading_filtered + 0.3*heading_raw ;

}

// Simple function for the robot to determine the heading is reached.
// TODO: Improve the preformance of these functions.
int reachTargetHeading( float currentAng){

  degreeToPos = rotateTargetHeading - currentAng;

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

