
void imuRead()
{  
   magX = imu.calcMag(imu.mx);
   magY = imu.calcMag(imu.my);

  heading_raw = atan2(magY,magX)*rad_to_deg*2;
  heading_filtered = 0.8*heading_filtered + 0.2*heading_raw ;

}


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

