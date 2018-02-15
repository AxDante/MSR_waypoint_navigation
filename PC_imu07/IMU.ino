
void imuRead()
{  
   magX = imu.calcMag(imu.mx);
   magY = imu.calcMag(imu.my);

  heading_raw = atan2(magY,magX)*rad_to_deg*2;
  Total_angle[2] = 0.8*Total_angle[2] + 0.2*heading_raw ;

}


int reachTargetArea( float currentAng){

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


/*
  if  ( rotateTargetLower > 360){
    if (currentAng > 180 && currentAng < 360){
      return 1;
    }
    else if (currentAng < 180 && currentAng + 360 < rotateTargetLower){
      return 1;
    }
    else if (currentAng < 180 && currentAng + 360 > rotateTargetUpper){
      return -1;
    }
    else if (currentAng < 180 && currentAng + 360 > rotateTargetLower && currentAng + 360 <rotateTargetUpper){
      return 0;
    }
  }
  if  ( rotateTargetLower < 360 && rotateTargetHeading > 360){
    
    if (currentAng > 180 && currentAng <  rotateTargetLower){
      return 1;
    }
    if (currentAng > 180 && currentAng >  rotateTargetLower){
      return 0;
    }
    else if (currentAng < 180 && currentAng + 360 > rotateTargetUpper){
      return -1;
    }
    else if (currentAng < 180 && currentAng + 360 < rotateTargetUpper && currentAng + 360 > rotateTargetLower ){
      return 0;
    }
  }
  if  ( rotateTargetHeading < 360 && rotateTargetUpper > 360){
    if (currentAng > 180 && currentAng <  rotateTargetLower){
      return 1;
    }
    if (currentAng > 180 && currentAng >  rotateTargetLower){
      return 0;
    }
    else if (currentAng < 180 && currentAng + 360 >rotateTargetUpper){
      return -1;
    }
    else if (currentAng < 180 && currentAng + 360 < rotateTargetUpper && currentAng + 360 > rotateTargetLower ){
      return 0;
    }
  }
  
  if  (rotateTargetUpper < 360 &&rotateTargetLower > 0){
    if (currentAng <  rotateTargetLower){
      return 1;
    }
    if (currentAng >  rotateTargetUpper){
      return -1;
    }
    else if (currentAng <rotateTargetUpper && currentAng > rotateTargetLower){
      return 0;
    }
  }
 if  ( rotateTargetUpper < 0){
    if (currentAng < 180 && currentAng > 0){
      return -1;
    }
    else if (currentAng > 180 && currentAng - 360 < rotateTargetLower){
      return 1;
    }
    else if (currentAng > 180 && currentAng - 360 > rotateTargetUpper){
      return -1;
    }
    else if (currentAng > 180 && currentAng - 360 <rotateTargetUpper && currentAng - 360 > rotateTargetLower){
      return 0;
    }
  }
  if  ( rotateTargetLower < 0){
    if (currentAng < 180 && currentAng > rotateTargetUpper){
      return -1;
    }
    if (currentAng < 180 && currentAng < rotateTargetUpper){
      return 0;
    }
    else if (currentAng > 180 && currentAng - 360 < rotateTargetLower){
      return -1;
    }
    else if (currentAng < 180 && currentAng - 360 < rotateTargetUpper && currentAng - 360 > rotateTargetLower ){
      return 0;
    }
  }
  */
}

