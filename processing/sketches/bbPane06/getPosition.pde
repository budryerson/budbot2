// File Name: getPosition.pde
// Supporting:  bbPane06.pde
// Creator:   Bud Ryerson
// Inception: 16MAR2019
// LastTouch:
//
// Described:
//   Called by 'getData' to do get platform position
//   from odometry and IMU data
//
    // IMU Data
    // ( Dead Reckoning position and IMU position should equal each other )
    // ------------------------------------------------------------------------
    // | d = ( v * t) + ( ( a * pow( t, 2) / 2);                              |
    // | d = distance, v = initial velocity, t = time, a = acceleration       |
    // | f = v + ( a * t)                                                     |
    // | f = final velocity, v = initial velocity, a = acceleration, t = time |
    // ------------------------------------------------------------------------
//

  float drDist;   // dead reckoning distance
  float radHead;  // radian value of heading
  int deltaPosX;  // change in X and Y dead reckoning position
  int deltaPosY;  // these are shared by at least two routines

  //  =======  Find new  X & Y Position by Dead Reckoning ======
  //  Find new distance using odometry from wheel encoders data
  //  Plot XY position using 'heading' angle
  void getPosition()
  {
      int clkL;       // wheel encoder clicks
      int clkR;
      clkL = intCountL; // wheel encoder clicks
      clkR = intCountR;
      // If either wheel is stopped then robot is spinning
      // and distance is zero
      if( ( clkL == 0) || ( clkR == 0))
      {
          drDist = 0;
          deltaPosX = 0;
          deltaPosY = 0;
      }
      else     // Otherwise ...
      {
        //  reverse the sign if either wheel is going backwards
        if( motSpeedL < 0) clkL *= -1;
        if( motSpeedR < 0) clkR *= -1;

        //  Add the clicks, divide by 2 to get an average, then multiply
        //  by 0.57 to get distance in millimeters for each 50ms loop
        //  Formula: 166.66 encoder state changes per wheel rotation
        //  divided by 190mm (7.5inches) traveled per wheel rotation
        //  equals 1.14mm of travel per each encoder click.
        //
        //drDist = ( clkL + clkR) / 3.46; // old calculation - why?
        drDist = ( clkL + clkR) * 0.285;
        // direction platform is traveling (heading) in radians
//        radHead = imuDat.Euler.Head / 57.2957795;  // 180/Pi
        radHead = heading / 57.2957795;  // 180/Pi
        // Use trig to roughly get the X and Y components of the dead
        // reckoning distance traveled from the angle of the heading.
        //   sin( angle) * hypotenuse = opposite
        //   cos( angle) * hypotenuse = adjacent
        deltaPosX = Math.round( sin( radHead) * drDist);  // in millimeters?
        deltaPosY = Math.round( cos( radHead) * drDist);
      }

      // If flagBit value 14 'fbPosReset' is TRUE
      // then reposition robot to point of origin
      // and zero all motion and navigation values.
      if( fbVal[ fbPosReset])
      {
          drPosX = 0;    // clear dead reckoning
          drPosY = 0;    // position
          drSpdX = 0;    // and speed
          drSpdY = 0;
          imAccX = 0;    // clear IMU
          imAccY = 0;    // acceleration
          imSpdX = 0;    // and speed
          imSpdY = 0;
         // fbCLR( fbPosReset);  // clear Position Reset flag
      }
      else
      {
        // Find new dead reckoning (dr) distance from point
        // of origin ( 0, 0) in millimeters plus or minus.
        // Map of room is -2250 millimeters (West) to 2250 mm (East),
        // and -3350 millimeters (South) to 3350 mm (North).
        drPosX += deltaPosX;
        drPosY += deltaPosY;

        // And multiply by 20 to get plus or minus speed in mm/sec
        drSpdX = deltaPosX * 20;    //  mm/sec
        drSpdY = deltaPosY * 20;    //  mm/sec
      }

      // If the robot is spinning in place, not moving laterally
      // (Is this a good idea? It does not account for skidding)
      if( drDist == 0)
      {
          imAccX = 0;    // clear the acceleration
          imAccY = 0;
          imSpdX = 0;    // clear the speed
          imSpdY = 0;
      }
  }
