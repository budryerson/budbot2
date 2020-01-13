// File name: drawMOT.pde
// Supporting:  bbPane05.pde
// Creator:   Bud Ryerson
// Date:      21JUL2016
// Described: Draw the motor instrument screen
//            and display motror datq

// - - - - thermometer instruments - - - - - - -
void drawMotScreen( int xOrg, int yOrg)
{
    pushMatrix();
    translate( xOrg, yOrg);

      //  draw box
      strokeWeight( 1);
      stroke( hermesHigh);
      fill( ccBlk);
      rect( 0, 0, 200, 550);
      line( 100, 85, 100, 360);

      //  draw titles
      fill( ccWht);
      textFont( faceMSR20);
      text( "SPD|ENC SPD|ENC", 11, 75);
      textFont( faceMSR28);
      text( "LEFT", 17, 390);
      text( "RIGHT", 110, 390);

      //  motor thermometer displays
      int thermOrg = 220;  //  thermometer vertical origin
      strokeWeight( 20);

      // left speed graph
      int thermPos = motSpeedL >> 1;   //  divide value by 2 for display
      if( thermPos < 0) stroke( ccRed); else stroke( ccGreen);      // change color on polarity
      line( 30, thermOrg, 30, thermOrg - thermPos);

      // right speed graph
      thermPos = motSpeedR >> 1;
      if( thermPos < 0) stroke( ccRed); else stroke( ccGreen);
      line(  130, thermOrg, 130, thermOrg - thermPos);

      // left encoder graph
      stroke( hermesHigh);         // thermometer fill color
      thermPos = intCountL << 2;   // multiply value by 4 for display
      line(   70, thermOrg,  70, thermOrg - thermPos);

      // right encoder graph
      thermPos = intCountR << 2;
      line(  170, thermOrg, 170, thermOrg - thermPos);
      // - - - - - - - - - - - - - - - - - - - - - -

      //  draw motor speed data
      strokeWeight( 1);
      stroke( ccWht);
      fill(30);
      rect(  10, thermOrg - 10, 40, 20);  // left motor speed box
      rect( 110, thermOrg - 10, 40, 20);  // right motor speed box
      rect(  50, thermOrg - 10, 40, 20);  // left wheel click box
      rect( 150, thermOrg - 10, 40, 20);  // right wheel click box
      fill( 255);
      textFont( faceMSR14);
      text( String.format( "% 4d", motSpeedL),  13, thermOrg + 6);  //  values back from PID controller
      text( String.format( "% 4d", motSpeedR), 113, thermOrg + 6);  //  dist: -255 to 255
      text( String.format( "% 4d", intCountL),  53, thermOrg + 6);  //  wheel encoder interrupts per 50msec.
      text( String.format( "% 4d", intCountR), 153, thermOrg + 6);  //  dist: 0 to 60 (approximately)

      //  draw current guages
      drawCurrentGauge("LF",  60, 440, lfCurrent);
      drawCurrentGauge("RF", 145, 440, rfCurrent);
      drawCurrentGauge("LR",  60, 525, lrCurrent);
      drawCurrentGauge("RR", 145, 525, rrCurrent);

      textFont( faceMSR20);
      fill( ccWht);
      text( "Motor Current", 25, 480);

      fill( bigLetter);  //  Hermes Color Low
      textFont( faceBPB56);
      text( "MOT", 55, 50);

    popMatrix();
}

//  draw and animate the four current guages
void drawCurrentGauge( String cgLabel, int xOrg, int yOrg, int cgCurrent)
{
    pushMatrix();             //  save the coordinate system
    translate( xOrg, yOrg);   //  move to the gauge position

      stroke( hermesLow);     //  draw guage background
      strokeWeight( 16);
      noFill();
      arc( 0, 0, 50, 50, 3, 6.42);

      textFont( faceMSR14);   // label the guage
      fill( ccWht);
      text( cgLabel, - 7, 5);


      strokeWeight( 6);       // draw a line to show gauge value
      stroke( yellow);        // color yellow
      float cgV = 3 + radians( cgCurrent);  //  current guage Vertex
      line( cos( cgV) * 20, sin( cgV) * 20,
            cos( cgV) * 30, sin( cgV) * 30);

    popMatrix();              // restore the coordinate system
}

//  - - - - - - - -  - - - - - - - - - - - - - - - -