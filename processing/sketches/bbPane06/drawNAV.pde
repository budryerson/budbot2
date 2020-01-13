// File name: drawNAV.pde
// Supporting:  bbPane06.pde
// Creator:   Bud Ryerson
// Date:
// Described:  support of 'bbPane04.pde'
//             draw and fill navigation display
// 18AUG16 - simplified Object Detect light routine
// 17AUG16 - lost an eye

int navWide = 500;
int navHigh = 550;

// main routine - called by bbData_10
void drawNAVscreen( int xOrg, int yOrg)
{
    pushMatrix();
    translate( xOrg, yOrg);

      //  Draw border of NAV screen area
      fill( ccBlk);
      stroke( hermesHigh);
      strokeWeight( 1);
      rect( 0, 0, navWide, navHigh);

      //  Draw the radar display
      translate ( 250, 300);  //  move to center of display
      drawRadar();            //  draw the radar scan in green

      //  draw the radar grid, various legends and object detect lights
      //  on top of the scan shape
      stroke( hermesLow);
      strokeWeight( 1);

      // draw 5 concentric circles at 50 pixel intervals
      for( int i = 1; i <= 5; i++)
      {
          ellipse( 0, 0, ( i * 100), ( i * 100));
      }
      // draw 24 lines at 15 degree intervals
      // from origin, out to edge of scan area
      // sin A = opp/hyp |  cos A = adj/hyp
      for( int i = 1; i <= 24; i++)
      {
        line( 0, 0,
          cos( radians( 15 * i)) * 250,
          sin( radians( 15 * i)) * 250);
      }

      // Add two text markers at plus and minus 45°
      fill( hermesLow);
      textFont( faceMSR20);
      textAlign( CENTER);
      text("-45°", xPosvRay[  45] * 280, yPosvRay[  45] * 280);
      text("+45°", xPosvRay[ 135] * 280, yPosvRay[ 135] * 280);
      textAlign( LEFT);  //  return text alignment to normal(left) side

      // Draw a 6-pixel-wide yellow line to show servo position angle
      strokeWeight( 6);
      stroke( yellow);  //  color yellow
      line(  0, 0,
          xPosvRay[ azDex] * constrain( distRay[ azDex], 0, 250),  //dist/2,
          yPosvRay[ azDex] * constrain( distRay[ azDex], 0, 250)  // dist/2);
      );

      //  Draw and illuminate four object detection lights in each
      //  corner of the radar grid using a predefined diamond pShape.
      noStroke();
      drawODLight( odFlagLF, -180, -180);
      drawODLight( odFlagRF, 360, 0);
      drawODLight( odFlagRR, 0, 360);
      drawODLight( odFlagLR, -360, 0);

      //  Write platform navigation data
      translate ( -60, -470);
      fill( 255);
      textFont( faceMSR20);
      text( String.format( "Course:  %03d°", course), 0, 20);    //  auto course: 0 to 360°
      text( String.format( "Bearing:% 04d°", bearing), 0, 45);   //  bearing to target: -180 to 180°

      // draw and fill servo data
      fill( 255);
      textFont( faceMSR20);
      //  servo Pan/Tilt position from 0 to 180°, center/level = 90°
      text( String.format( "Azim:   %03d°", azPos), 0, 315);
      //  distance as a running average of ten readings
      text( String.format( "Flux:  %04d", flux), 0, 340);
      //  distance as a running average of ten readings
      text( String.format( "Dist:   %03d", dist), 0, 365);
      //  nearest position
      text( String.format( "Target: %03d", nearPos), 0, 390);
      // Write clock data
      //textFont( faceMSR20);
      text( "Sys Clock:  " + clockStr, 120, 465);
      text( "Loop Count: ", 120, 491);
      text( String.format( "%07d", loopCount), 275, 490);
      text( "Loop Time: ", 120, 516);
      text( String.format( "%04d", loopTime), 275, 515);

      //  distance displayed large
      textFont( faceBPB56);
      textAlign( CENTER);
      text( String.format( "%02d", ( dist)), 240, 340);
      textFont( faceMSR28);
      text( "cm", 240, 365);
      textAlign( LEFT);

      //  Draw a ccWht box at top of the radar grid and
      //  write the platform Heading as 0 to 360 degrees.
      translate ( 210, 0); //  move to Heading box position
      fill( 30);            //  make box background dark grey
      stroke( 255);         //  make box border bright ccWht
      strokeWeight( 1);
      rect( 0, 0, 60, 30);  //  draw the Heading box
      // Write compass heading value: 0 to 360° in the box
      fill( 255);           //  make the text bright ccWht
      textFont( faceMSR20);
      text( String.format( "%03d°", heading), 10, 23);  // Heading in degrees

      //  Write big letters
      translate( 175, 35);   //  move back to upper right corner
      fill( bigLetter);      //  Hermes Color Low
      textFont( faceBPB56);
      text( "NAV", 0, 0);

    popMatrix();    // recall the coordinate system from the stack
}

// draw single Object Detect Light at given position
void drawODLight( int fb, int xPos, int yPos)
{
      translate( xPos, yPos);
      if( fbVal[ fb]) fill( ccRed); else fill( ccOff);
      quad( +15, 0, 0, +15, -15, 0, 0, -15);
}

//  draw rectangular boxes for IR eyes, illuminate and
//  write data in each to indicate light level received
void drawEyeLight( int xPos, int yPos, int value)
{
    //  fill boxes with a dist of yellow color brightness
    //fill( value, value, 0);
    fill( 0, 0, 0);
    rect( xPos, yPos, 85, 40);
    //  draw numerical value inside the light
    //fill( 180);
    fill( 250);
    textFont( faceMSR28);
    text( String.format( "%03d", value), xPos + 15, yPos + 30);
    //  reset
    noFill();
}

//  - - - - - - - -  - - - - - - - - - - - - - - - -
