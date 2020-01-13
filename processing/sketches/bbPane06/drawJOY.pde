//  File Name: drawJOY.pde
//  Supporting:  bbPane06.pde
//  Creator:   Bud Ryerson
//  Inception: 21 OCT2016
//  Described: Draw the Joystick (JOY) target box, Rotation screen,
//  Communications (COM) and Copyright screens, all stacked in a
//  vertical column along on the left side of the display pane

//  draw and fill the joystick (JOY) posititon display
void drawJOYscreen( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      // draw joystick box
      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, 300, 300);

      //  draw joystick position display diagram
      fill( ccBlk);
      strokeWeight( 1);
      stroke( hermesHigh);
      ellipse( 150, 150, 280, 280);  // draw outermost circle
      stroke( hermesLow);            // decrease color brightness
      ellipse( 150, 150, 200, 200);  // draw two more inner circles
      ellipse( 150, 150, 120, 120);
      line( 150, 10, 150, 290);      // draw vertical axis
      line( 10, 140, 290, 140);      // draw two horizontal
      line( 10, 160, 290, 160);      // lines for "Spin Zone"
      ellipse( 150, 150, 32, 32);    //  draw center circle "Dead Zone"
      fill( hermesLow);
      textFont( faceMSR20);
      text( "SPIN      ZONE", 65, 157);  //  add "Spin Zone" text

      //  draw joystick raw data
      fill( ccWht);
      textAlign( CENTER);
      textFont( faceMSR20);
      text( "Xpos", 30,265);
      text( String.format( "%4d", joyX),  30, 290);
      text( "Ypos", 270, 265);
      text( String.format( "%4d", joyY), 270, 290);
      textAlign( LEFT);

      // draw joystick position indicator
      noStroke();
      fill( yellow);   //color yellow
      //  divide joystick values by 4 to place a 12 pixel yellow dot
      ellipse( 23 + ( joyX >> 2), 277 - ( joyY >> 2), 12, 12);

      //  draw box title on top of everything
      fill( bigLetter);  //  Hermes Color Low
      textFont( faceBPB56);
      text( "JOY", 10, 50);

    popMatrix();             // restore the coordinate system
}

// velocity and rotation data
void drawRotationScreen( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);    //  set stroke color to Hermes High
      rect( 0, 0, 300, 60);
      fill( 255);
      textFont( faceMSR20);
      text( String.format( "Velocity: % 4d", velocity), 35, 25);
      text( String.format( "Rotation: % 4d", rotation), 35, 50);

    popMatrix();             // restore the coordinate system
}

void drawCOMscreen( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);    //  set stroke color to Hermes High
      rect( 0, 0, 300, 165);                 // draw joystick box

      fill( ccWht);
      textFont( faceMSR20);
      text( "Ports: " + (serialNumber + 1), 30, 75);
      text( "Select: " + serialName, 30, 100);
      text( "Rate: " + serialRate, 30, 125);
      text( "Size: ", 30, 150);
      noStroke();
      if( goodData) fill( nsbGreen);
      else fill( nsbRed);
      rect( 129, 132, 50, 22);    // background for data size
      fill( ccWht);
      text( String.format( "%4d", strBufSize), 125, 150);  // this comes from getData
      //text( String.format( "%4d", byteCount), 125, 150);  // this comes from getData
      //  box title on top of everything
      fill( bigLetter);  //  Hermes Color Low
      textFont( faceBPB56);
      text( "COM", 10, 45);

    popMatrix();             // restore the coordinate system
}

void drawCopyright( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      // draw Copyright box
      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, 300, 30);

      fill( ccWht);
      textFont( faceBPB18);
      text( "Copyright (C)2016 Bud Ryerson", 16, 21);

    popMatrix();             // restore the coordinate system
}

// test routine to display the state of bits in the signal string
// called or commented out in main 'draw' loop in 'bbPane06'
void drawSignalString( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      // draw Signal String box
      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, 300, 30);

      fill( ccWht);
      textFont( faceBPB18);

      text( "Out: ", 10, 22);
      for( int x = 0; x < 16; ++x)
      {
          text( outFlags[ x], 50 + (x * 8), 22);
          outFlags[ x] = '0';    // reset each flag character after printing
      }
      // print a separator character followed by the output
      // flag word in five decimal places with leading zeros
      text( String.format( " | %05d", outFlagWord), 180, 22);
      // then clear the output flag word
      outFlagWord = 0;

    popMatrix();             // restore the coordinate system
}

// test routine to convert byte array members to integers
// called or commented out in main 'draw' loop in 'bbPane06'
void drawRdat1( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      // draw Copyright box
      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, 300, 30);

      fill( ccWht);
      textFont( faceBPB18);
      for( int x = 0; x < 4; ++x)   // serial write each flag character
      {
          text( ", " + buff[253+x], 16 + (64 * x), 21);
      }

    popMatrix();             // restore the coordinate system
}

// test routine to convert byte array members to integers
// called or commented out in main 'draw' loop in 'bbPane06'
byte[] tbr = {1,2,3,4,5,6,-7,0};  // test byte array
int tbx = 0;               //  test byte array index
int[] tir = new int[4];    // test integer array
void testByte2Short( int xOrg, int yOrg)
{
    tbx = 0;
    for( int x = 0; x < 4; ++x)   // serial write each flag character
    {
        int hi = tbr[ tbx] & 0x00FF;
        ++tbx;
        int lo = tbr[ tbx] & 0x00FF;
        ++tbx;
        tir[ x] = (short)((hi << 8) | (lo));
    }
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      // draw Copyright box
      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, 300, 30);

      fill( ccWht);
      textFont( faceBPB18);
      for( int x = 0; x < 4; ++x)   // serial write each flag character
      {
          text( ", " + tir[x], 16 + (64 * x), 21);
      }

    popMatrix();             // restore the coordinate system
}

// test routine for motor current
// called or commented out in main 'draw' loop in 'bbPane06'
void drawCurrent( int xOrg, int yOrg)
{
    pushMatrix();            //  save the coordinate system
    translate( xOrg, yOrg);  //  move to JOY screen

      // draw Copyright box
      fill( ccBgd);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, 300, 30);

      fill( ccWht);
      textFont( faceBPB18);
      text( ", " + lfCurrent, 16 + (64 * 0), 21);
      text( ", " + rfCurrent, 16 + (64 * 1), 21);
      text( ", " + lrCurrent, 16 + (64 * 2), 21);
      text( ", " + rrCurrent, 16 + (64 * 3), 21);

    popMatrix();             // restore the coordinate system
}


//  - - - - - - - -  - - - - - - - - - - - - - - - -
