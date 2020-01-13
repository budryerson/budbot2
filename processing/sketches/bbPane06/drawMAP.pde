// File Name: drawMAP.pde
// Supporting:  bbPane06.pde
// Creator:   Bud Ryerson
// Described: support of 'bbData_10.pde'
//            draw and fill the map display
// 10OCT16 - relocated here from bbData_10
// 01NOV16 - the 'clip' function makes things easier
// 05DEC19 - changed scale of MAP display in 'drawRadar.pde'

int oldTenths = 0;
int newTenths = 0;
boolean fadeTime = false;

int mapWidth = 550;
int mapHeight = 370;
//int mapMaskSize = 125;
int mapCenterX = 275;
int mapCenterY = 185;
int mapOffsetX =  25;
int mapOffsetY =  80;

// draw an overhead MAP viewport
// 600 pixels left too right, and 550 pixels top to bottom
void drawMAPscreen( int xOrg, int yOrg)
{
    pushMatrix();                 // save current screen coordinates

      translate( xOrg, yOrg);     // move to MAP display screen origin
      fill( ccBgd);
      stroke( hermesHigh);
      strokeWeight( 1);
      rect( 0, 0, 600, 550);     // create a background for the viewport
      clip( mapOffsetX, mapOffsetY, mapWidth, mapHeight);
      pushMatrix();              //  save MAP display screen origin

        translate( mapOffsetX, mapOffsetY);    // move to MAP viewport origin
        fill( ccBlk);
        noStroke();
        rect( 0, 0, mapWidth, mapHeight);      // draw the viewport background

/*      // draw cut-out areas of bocana bedroom in brown
        stroke(brown);
        strokeWeight( 1);
        rect( 0, 270, 275, 100);
        rect( 475, 0, 75, 150);    */
  
        pushMatrix();                            // save the viewport origin
          translate( mapCenterX, mapCenterY);    // move to center of viewport area
    
          //  Fade all scans every 1/10 of a second
          newTenths = sysMillis / 100;
          if( oldTenths < newTenths)
          {
              fadeTime = true;
              oldTenths = newTenths;
          }
          else fadeTime = false;

          //  draw the entire array of scan images
          //  in their various locations and rotations
          //  'showScan' and 'fadeScan' are in 'drawRadar.pde'
          for ( int i = 0; i < scans.size(); ++i)
          {
              Scan s = (Scan)scans.get( i);
              s.showScan();            
              if( fadeTime) s.fadeScan();
          }
  
          // move to platform location in MAP viewport
          translate( drPosX / 10, drPosY / -10);  //  move to platform position
          float headRad = radians( heading);      //  convert heading to radians
          rotate( headRad);
    
          //  draw the icon of budbot2
          stroke( hermesHigh);
          strokeWeight(5);           //  draw two Hermes colored treads
          line( -8, -6, -8, 8);      //  each five pixels wide
          line(  8, -6,  8, 8);      //  on either side of
          stroke( yellow);           //  an open, yellow triangle
          strokeWeight(2);
          noFill();
          triangle( -10, 7, 0, -9, 10, 7);

      popMatrix();    //  restore viewport origin coordinates
      noClip();       //  stop clipping
      noFill();
      stroke( hermesHigh);    //  draw viewport border
      strokeWeight( 1);
      rect( 0, 0, mapWidth, mapHeight);

    popMatrix();   //  restore MAP display screen coordinates

    //  Display displacement distance from center in millimeters
    //  Calculatred by dead-reckoning odometry from 'getPosition'
    fill(255);
    textFont( faceMSR20);     // Mono Space Roman 20
    text( "Track Encoder:", 30, 30);
    text( String.format( "PosX: %04dmm", drPosX), 30, 50);
    text( String.format( "PosY: %04dmm", drPosY), 30, 70);
    
    //  Display legend for scale of the MAP space
    //  show five meters with tic for each meter
    text( "Scale", 290, 30);
    text( "0 1 2 3 4 5m", 247, 55);
    stroke( ccWht);            //  color white
    strokeWeight(2);           //  three pixel wide
    line( 250, 70, 375, 70);   //  horizontal line
    line( 250, 65, 250, 70);   //  vertical line
    line( 275, 65, 275, 70);   //  vertical line
    line( 300, 65, 300, 70);   //  vertical line
    line( 325, 65, 325, 70);   //  vertical line
    line( 350, 65, 350, 70);   //  vertical line      
    line( 375, 65, 375, 70);   //  vertical line

    //  Display IMU linear acceleration, speed and Euler heading
    text( "Inertial Measurement Unit:", 30, 480);
    text( String.format( "X: Acc: %04dcm/s/s", imAccX), 30, 505);
    text( String.format( "Y: Acc: %04dcm/s/s", imAccY), 30, 530);
    text( String.format( "Spd: %04dmm/s", imSpdX), 270, 505);
    text( String.format( "Spd: %04dmm/s", imSpdY), 270, 530);
    text( String.format( "Head: %04d°", heading), 450, 505);

    //  Display MAP display screen title on top of everything
    fill( bigLetter);  //  Hermes Color Low
    textFont( faceBPB56);    //  Blener Pro Book 56
    text( "MAP", 495, 50);

    popMatrix();  // restore the saved screen cooordinates
}

//  - - - - - - - -  - - - - - - - - - - - - - - - -
