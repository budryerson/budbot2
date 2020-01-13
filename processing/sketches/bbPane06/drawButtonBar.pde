//  File Name: drawButtonBar.pde
//  Supporting:  bbPane06.pde
//  Creator:   Bud Ryerson
//  Inception: 21 OCT2016
//  Described: draw and illuminate a row of buttons
//             and lights across the top of the display
//  25DEC16 - added mouse over to button bar

int bbHigh = 50;   // button bar height
int bbWide = 1600; // button bar width
int butWidth = 80;
int butHeight = 30;

void drawButtonBar( int xOrg, int yOrg)
{
    pushMatrix();
    translate( xOrg, yOrg);

      // draw button box
      fill( ccBlk);
      strokeWeight( 1);
      stroke( hermesHigh);
      rect( 0, 0, bbWide, bbHigh);

      // set button label characteristics
      fill( ccWht);          //  white
      textAlign( CENTER);
      textFont( faceMSR20);  //  Mono Space Roman 20 pixels
      noStroke();

      //  label and light each button
      translate( 10, 10);    //  move to first button position
      drawButton( fbMotor,    ccMotor,    "MOTOR");
      drawButton( fbAuto,     ccAuto,     "AUTO");
      drawButton( fbServo,    ccServo,    "SERVO");
      drawButton( fbRadio,    ccRadio,    "RADIO");
      drawButton( fbSerial,   ccSerial,   "SERIAL");
      drawButton( fbEvade,    ccEvade,    "EVADE");
      drawButton( fbHalt,     ccHalt,     "HALT");
      drawButton( fbPivot,    ccPivot,    "PIVOT");
      drawButton( fbTrack,    ccTrack,    "TRACK");
      drawButton( fbBear,     ccBear,     "BEAR");
      drawButton( fbProgram,  ccProgram,  "PRGM");
      drawButton( fbFault,    ccFault,    "FAULT");
      drawButton( fbSeek,     ccSeek,     "SEEK");
      drawButton( fbObject,   ccObstacle, "OBST");
      drawButton( fbPosReset, ccPosSet,   "RESET");
      drawButton( fbManual,   ccManual,   "MANUAL");
      textAlign( LEFT);

    popMatrix();
}

//  For each of certain buttons, set 'outFlags' bit to '1' if clicked.
//  Bit states are held until 'putData' routine.  Then each 'outFlags'
//  bit is reset to zero after it is serial written.
void drawButton( int dbN, color dbColor, String dbLabel)
{
    //  enable mouse click for only the following signal flags
    if( dbN == fbMotor || dbN == fbAuto || dbN == fbServo ||
        dbN == fbProgram || dbN == fbObject || dbN == fbPosReset)
    {
        if( overButton( dbN))  //  highlight the button when mouse is over
        {
            stroke( 250);
            strokeWeight( 2);
            rect( 0, 0, butWidth, butHeight, 3);     // round corners with 3px radius
            if( pressButton( dbN))
            {
                fill( orange);
                rect( 0, 0, butWidth, butHeight, 3);
                outFlags[ dbN] = '1';   //  set a bit flag binary character
                outFlagWord |= 0x0001 << dbN;
            }
        }
        noStroke();
    }
    if( fbVal[ dbN]) fill( dbColor); else fill( ccOff);
    rect( 0, 0, butWidth, butHeight, 3);     // round corners with 3px radius
    fill( ccWht);               // text color White
    text( dbLabel, 40, 22);     // write button label
    translate( 100, 0);         // move to next button position
}


//  This provides a 10 millisecond timer to debounce the mouse click
//  If mouse is pressed, it cannot be pressed again for 'pbTimeOut' milliseconds
int[] pbTimerRay = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int pbTimeOut = 10;
boolean pressButton( int pbN)
{
  if( mousePressed)
  {
      if( pbTimerRay[ pbN] < millis())
      {
          pbTimerRay[ pbN] = millis() + pbTimeOut;
          return true;
      }
  }
  return false;
}

// Determine if the mouse pointer is over a button field
boolean overButton( int obN)
{                             // find horizontal distance from the UprLft corner of the main window
    int posX = 20 +           // to the UprLft corner of the button bar
               10 +           // plus the distance to the UprLft corner of the first button shape
               ( obN * 100);  // plus the distance between each button shape
    int posY = 20 + 10;
    if( mouseX >= posX  &&
        mouseX <= posX + butWidth &&
        mouseY >= posY &&
        mouseY <= posY + butHeight)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//  - - - - - - - -  - - - - - - - - - - - - - - - -
