//  File name: drawRadar.pde
//  Support:   bbPane06.pde
//  Creator:   Bud Ryerson
//  Date:      14OCT2016
//  called by the drawNAV routine
//  draw the RADAR info shape over the NAV grid screen
//  at an apparent ratio of 2cm per pixel. 
//
//  Includes the 'Scan' shape class definition at the end
//  05DEC19 - changed scale of MAP display in 'Scan' class definition
//            so that 1 pixel = 4 centimeter

PShape scanShape1;   // shape variable of Lidar scan for NAV (Radar)
                     // and MAP displays defined in 'bbPane06' setup
ArrayList scans;     // array of scan shapes for MAP
                     // scans Class definition is at bottom
int scansArrayMaxSize = 30;  // maximum number of scan shapes in ArrayList

int motion = 0;                // scan moving left or right
int fillOff = 0;               // scan fill offset
int scanCount = 0;

// Constant values for robot platform servo position 
//  • Zero (0) degrees is directly ahead of the platform
//  • Negative values are counter-clockwise, toward the left (West)
//  • Positive values are clockwise, toward the right (East). 
static final int  azMin = -60;    // far left (West) limit (-60°)
static final int  azMid =   0;    // straight ahead, middle (0°)
static final int  azMax =  60;    // far right (East) limit (+60°)
static final int  elMin = -60;    // low, bottom limit (-60°)
static final int  elMid =   0;    // straight ahead, horizontal (0°)
static final int  elMax =  60;    // high, upper limit (+60°)
// Variables for robot platform servo position
int  azPos =  azMid;     //  Azimuth servo position in degrees
int  elPos =  elMid;     //  Elevation servo position in degrees

// Constant values for azimuth and elevation data array indices
static final int  adMin =  30;    // far left scan limit   (  30)
static final int  adMid =  90;    // center straight ahead (  90)
static final int  adMax = 150;    // far right scan limit  ( 150)
static final int  edMin =  30;    // upper scan limit (  30)
static final int  edMid =  90;    // level horizon    (  90)
static final int  edMax = 150;    // lower scan limit ( 150)
// Variables for azimuth and elevation data array indices
int  azDex =  adMid;     //   First Azimuth data array index value
int  azDex2 = adMid;     //   Second Azimuth data array index value
int  elDex =  edMid;     //   Elevation data array index value

// 'distRay' is continuously filled during 'getData'
int[] distRay  = new int[ 181];  // Lidar distance values by servo positions: 0 to 180 degrees
int[] distRay0 = new int[ 181];  // additional array of distance for comparison
int[] fluxRay  = new int[ 181];  // Lidar return signal strength array

//  To save doing repeated sine and cosine calculations
//  arrays of X and Y scan shape trigonometric values
//  are precalculaated in 'setupoRadar()'
float[] xPosvRay = new float[ 181];
float[] yPosvRay = new float[ 181];

int radarRadius = navWide/2;   // radar screen radius is 1/2 of NAV screen width

void setupRadar()
{
    //  Initalize scanning arrays
    for( int i =  0; i < 181; ++i)
    {
        //  Clear the Lidar sensor dist value array.
        distRay[ i] = 0;
        //  Fill X and Y scan shape trigonometric position
        //  array with precalculated values        
        xPosvRay[ i] = cos( PI - radians( i));
        yPosvRay[ i] = sin( TWO_PI - radians( i));
    }

    // Setup the shape of the scan image
    noStroke();                        // no outline
    fill( scanGreen);                  // predefined green color
    scanShape1 = createShape();        // Create a shape object.
    scanShape1.beginShape();           // Establish a scanShape with
    scanShape1.vertex( 0, 0);          // the first vertex at the origin.
    // Then for each degree in the scan, create another vertex.
    // using the Min/Max constant values defined above.
    for (int i = azMin; i <= azMax; ++i) scanShape1.vertex( 0, 0);
    scanShape1.endShape( CLOSE);       // And connect back to the origin.

    scans = new ArrayList();
    scans.add( new Scan( 0, 0, 0, distRay));
}

void fillArray()
{
  // Copy the Lidar value array to the comparison array.
  for( int i = adMin; i <= adMax; i++) distRay0[ i] = distRay[ i];
  // If wheels are not turning, add new scan to end of arrayList.
  if( !(intCountL + intCountR > 0))
  {
      // Add new scan to array with Dead Reckoning positions,
      // heading, and array of Lidar values.
      scans.add( new Scan( drPosX, drPosY, heading, distRay0));
      // If the variable length AarrayList gets too big,
      // remove the earliest scanShape.
      if (scans.size() > scansArrayMaxSize) scans.remove(0);
  }
}

void drawRadar()
{
  //  Get x and y vertex positions from predefined array of cosines and sines
  //  (Cosine for the x or left/right value and Sine calculates the y or Up/Down value)
  //  and multiply times the dist value to get the radius betweeen 0 and 500.

  // initialize motion direction
  if( ( azDex >= 150) && ( motion == 0))    //  if at far right of sweep
  {
      fillArray();
      fillOff = 0;
      azDex = 150;
      motion = 1;           //  set animation to run right to left
  }
  if( ( azDex <= 30) && ( motion == 1))    //  if at far right of sweep
  {
      fillArray();
      fillOff = 0;
      azDex = 30;
      motion = 0;         //  run animation left to right
  }

/*
  // = = = =  create a 'fading trail' overlay for the radar sweep  = = = =
  //  Only create effect if Servo is ON
  //  Draw 20 full ines, each one degree before (or after) the 'azPos'.
  //  Fade the green color 10 units each time through the loop.
  if( fbVal[ fbServo])
  {
      strokeWeight(7);         //  set the thickness of the lines
      int sPos = azDex;        //  set shape position to azimuth index value
      for( int i = 0; i < 20; i++)
      {
          stroke( 0, 200 - ( 10 * i), 0);        // reset RGB color so G value is 'i x 10'
          if( motion != 0) ++sPos; else --sPos;  // fade right or left
          if( ( sPos >= 30) && ( sPos <= 150))   // only draw within azPos limiits
              line( 0, 0, xPosvRay[ sPos] * radarRadius, yPosvRay[ sPos] * radarRadius);
      }
  }
  // - - - -  End of Fading Trail effect - - - -
*/

  //  modify the shape of a PGraphic scanShape image and draw it
  int vPos = 0;
  for( int i = adMin; i <= adMax; i++)  // for each degree of the scan
  {
    ++vPos;
    scanShape1.setVertex
    (
        vPos,                        //  Vertex index
        // X position for dist constrained within radar scope radius
        xPosvRay[ i] * constrain( distRay[ i], 0, 250),
        // Y position for dist constrained within radar scope radius
        yPosvRay[ i] * constrain( distRay[ i], 0, 250)
     );
  }
  // Dont draw green shape on RADAR screen, save for MAP screen
  // shape( scanShape1);      //  draw the scan shape

  
  //  Draw a green line connecting all the distance values
  //  Draw a green dot to represent the brightness of the flux
  strokeWeight( 2);
  noFill();
  for( int i = adMin; i < adMax; i++)
  {
      stroke( 0, fluxRay[ i], 0);
      line( xPosvRay[ i] * constrain( distRay[ i], 0, 250),
            yPosvRay[ i] * constrain( distRay[ i], 0, 250),
            xPosvRay[ i+1] * constrain( distRay[ i+1], 0, 250),
            yPosvRay[ i+1] * constrain( distRay[ i+1], 0, 250)
          );
  }
  noFill();

  //  Draw red circle at nearest object
  int nDex = adMid;
  int fDex = adMid;
  for( int i = adMin; i <= adMax; i++)
  {
      // find the nearest point
      if( distRay[ i] < distRay[ nDex]) nDex = i;
      // find the furthest point
      if( distRay[ i] > distRay[ fDex]) fDex = i;
  }
  strokeWeight( 0);
  fill( 200, 0, 0);
  ellipse( xPosvRay[ nDex] * constrain( distRay[ nDex], 0, 250), yPosvRay[ nDex] * constrain( distRay[ nDex], 0, 250), 12, 12);
  //strokeWeight( 0);
  fill( 0, 0, 250);
  ellipse( xPosvRay[ fDex] * constrain( distRay[ fDex], 0, 250), yPosvRay[ fDex] * constrain( distRay[ fDex], 0, 250), 12, 12);
  noFill();


/*
  //  Where dist differs more than 35cm from previous scan
  //  draw red circle at location of previous scan
  stroke( 150, 0, 0);
  strokeWeight( 2);
  noFill();
  for( int i = adMin; i <= adMax; i++)
  {
    if ( distRay0[ i] - distRay[ i] > 35 || distRay[ i] - distRay0[ i] > 35)
    {
      ellipse( xPosvRay[ i] * distRay0[ i], yPosvRay[ i] * distRay0[ i], 10, 10);
    }
  }
  noFill();
*/
}

// Foundation class for multiple Lidar dist finder scan shapes
// Used to fill variable length ArrayList declared above
class Scan
{
    color sColor = color( 0, 150, 0, 255);  //  150 levels of green
    PShape sShape;  //  local PShape object
    int sPosX;
    int sPosY;
    float sHead;

    // Save the MAP screen position, the Heading direction and an array of dist values
    // to the 'scans' ArrayList when instantiated by each sweep of the RADAR routine.
    Scan( int xPos, int yPos, int hDir, int[] vRay)
    {
        // MAP positions are scaled so that 1 pixel = 1 centimeter
        // and constrained to remain within the MAP boundaries.
        sPosX = constrain( ( xPos / 10), -275, 275);
        sPosY = constrain( ( yPos / -10), -235, 135);  // y value is inverted
        // Convert heading to radians
        sHead = radians( hDir);

        // Setup the shape of the scan image
        noStroke();               // no outline
        fill( sColor);            // green color
        sShape = createShape();
        // Establish a shape with the first vertex at the origin
        // and another vertex for each degree of the servo scan
        sShape.beginShape();
        sShape.vertex( 0, 0);
        for( int i = adMin; i <= adMax; i++)
        {
          // display MAP at 1pix:1cm
          // sShape.vertex( xPosvRay[ i] * vRay[ i], yPosvRay[ i] * vRay[ i]);
          // display the MAP at 2px:1cm (1/2 scale)
            sShape.vertex( xPosvRay[ i] * vRay[ i]/2 , yPosvRay[ i] * vRay[ i]/2);
        }
        sShape.endShape( CLOSE);          // And connect back to the origin.
    }

    //  Reduce opacity of the scan shape color.
    //  Called by a timer in the 'drawMAP' routine.
    void fadeScan()
    {
        float sAlpha = alpha(sColor);        // get the alpha level of the color
        if( sAlpha > 0) --sAlpha;            // reduce the alpha level
            else sAlpha = 0;                 // until it is zero
        sColor = color( 0, 150, 0, sAlpha);  // recreate color with new alpha level
    }

    //  Draw the shape onto the MAP
    //  Called by the MAP routine
    void showScan()
    {
        pushMatrix();                 //  Push current matrix onto the stack
          translate( sPosX, sPosY);   //  Reposition scan on the MAP
          rotate( sHead);             //  Rotate scan to match platform Heading
          sShape.setFill( sColor);    //  Fade the scan color's opacity
          shape( sShape);             //  Draw the actual scan shape
        popMatrix();                  //  Pop the matrix off the stack
    }
}

//  - - - - - - - -  - - - - - - - - - - - - - - - -
