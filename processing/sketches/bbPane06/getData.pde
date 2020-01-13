// File Name: getData.pde
// Supporting:  bbPane05.pde
// Creator:   Bud Ryerson
// Inception: 20JAN2016
// LastTouch:
// 24MAR16 - We have some deep and serious timing problems.
//           Created new version 6 to mess with it.
// 04MAY16 - Created version 7 to resolve problems parsing
//           integers from the inputBuffer string.  Now we
//           determine ability to parse, convert to integers,
//           and format output strings in the draw routines.
//           We eliminated all subString variables.
//           Also, found and fixed a serious formatting error
//           with the "bearing" data.
// 04JUN16 - added "isNotCenter" to flag list
// 20JUL16 - converted to 32bit flagBit string
// 23SEP16 - lost the second eye
// 29JAN17 - added initializing character to signal flag array
// Described:
//   Called by the main program loop in 'bbPane06.pde'
//   Provide a 'calling' loop to wait for serial joystick contact.
//   Read serial string from the joystick
//   Parse the data into integer format
//   Write signal flag characters back to the joystick
//

// Variables for the getData serial data read
String strBuffer = "";  // holder for the input string to be chopped up
int strPos = 3;         // data buffer substring position indicator
int strBufSize = 0;     // this is for the COM display

//boolean firstContact = false;  // Whether we've heard from the joystick

boolean goodData = false;     //  false only if joystick data received not formatted correctly
                              //  otherwise true even if no data is recived
boolean putFlag = false;      //  clear to send signal string to joystick
//boolean getFlag = true;
boolean errFlag = false;

//RobotData1 rDat1 = new RobotData1();

int  joyX = 512;
int  joyY = 512;
int  velocity = 0;     //  platform velocity: -255 to 255
int  rotation = 0;     //  platform rotation: -255 to 255

int  heading = 0;      //  compass heading: 0 - 360 degrees
int  course = 0;
int  bearing = 0;      //  bearing to target: -180 to 180 degrees
int  euHead = 0;       //  Euler Heading

int  motSpeedL = 0;    //  dist: -255 to 255
int  motSpeedR = 0;    //  dist: -255 to 255
int  intCountL = 0;    //  wheel encoder interrupt counters
int  intCountR = 0;    //    from 0 to about 60 (reset every 50 msec.)

int  tempF =  66;      //  Lidar temperature in degrees Farenheit
int  dist  =  60;      //  Lidar distance in centimeters
int  flux  =  100;     //  Lidar return signal strength
int  nearPos = 90;     //  position of shortest dist

//char[] bitFlags = new char[32];  // declare character array
String bitFlags = "";
char[] outFlags = new char[16];  // char array of output signal flags
int outFlagWord = 0;
int flagsInt = 0;             // received platform flags as a 32 bit number
boolean[] fbVal = new boolean[ 32];

//  Define flag bit position within 32 bit flag word.
  static final int fbMotor  = 0;  // power to motors
  static final int fbAuto   = 1;  // autonomous mode
  static final int fbServo  = 2;  // IR sensor servo is on
  static final int fbRadio  = 3;  // link between joystick and platform
  static final int fbSerial = 4;
  static final int fbEvade  = 5;  // evade mode
  static final int fbHalt   = 6;  // wheels are not turning
  static final int fbPivot  = 7;  // platform pivoting
  static final int fbTrack  = 8;
  static final int fbBear   = 9;
  static final int fbProgram  = 10;  // running a predefined course program
  static final int fbFault    = 11;  // unknown fault condition
  static final int fbSeek     = 12;  // wheel motor current overload
  static final int fbObject   = 13;  // one of four corner sensors triggered
  static final int fbPosReset = 14;  // click to re-center positions on map
  static final int fbManual   = 15;  // joystick not centered - This will reset
                        // all auto functions such as evade and program.

  // individual corner proximity sensors
  static final int odFlagLF  = 16; // Left Front object detector
  static final int odFlagRF  = 17;
  static final int odFlagLR  = 18;
  static final int odFlagRR  = 19;

  //  robot platform buttons
  static final int fbRoBut1 = 20;  // robot button #1 - reset compass
  static final int fbRoBut2 = 21;  // #2 - run program
  static final int fbRoBut3 = 22;  // #3 - reseet position
  static final int fbRoBut4 = 23;  // #4 - toggle object detect
  static final int fbRoTim1 = 24;
  static final int fbRoTim2 = 25;

  // joystick console buttons
  static final int fbJoBut1 = 26;
  static final int fbJoBut2 = 27;
  static final int fbJoBut3 = 28;
  static final int fbJoTim1 = 29;
  static final int fbJoTim2 = 30;
  static final int fbJoTim3 = 31;

// platform motor current sensors
int  mcInteger = 0;  // motor current integer
int  lfCurrent = 0;  // left front current value
int  rfCurrent = 0;
int  lrCurrent = 0;
int  rrCurrent = 0;

//  Wheel Encoder or Dead Reckoning (dr) values
int drPosX = 0;  // Dead Reckoning position in millimeters is derived from
int drPosY = 0;  //  the average of both wheel encoder interrupt counters.
int drSpdX = 0;  // Dead Reckoning speed in millimeters per second,
int drSpdY = 0;  //  also derived from wheel encoder interrupt counters.

//  IMU - Inertial Measurement Unit (im) values
int imAccX = 0;  // IMU Linear Acceleration in millimeters per second per second
int imAccY = 0;
int imSpdX = 0;  // IMU speed in millimeters per second,
int imSpdY = 0;  // derived from accelration and course

String clockStr = "00:00:00";   //  HH:MM:SS since last platform reset
int sysMillis  = 0;
int sysHours   = 0;
int sysMinutes = 0;
int sysSeconds = 0;

int loopCount  = 0;
int loopTime   = 0;
int comTime    = 0;


//  Input string and radix and
//  return whether string can be parsed.
boolean isParsable( String input, int radix)
{
    boolean parsable = true;
    try
    {
        Integer.parseInt( input, radix);
        // Integer.parseInt( input, 16);   // Using hexadecimal radix
    }
    catch( NumberFormatException e)
    {
        parsable = false;
    }
    return parsable;
}

//  Extract decimal data substring and check whether string can be parsed.
//  If so, parse string and return integer value;
//  if not, report error and return former value.
Integer parseDataSubString( int formerValue, int strLen)
{
    String dataString = trim( strBuffer.substring( strPos, strPos + strLen));
    strPos += ( strLen + 3); // advance position 'strLen' plus the three ID characters
    if( isParsable( dataString, 10))
    {
        return Integer.parseInt( dataString);
        // return Integer.parseInt( dataString, 16);   // Using hexadecimal radix
    }
    println( "ERROR: Data substring not parsable.");
    return formerValue;
}

//  Extract hex data substring and check whether string can be parsed.
//  If so, parse string and return integer value;
//  if not, report error and return former value.
Integer parseHexSubString( int formerValue, int strLen)
{
    String dataString = trim( strBuffer.substring( strPos, strPos + strLen));
    strPos += ( strLen + 3); // advance position 'strLen' plus the three ID characters
    if( isParsable( dataString, 16))
    {
        return Integer.parseInt( dataString, 16);   // Using hexadecimal radix
    }
    println( "ERROR: Data substring not parsable.");
    return formerValue;
}

void getDataValues()
{
    // - - - Parse strings into integers for analog display - - -
    // Ensure that this sequence matches the joystick print sequence
    strPos = 3;  //  set position indicator to start of first data substring

    // JOY Screen X and Y values
    joyX = parseDataSubString( joyX, 4);
    joyY = parseDataSubString( joyY, 4);

    // Rotation subScreen values
    velocity = parseDataSubString( velocity, 4);
    rotation = parseDataSubString( rotation, 4);

    heading = parseDataSubString( heading, 3);
    //heading = intRay[ 3];
    course  = parseDataSubString( course,  3);
    bearing = parseDataSubString( bearing, 4);
    //euHead  = parseDataSubString( euHead, 4);

    // Motor Speeds and Encoder Counts
    motSpeedL = parseDataSubString( motSpeedL, 4);
    motSpeedR = parseDataSubString( motSpeedR, 4);

    intCountL = parseDataSubString(  intCountL,  3);  //  wheel encoder interrupts
    intCountR = parseDataSubString(  intCountR,  3);  //  dist: 0 - 60 (reset every 50 msec.)

    azDex = parseDataSubString( azDex, 3);   //  azimuth index value: 30 to 150
      azDex = constrain( azDex, 30, 150);
      azPos = azDex - 90;                    //  get azimuth postion in degrees
    dist  = parseDataSubString( dist, 3);    //  Lidar distance value
    if( dist > 0) distRay[ azDex] = dist/2;        //  Save dist/2 in Azimuth distance array
      else distRay[ azDex] = distRay[ azDex -1];   //  and try to smooth over zero values

    flux    = parseDataSubString( flux, 5);      //  Lidar return signal strength
      fluxRay[ azDex] = mapFlux();
    azDex2 = parseDataSubString( azDex2, 3);   //  second azimuth index value

    flagsInt = parseHexSubString( flagsInt, 8);  // array of flag bits as 32 bit integer
    for( int x = 0; x < 32; ++x)                 // convert the flags integer to boolean array
    {
        fbVal[x] =  boolean( ( flagsInt >>> x) & 1);
    }

    mcInteger = parseHexSubString( mcInteger, 8);  // motor current array as 32 bit number
      lfCurrent   = int( ( mcInteger >>>  0) & 0xFF);
      rfCurrent   = int( ( mcInteger >>>  8) & 0xFF);
      lrCurrent   = int( ( mcInteger >>> 16) & 0xFF);
      rrCurrent   = int( ( mcInteger >>> 24) & 0xFF);

    int dummy;
    dummy = parseDataSubString( drPosX, 7);  // dead reckoning X & Y positions
    dummy = parseDataSubString( drPosY, 7);  // +/- 32,000mm or 32 meters
    imAccX = parseDataSubString( imAccX, 7);  // IMU X&Y Linear Acceleration
    imAccY = parseDataSubString( imAccY, 7);  // +/- 32,000 mm/s/s
    imSpdX = parseDataSubString( imSpdX, 7);  // X&Y Speed derived from Acc and course
    imSpdY = parseDataSubString( imSpdY, 7);  // +/- 32,000 meters/second
    //getPosition();

    sysMillis   = parseHexSubString( sysMillis, 8); // platform time since start
    loopCount   = parseHexSubString( loopCount, 8); // platform loops since start
    loopTime    = parseDataSubString( loopTime,   4); // platform loop interval set by Timer 3
                                                      // should always be ~0.050 seconds

    sysHours = ( sysMillis / ( 1000 * 60 * 60)) % 24;
    sysMinutes = ( sysMillis / ( 1000 * 60)) % 60;
    sysSeconds = ( sysMillis / 1000) % 60;
    clockStr = String.format( "%02d:%02d:%02d", sysHours, sysMinutes, sysSeconds);

//    String headString = trim( strBuffer.substring( 275, 279));
//    char[] headChar = strBuffer.substring( 275, 275).toCharArray();
}

byte[] buff = new byte[300];
ByteBuffer byteBuff = ByteBuffer.wrap( buff, 20, 10);

//  = = = =    getData from Joystick   = = = =
//  Rread a string from the joystick and parse the characters as data
//  String format: "|JX=  493|JY=  512|PV=    0|PR=    0|PH=  000, etc."
static int gpdCountMax = 4; // max read retries
static int byteCount = 0;
void getData()
{
    int gpdCount = 0;   // zero counter for data read retres
    putFlag = false;
    strBuffer = null;   //  clear data string buffer
    while( gpdCount < gpdCountMax)
    {
        if( myPort.available() > 0)
        {
            strBuffer = "";   //  clear data string buffer
/*            myPort.readBytesUntil( '\n', buff);
            for( int x = 0; x < 253; ++x)   // convert array of bytes to a string
            {
                strBuffer += char( buff[ x]);
            }
            int bfx = 256;  // set buff index to the beginning of rdat1
            for( int x = 0; x < 16; ++x)   // serial write each flag character
            {
                int hi = buff[ bfx] & 0x00FF; // convert hi order byte to integer
                ++bfx;                       // increment buff index
                int lo = buff[ bfx] & 0x00FF;
                ++bfx;
                intRay[ x] = (short)((hi << 8) | (lo));
            }
*/
            // Examine the serial port looking for an EOL character
            strBuffer = myPort.readStringUntil('\n');
            //strBuffer = strBuff;
            // Do not continue unless an EOL character has been received
            if ( strBuffer != null)
            {
                 strBufSize = strBuffer.length();   //  this data element is for the COM display
                // test first characters and length
                //if( strBuffer.startsWith( "|JX") && strBuffer.length() == 253)
                //if( strBuffer.startsWith( "|JX") && strBuffer.length() == 225)
                //if( strBuffer.startsWith( "|JX") && strBuffer.length() == 214)
                if( strBuffer.startsWith( "|JX") && strBuffer.length() == 212) // 20190514
                {
                    // - - - Parse strings into integers for analog display - - -
                    // Ensure that this sequence matches the joystick print sequence
                    getDataValues();

                    goodData = true;
                    putFlag = true;         // say okay to write to PC,
                    gpdCount = gpdCountMax; // and kill the loop
                }
                else
                {
                    goodData = false;   //  set goodData flag FALSE
                }
            }
      	}
        ++gpdCount;   // advance the count
        delay( 1);    // and wait to try again
    }
}
// - - - -    End of getData    - - - -

//  = = = =    putData is second    = = = =
//  After all the screen drawing and data display is done,
//  write sixteen signal characters back to the joystick.
//  Bits of 'outFlags' can be set during 'drawButtonBar'
//  and each is reset to zero here after being written.
void putData()
{
    myPort.write( '|');            // serial write initializing character
    for( int x = 0; x < 16; ++x)   // serial write each flag character
    {
        myPort.write( outFlags[x]);
        outFlags[ x] = '0';        // reset each flag character sending
                                   // set to all zeros in program 'setup'
    }
    myPort.write( "\n");           // write string terminating characters

/*  if( outFlagWord > 0)
  {
    print( "Out: ");
    for( int x = 0; x < 16; ++x)
    {
        print( outFlags[ x]);
        outFlags[ x] = '0';    // reset each flag character after printing
    }
    // print a separator character followed by the output
    // flag word in five decimal places with leading zeros
    System.out.printf( " | %05d", outFlagWord);
    println();
    // then clear the output flag word
    outFlagWord = 0;
  } */

}

//  13APR19
//  Return screen brightness values (0 - 255)
//  from raw 'flux' values (0-5000)
int mapFlux()
{
    int fluxMin = 300;
    int fluxMax = 3000;
    int fluxRange = (fluxMax - fluxMin);
    int fluxValue;
    int brightMin = 100;
    int brightMax = 255;
    int brightRange = ( brightMax - brightMin);
    //static int brightValue = 255;
 
    fluxValue = constrain( flux, 300, 3000); 
    return (((flux - fluxMin) * brightRange) / fluxRange) + brightMin;
}
// - - -    End of PutData    - - - -
