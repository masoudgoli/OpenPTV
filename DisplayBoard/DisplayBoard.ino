/*
Version 04: changed the variable 'voltage' (from display board via SPI) to voltage_D
reason: in Mainboard we have also the variable 'voltage'

###################################################
Pinout:

Voltage  A0
Steering A1   
Throttle A2   
CS L     A3
CS R     A4
res      A5 // we use A5 as D19 for SoftwareSerial of the RFID reader
res      A6
res      A7
speed L  D2
speed R  D3
LCD      D4  
Speaker  D5
LED red  D6
RFID_OK  D7
ID-20_P2 D8
res      D9  // currently used as strike pin; will be RFID_OK to MEGA
SS       D10
MOSI     D11
MISO     D12
SCK      D13
shall breaklight be defined?
###################################################
Change log:
changed Serial-LCD from SoftwareSerial to HardwareSerial
#define LCD Serial // change txPin 4 to TX (pin1)
Now Pin1 has to be removed when uploading a new sketch

The change to HW-Serial on RFID had only a 2ms advantage
readBatteryVoltage() - - nextPrintTime = millis()+100;  // changed from 1000 to 100

reduced the LCDdelay from 10 to 5 (2 doesn't work)
made the Throttle display leaner
included Battery symbol for alternative Battery status

*/

#include <SPI.h>
#define SPI_SS_PIN SS
#include <SoftwareSerial.h>
#include <serLCD.h>
SoftwareSerial mySerial(19, 20); //pin A5 (=D19) Rx, pin A6 (=D20) Tx // Tx does not need to be connected!
// collision with CS_R : try out whether 19, 20 also works.(can only done on the Wattuino)
// optional add a 470 Ohm resistor for better readings!

//#define txPin 4 	// changed
//SoftwareSerial LCD = SoftwareSerial(0, txPin);

#define LCD Serial // change txPin 4 to TX (pin1)
const int LCDdelay=5;  // conservative, 2 actually works  was 10
//#define DbgPrint Serial.println
#define DbgPrint(x) 
#define MAX_RFID_READ_TIME 250  // In milliseconds // was 250

enum // Message Types
  {
    NothingMsgId = 0,
    HelloMsgId = 1,
    ThrottleMsgId = 2,

    WaitingMsgId = 100,
    WaitingForCmdMsgId = 101,
    OkMsgResponse = 102,
   
    AbortMsgResponse = 0xff,

  };

// ----------------  declare Pins and variables ---------------------

// Set your left and right throttle sensors on pins A0 and A1 (analog 0 and 1)
const int throttlePin = A2;   // input for throttle // was A0

//const int throttleMin = 151;  // min/max values of throttle
//const int throttleMax = 823;  // values measured
double tempTrottleValue = 0;

const int steeringPin = A1;       // input for steering pot
int steeringValue = 0;
const int steeringZeroLeft = -5;  // define range for going straight
const int steeringZeroRight = 5;  // "backlash" or "lost motion"
double tempSteeringValue = 0;
double subtractSteeringValue = 0;

int ledpin = 6;
int potpin = A0;
int ledvalue = 0;
int voltage_D =0;  
boolean lowVoltage = false;
int V1;

int CS_L = A3;
int CS_R = A4;
int CS_L_Value = 0;
int CS_R_Value = 0;

int speedL = 2;
int speedR = 3;
volatile byte rpmcountL;
volatile byte rpmcountR;
unsigned int rpmL;
unsigned int rpmR;
unsigned long timeoldL;
unsigned long timeoldR;
unsigned int  averageRPM;
unsigned int  vehicleSpeed;

// These 2 variables will hold the throttle values that will be sent to the Mega
int Motor_LValue = 0;            // output for motor h-bridge
int Motor_RValue = 0;
char Motor_Lstring[10], Motor_Rstring[10]; // create string arrays


// ------------------------------- Functions ---------------------------------

// BSD licensed from Dave Hylands http://websvn.hylands.org
unsigned char crc8( unsigned char inCrc, unsigned char inData )
{
    unsigned char   i;
    unsigned char   data;

    data = inCrc ^ inData;
  
    for ( i = 0; i < 8; i++ ) 
    {
        if (( data & 0x80 ) != 0 )
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
    }
    return data;
} 

byte runningCrc = 0;
byte Stransfer(byte b)
{
  SPDR = b;
  runningCrc = crc8(runningCrc,b);
  while (!(SPSR & (1<<SPIF)));
  return SPDR;
}

// SerialLCD calls *******************************************************************************************
// wbp: goto with row & column
#define LCD_SETCGRAMADDR	0x40

void lcdCommand(uint8_t value)
{
  LCD.write(0xFE);
  LCD.write(value);
  delay(5);
}

void lcdCreateChar(int location, uint8_t charmap[])
{
  location -= 1;
  location &= 0x07;
  for (int i=0; i<8; i++){
    lcdCommand(LCD_SETCGRAMADDR | (location << 3) | i);
    LCD.write(charmap[i]);
  }
}

void lcdPosition(int row, int col) {
  LCD.write(0xFE);   //command flag
  LCD.write((col + row*64 + 128));    //position 
  delay(LCDdelay);
}
void clearLCD(){
  LCD.write(0xFE);   //command flag
  LCD.write(0x01);   //clear command.
  delay(LCDdelay);
}
void backlightOn() {  //turns on the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(157);    //light level: 128 = 0ff, 140 = 40%, 150 = 73%, 157 = 100%
  delay(LCDdelay);
}
void backlightOff(){  //turns off the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(128);     //light level for off.
   delay(LCDdelay);
}
void serCommand(){   //a general function to call the command flag for issuing all other commands   
  LCD.write(0xFE);
}
// the 8 arrays that form each segment of the custom numbers
byte Bat_00Char[8] = {
	0b01110,
	0b11011,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b11111
};

byte Bat_20Char[8] = {
	0b01110,
	0b11011,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b11111,
	0b11111
};

byte Bat_40Char[8] = {
	0b01110,
	0b11011,
	0b10001,
	0b10001,
	0b10001,
	0b11111,
	0b11111,
	0b11111
};

byte Bat_60Char[8] = {
	0b01110,
	0b11011,
	0b10001,
	0b10001,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

byte Bat_80Char[8] = {
	0b01110,
	0b11011,
	0b10001,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

byte Bat_100Char[8] = {
	0b01110,
	0b11011,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};



//---------------------------------------------------------------------------------------------
// Melodies part
#include "pitches.h"
int j_melody[] = {NOTE_G4,0,NOTE_A4,0, NOTE_B4,0,NOTE_A4,0,NOTE_B4,0, NOTE_C5,0};
int j_noteDurations[] = {8,8,8,8,8,4,8,8,8,8,8,4};
int d_melody[] = {NOTE_C4,0,NOTE_D4,0,NOTE_F4,0,NOTE_D4,0,NOTE_F4,0,NOTE_G4,0};
int d_noteDurations[] = {8,8,8,8,8,4,8,8,8,8,8,4};
int fail_melody[] = {NOTE_G2,0,NOTE_F2,0,NOTE_D2,0};
int fail_noteDurations[] = {8,8,8,8,8,4};
int R2D2_beep[] = {NOTE_C3, NOTE_C4,0}; // there was a 5 instead of 0 
int R2D2_beep_noteDurations [] = {8,4};
int R2D2_melody[] = {NOTE_A7,0,NOTE_G7,0, NOTE_E7,0, NOTE_C7,0, NOTE_D7,0, NOTE_B7,0, NOTE_F7,0, NOTE_C8,0, NOTE_A7,0, NOTE_G7,0, NOTE_E7,0, NOTE_C7,0, NOTE_D7,0, NOTE_B7,0, NOTE_F7,0, NOTE_C8,0};
int R2D2_melody_noteDurations [] = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};  
// try A7 G7 E7 C7 D7 B7 F7 C8 A7 G7 E7 C7 D7 B7 F7 C8; same duration
int R2D2_close[] = {NOTE_A7,0,NOTE_G7,0, NOTE_E7,0, NOTE_C7,0, NOTE_D7,0, NOTE_B7,0, NOTE_F7,0, NOTE_C8,0, NOTE_A7,0, NOTE_G7,0, NOTE_E7,0, NOTE_C7,0, NOTE_D7,0, NOTE_B7,0, NOTE_F7,0, NOTE_C8,0};
int R2D2_close_noteDurations [] = {8,8,8,8,4,8,8,8,8,4,8,8,8,8,4};  

int beep_beep_melody[] = {NOTE_GS2, NOTE_GS2};
int beep_beep_noteDurations[] = {6,5};   
int beep_beep_beep_melody[] = {NOTE_C4, NOTE_B3, NOTE_C4};
int beep_beep_beep_noteDurations[] = {8,4, 1};   // eigth, quarter, full note

int speaker_pin = 5;
// Melodies end
//---------------------------------------------------------------------------------------------
// Toggle ON/OFF part
boolean wasOff = true;
boolean wasOn = false;
//***********************************************************************************
// Set up outputs for the strike plate and status LEDs.
// If you have built the circuit exactly as described in Practical
// Arduino, use pins D12 and D13:
#define strikePlate 9    // this might be changed later
#define ledPin 6
int AUTHORIZED_MODE_1 = 0;

// Specify how long the strike plate should be held open.
#define unlockSeconds 2

// The tag database consists of two parts. The first part is an array of
// tag values with each tag taking up 5 bytes. The second is a list of
// names with one name for each tag (ie: group of 5 bytes).
char* allowedTags[] = {
  "0000000000", // Tag 1 card red dot    "030030C8DE" with "000..00" it returns "not authorized"
  "030030DBC6", // Tag 2 card no dot
  "05007E4DC0", // Tag 3 blue chip
  "0700455827", // Tag 4 yellow chip
  "05003DE8BB", // Tag 5 red chip
  "0047E21A5A", // Tag 6 car key
  "0047E21AE5", // Tag 7 glass tube
};
// List of names to associate with the matching tag IDs
char* tagName[] = {
  "card with red dot", // Tag 1
  "card without dot",  // Tag 2
  "blue chip",         // Tag 3
  "yellow chip",       // Tag 4
  "red chip",          // Tag 5
  "car key",           // Tag 6
  "glas tube",         // Tag 7
};

// Check the number of tags defined
int numberOfTags = sizeof(allowedTags)/sizeof(allowedTags[0]);
int incomingByte = 0; // To store incoming serial data
//********************************************************************************************

void setup() 
{
  // Open Serial Ports
  Serial.begin(9600);			
//  Serial.println("Display Board");
  LCD.begin(9600);  // can also be 19200
  mySerial.begin(9600);
  pinMode (ledPin, OUTPUT);
  digitalWrite (ledPin, LOW);
  LCD.print("P-REx starting !");
  delay (500);
  clearLCD();
  LCD.print("Please RFID card");
 
// pins for RFID
  pinMode (ledpin, OUTPUT);
  digitalWrite (ledpin, LOW);
  pinMode (potpin, INPUT);

// pins for SPI: SS, MOSI, MISO, SCK
  digitalWrite(SPI_SS_PIN, HIGH);  // Pull it up so SPI is off unless actively pulled down by master
  pinMode(SPI_SS_PIN, INPUT);
 
  digitalWrite(MOSI,LOW);
  pinMode(MOSI,INPUT);
 
  digitalWrite(MISO,LOW);
  pinMode(MISO, OUTPUT);
 
  digitalWrite(SCK, LOW);
  pinMode(SCK,INPUT);
 
  //SPI.setDataMode(SPI_MODE0); SPI.setBitOrder(MSBFIRST); SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPCR |= _BV(SPE);
 
  // Switch to slave mode -- don't need this b/c its slave by default
  //SPCR &= ~(1<<MSTR);

  pinMode (throttlePin, INPUT); 
  pinMode (steeringPin, INPUT); 

//  pinMode(speedL, OUTPUT);
//  pinMode(speedR, OUTPUT);
//  attachInterrupt (0, speedreadingL, CHANGE);
//  attachInterrupt (1, speedreadingR, CHANGE);
 
//variables for rpm and speed calculaton 
  rpmcountL = 0;
  rpmcountR = 0;
  rpmL = 0;
  rpmR = 0;
  timeoldL = 0;
  timeoldR = 0;
  averageRPM = 0;
  vehicleSpeed = 0;
  
  DbgPrint("PREx Display Sketch V0.1");
  readBatteryVoltage();
  
   // assignes each segment a write number
  lcdCreateChar(1,Bat_00Char);
  lcdCreateChar(2,Bat_20Char);
  lcdCreateChar(3,Bat_40Char);
  lcdCreateChar(4,Bat_60Char);
  lcdCreateChar(5,Bat_80Char);
  lcdCreateChar(6,Bat_100Char);   
}


byte sendThrottle(void)
//byte sendVoltage(void)
{
//Serial.println ("send data");
  byte data;
  
  runningCrc=0;  // Reset the checksum since this is a new data transmission

  data = Stransfer(Motor_LValue>>8);
  if (data != WaitingMsgId) return 1;
  data = Stransfer(Motor_LValue&255);
  if (data != WaitingMsgId) return 2;

  data = Stransfer(Motor_RValue>>8);
   if (data != WaitingMsgId) return 3;
  data = Stransfer(Motor_RValue&255);
  if (data != WaitingMsgId) return 4;

  data = Stransfer(voltage_D>>8);
  if (data != WaitingMsgId) return 5;
  data = Stransfer(voltage_D&255);
  if (data != WaitingMsgId) return 6;

  // checksum
  data = Stransfer(runningCrc);
  if (data != WaitingMsgId) return 7;

  return 0;
}

void calcMotorValues(int rawThrottleValue, int rawSteeringValue)
{
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
  int throttleValue = constrain(rawThrottleValue, 184, 800);      // cut off wrong readings
  throttleValue = map(throttleValue, 189, 800, 0, 255);    // and scale it to 0-255 for motor PWM // make lower end less sensitive (189 instead of 184)
if (rawThrottleValue < 160) { 

// put in here the procedure for breaking
// maybe just brakelight?
  lcdPosition(0,0);
  LCD.print("                ");
  lcdPosition(0,0);
//  digitalWrite (breaklight, HIGH);  
  LCD.print("   B R E A K");
  delay(100);
  clearLCD();
}
  
    //steeringValue = constrain(steeringValue, 0, 1024);             // cut off wrong readings
  int steeringValue = map(rawSteeringValue, 0, 1024, -100, 108);     // 108 instead of 100 to adjust full range of pot 

  if (steeringValue < steeringZeroLeft && throttleValue > 0)         // drive left (keep Motor_R on gas value)
  {                                                                  // steering is -100..0
    tempSteeringValue = map(steeringValue, -100, steeringZeroLeft, 1, 100);    
    subtractSteeringValue = tempSteeringValue/100 * throttleValue ;  // calculate percentage how much less motor_L has to run
    Motor_LValue = subtractSteeringValue ;      				     // subtract percentage from throttle value
    Motor_RValue = throttleValue;                                    // keep right motor on throttle value
  }

  else if (steeringValue > steeringZeroRight && throttleValue > 0)            // drive right (keep Motor_L on gas value)
  {
    tempSteeringValue = map(steeringValue, steeringZeroLeft, 100, 100, 1);    // calculate percentage how much less motor_L has to run
    subtractSteeringValue = tempSteeringValue/100 * throttleValue ;
    Motor_LValue = throttleValue ;
    Motor_RValue = subtractSteeringValue ;    
  }

  else if (steeringValue <= -95 && throttleValue <= 3)       // if handle bar is fully towards LEFT and NO gas
  {
    Motor_LValue =  0 ;                                      // drive slowly on the spot to left
    Motor_RValue = 15 ;      
  }

  else if (steeringValue >= 95 && throttleValue <= 3)        // if handle bar is fully towards RIGHT and NO gas
  {
    Motor_LValue = 15 ;                                      // drive slowly on the spot to right
    Motor_RValue =  0 ;      
  }

  else   // drive straight (M_L and M_R same value)
  {
    Motor_LValue = throttleValue;
    Motor_RValue = throttleValue;   
  }
/*
// test to calibrate steering pot
Serial.print("Steering ");
Serial.println(steeringValue );
*/
}

void readBatteryVoltage()
{
  unsigned long int potvalue = analogRead (potpin);   // later read A0
  // potvalue ranges from 0 to 1023, representing a voltage range from 0 to 29
  // Transform it into a fixed point number denoting hundredths of a volt.  i.e 100 = 1v
  voltage_D = (potvalue * 29) / 10.23;  
  V1 = (voltage_D/28);    // ought to be 29 ??
//* commented out 06/14/13
//here was the voltage calculation
//*/

  // The Mega will poll us when it wants to know the data.
  // delay (5000);   // send every 5 seconds only
}



void dumpReadings(int throttleValue, int steeringValue, int voltage_D)  
{
  // output on display
//clearLCD();
//delay(200);

/*
  LCD.print("Left Right  Volt");
  lcdPosition (1,1);
  LCD.print(Motor_LValue);
  lcdPosition (1,6);
  LCD.print(Motor_RValue);
//  lcdPosition (1,12);
//  LCD.print(tempSteeringValue);
  lcdPosition (1,13);
  LCD.print(voltage_D/100);
  lcdPosition (0,0);
  delay(100);  
*/  

//  backlightOn();
  clearLCD();
  LCD.print("L     R");
  lcdPosition (0,2);
//  LCD.print(Motor_LValue);  // https://www.sparkfun.com/tutorials/246
  sprintf(Motor_Lstring, "%3d", Motor_LValue);
  LCD.write(Motor_Lstring);
//  lcdPosition (0,6);  
//  LCD.print("R");
  lcdPosition (0,8);  
//  LCD.print(Motor_RValue);
  sprintf(Motor_Rstring, "%3d", Motor_RValue);
  LCD.write(Motor_Rstring);
//clearLCD();
  lcdPosition (1,0);
  display_voltage ();
  lcdPosition (0,0);
  delay(2);   //was 200
}
 

int checkRFID(char* tagValue)
{
//  byte i = 0;
  byte val = 0;
  byte checksum = 0;
  byte bytesRead = 0;
  byte tempByte;
  byte tagBytes[6]; // "Unique" tags are only 5 bytes but we need an extra byte for the checksum
//  char tagValue[10];
//  unsigned long int start = millis();

  // Check for the STX Header (02 ASCII Value)
  if((val = mySerial.read()) == 2) 
    {                 
      bytesRead = 0; 
      // Read the RFID 10 digit code & the 2 digit checksum
      while (bytesRead < 12) 
	{
	  if( mySerial.available() > 0) 
	    { 
	      val = mySerial.read();
	      // Append the first 10 bytes (0 to 9) to the raw tag value
    //Serial.println(val);
	      if (bytesRead < 10)
		{
		  tagValue[bytesRead] = val;
		}

	      // Check for ETX | STX | CR | LF
	      if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) 
		{  
		  // Stop Reading - There is an Error.
//                break;
		  return 0;
		}
 
	      // Convert Hex Tag ID
	      if ((val >= '0') && (val <= '9')) 
		{
		  val = val - '0';
		} 
	      else if ((val >= 'A') && (val <= 'F')) 
		{
		  val = 10 + val - 'A';
		}

/* when commented out constant "rejects"
	      else if ((val >= 'a') && (val <= 'f')) 
		{
		  val = 10 + val - 'a';
		}
              else  // Error, garbage value
		{
		  return 0;
		}
*/
	      // Every two hex-digits, add byte to code:
	      if (bytesRead & 1 == 1) 
		{
		  // make some space for this hex-digit by
		  // shifting the previous hex-digit with 4 bits to the left:
		  tagBytes[bytesRead >> 1] = (val | (tempByte << 4));
 
		  if (bytesRead >> 1 != 5) 
		    {                
		      // If we're at the checksum byte,
		      // Calculate the checksum... (XOR) - Exclusive OR
		      checksum ^= tagBytes[bytesRead >> 1];      
		    };   
		} 
	      else 
		{
		  tempByte = val;
  	        };
	      // ready to read next digit
	      bytesRead++;                               
	    }
//          else  // Check to make sure that the RFID reader hasn't just hung...
//	    {
//              if (millis() - start > MAX_RFID_READ_TIME) return 0;  // RFID timeout
//	    } 
	}
 
      // Print
      if (bytesRead == 12) 
	{                          
	  tagValue[10] = '\0'; // Null-terminate the string
//	  LCD.print("5-byte code: ");
	  for (byte i=0; i<5; i++) 
	    {
	      if (tagBytes[i] < 16) Serial.print("0");
//	      LCD.print(tagBytes[i], HEX);
//	      LCD.print(" ");
	    }
//	  LCD.println();
 
//	  LCD.print("Checksum: ");
//	  LCD.print(tagBytes[5], HEX);
//	  LCD.print(tagBytes[5] == checksum ? " -- passed." : " -- error.");
          if (tagBytes[5] == checksum) return 1;  // We got a good tag, so return success!
	}
    }

  return 0;  // Failed, not enough bytes read or serial not available or serial garbage
}



  void loop () 
  {
    byte i = 0;
    char tagValue[11];


    if (checkRFID(tagValue))
      {
	// Show the raw tag value
	// Serial.print("VALUE: ");
	// Serial.println(tagValue);
	//*******************************************************************
	// Search the tag database for this particular tag
	int tagId = findTag( tagValue );

	// Only fire the strike plate if this tag was found in the database
	if( tagId > 0 )
	  {
	    clearLCD();
	    lcdPosition(0,0);
	    LCD.print("Authorized tag");
	    lcdPosition(1,0);
	    LCD.print(tagId);
	    lcdPosition(0,0);
	    delay(500);
	    LCD.print("unlocking for ");
	    lcdPosition(1,0);
	    LCD.print(tagName[tagId - 1]); // Get the name for this tag from the database
	    playToneAccept1();
	    runUnlockedState(); // Fire the strike plate to open the lock
	    lcdPosition(0,0);
	    clearLCD();
// clearLCD();
            LCD.print("Please RFID card");  
          }  
	  else {
	    clearLCD();
	    lcdPosition(0,0);
	    LCD.print("NOT authorized");
	    playToneReject1 ();
	    clearLCD();  
            LCD.print("Try again");
	  }
	//      Serial.println(); // Blank separator line in output
	//*******************************************************************
      }
//delay(100);
  }


//----------------------------------------------------------------------------------
// Switch unit ON and OFF each time an authorized card is held to the reader
void runUnlockedState () 
{
    char tagValue[11];

    if (1)
//  if ((wasOn == false) && (wasOff == true))
    {
      digitalWrite  (ledPin, HIGH);
      wasOff = false;
      wasOn = true; 
      AUTHORIZED_MODE_1 = 1;

      while (AUTHORIZED_MODE_1 == 1)    // AS LONG AS THE MODE IS TRUE
	{
	  byte cmd;
	  unsigned long int nextPrintTime=0;
	  SPDR = WaitingForCmdMsgId;  
	  while (1)
	    {  
   /// check whether RFID tag is present ------------------------------------------------------
              if (checkRFID(tagValue))
		{
  	          int tagId = findTag( tagValue );
	          if( tagId > 0 )
		    {
		      AUTHORIZED_MODE_1 = 0; 
		      cmd = NothingMsgId;
		 break;  // Kick out of the while loop so we can drop back into "parked" mode.
                  }
  // end RFID check --------------------------------------------------------------------------             
		}

	      if (SPSR & (1<<SPIF)) { cmd = SPDR; break; }    
  
	      // Note, a pot should provide a reasonable current so a delay is not necessary.
	      // But for high impedence sensors it is necessary to sample, delay, sample again, and only take the 2nd result.
	      int throttleReading = analogRead(throttlePin);                 // read throttle 
	      if (SPSR & (1<<SPIF)) { cmd = SPDR; break; }
/*
  /// check whether RFID tag is present ------------------------------------------------------  // uncomment for better sensitivity
              if (checkRFID(tagValue))
		{
  	          int tagId = findTag( tagValue );
	          if( tagId > 0 )
		    {
		      AUTHORIZED_MODE_1 = 0; 
		      cmd = NothingMsgId;
		 break;  // Kick out of the while loop so we can drop back into "parked" mode.
                  }
		}
  // end RFID check --------------------------------------------------------------------------             
*/

	      int steeringReading = analogRead(steeringPin);                 // read steering angle
	      if (SPSR & (1<<SPIF)) { cmd = SPDR; break; }

  /// check whether RFID tag is present ------------------------------------------------------
              if (checkRFID(tagValue))
		{
   	          int tagId = findTag( tagValue );
	          if( tagId > 0 )
		    {
 		      AUTHORIZED_MODE_1 = 0; 
		      cmd = NothingMsgId;
		 break;  // Kick out of the while loop so we can drop back into "parked" mode.
                  }
                }
  // end RFID check --------------------------------------------------------------------------             

	      calcMotorValues(throttleReading, steeringReading);
	      if (SPSR & (1<<SPIF)) { cmd = SPDR; break; }
/*
  /// check whether RFID tag is present ------------------------------------------------------ // uncomment for better sensitivity
              if (checkRFID(tagValue))
		{
  	          int tagId = findTag( tagValue );
	          if( tagId > 0 )
		    {
        	      AUTHORIZED_MODE_1 = 0; 
		      cmd = NothingMsgId;
		 break;  // Kick out of the while loop so we can drop back into "parked" mode.
                  }
		}
  // end RFID check --------------------------------------------------------------------------             
*/


	      readBatteryVoltage();
//		  dumpReadings(throttleReading,steeringReading, voltage_D);

	      // Note USB prints take a lot of time, so this code should be commented out
	      // when you are confident the sensors are working properly.
	      if (millis() > nextPrintTime) 
		{
		  dumpReadings(throttleReading,steeringReading, voltage_D);
		  nextPrintTime = millis()+1000; 													 // changed from 1000 to 100

		}


	    }

	  // The SPI Master communicated with us so let's figure out what it wants
	  // and respond with the proper data.
	  switch(cmd)
	    {
	    case NothingMsgId:
	      break;
	    case WaitingMsgId:  // Yikes this is the master telling us he is waiting for data from us.  Should never happen at the top level
	      // Receiving the NothingMsgId should kick it out.
	      DbgPrint("ERROR: Received incorrect WaitingMsgId at command context");
	      break;

	    case HelloMsgId:
	      {
		DbgPrint("INFO: Sending hello response");
		Stransfer(OkMsgResponse);
	      } break;

	    case ThrottleMsgId:      // I want to send instead throttle data the calculated result from throttle and steering as Motor_RValue and Motor_LValue
	      {
		byte tmp;

		DbgPrint("INFO: Sending Motor data");
		if ((tmp = sendThrottle())==0)
		  {
		    DbgPrint("Motor_L ");
		    DbgPrint(Motor_LValue);
		    DbgPrint("Motor_R ");
		    DbgPrint(Motor_RValue);
  
		    DbgPrint("INFO: Motor data sent");
		  }
  
		else
		  {
		    DbgPrint("ERROR: Motor data send aborted");
		    DbgPrint(tmp);
		  }
	      } break;
	    default:
	      DbgPrint("Bad command");
	      DbgPrint(cmd);
	      // If we get a bad command, it may be an off-by-one bit issue.  So reset the SPI to reset the hardware bit counter
	      SPCR = 0;  // RESET SPI
	      SPCR |= _BV(SPE);

	      Stransfer(AbortMsgResponse);
	      break;
	    }

	  if (rpmcountL >= 4)  
	    { 
	      //Update RPM every 4 counts, increase this for better RPM resolution,
	      //decrease for faster update
	      rpmL = 5*1000/(millis() - timeoldL)*rpmcountL;  
	      // encoder disk has 6 windows; change: 12 events per rotation
	      // 60 = 1 event per rotation, 10 = 6 events, 5 = 12 events  
	      timeoldL = millis();
	      rpmcountL = 0;
	      //Serial.println(rpmL,DEC);
	    }
	  if (rpmcountR >= 4)
	    { 
	      rpmR = 5*1000/(millis() - timeoldR)*rpmcountR;
	      timeoldR = millis();
	      rpmcountR = 0;
	      //Serial.println(rpmR,DEC);
	    }
	  // calculate average
	  averageRPM = (rpmL + rpmR) /2;
	  //Serial.println(averageRPM,DEC);
	  // calculate speed
	  // the wheel diameter is 406.4 mm
	  // circumfence = Pi * diameter = 1,305 m
	  vehicleSpeed = averageRPM * 60 * 1.305;
	  //Serial.println(vehicleSpeed, DEC);	
	}

      ///////////////////////////////////////////////////////////////////////////////////
    }
//  else
    if (1)
    {
      digitalWrite (ledPin, LOW);
      playTone_3_beep();
      wasOff = true;
      wasOn = false;
      AUTHORIZED_MODE_1 = 0;    
    }
}
//* Fire the relay to activate the strike plate for the configured                                             
//* number of seconds.
/*
void unlock() {
  digitalWrite(ledPin, HIGH);
  digitalWrite(strikePlate, HIGH);
  delay(unlockSeconds * 1000);
  digitalWrite(strikePlate, LOW);
  digitalWrite(ledPin, LOW);
}
*/
//**********************************************************************************
/**
* Search for a specific tag in the database
*/
int findTag( char tagValue[10] ) {
  for (int thisCard = 0; thisCard < numberOfTags; thisCard++) {
    // Check if the tag value matches this row in the tag database
    if(strcmp(tagValue, allowedTags[thisCard]) == 0)
    {
      // The row in the database starts at 0, so add 1 to the result so
      // that the card ID starts from 1 instead (0 represents "no match")
      return(thisCard + 1);
    }
  }
  // If we don't find the tag return a tag ID of 0 to show there was no match
  return(0);
}
//**********************************************************************************
/////////////////////////////////////////////////////////////////////////////////////////////
void playToneAccept1 () {
     for (int i = 0; i < 12; i++)
     {
/*
       int j_noteDuration = 1000/j_noteDurations[i];
       tone(speaker_pin, j_melody[i],j_noteDuration);
       int j_pauseBetweenNotes = j_noteDuration * 1.30;
       delay(j_pauseBetweenNotes);
       noTone(speaker_pin);
*/
       int d_noteDuration = 1000/d_noteDurations[i];
       tone(speaker_pin, d_melody[i],d_noteDuration);
       int d_pauseBetweenNotes = d_noteDuration * 1.30;
       delay(d_pauseBetweenNotes);
       noTone(speaker_pin);

     }
   }
//-------------------------------------------------------------------------------------------
void playToneAccept3 () {
     for (int i = 0; i < 16; i++)
     {
       int R2D2_melody_noteDuration = 1000/R2D2_melody_noteDurations[i];
       tone(speaker_pin, R2D2_melody[i],R2D2_melody_noteDuration);
       int R2D2_melody_pauseBetweenNotes = R2D2_melody_noteDuration * 1.30;
       delay(R2D2_melody_pauseBetweenNotes);
       noTone(speaker_pin);

     }
   }       
//-------------------------------------------------------------------------------------------
void playToneAccept4 () {
     for (int i = 0; i < 15; i++)
     {
       int R2D2_close_noteDuration = 1000/R2D2_close_noteDurations[i];
       tone(speaker_pin, R2D2_close[i],R2D2_close_noteDuration);
       int R2D2_close_pauseBetweenNotes = R2D2_close_noteDuration * 1.30;
       delay(R2D2_close_pauseBetweenNotes);
       noTone(speaker_pin);

     }
   }       
//-------------------------------------------------------------------------------------------
void playTone_2_beep () {
     for (int i = 0; i < 2; i++)
     {
       int beep_beep_noteDuration = 1000/beep_beep_noteDurations[i];
       tone(speaker_pin, beep_beep_melody[i],beep_beep_noteDuration);
       int beep_beep_pauseBetweenNotes = beep_beep_noteDuration * 1.30;
       delay(beep_beep_pauseBetweenNotes);
       noTone(speaker_pin);
     }
   }         
//-------------------------------------------------------------------------------------------
void playTone_3_beep () {  //already good
     for (int i = 0; i < 3; i++)
     {
       int beep_beep_beep_noteDuration = 1000/beep_beep_beep_noteDurations[i];
       tone(speaker_pin, beep_beep_beep_melody[i],beep_beep_beep_noteDuration);
       int beep_beep_beep_pauseBetweenNotes = beep_beep_beep_noteDuration * 1.30;
       delay(beep_beep_beep_pauseBetweenNotes);
       noTone(speaker_pin);
     }
   }         
//-------------------------------------------------------------------------------------------
void playToneReject1 () {
     for (int i = 0; i < 6; i++)
     {
       int fail_noteDuration = 1000/fail_noteDurations[i];
       tone(speaker_pin, fail_melody[i],fail_noteDuration);
       int fail_pauseBetweenNotes = fail_noteDuration * 1.30;
       delay(fail_pauseBetweenNotes);
       noTone(speaker_pin);
     }
   }  
/////////////////////////////////////////////////////////////////////////////////////////////

void display_voltage() {
  unsigned long int potvalue2 = analogRead (potpin);  
  // potvalue ranges from 0 to 1023, representing a voltage range from 0 to 29
  // Transform it into a fixed point number denoting hundredths of a volt.  i.e 100 = 1v
  voltage_D = (potvalue2 * 29) / 10.23;  

  V1 = (voltage_D/28);    // ought to be 29 ??
      int vom1 = (V1 / 12);  //divide reading for solid blocks // 6 = 16 blocks, 12 = 8 blocks
      int rem1 =  (V1 % 12 ); //get modulo for remainder
      backlightOn;
        lcdPosition(1,0);       //move cursor to first meter
        
         for (int i=0; i < vom1; i++) {
           LCD.print((char)255);  //print solid blocks 
         }
         LCD.print("-"); //print the remainder which is meter segments within character
          for (int i=(vom1 + 1); i<8; i ++)  // 16: fill up whole line, 8 fill half line 
          {
          LCD.print("-"); //blank spaces after so there is no leftover mess after fast moving meter
          }  
          lcdPosition (1, 12);
          LCD.print(vom1*4);    // position for the calculated remaining range // calculation still wrong
          lcdPosition(1, 14);
          LCD.print("Km");
//        clearLCD();
        lcdPosition (0,0);
//  ledvalue = map (potvalue, 0, 1023, 0, 255);    // later no LED
//  analogWrite (ledpin, ledvalue);
  if (voltage_D < 2500) {    //equivalent to 25.00V
    lowVoltage = true;
  }
  else {
    lowVoltage = false;
  }
}

//**************************************************************
void speedreadingL() {
 rpmcountL++;
}
void speedreadingR() {
  rpmcountR++;
}
//**************************************************************


