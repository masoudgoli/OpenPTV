/*
'Tinkering series'
02: basic balancing, a bit rough in soft deviation range; build up when left alone
03: introduced 3rd controller curve for angles > GABCONST_H (15 degrees)
04: introduced a hard 'erect sequence' in 3r controller and got the safety switch working within.
    BUT: it always does it 2 times !???
05: added processing read out - found that my switch-off turned the controller from PD to P only   
    starting to get the PID values straight again
    http://forum.arduino.cc/index.php?PHPSESSID=c6f9ks6kq5kcvtoe2jjm130pm3&topic=8871.0
    
06: eliminated the Kpid settings in the main loop and try to get it balanced by PID settings in MainBoard.cpp

07: adjusted acc values according to voltage by 1.4
    working with http://www.kerrywong.com/2012/03/21/a-self-balancing-robot-iii/
    
    Have a look at http://www.starlino.com/imu_guide.html  for the acc sensor 

08: changed GFAngle into GFORCE (this always confused me)
   
    increase MAXSPEED from 100 to 122
    
    
 
 OpenPTV Mainboard Firmare 1.0b for Arduino Mega 2560 Licenced unter CC by-nc not yet for public circulation.
 Not for commercial Use.
 USE AT OWN RISK!
 Kalman-Filter and PID-Controller from www.x-firm.com
 www.OpenPTV.org

 /////////////////////////////////////////////////////////////////////////////////////////////////

    MB.setLED(false, false, false, false, false, true);
              red  - red  - red  - red  - yel  - green
 */

/*hier werden die headerfiles der librarys importiert die wir im hauptprogramm verwenden werden.
 import header files of libraries used in main program
 */
#include "MainBoard.h"
#include "math.h"
#include "MotorController.h"
#include "SoftwareSerial.h"
#include "b2b.h"
#include "misc.h"
#include <EEPROM.h>
#include <SPI.h>

// Testkonstanten zum Bremsen bei zu hoher Drehzahl der Motore
// test constants to brake motors when rpms are too high 
#define GABCONST 3      //was 3
#define GABCONST_H 10
#define SPEEDCONST 40
#define MAXSPEED 100    // was 100  then 142 increased with calculated values 122 and 142 for real values

//#define DbgPrint Serial.print  // little tool to erase all Serial.print after debugging; DbgPrint: normal messages
#define DbgPrint(x)                // after debugging just use (x) and all the Serial.prints are gone 

//#define DbgPrint2 Serial.print  // little tool to erase all Serial.print after debugging; DbgPrint2: messages for Processing Window "BalancingBotControl" 
#define DbgPrint2(x)                // after debugging just use (x) and all the Serial.prints are gone 


// Variablen die im Hauptprogramm verwendet werden.
// variables used in main program
int del = 10;
int time = 0;

int motorra = 0;
int motorla = 0;

int motorr = 0;
int motorl = 0;

int motorLeft, motorRight;
int voltage_D;

int PID = 0;
int gab = 0;
int targetposition = 0;


// Deklaration der Motorcontroller und Mainboard Objekte
// declaration of motor controllers and mainboard objects
MotorController motorR;
MotorController motorL;
MainBoard MB;
MotorController MC;


// Intialisierung der Kommunikation zwischen HP und DP 
// initialization of communication between main board and display board
// SoftwareSerial ser(DB1, DB2);
// we have changed over to SPI

Board2Board b2b;


int drive = 0;   // I don't know what this variable is used for 
boolean fs = 0;  // failsafebit

float angle = 0.0;                      // Winkelwert                       // horizontal vehicle angle 
//int steering = 0;                       // Lenkungswert                     // steering value
int looptime = 20;                      // Anfangsschleifenzeit...          // initial loop time
float gyrorate = 0.0;                   // Winkel aus Kreiselsensor         // gyro value
float gfangle = 0.0;                    // Wert aus Beschleunigungssensor   // acc sensor value  //wo wird diese Variable benötigt??
float calculatedvoltage = 0.0;          // Spannung der Batterie            // battery voltage



void setup() 
{
  // Startupfunktionen
  // startup functions

  Serial.begin(9600);
//  Serial.println("testPersonal Range Extender V1.0");
//  Serial.println();
  //  delay(500);  
  MB.startUp();
  motorR.startUp(23, 2, 3, 27, 31, 33, 9); //(MotorController.ccp:  pinDir, pinPwmLow, pinPwmHigh, pinReset, pinFF1, pinFF2, pinCS)
  motorL.startUp(25, 11, 5, 29, 35, 37, 8);  // in Excel list PwmLow is defined as 4 - but soldered at 11 on reserve plug !?
  // IMPORTANT:  I would like to change L and R so I do not need to cross the Motor Cables - revisit this topic later (!)
//  Serial.println("Motors Initialized");  

  //  delay(500); 
  //  pinMode(DB4, OUTPUT);   // we don't use this communication DB4 is pin D9
  //  digitalWrite(DB4, LOW);

  //  Intialisierung der Kommunitaion der Platinen
  //  initialization of communication between boards
  //  pinMode(DB1, OUTPUT);
  //  pinMode(DB2, OUTPUT);
  //  ser.begin(19200); 

  // SPI communication between MainBoard (Master) and DisplayBoard (Slave)
  // optional a FollowBoard as second slave
//  Serial.println("Initializing board 2 board communications");
  b2b.begin();
  if (1)
  {
    byte inited =0; 
    byte count = 0;
    while ((inited!=3)&&(count<3)) // was count<5  // check three times whether the slaves are responding
    {
      delay(500);
//      if (!b2b.hello(Displayboard))  Serial.println("Display board is not responding...");
//      else inited |= 2;
//      if (!b2b.hello(Followboard)) Serial.println("Follow board is not responding..."); 
//      else inited |= 1;
      count++;
    }
  }
//   digitalWrite(Displayboard_SS, LOW);    // start communication to Displayboard - this is also in the b2b.getThrottles  (!)   
  /*  
   // use this to read combined throttle and steering values (as motorL and motorR outputs) and Voltage  
   Serial.println("Throttle test");
   while(1)
   {
   int motorLeft, motorRight;
   int voltage_D;
   delay(200);  // was 1000
   if (b2b.getThrottles(&motorLeft, &motorRight, &voltage_D))
   {
   ps("Display throttle values received %d %d. voltage_D %d.\n", motorLeft,motorRight,voltage_D);
   Serial.print("left  ");
   Serial.println(motorLeft);
   Serial.print("right  ");
   Serial.println(motorRight);        
   Serial.print("Volt  ");
   Serial.println(voltage_D /100);
   }
   else
   {
   ps("getThrottle failed\n");
   }
  /*
   if (b2b.getVoltage(&voltage_D))
   {
   ps("Follower voltage value received %d\n", voltage_D);
   ps("\n");
   }
   else
   {
   ps("getVoltage failed\n");
   }
   */
  /*
    }
   */

  //  pinMode(DB4, OUTPUT); //A0  // erased because we don't use Display port
  //  pinMode(DB5, INPUT); //D6

//  Serial.println("Waiting for RFID key");

  // Wegfahrsperre auslesen. Setzt zu Testzwecken nur die LEDs.
  // read RFID keys. For testing purposes only LEDs are set.
  while(!MB.getRFID())    //RFID.ccp
  {
    MB.setLED(true, true, true, true, false, false);

    delay(100);

    MB.setLED(false, false, false, false, false, false);

    delay(100);

  }
  // delay (2000);

  // Endlosschleife falls beim Einschalten schon ein Taster gedrückt wird und für den fall falls ein oder zwei Taster defekt sind.
  // endless loop when a button is already pressed when switched on and for the case that on or two buttons are defect. 
//  Serial.println ("are switches pressed?");
  /*
  Serial.println("left  right");
   Serial.print(trkl);
   Serial.print("\t");
   Serial.println(trkr);
   */

/*
  // this section prevents starting of the PREx if during initialization a safety switch is pressed 
  // un-comment it later - this is an important feature.
  if(MB.getTRK() != 0) //FAILSAFE   (MainBoard.ccp)
  {

    while(true)      // was true
    {
      Serial.println("Safety Switch is pressed - please restart!");
      MB.setLED(true, true, true, true, false, false);

      delay(1000);

      MB.setLED(false, false, false, false, false, false);

      delay(1000);

    }
  }
*/  
  // delay (2000);


  MB.updateSensors();    
//  Serial.println("updating Sensors");

  /*  This procedure is taken out for now
   // Endlosschleife falls beim Einschalten die Lenkung schon betätigt ist und für den Fall, dass das Potentiometer defekt ist.
   // endless loop when the steering is already out of zero position when switched on and for the case that the steering pot is defect. 
   // in this case let's check the steering/throttle values.
   
   if(MB.getSteering() > 5 || MB.getSteering() < -5) //FAILSAFE steering
   {
   while(true)
   {
   MB.setLED(true, true, true, true, false, false);
   
   delay(200);
   
   MB.setLED(false, false, false, false, false, false);
   
   delay(200);
   
   }
   }
   */

  // we want to read the sensors from the display board
  /*
     if (motorLeft > 10 || motorRight > 10)
   {
   Serial.println("Steering or Throttle already applied");
   }
   */

  /* Wenn beim Einschalten der Jumper gesetzt ist, geht das Fahrzeug in den Kalibrierungsmodus.
   Man hat Zeit das Fahrzeug auszurichten und wenn dann ein Taster gedrückt wird, werden die Kalibrierungswerte eingelesen.  // WELCHER TASTER ????
   Wenn die LEDs erloeschen hat man 3 Sekunden um den Taster loszulassen...
   Bei der Kalibrierung muss das Fahrzeug in der gewünschten "Nullposition" stehen und darf nicht bewegt werden!
   
   If calibration jumper on main board is plugged when switched on, the vehicle goes into calibration mode.
   There is time to adjust the vehicle. After adjustment when a button is pressed, the calibration values are recorded.
   When the LEDs turn off there are 3 seconds to release the button ...
   During recording of calibration values the vehicle has to be in the desired "zero position" and cannot be moved!   
   */



  if(MB.getJumper())    // mainboard.cpp; true = jumper present / false no jumper
  {

    Serial.println("Jumper is present");
    Serial.println("Calibration press a switch and wait that the LEDs turn off");
    while(STATE_TRKN == MB.getTRK())
    {
      MB.setLED(true, true, true, true, false, false);
      delay(500);
      MB.setLED(false, false, false, false, false, false);
      delay(500);
    }

    MB.setLED(false, false, false, false, false, false);
    MB.calibrateSensors();
    Serial.println("Calibration done, now 3 secs to switch off");
    delay(3000);
  }
//  Serial.println("starting MAIN Loop");
//  Serial.println("angle - GFORCE -GyroRate - PID -  raw LEFT   - raw RIGHT  - Motor LEFT - Motor Right");

}

/* we got this covered in b2b.getThrottles routine
 void getThrottleValues(void)
 {
 // Talk to Nano via SPI to get the throttle values.
 }
 void getVoltageValues(void)
 {
 }
 */

// Hauptschleife
// main loop
void loop() 
{

  //Serial.println("starting MAIN Loop");
  MB.updateSensors();                                                                    // Sensorwerte updaten                // update sensor data

// this is the section where the MainBoard requests Throttle and Steering from the Displayboard
//   digitalWrite(Displayboard_SS, LOW);    // start communication to Displayboard   (!)   
//   int motorLeft, motorRight;
//   int voltage_D;
	digitalWrite(Displayboard_SS, LOW);    // start communication to Displayboard
   b2b.getThrottles(&motorLeft, &motorRight, &voltage_D);      // this location is much better. now sensor readings without 200ms delay possible (!)
   if ((motorLeft == 512) &&  (motorRight == 44)) {      // if display board doesn't respond and sometimes @ brake the values are L = 512 and R = 44
   motorLeft = 0;                                      // set in this case values to 0 not that wrong readings accelerate the vehicle unintentionally
   motorRight = 0;
   }
   delay(46);  
   // works from 158ms w/SoftwareSerial on LCD; 
   // 156ms w/HardwareSerial on RFID; 
   // 118 w/HardwareSerial on LCD;  
   // 111ms with digitalWrite in this section; 
   // 56ms with reducing the LCDdelay from 10 to 5
   // 46ms with making the display leaner 
   // next step looking at the voltage  - - try to delay less !
   digitalWrite(Displayboard_SS, HIGH);   // stop communication to Displayboard
/*
   Serial.print(motorLeft);
   Serial.print("\t");
   Serial.println(motorRight);
*/
  //
  //  b2b.getThrottles(&motorLeft, &motorRight, &voltage_D);
  /*
          ps("Display throttle values received %d %d. voltage_D %d.\n", motorLeft,motorRight,voltage_D);
   Serial.print("left  ");
   Serial.println(motorLeft);
   Serial.print("right  ");
   Serial.println(motorRight);        
   Serial.print("Volt  ");
   Serial.println(voltage_D /100);
   */
	  
 

  //  steering = MB.getSteering();                                                           // Potentiometerwert ausgeben         // read steering pot
//  angle = MB.getKalman((float)MB.getGFORCE(), (float)MB.getGyroRate(), looptime);       // had to change order of GFORCE and GYRO as well !!
  angle = MB.getKalman((float)MB.getGyroRate(), (float)MB.getGFORCE(), looptime);       // Winkel mit Kalmanfilter berechnen  // calculate angle with Kalman filter

  // Setzen der Werte für den PID Regler !!!EXPERIMENTAL!!!
  // setting of values for the PID controller   !!! EXPERIMENTAL !!!  

  gab = angle;
  abs (gab);    // changed this for absolute value
//  if(gab < 0) gab = gab * -1;

  // angles in balancing mode -0.6 ... 2.49 degrees

//  if(gab < GABCONST)// || ((motorr > SPEEDCONST || motorr < SPEEDCONST * -1) || (motorl > SPEEDCONST || motorl < SPEEDCONST * -1)))
  // if angle between 0 .. 3

  // changed . . . 
  {  

    // proportional gain   float Kp=7.5;
    // deferential gain    float Kd=0.5;
    // integral gain       float Ki=12.5;
    /* aus Segwii:
     float K = 1.9 ;      // wheels 100mm
     float Kp = 14.01;    //16.37                       
     float Ki = 1.0;                        
     float Kd = -13.30; //-7.23      
     http://3digi.wikidot.com/allgemeines-zu-pid-reglern
     */

/* in Mainboard.cpp defined:
  //PIDvariables
  K =  1.0;  // 1.0 //1.4
  Kp = 1.0;  // 1   //6                  
  Ki = 0.0;  // 0   // nicht anrühren       
  Kd = 7.0;  // 7   //11
*/
  /*
   For manual tuning, set all three constants to zero, then turn up Kp until oscillation occurs.
   Then turn up Kd until oscillation disappears. Adjust Kd until the system is critically damped, i.e. there’s no overshoot.
   Then increase Ki until the steady-state error goes to zero in a reasonable time.
   */

    // small derivation
//    MB.setK((1.2 + (gab * 0.35)));  // MB.setK((0.8 + (gab * 0.02)));
//    MB.setKp(1);            // Not in org sketch     //5.0
//    MB.setKi(0);            // Not in org sketch //0.05
//    MB.setKd(gab * 1.8);
//    MB.setKd((3.0 - (gab * 4.0)));            // MB.setKd(gab * 1.8);
    MB.setLED(false, false, false, false, true, false);
  }
//  if (gab >= GABCONST && gab <= GABCONST_H) 
  
/*
// try first only one
    else
  {
    // if angle is between 4 and 15
    // larger derivation
    MB.setK((1.1  + (gab * 0.1)));  // MB.setK((1.0  + (gab * 0.05)));
//    MB.setKp(1);            // Not in org sketch 
//    MB.setKi(0.1);            // Not in org sketch 
    MB.setKd(gab * 2.5 );            // MB.setKd(gab * 3.0 );  //5
    MB.setLED(false, false, false, false, false, true);
  }
  
*/  
//  if ((-angle > GABCONST_H)  &&  (0 < MB.getTRK()))  // if motor settings are used    // if foot switch is pressed

/*
  if (-angle > GABCONST_H)  

  // when backwards
  {
/* just for debugging
    Serial.println ("gab    angle");
Serial.print (gab);
Serial.print("\t");
Serial.println(angle);
*/

    // if angle larger than 15
    // larger derivation
/*
 motorL.setSpeed(-55);
 motorR.setSpeed(-55);
 delay(100);
 motorL.setSpeed(-255);
 motorR.setSpeed(-255);
delay(500); 
 motorL.setSpeed(-35);
 motorR.setSpeed(-35);
*/
/*
    // when backwards
    MB.setK((2.0  + (gab * 0.10)));  // MB.setK((1.0  + (gab * 0.05)));
//    MB.setKp(1);            // Not in org sketch 
//    MB.setKi(0.1);            // Not in org sketch 
    MB.setKd(gab * 4.0 );            // MB.setKd(gab * 3.0 );  //.70
    MB.setLED(true, true, false, false, false, false);
  }

//  if ((angle > GABCONST_H) &&  (0 < MB.getTRK()))  // wenn Trittkontakttaster gedrückt    // if foot switch is pressed
  if (angle > GABCONST_H) 
  {
    // if angle larger than 15
    // larger derivation
/*
    Serial.println ("gab    angle");
    Serial.print (gab);
    Serial.print("\t");
    Serial.println(angle);
*/
/*
 motorL.setSpeed(55);
 motorR.setSpeed(55);
 delay(100);
 motorL.setSpeed(255);
 motorR.setSpeed(255);
delay(300); 
 motorL.setSpeed(35);
 motorR.setSpeed(35);
*/ 
/*
    MB.setK((2.0  + (gab * 0.10)));  // MB.setK((1.0  + (gab * 0.05)));
//    MB.setKp(0.1);            // Not in org sketch 
//    MB.setKi(0.1);            // Not in org sketch 
    MB.setKd(gab * 4.0 );            // MB.setKd(gab * 3.0 );  //.70
    MB.setLED(true, true, false, false, false, false);
  }
*/


  
  if ((angle > -1.5) && (angle < 1.5))   // balanced
    MB.setLED(false, false, true, true, false, false);  

  /* // backward
   MB.setK((0.6 + (gab * 0.02)));      // K is multiplied to the sum of Kp, Ki, and Kd
   MB.setKd(gab * 3.8);            // 3.8
   MB.setKp(gab * 5.0);            // Not in org sketch     //5.0
   MB.setKi(gab * -5.5);            // Not in org sketch     //0.05
   */

  /*  // forward
   MB.setK((0.6  + (gab * 0.02)));  //original values
   MB.setKd(gab * 3.8 );            // 3.8
   MB.setKp(gab * 5.0 );            // Not in org sketch 
   MB.setKi(gab * -5.5);             // Not in org sketch 
   */


  if(motorr < MAXSPEED * -1) targetposition = 0;    // was  2
  else targetposition = 0;


  // Wert durch den PID Regler laufen lassen.
  // process value in PID controller 

  PID =  MB.updatePid(targetposition, angle);
  /*
  Serial.print("PID");
   Serial.print("\t");
   Serial.println(PID); 
   */
  /*
  // zu Testzwecken wird die Spannung auf den LEDs ausgegeben...
   // for testing purposes the voltage is shown with the LEDs ....
   calculatedvoltage = MB.calculateVoltage(looptime);
   Serial.print ("calc. Voltage ");
   Serial.println(calculatedvoltage);
   Serial.println();
   if(calculatedvoltage > 26) MB.setLED(false, false, false, false, false, true);
   else if(calculatedvoltage <= 22) MB.setLED(false, false, false, true, false, false);
   else if(calculatedvoltage <= 23) MB.setLED(false, false, true, false, false, false);
   else if(calculatedvoltage <= 24) MB.setLED(false, true, false, false, false, false);
   else if(calculatedvoltage <= 26) MB.setLED(true, false, false, false, false, false);
   */

  // zu testzwecken kann der motorwert auf den LEDs ausgegeben werden.
  // for testing purposes motor value can be shown with the LEDs.
  /*  
   if(motorr < -200) MB.setLED(false, false, false, false, false, true);
   else if(motorr >= -30) MB.setLED(false, false, false, true, false, false);
   else if(motorr >= -80) MB.setLED(false, false, true, false, false, false);
   else if(motorr >= -120) MB.setLED(false, true, false, false, false, false);
   else if(motorr >= -170) MB.setLED(true, false, false, false, false, false);
   */
  /* debug
   Serial.println("angle - GFORCE -GyroRate - PID");
   Serial.print(angle);
   Serial.print("     ");
   Serial.print(MB.getGFORCE());
   Serial.print("     ");
   Serial.print(MB.getGyroRate());
   Serial.print("     ");
   Serial.print(PID);
   Serial.println("     ");
   // end debug
   */
  /*
   // Senden der Werte an die DP -  funktioniert noch nicht richtig...
   // send values to diplay board - does not work properly right now ...
   transmitData(motorL.getCurrent(), motorR.getCurrent(), MB.getVoltage(), motorR.getFail(), motorr, looptime);
   */



  if(0 < MB.getTRK())  // wenn Trittkontakttaster gedrückt    // if foot switch is pressed
  {

    //     Serial.println("get Switches");   

    if(angle > -1 || angle < 1) fs = true;  // Failsafe switches work
    // wenn der winkelwert ca. bei der nullposition ist wird fs (FAILSAFE) auf 1 gesetzt 
    // das verhindert, dass das Fahrzeug gleich voll beschleunigt wenn man bei einer großen neigung aufsteigt.
    // when angle value is approx. at zero position fs (failsafe) is set to 1
    // this prevents the vehicle to accelerate unintenionally when rider climbs on while the angle is too far from zero position
    //    Serial.println (fs);
    if(fs) // wenn fs gesetzt ist werden die Werte an die Motorcontroller gegeben
      // when fs is set values are handed to the motor controller
    {

      motorr = motorr + PID;  // der neue Wert ist der Alte plus den PID Wert   // vielleicht ist das der Wert nur um zu balanzieren . . 
      motorl = motorl + PID;  // new value is old value plus PID value          // perhaps this is the value zu for balancing
      /*
  Serial.println("raw motor values");
       Serial.print("left  ");
       Serial.println(motorl);
       //        Serial.println(motorLeft);
       Serial.print("right  ");
       Serial.println(motorr);   
       */


      motorR.setBrakeType(DRIVE_BRAKE);  // hier wird die Bremseinstellung gesetzt
      motorL.setBrakeType(DRIVE_BRAKE);  // setting of brake mode

	  
	  
/*      
//  Serial.println("raw throttle values");
//       Serial.print("left  ");
        Serial.print(motorLeft);
		Serial.print("\t");
	   //       Serial.println("right  ");
       Serial.println(motorRight);       
*/      
      //  digitalWrite(Displayboard_SS, HIGH);  // shall I switch it off every time the value was transmitted to allow later on to switch to Followboard_SS ?


      //Serial.println("adding Steering to Motors");
      //      motorR.setSpeed(motorr + steering);  // + GasGriff-Wert  // hier werden die Sollwerte mit der Lenkung addiert und endgültig an die Motorcontroller gegeben
      //      motorL.setSpeed(motorl - steering);  // + throttle-Value // target values are added to steering values and then finally handed to motor controllers

      // test: not adding any values   
     // motorR.setSpeed((motorr * 1 ) + (motorRight));  // increase torque calculated value = 0.83  
     // motorL.setSpeed((motorl * 1 ) + (motorLeft));  
	 // motorR.setSpeed((motorr * 1 ) );  // first step: do not add values from steering
     // motorL.setSpeed((motorl * 1 ) );  
     
     motorR.setSpeed(motorr + motorRight);  //  hier werden die Sollwerte mit der Lenkung und Gas addiert und endgültig an die Motorcontroller gegeben
/*
	 int finalR = (motorr + (motorRight /7));   // finalR is just a variable to display the final value added on the serial monitor
       int finalR = (motorr + motorRight);   // finalR is just a variable to display the final value added on the serial monitor
        //        Serial.print("right  ");
     Serial.print(motorr);
	 Serial.print("\t");
     Serial.print(motorRight);
	 Serial.print("\t");
	 Serial.print(finalR);       
	 Serial.print("\t");
	 Serial.print("\t");
	*/ 
	 motorL.setSpeed(motorl + motorLeft);  // target values are added to steering and throttle values and then finally handed to motor controllers
/* 
 int finalL = (motorl + motorLeft);
	   //        Serial.print("left  ");
          Serial.print(motorl);
	 Serial.print("\t");
     Serial.print(motorLeft);
	 Serial.print("\t");
	 Serial.println(finalL);       
*/
     

      // alternative serial print output:
      //Serial.println("angle - GFORCE -GyroRate - PID -  raw LEFT   - raw RIGHT  - Motor LEFT - Motor Right");


//  Uncomment to display the values on processing 
//  The message from the arduino should be like below.
  
//  | ACC_angle, actAngle, ACC_X, ACC_Z, GYR_Y, GYR_X, pTerm, iTerm, dTerms, drive \n |

//  DbgPrint2 (((MB.gyrovalx) + (MB.gforcevaly)) /2);
//  Serial.print (MB.getGyroRate());
  DbgPrint2 (",");
  DbgPrint2 (MB.x_angle);
//  DbgPrint2 ("\n");
  DbgPrint2 (",");
  DbgPrint2 (MB.gforcevalx);
  DbgPrint2 (",");
  DbgPrint2 (MB.gforcevaly);
  DbgPrint2 (",");
  DbgPrint2 (MB.gyrovalz);
  DbgPrint2 (",");
  DbgPrint2 (MB.gyrovalx);
  DbgPrint2 (",");
  DbgPrint2 (MB.pTerm);
  DbgPrint2 (",");
  DbgPrint2 (MB.iTerm);
  DbgPrint2 (",");
  DbgPrint2 (MB.dTerm);  
  DbgPrint2 (",");
//  DbgPrint2 (MC.speed);
  DbgPrint2(MB.PID_sum);
  DbgPrint2 ("\n");



//      Serial.print (angle);
//      Serial.print("           ");
//      Serial.print (gab);
//      DbgPrint(angle);
//      Serial.print("     ");
      DbgPrint(MB.getGFORCE());
      DbgPrint("     ");
      DbgPrint(MB.getGyroRate());
      DbgPrint("     ");
      DbgPrint(PID);
      DbgPrint("         ");

      DbgPrint(motorLeft);   // raw throttle values
      DbgPrint("           ");
      DbgPrint(motorRight);  // raw throttle values
 //     Serial.print(motorl);   // motor values
 //     Serial.print("           ");
 //     Serial.println(motorr);  // motor values

      DbgPrint("           ");
//      Serial.println(motorl * 1.42);     // motor values

      //DbgPrint(finalL);  // those are the throttle values added to the motor values 
      DbgPrint("           ");
      //DbgPrintln(finalR);
      DbgPrint(motorr);
      DbgPrint("\n");
//      delay (100); // erase this later 
      // this can be copied into the Serial Display input line:
      // angle - GFORCE -GyroRate - PID -             raw LEFT                 - raw RIGHT          - Motor LEFT       - Motor Right

    }

  }


  else // wenn man von den TRKS absteigt werden die Motore abgeschaltet und fs wieder ausgeschaltet.
  // when getting off (or interrupting the foot switches) motors are shut off and fs turned off
  {
    fs = false;
    motorr = 0;
    motorl = 0;
    motorR.setSpeed(motorr);
    motorL.setSpeed(motorl);
 
 /*
    // when we switch off, then we set the PID Values to 0   this turns the thing in a P-controller only (WRONG !)
    angle = 0;
    MB.setK (0);
    MB.setKi (0);
    MB.setKp (0);
    MB.setKd (0);
*/  
}



  looptime = MB.getLoopTime();
}

/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// reminder to make an autoGetUp later
  void autoGetUp() {
  if (fall == 25 || fall == - 25) {
  left.write(servoZero);
  right.write(servoZero);
   delay(250);
   fall = 0;
   I = 0;
}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/





/* we don't send data to Display board
 // Funktion zum Senden der Daten an die DP - noch nicht funktionsfähig
 // function to send data to the display board - does not work yet
 void transmitData(float ampleft, float ampright, float voltage, int fail, int spd, int looptime)
 {
 
 ser.print("l");
 ser.print(ampleft);
 ser.print("r");
 ser.print(ampright);
 ser.print("v");
 ser.print(voltage);
 ser.print("f");
 ser.print(fail);
 ser.print("s");
 ser.print(spd);
 ser.println(";");
 
 */


/*time = time + looptime;
 
 if(time > SENDDELAY)
 {
 digitalWrite(DB4, HIGH);
 MB.setLED(false, false, false, true, false, false);
 
 if(analogRead(DB5) > 800)
 {
 MB.setLED(false, false, false, false, true, false);
 
 ser.print("l");
 ser.print(ampleft);
 ser.print("r");
 ser.print(ampright);
 ser.print("v");
 ser.print(voltage);
 ser.print("f");
 ser.print(fail);
 ser.print("s");
 ser.print(spd);
 ser.println(";");
 
 time = 0;
 
 digitalWrite(DB4, LOW);
 }
 
 }
 
 }
 */

/* ///////////////////////////////////////////////////////////////////////

PINOUT

 A 0	GFORCE X			
 A 1	GFORCE Y			
 A 2	connected to GFORCE Z but not used			
 A 3	GYROREF			
 A 4	GYRO4Z			
 A 5	GYRO4X			
 A 6	GYRO1Z			
 A 7	GYRO1X			
 A 8	CL L			
 A 9	CS R			
 A 10	Vref			geht auf Masse LM705 ??
 A 11	Poti	erase		
 A 12	DB5 A12	??		
 A 13				
 A 14				
 A 15				
 				
 D 0	TX			
 D 1	RX			
 D 2	PWML R			
 D 3	PWMH R			
 D 4				
 D 5	PWMH L			
 D 6	SS Display	DB1 	Sserial	Displaystecker
 D 7	SS Follow	DB 2	Sserial	Displaystecker
 D 8		DB 3	not used?	
 D 9		DB 4 	display comm	we do not use 
 D 10				
 D 11	PWML L			
 D 12				
 D 13	Ubat??			
 D 14				
 D 15				
 D 16				
 D 17				
 D 18				
 D 19				
 D 20				
 D 21				
 D 22	LED GR			
 D 23	DIR R			
 D 24	LED YL			
 D 25	DIR L			
 D 26	LED 4			
 D 27	Reset R			
 D 28	LED 3			
 D 29	Reset L			
 D 30	LED 2			
 D 31 	FF1 R			
 D 32	LED 1			
 D 33	FF2 R			
 D 34				
 D 35	FF1 L			
 D 36	GYROST			
 D 37	FF2 L			
 D 38	GYRODP			
 D 39	SW L			
 D 40 	GYROHP			
 D 41	SW R			
 D 42	DB RFID	connect here Display board D7		
 D 43				
 D 44	calibration JUMPER // changed from 53		
 D 45				
 D 46    				
 D 47				
 D 48
 D 49
 D 50    MISO
 D 51    MOSI
 D 52    SCK
 D 53	 SS   // was originally calibration Jumper			
///////////////////////////////////////////////////////////////////////////////////////
*/


