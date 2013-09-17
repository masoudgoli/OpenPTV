/*

 // changed vrefval of ADXL335 from analogRead(VREF) to analogRead(GYROREF)
 
 
 
 OpenPTV Mainboard Firmare 1.0b Licenced unter CC by-nc not yet for public circulation.
 Not for commercial Use.
 
 USE AT OWN RISK!
 Kalman-Filter and PID-Controller from www.x-firm.com
 
 www.OpenPTV.org
 
 
 ****************** Mainboard Library ******************
 */


#include "Arduino.h"
#include "MainBoard.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>


// startUp(): definiert die Variablen, setzt die Pinmodes auf die jeweiligen Ausgangswerte.
// startUp(): defines variables and sets pinmodes to start values.
void MainBoard::startUp(void)
{
  i = 0;

  voltcount = 0;
  volttime = 0;
  voltage = 0;
  oldvoltage = 26.5;

  trkr = 0;
  trkl = 0;

  gyrovalx = 0;
  gyrovalz = 0;
  gyrovalref = 0;

  gforcevalx = 0;
  gforcevaly = 0;
  vrefval = 0;

  gyrovalxzero = 0;
  gyrovalzzero = 0;

  gforcevalxzero = 0;
  gforcevalyzero = 0;

  lastlooptime = 20;
  loopstarttime = 0;

//  potival = 0;

  //Kalman variables
  Q_angle = 0.001;  //0.001
  Q_gyro  = 0.003;  //0.003
  R_angle = 0.03;    //0.03

  x_angle = 0;
  x_bias = 0;
  P_00 = 0;
  P_01 = 0;
  P_10 = 0;
  P_11 = 0;
  //end Kalman variables

  //PID variables w/o additional Serial connection and steering
  K =  1.0;   
  Kp = 0.78;                   
  Ki = 0.020; 
  Kd = 2.5;      
  
/*
  //PID variables w/ additional Serial connection (Processing "BalancingBotControl") : 
  K =  1.0;   
  Kp = 0.78;                   
  Ki = 0.020; 
  Kd = 2.5;       
 */
  /*
    //PID variables w/o any additional communication 
  K =  1.4;  
  Kp = 3.9;                   
  Ki = 0.020;
  Kd = 9.0;       
  */
/*
  //PID variables with SPI communication (Steering control": 
  K =  1.2;   
  Kp = 0.8;                    
  Ki = 0.10; 
  Kd = 0.0;        
*/    
  
/*
1. Set I and D term to 0, and adjust P so that the robot starts to oscillate 
(move back and forth) about the balance position. P should be large enough 
for the robot to move but not too large otherwise the movement would not be smooth.
2. With P set, increase I so that the robot accelerates faster when off balance. 
With P and I properly tuned, the robot should be able to self-balance for at least a few seconds.
3. Finally, increase D so that the robot would move about its balanced position more gentle, 
and there shouldnâ€™t be any significant overshoots.
*/
  last_error = 0.0;          // was 0
  integrated_error = 0.0;    // was 0
  pTerm = 0; 
  iTerm = 0; 
  dTerm = 0;
  //ENDPID variables
  /*


1.Set all gains to 0.
2.Increase Kd until the system oscillates.    //38
3.Reduce Kd by a factor of 2-4.               //19..9.5 -> 10
4.Set Kp to about 1% of Kd.                   // 1
5.Increase Kp until oscillations start.       // 4
6.Decrease Kp by a factor of 2-4.             // 2..1 -> 1.5
7.Set Ki to about 1% of Kp.                   // 0.01 .. 0.02 -> 0.015
8.Increase Ki until oscillations start.       //0.01
9.Decrease Ki by a factor of 2-4.             //0.005..0.0025  -> 0.003

Effects of independent P, I, and D tuning on closed-loop response.
For example, while KI and KD are fixed, increasing KP alone can decrease rise time, 
increase overshoot, slightly increase settling time, decrease the steady-state error, 
and decrease stability margins.

                Rise Time 	Overshoot 	Settling Time 	Steady-State 	Error Stability

Increasing KP	Decrease 	Increase 	Small Increase 	Decrease 	Degrade
Increasing KI	Small Decrease 	Increase 	Increase Large 	Decrease 	Degrade
Increasing KD	Small Decrease	Decrease	Decrease 	Minor Change 	Improve



Set all gains to zero.
Increase the P gain until the response to a disturbance is steady oscillation.
Increase the D gain until the the oscillations go away (i.e. it's critically damped).
Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
Set P and D to the last stable values.
Increase the I gain until it brings you to the setpoint with the number of oscillations desired (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)
What disturbance you use depends on the mechanism the controller is attached to. Normally moving the mechanism by hand away from the setpoint and letting go is enough. If the oscillations grow bigger and bigger then you need to reduce the P gain.

If you set the D gain too high the system will begin to chatter (vibrate at a higher frequency than the P gain oscillations). If this happens, reduce the D gain until it stops.


   For manual tuning, set all three constants to zero, then turn up Kp until oscillation occurs.
   Then turn up Kd until oscillation disappears. Adjust Kd until the system is critically damped, i.e. thereâ€™s no overshoot.
   Then increase Ki until the steady-state error goes to zero in a reasonable time.
   */



  // pinModes fÃ¼r die LEDs grÃ¼n, gelb, debug 1-4
  // pinModes for green, yellow, and debug 1-4 LEDs
  pinMode(GR, OUTPUT);
  pinMode(YL, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);

  // setzen der pinModes fÃ¼r Beschleunigungssensor, Kreisel, (KALIBRIERUNGS-)Jumper ??, Trittkontakttaster und dem "Entsperrungspin" von der displayplatine
  // pinModes for acceleration sensor, gyro, calibration jumper, foot switch, and "start pin" on display board
  pinMode(GFORCEST, OUTPUT);

  pinMode(GYROST, OUTPUT);
  pinMode(GYROHP, OUTPUT);
  pinMode(GYRODP, OUTPUT);

  pinMode(JUMPER, INPUT);

  pinMode(TRKR, INPUT);
  pinMode(TRKL, INPUT);

  pinMode(DBRFID, INPUT);



  // am Beginn alle LEDs abschalten
  // for starting switch all LEDs off 
  MainBoard::setLED(false, false, false, false, false, false);

  // setzen der Pins fÃ¼r die Sensoren. 
  // set pins for sensors. Refer to data sheets of the breakout boards for the pin functions. 
  digitalWrite(GFORCEST, LOW);  // Self-test (LOW: normal mode; HIGH: self-test)

  digitalWrite(GYROST, LOW);    // Self-test (LOW: normal mode; HIGH: self-test)
  digitalWrite(GYROHP, LOW);    // High pass filter reset (LOW: normal operation mode; HIGH: external high pass filter is reset)
  digitalWrite(GYRODP, LOW);    // Power-down (LOW: normal mode; HIGH 1: power-down mode)
 
  // +++ ATTENTION: DO NEVER SET VALUES TO HIGH: SENSOR OPERATES WITH 3.3V AND WOULD GET DESTROYED +++
 
  // die Kalibrationswerte aus dem EEPROM laden.
  // read calibration values from EEPROM
  MainBoard::loadZeroVal();

  // debug: read values from EEPROM
  Serial.println("loaded values from EEPROM");  // erase later 
  byte sign_GyroX = EEPROM.read(0);
  byte GyroX = EEPROM.read(1); 
  byte sign_GyroZ = EEPROM.read(2); 
  byte GyroZ = EEPROM.read(3); 
  byte sign_GForceX = EEPROM.read(4); 
  byte GForceX = EEPROM.read(5); 
  byte sign_GForceY = EEPROM.read(6); 
  byte GForceY = EEPROM.read(7);
//  byte value8 = EEPROM.read(8);  // not used (just to see that there is nothing in this cell
//  byte value9 = EEPROM.read(9);  // not used

  Serial.println("-     GyroX     -     GyroZ     -     GForceX   -      GForceY");
  Serial.print (sign_GyroX);
  Serial.print ("\t");
  Serial.print (GyroX);
  Serial.print ("\t");
  Serial.print (sign_GyroZ);
  Serial.print ("\t");
  Serial.print (GyroZ);
  Serial.print ("\t");
  Serial.print (sign_GForceX);
  Serial.print ("\t");
  Serial.print (GForceX);
  Serial.print ("\t");
  Serial.print (sign_GForceY);
  Serial.print ("\t");
  Serial.print (GForceY);
//  Serial.print ("\t");
//  Serial.print (value8);
//  Serial.print ("\t");
//  Serial.print (value9);
  Serial.println ();
  // end debug

}
/*
 getVoltage() lieÃŸt die Batteriespannung ein. die Konstante VOLTAGE_DIVISOR wird in MainBoard.h definiert. 
 BETATESTER!!! -> der wert der eingetragen werden muss steht auf den kleinen zusatzplatinen die an der HP hÃ¤ngen.
 
 getVoltage() reads battery voltage. The constant VOLTAGE_DIVISOR is defined in MainBoard.h .
 BETA TESTER !!! -> the value is printed on the small board connected on the main board.
 
 */
float MainBoard::getVoltage(void)
{
  return((float)analogRead(BatteryVoltage) / VOLTAGE_DIVISOR);
}

// setLED setzt die LEDs auf der HP. zb. setLED(HIGH, HIGH, HIGH, HIGH, HIGH, HIGH) lÃ¤sst alle LEDs leuchten.
// sets LEDs on main board (HIGH is ON) 
void MainBoard::setLED(bool debug1, bool debug2, bool debug3, bool debug4, bool yellow, bool green)
{
  if(debug1) digitalWrite(D4, HIGH);
  else digitalWrite(D4, LOW);

  if(debug2) digitalWrite(D3, HIGH);
  else digitalWrite(D3, LOW);

  if(debug3) digitalWrite(D2, HIGH);
  else digitalWrite(D2, LOW);

  if(debug4) digitalWrite(D1, HIGH);
  else digitalWrite(D1, LOW);

  if(yellow) digitalWrite(YL, HIGH);
  else digitalWrite(YL, LOW);

  if(green) digitalWrite(GR, HIGH);
  else digitalWrite(GR, LOW);
}

// getJumper() lieÃŸt den Status des Jumpers der HP ein und gibt HIGH oder LOW zurÃ¼ck
// reads status of main board jumper and returns HIGH or LOW
bool MainBoard::getJumper(void)
{
  if(digitalRead(JUMPER) == HIGH) return(true);
  else return(false);
}

// getTRK() gibt den Status der Trittkontakttaster aus. Die Werte der Konstanten sind in der Headerdatei hinterlegt
// returns stati of foot switches. The constant values are defined in MainBoard.h

int MainBoard::getTRK(void)
{
  trkl = digitalRead(TRKL);
  trkr = digitalRead(TRKR);

  if(trkl != LOW && trkr != LOW) return(STATE_TRKB); //beide    // both
  else if(trkl != LOW) return(STATE_TRKL);           //links    // left
  else if(trkr != LOW) return(STATE_TRKR);           //rechts   // right
  else return(STATE_TRKN);                           //keiner   // none

}

/*
 updateSensors() liest die Werte der Sensoren ein und speichert diese in den Variablen. 
 Um mit den Werten arbeiten zu kÃ¶nnen, mÃ¼ssen sie mit anderen Funktionen ausgelesen werden. (getGFORCE() getGyroRate() getSteering())
 Diese eingelesenen Werte sind die Rohwerte der Sensoren (0 - 1023)!
 
 updateSensors() reads sensor values and stores them in variables
 To process the values, they must be read by other functions (getGFORCE() getGyroRate() getSteering())
 The sensor values are raw data (0 .. 1023)!
 */
void MainBoard::updateSensors(void)
{
  gyrovalx = 0;
  gyrovalz = 0;

  gforcevalx = 0;
  gforcevaly = 0;

  // referenzwerte werden eingelesen.
  // read reference values
  //  vrefval = analogRead(VREF);  // this was the 5V vref for the ADXL322; the new ADXL335 requires 3.3 V
  vrefval    = analogRead(GYROREF); // changed over to same 3.3V as for Gyro
  gyrovalref = analogRead(GYROREF);
  //  potival = analogRead(POTI) - vrefval / 2;  //we use steering


  // es werden die sensorwerte UTIMES mal eingelesen ...
  // read sensor values UTIMES ...

  // ANGLE ///////////////////////////////////////////////////////
  for(i = 0; i < UTIMES; i++)
  {
    gyrovalx = gyrovalx + (analogRead(GYRO4X) - gyrovalref);
  }
  gyrovalx = gyrovalx / UTIMES - gyrovalxzero; // ... und dann der Mittelwert des Winkels gebildet.
  // ... and then calculate the mean of the angle
  for(i = 0; i < UTIMES; i++)
  {
    gyrovalz = gyrovalz + (analogRead(GYRO4Z) - gyrovalref);
  }
  gyrovalz = gyrovalz / UTIMES - gyrovalzzero;
  // END ANGLE ///////////////////////////////////////////////////////

  // ACCELERATION //////////////////////////////////////////////////////
  for(i = 0; i < UTIMES; i++)
  {
     gforcevalx = gforcevalx + (analogRead(GFORCEX) - vrefval );    
  }
  gforcevalx = gforcevalx / UTIMES - gforcevalxzero;

  for(i = 0; i < UTIMES; i++)
  {
    gforcevaly = gforcevaly + (analogRead(GFORCEY) - vrefval );  
  }
  gforcevaly = gforcevaly / UTIMES - gforcevalyzero;
  // END ACCELERATION //////////////////////////////////////////////////
}

/*
 calibrateSensors() sollte am Anfang bei der Kalibrierung ausgefÃ¼hrt werden um die "0-Lage" des Fahrzeugs zu bestimmen.
 Diese Funktion eliminiert den Effekt einer ggf. schlecht ausgerichteten Platine / Sensoren.
 
 calibrateSensors() should be executed at the beginning of calibration to determine the "zero-position" of the vehicle.
 This function eliminates the effect of improperly adjusted board / sensors
 */
void MainBoard::calibrateSensors(void)
{
  gyrovalxzero = 0;
  gyrovalzzero = 0;

  gforcevalxzero = 0;
  gforcevalyzero = 0;

  // Referenzwerte werden eingelesen und gespeichert.
  // reference values are read and stored.
  vrefval = analogRead(GYROREF);   /// was VREF before ...
  gyrovalref = analogRead(GYROREF);

  // hier wird wieder der Mittelwert gebildet...
  // here the mean is calculated ...
  for(i = 0; i < UTIMESCAL; i++)
  {
    gyrovalxzero = gyrovalxzero + (analogRead(GYRO4X) - gyrovalref);
  }
  gyrovalxzero = gyrovalxzero / UTIMESCAL;


  for(i = 0; i < UTIMESCAL; i++)
  {
    gyrovalzzero = gyrovalzzero + (analogRead(GYRO4Z) - gyrovalref);
  }
  gyrovalzzero = gyrovalzzero / UTIMESCAL;


  for(i = 0; i < UTIMESCAL; i++)
  {
    gforcevalxzero = gforcevalxzero + (analogRead(GFORCEX) - vrefval );    
  }
  gforcevalxzero = gforcevalxzero / UTIMESCAL;


  for(i = 0; i < UTIMESCAL; i++)
  {
    gforcevalyzero = gforcevalyzero + (analogRead(GFORCEY) - vrefval );    
  }
  gforcevalyzero = gforcevalyzero / UTIMESCAL;
  Serial.println("Sensors calibrated successfully");
  //delay (2000);
/* These are the actual values from the Mini-PREx
-  GyroX  -  GyroZ  -  GForceX  -  GForceY
0	83	  0	  80	1	  8		1	  2
*/
  
  /*  
   Hier werden die Werte in das EEPROM gespeichert.
   Wir brauchen ein zusÃ¤tzliches byte, welches speichert ob ein Wert positiv oder negativ ist.
   
   Here the values are stored in the EEPROM.
   We need an additional byte to make the value signed (positive or negative).
   */



  if(gyrovalxzero < 0) 
  {
    EEPROM.write(0, 1);                 // Vorzeichen  //sign    
    EEPROM.write(1, gyrovalxzero * -1); // Wert        // value
  }
  else 
  {
    EEPROM.write(0, 0);
    EEPROM.write(1, gyrovalxzero);
  }

  if(gyrovalzzero < 0) 
  {
    EEPROM.write(2, 1);
    EEPROM.write(3, gyrovalzzero * -1);
  }
  else 
  {
    EEPROM.write(2, 0);
    EEPROM.write(3, gyrovalzzero);
  }

  if(gforcevalxzero < 0) 
  {
    EEPROM.write(4, 1);
    EEPROM.write(5, gforcevalxzero * -1);
  }
  else
  { 
    EEPROM.write(4, 0);
    EEPROM.write(5, gforcevalxzero);
  }

  if(gforcevalyzero < 0) 
  {
    EEPROM.write(6, 1);
    EEPROM.write(7, gforcevalyzero * -1);
  }
  else 
  {
    EEPROM.write(6, 0);
    EEPROM.write(7, gforcevalyzero);
  }

}

// loadZeroVal() lÃ¤dt die Korrekturwerte aus dem EEPROM und setzt auch gleich das Vorzeichen.
// loadZeroVal() reads correction values from EEPROM and also signs the value.
void MainBoard::loadZeroVal(void)
{
  if(EEPROM.read(0) == 1) gyrovalxzero = -1 * EEPROM.read(1); // wenn vorzeichenbyte == 1 wert ist negativ  //if sign_byte == 1  value is negative
  else gyrovalxzero = EEPROM.read(1);                         // wenn vorzeichenbyte == 0 wert ist positiv  //if sign_byte == 0  value is positive

  if(EEPROM.read(2) == 1) gyrovalzzero = -1 * EEPROM.read(3);
  else gyrovalzzero = EEPROM.read(3);

  if(EEPROM.read(4) == 1) gforcevalxzero = -1 * EEPROM.read(5);
  else gforcevalxzero = EEPROM.read(5);

  if(EEPROM.read(6) == 1) gforcevalyzero = -1 * EEPROM.read(7);
  else gforcevalyzero = EEPROM.read(7);

}

// getLoopTime() gibt die Schleifendurchlaufzeit in ms zurÃ¼ck.
// getLoopTime() returns the loop time in ms.
int MainBoard::getLoopTime(void)
{
  lastlooptime = millis() - loopstarttime;
  loopstarttime = millis();
  return(lastlooptime);
}


int MainBoard::getGyroRate(void)
{
// gibt den Winkel des Kreisels zurÃ¼ck (dual axis pitch and roll +/- 300Â°/s
// returns gyro angle from the LPR530AL  (A4 = GYRO4Z, A5 = GYRO4X)
  
    return((float)((gyrovalx -0) * 2.7525)); // ARef=3.3V, Gyro sensitivity=3.33mV/(deg/sec)
                                          // (1024/360)/1024 * 3.3/0.0033  =  2.7525 ist der Korrekturwert fÃ¼r diesen Sensor
                                          // 2.7525 is the correction value for this sensor
//  return(((float)((gyrovalx -3) + gyrovalz) / 2) * 2.7525);  // if X and Z is used 
						// -3 is a point of gravity adjustment (in degrees)	
  }


int MainBoard::getGFORCE(void)
{
// gibt den Wert des Beschleunigungssensors zurÃ¼ck
// returns value of the ADXL355 accelerometer (+/- 3g; 300mV/g)
   //  return((float)(gforcevalx + gforcevaly) / 2) * (float)(110/90);   // (73/90)  // was orig. (110/90), changed because Vref was changed to GYROREF  
  return(((float)(gforcevalx + gforcevaly) / 2) * 2.7525);   //    1.23 1.86  // or  /2 * (90/74)  // empiricially the value is 1.86  
// orig. ADXL322 has 420mV/g, used ADXL335 has 300mV/g: this is 1.4 times less
  // es wird der Mittelwert der beiden Achsen gebildet und dann mit dem Korrekturwert multipliziert.
  // calculate mean of both axis and then multiplied by correction value

  // Acquisition of Correction Value:
  // use Gyro_and_Gforce_tool, set the vehicle plain on a stand to calibrate sensors;
  // tilt board 90Â° and note this value; here: 73
  // This angle is here devided by 90 to get 1Â°
  // ********* BUT : ******************************
  // This method returned in the Main.ino wrong results. Empiricially the correction value was 1.86
  
}

/*  we read the steering the display routine
 // gibt den Wert des Lenkpotentiometers zurÃ¼ck
 // returns steering pot value
 int MainBoard::getSteering(void)
 {  
 potival = potival / 7;                           // da der Wert direkt zu den motorausgabewerten addiert wird mÃ¼ssen wir ihm mal verkleinern.
 // as the value is added directly to the motorPWMs we have to scale it down. 
 
 if(potival < -25 || potival > 25) potival = 0;   // eliminiert die ungenauigkeit um den mittelpunkt  // was &&
 // eliminates jitter around the center
 return(potival);                                 
 }
 */

// Kalmanfilter vom x-firm projekt  // Kalman filter from x-firm project
float MainBoard::getKalman(float newAngle, float newGForce_Rate, int looptime)
{
  dt = float(looptime)/1000;
  x_angle += dt * (newGForce_Rate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return((float)x_angle);
// should here be the compensation for the angle offset
}


// sendet die daten an die displayplatine funktioniert noch nicht.
// sending data to the display board does not work yet
/*void MainBoard::transmitData(float ampleft, float ampright, float voltage, int fail, int spd, int looptime)
 {
 time = time + looptime;
 
 if(time > SENDDELAY)
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
 ser.println("e");
 
 time = 0;
 }
 
 }*/

// gibt den status vom RFID sensors zurÃ¼ck. Wird direkt vom Controller der DP angesteuert und dient als Wegfahrsperre.
// returns status from RFID reader. This is controlled directly from the display board Arduino and acts as anti theft device.
bool MainBoard::getRFID(void)
{
#ifdef NO_RFID
  return(true);
#else  
  if(digitalRead(DBRFID) == HIGH) return(true);    // change over to SPI reading
  else return(false);
#endif  
}


// berechnet die Spannung der Batterien.
// calculates the battery voltage
float MainBoard::calculateVoltage(int looptime)
{
  voltage = voltage + MainBoard::getVoltage();
  voltcount++;
  volttime = volttime + looptime;

  if(volttime > VOLTDELAY)
  {
    voltage = voltage / voltcount;
    voltcount = 0;
    volttime = 0;
    oldvoltage = voltage;
    voltage = 0;
  }
  return(oldvoltage);


}


// PID-Regler vom x-firm projekt ACHTUNG BEIM Ã„NDERN DES INTEGRAL WERTS!!!
// PID controller by x-firm project  - CAREFUL WHEN INTEGRAL VALUE IS CHANGED !!!
/*
http://forum.arduino.cc/index.php?topic=8871.30;wap2
 
 
 */
int MainBoard::updatePid(int targetPosition, int currentPosition)   
{
  int error = targetPosition - currentPosition; 
  pTerm = Kp * error;
  integrated_error += error;                                  
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);                            
  last_error = error;

/*
// debugging
//  Serial.println ("P    I     D");
  Serial.print(pTerm);
  Serial.print("\t");
  Serial.print(iTerm);
  Serial.print("\t");
  Serial.println(dTerm);
*/
  PID_sum = (-constrain(K*(pTerm + iTerm + dTerm), -254, 254));
//  return(-constrain(K*(pTerm + iTerm + dTerm), -254, 254));  
  return (PID_sum);
}
// http://forum.arduino.cc/index.php?PHPSESSID=g8qsgiv792rs7d2f0jek06t6u7&topic=8871.60;wap2

void MainBoard::setKp(float kpval)
{
  Kp = kpval;
}

void MainBoard::setKi(float kival)
{
  Ki = kival;
}

void MainBoard::setKd(float kdval)
{
  Kd = kdval;
}

void MainBoard::setK(float kval)
{
  K = kval;
}

