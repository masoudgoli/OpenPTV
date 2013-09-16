/*
OpenPTV Mainboard Firmare 1.0b Licenced unter CC by-nc not yet for public circulation.
Not for commercial Use.

USE AT OWN RISK!
Kalman-Filter and PID-Controller from www.x-firm.com

www.OpenPTV.org

****************** MotorController Library ******************
*/


#include "MotorController.h"
#include "Arduino.h"

// startupfunktion um die motorcontroller verwenden zu können.
// startup funtion to initialize motor controllers
void MotorController::startUp(int pinDir, int pinPwmLow, int pinPwmHigh, int pinReset, int pinFF1, int pinFF2, int pinCS) // hier müssen die nummern der pins übergeben werden.
                                                                                                                          // assignment of pin numbers
{
  pwmLow = pinPwmLow;
  pwmHigh = pinPwmHigh;
  dir = pinDir;
  ff1 = pinFF1;
  ff2 = pinFF2;
  reset = pinReset;
  cs = pinCS;
  
  i = 0;
  current = 0;
  braketype = 1;
  FF1 = 0;
  FF2 = 0;
  
  setCurrentZero(); // Stromaufnahmewert nullen  
                    // zero current-values
  // define pinModes 
  pinMode(pwmLow, OUTPUT);
  pinMode(pwmHigh, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(reset, OUTPUT);
  pinMode(ff1, INPUT);
  pinMode(ff2, INPUT);
  
  // set pins 
  digitalWrite(reset, HIGH);
  digitalWrite(pwmLow, HIGH);

}

// Drehrichtung der Motoren ändern
// change motor direction
void MotorController::setDir(int direc)
{
  digitalWrite(dir, direc); 
}

// art der bremse einstellen. Werte sind in der Header Datei und im Datenblatt der Motorcontroller.
// adjust break mode. Values are in MotorController.h and taken from the data sheets.
void MotorController::setBrakeType(int brk)
{
  braketype = brk;
}

// setzt die Geschwindigkeit der Motoren; es sind Werte zwischen -254 und 254 zulässig.
// sets motor speeds; values allowed from -254 .. 254.
void MotorController::setSpeed(int speed)
{
  
  /*if(speed > 254) speed = 254;
  else if(speed < -254) speed = -254;*/
  speed = constrain(speed, -254, 254); // kontrolliert die Werte, z.B. 42 = 42, -1337 = -254,  ...
                                       // constrains values, e.g. 42 = 42, -1337 = -254,  ... 
  
  if(speed < 0) // hier wird die drehzahl geändert falls die Werte negativ/positiv ist.
                // here the motor speed is changed if values are negative/positive.
  {
    speed = speed * -1;
    setDir(FORWARD);
  }
  else setDir(BACKWARD);
// Serial.println(speed);  //debugging
  switch(braketype) // hier wird der Bremstyp und die Geschwindigkeit and die MC übergeben  
                    // here the brake mode and the speed are handed to the motor controllers.
{
   case DRIVE_BRAKE:
/*    The duty cycle of the PWM controls the speed of the motor and the DIR pin controls the direction. 
      During the active (high) portion of the PWM, the motor outputs drive the motor by putting the 
      full V+ voltage across the motor in the direction determined by the DIR pin; during the low 
      portion of the PWM, the motor outputs brake the motor by shorting both motor terminals to ground. 
      This means that the motor alternates between drive and brake at the PWM frequency with the 
      percentage of the driving time determined by the duty cycle.
*/
       digitalWrite(pwmLow, HIGH);
       analogWrite(pwmHigh, speed);
   break;

   case DRIVE_COAST:
/*    During the active (high) portion of the PWM, the motor outputs drive the motor by putting the 
      full V+ voltage across the motor in the direction determined by the DIR pin; during the low 
      portion of the PWM, the motor outputs are disconnected and the motor is allowed to coast. 
      This means that the motor alternates between drive and coast at the PWM frequency with the 
      percentage of the driving time determined by the duty cycle. Drive-coast operation can draw 
      less power than drive-brake operation, but drive-brake operation can produce a more linear 
      relationship between duty cycle and motor speed.
*/
       analogWrite(pwmLow, speed);
       analogWrite(pwmHigh, speed);
   break;

   case BRAKE_COAST:
/*    During the active (high) portion of the PWM, the motor outputs brake the motor by shorting both 
      motor terminals to ground; during the low portion of the PWM, the motor outputs are disconnected 
      and the motor is allowed to coast. This means that the motor alternates between brake and coast 
      at the PWM frequency with the percentage of the braking time determined by the duty cycle.
*/
       digitalWrite(pwmHigh, LOW);
       analogWrite(pwmLow, speed);
   break;
}
  
}


// wenn diese Funktion ausgeführt wird, wird der Motorcontroller resetet.
// if this function is executed the motor controller is reset.
void MotorController::resetController()
{
  digitalWrite(reset, LOW);
  delay(1);
  digitalWrite(reset, HIGH);
}


// die Stromaufnahmen werden ausgelesen. Um Spitzen herauszufiltern kann es sinnvoll sein, den CS Kondensator auf dem Motorcontroller einzulöten; siehe Datenblatt.
// the current sensors are read. To filter peaks it might make sense to solder the CS capacitor onto the motor controller; refer to data sheet.
float MotorController::getCurrent()
{
  current = 0;
  for(n = 0; n < 3; n++) current += analogRead(cs);
  
  current = ((current / 3) - currentzero) / CURRENT_DIVISOR;
  
  if(current < 0) current = current * -1;
  
  return(current);
  
}


// auslesen der Fehlercodes des Motors.
// read motor fault flag 
int MotorController::getFail()
{
  FF1 = digitalRead(ff1);
  FF2 = digitalRead(ff2);
  
  if(FF1 == LOW && FF2 == LOW) return(NO_FAIL);                  // kein Fehler    // no fault
  else if(FF1 == LOW && FF2 == HIGH) return(SHORT_CIRCUIT);      // "Kurzschluss"  // short circuit
  else if(FF1 == HIGH && FF2 == LOW) return(OVER_TEMP);          // Übertemperatur // over temperature
  else if(FF1 == HIGH && FF2 == HIGH) return(UNDER_VOLTAGE);     // Unterspannung  // under voltage
  
}


// nullen der stromaufnahme
// zero current-sensors
void MotorController::setCurrentZero()
{
  for(n = 0; n < 50; n++) currentzero += analogRead(cs);
  currentzero = currentzero / 50;
}
