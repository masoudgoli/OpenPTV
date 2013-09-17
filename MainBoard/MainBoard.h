/*
OpenPTV Mainboard Firmare 1.0b Licenced unter CC by-nc not yet for public circulation.
Not for commercial Use.

USE AT OWN RISK!
Kalman-Filter and PID-Controller from www.x-firm.com

www.OpenPTV.org

****************** Mainboard Library ******************
*/

#define NO_RFID  // Define this to skip the RFID key verification

#include "SoftwareSerial.h"
// div constants
#define STATE_TRKN 0  // if no switch is pressed
#define STATE_TRKL 1  // if switch #1 is pressed
#define STATE_TRKR 2  // if switch #2 is pressed
#define STATE_TRKB 3  // if switch #1 & #2 is pressed

#define UTIMES 3      // number to build average of sensors during normal operation
#define UTIMESCAL 50  // number to build average of sensors during calibration

#define SENDDELAY 2000
#define VOLTDELAY 2000
#define GUARD_GAIN 20  // was 20
/*
GUARD_GAIN sets up a limit for iTerm.
The integral term allows a precise landing at setpoint, it is known as â€œsteady state errorâ€.
It is the only term that keeps changing no matter how small the error remains.
Youâ€™re basically telling the controller, â€œif you arenâ€™t quite getting to angle, keep increasing PWM over a period of time until you get thereâ€.
*/



//PinModes
#define BatteryVoltage 13
#define VOLTAGE_DIVISOR 10.13    // was 20.26  we are currently using 12V instead of 24V
//pin definitions
#define GR 22
#define YL 24
#define D1 32
#define D2 30
#define D3 28
#define D4 26

// pindefinition der pins die an das DB gehen.
// definitions for pins that go to the display board. 
//#define DB1 6  //A3    Softwareserial TX    // collision with SS_select
//#define DB2 7  //A2    SoftwareSerial RX


#define DB3 8  //A1    NOT USED ?
// #define DB4 9  //A0    used in Display communication
#define DB5 A12 //D6
#define DBRFID 42 //D7      // delete, Reading via SPI; assign variable

#define JUMPER 44      // was 53 but had a collision with SPI SS
#define POTI 11

#define TRKL 39    // switch #? (left)
#define TRKR 41    // switch #? (right)

#define GFORCEY 4  //was 1
#define GFORCEX 5  //was 0
#define VREF 10   // changed over to GYROREF (A3)
#define GFORCEST 34

#define GYRO4X 0  //was 5
#define GYRO4Z 1  //was 4
#define GYRO1X 7  // not used
#define GYRO1Z 6  // not used
#define GYROREF 3
#define GYROST 36
#define GYRODP 38
#define GYROHP 40


class MainBoard
{
  
// functions definition ...
public:
float getVoltage(void);
void startUp(void);
void setLED(bool, bool, bool, bool, bool, bool);
bool getJumper(void);
int getTRK(void);
void calibrateSensors(void);
int getLoopTime(void);
int getSteering(void);
int getGyroRate(void);
int getGFORCE(void);
float getKalman(float, float, int);
void transmitData(float, float, float, int, int, int);
bool getRFID(void);
void updateSensors(void);
float calculateVoltage(int);
int updatePid(int, int);

void setKp(float);
void setKi(float);
void setKd(float);
void setK(float);



// private:  //uncommented to try to read values directly
int i;
int voltcount;
float voltage;
long volttime;
float oldvoltage;

int trkr;
int trkl;

int potival;

int gyrovalx;
int gyrovalz;
int gyrovalref;

int gforcevalx;
int gforcevaly;
int vrefval;

int gyrovalxzero;
int gyrovalzzero;

int gforcevalxzero;
int gforcevalyzero;

int lastlooptime;
unsigned long loopstarttime;


// Kalman variables:
float Q_angle;
float Q_gyro;  
float R_angle;  

float x_angle;
float x_bias;
float P_00, P_01, P_10, P_11;
float dt, y, S;
float K_0, K_1;
    
// End Kalman variables    



// PID variables
float   K;
float   Kp;                     
float   Ki;               
float   Kd;
float last_error;
float integrated_error;
float pTerm, iTerm, dTerm;
float PID_sum;

// END PID variables


void loadZeroVal(void);


};


