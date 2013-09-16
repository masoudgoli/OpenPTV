/*
OpenPTV Mainboard Firmare 1.0b Licenced unter CC by-nc not yet for public circulation.
Not for commercial Use.
USE AT OWN RISK!
Kalman-Filter and PID-Controller from www.x-firm.com
www.OpenPTV.org


****************** MotorController Library ******************
*/

// Bremstypen
// brake modes
#define DRIVE_BRAKE 1
#define DRIVE_COAST 2
#define BRAKE_COAST 3


#define FORWARD LOW
#define BACKWARD HIGH


// Fehlertypen
// fault types
#define NO_FAIL 0
#define SHORT_CIRCUIT 1
#define OVER_TEMP 2
#define UNDER_VOLTAGE 3

// Korrekturwert der Stromaufnahme; siehe Datenblatt.
// current sensor correction value; see data sheet. 
#define CURRENT_DIVISOR 13.5168

class MotorController
{
  
public:
void startUp(int, int, int, int, int, int, int);
void setBrakeType(int);
void setSpeed(int);
void resetController(void);


float getCurrent(void);
int getFail(void);
int speed;

private:

void setDir(int);
void setCurrentZero(void);

int pwmLow;
int pwmHigh;
int dir;
int ff1;
int ff2;
int reset;
int cs;
int currentzero;

int i;
float current;
int braketype;
int FF1;
int FF2;
int n;



};
