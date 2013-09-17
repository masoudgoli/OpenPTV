// This class defines all communications between the Mega and its daughter
// processing boards (uno and nano)

#define Displayboard_SS 6      // definition of the slave select pins
#define Followboard_SS 7

enum BoardName
{ 
    Mainboard = 0,
    Displayboard = 2,
    Followboard = 1,
};

class Board2Board
{
public:
    
    // Establish communications with each board, returns bitmap describing what is alive.
    int  begin(void);

    // End communications
    void end(void);

    // See if the other board is alive
    bool hello(BoardName b);
    
    
    // Retrieve the throttle values from the Following Board.
    bool getThrottles(int* motorLeft, int* motorRight, int* voltage_D);
    bool getVoltage(int* voltage_D);    // try to take this out for now and see whether the SS line is LOW
};

