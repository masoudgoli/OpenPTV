#include <arduino.h>
#include <SPI.h>
#include "b2b.h"


enum // Message Types
  {
    NothingMsgId = 0,
    HelloMsgId = 1,
    ThrottleMsgId = 2,
    VoltageMsgId = 3,

    WaitingMsgId = 100,
    WaitingForCmdMsgId = 101,
    OkMsgResponse = 102,
    
    AbortMsgResponse = 0xff,
  };

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

class SpiSS
{
public:
  byte line;
  inline SpiSS(byte ssLine):line(ssLine) { digitalWrite(line, LOW); }
  inline ~SpiSS() { digitalWrite(line, HIGH); }
};

byte spiXfer(byte b)
{
  SPDR = b;
  delayMicroseconds(500);  // So the slave can react
  while(!(SPSR & (1<<SPIF)));
  return SPDR;
}

int Board2Board::begin(void)
{

  digitalWrite(Followboard_SS, HIGH);    //was HIGH
  digitalWrite(Displayboard_SS, HIGH);   //was HIGH
  pinMode(Followboard_SS, OUTPUT);
  pinMode(Displayboard_SS, OUTPUT);
  
  SPI.begin();
  //SPI.setDataMode(SPI_MODE0); SPI.setBitOrder(MSBFIRST); SPI.setClockDivider(SPI_CLOCK_DIV16);
/*
#if 0
  pinMode(2,OUTPUT);
  for (int i=0;i<200;i++)
    {
      digitalWrite(2,HIGH);
      digitalWrite(SS,HIGH);
      delay(1);
      digitalWrite(2,LOW);
      digitalWrite(SS,LOW);
      delay(1);     
    }
#endif    
*/
}

bool Board2Board::hello(BoardName b)
{
  bool result = false;
  if (b == Mainboard) return true; // I am the mainboard!
  
  SpiSS line((b==Displayboard) ? Displayboard_SS: Followboard_SS);  // Enable the proper chip select line
  
  spiXfer(HelloMsgId);  // Send the hello
  result = (spiXfer(WaitingMsgId)==OkMsgResponse);  // See if we get the correct response
  return result;  
}


bool Board2Board::getThrottles(int* motorLeft, int* motorRight, int* voltage_D)
{
  SpiSS line(Followboard_SS);  // Enable the following board SPI
  byte rcvd = spiXfer(ThrottleMsgId);
  if (rcvd != WaitingForCmdMsgId) return false;
  //delay(1);
  unsigned int tmp=0;
  unsigned char crc=0;
  
  // Grab motorLeft -- a 16 bit number in 2 8 bit chunks
  rcvd = spiXfer(WaitingMsgId);
  crc = crc8(crc,rcvd);
  tmp = (rcvd << 8);

  rcvd = spiXfer(WaitingMsgId); 
  crc = crc8(crc,rcvd);  
  *motorLeft = tmp | rcvd;

  // Grab motorRight -- a 16 bit number in 2 8 bit chunks
  rcvd = spiXfer(WaitingMsgId);
  crc = crc8(crc,rcvd);
  tmp = (rcvd << 8);

  rcvd = spiXfer(WaitingMsgId); 
  crc = crc8(crc,rcvd);  
  *motorRight = tmp | rcvd;  
  
  // Grab voltage -- a 16 bit number in 2 8 bit chunks
  rcvd = spiXfer(WaitingMsgId);
  crc = crc8(crc,rcvd);
  tmp = (rcvd << 8);

  rcvd = spiXfer(WaitingMsgId); 
  crc = crc8(crc,rcvd);  
  *voltage_D = tmp | rcvd;  
  
  byte checksum = spiXfer(WaitingMsgId);

  if (crc == checksum) return true;
  *motorLeft=0;  // Zero out the throttles so there is no confusion that
  *motorRight=0; // bad data was received
  *voltage_D=0;
  return false;
}


bool Board2Board::getVoltage(int* voltage_D)
{
  SpiSS line(Displayboard_SS);  // Enable the following board SPI

  byte rcvd = spiXfer(VoltageMsgId);
  if (rcvd != WaitingForCmdMsgId) return false;

  unsigned char crc=0;
  unsigned int tmp=0; 
  
  rcvd = spiXfer(WaitingMsgId);
  crc = crc8(crc,rcvd);
  tmp = rcvd << 8;

  rcvd = spiXfer(WaitingMsgId);
  crc = crc8(crc,rcvd);

  byte checksum = spiXfer(WaitingMsgId);

  if (crc != checksum) return false;
  // Don't modify voltage if the checksum is bad.  This will have the effect of
  // returning the previous good value (assuming the caller did not change his voltage variable)
  *voltage_D = tmp | rcvd;  
  return true;  
}

