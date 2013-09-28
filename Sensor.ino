/* 
Sketch for Sharp IR Distance + Gesture + Ambient Light Sensor GP2AP052A00F
This I2C device can be configured to return either distance values or detection/non-detection status.
It contains four built-in photo diodes for gesture and proximity sensing, a infra-red photo diode as
ambient light sensor and an infra red-LED
 
This Library contains the basic sensor functions and can be used for writing own applications
up to 128 devices can be addressed 
Sensor address is preset to 0x45   BIN [1000101]   DEC [69]

Open Source / Public Domain

Using Arduino 1.0.1 and later versions  
http://arduino.cc/en/Main/Software
It will not work with an older version, since Wire.endTransmission() uses a parameter 
to hold or release the I2C bus.

Arduino wiring:
Board	        I2C / TWI Pins
----------------------------------------------
Uno, Ethernet	A4 (SDA), A5 (SCL)
Mega2560		20 (SDA), 21 (SCL)
Leonardo		 2 (SDA),  3 (SCL)
Due	      		20 (SDA), 21 (SCL), SDA1, SCL1
*/


#include <Wire.h>

#define COMMAND_I		0x00
#define COMMAND_II		0x01
#define PS_I			0x02
#define PS_II			0x03
#define INT_LT_LSB		0x04
#define INT_LT_MSB		0x05
#define INT_HT_LSB		0x06
#define INT_HT_MSB		0x07
/*
#define OS_DATA0_MSB	0x08
#define OS_DATA0_MSB	0x09	// why 2 times same name??
#define OS_DATA1_MSB	0x0A
#define OS_DATA1_MSB	0x0B
#define OS_DATA2_MSB	0x0C
#define OS_DATA2_MSB	0x0D
#define OS_DATA3_MSB	0x0E
#define OS_DATA3_MSB	0x0F
*/
#define OS_DATA0_LSB	0x08
#define OS_DATA0_MSB	0x09	
#define OS_DATA1_LSB	0x0A
#define OS_DATA1_MSB	0x0B
#define OS_DATA2_LSB	0x0C
#define OS_DATA2_MSB	0x0D
#define OS_DATA3_LSB	0x0E
#define OS_DATA3_MSB	0x0F
#define DATA0_LSB		0x10
#define DATA0_MSB		0x11
#define DATA1_LSB		0x12
#define DATA1_MSB		0x13
#define DATA2_LSB		0x14
#define DATA2_MSB		0x15
#define DATA3_LSB		0x16
#define DATA3_MSB		0x17
#define DATA4_LSB		0x18
#define DATA4_MSB		0x19
#define GP2A_WHO_AM_I   0x45	// Default I2C address for the Sharp Sensor is 0x45.
								// However, the scanner also reports 0x39  - investigate further . . 
#define GP2A_I2C_ADDRESS 0x45

/*
Register Map

ADDR 	Register	Function				Parameters					recommended settings
0x00	OP3 		Software shutdown 			0:shutdown,      1:operation
	OP2 		Auto shutdown/Continuous operation 	0:auto shutdown, 1:continuous operating function
	PROX 		detection/non-detection 		0:non-detection, 1:detection
	FLAG 		interrupt result 			0:non-interrupt, 1:interrupt
-------------------------------------------------------------------------------------------------------------------------			
0x01	INTVAL[1:0]     Intermittent operating 			00: 0msec, 01: 1.56msec, 10: 6.25msec, 11: 25msec
	INTSEL[2:0]     The interrupt data setting 		000:D0[13:0], 001:D1[13:0], 010:D2[13:0], 011:D3[13:0], 100:D4[13:0], 101~111:not allowed
	PIN INT 	terminal setting 			0:FLAG, 1:PS(Detection/Non-detection)
	INTTYPE 	Interrupt type setting 			0:level, 1:pulse
	RST 		Software Reset 				0:not reset, 1:reset
------------------------------------------------------------------------------------------------------------------------			
0x02	PRST[2:0] 	Number of measurement cycles 		000:once - 111:8cycles
	RES[1:0] 	Resolution 				00:14bits(6.25msec),01:12bits(1.56msec),10:10bits(0.39msec),11:8bits(0.1msec)
	RANGE[2:0] 	Maximum measurable range 		000:Ã—1 - 111:Ã—128; 001:x2
-------------------------------------------------------------------------------------------------------------------------			
0x03	IS[2:0] 	LED drive peak current setting 		000:17.5mA, 001:35.0mA, 010:70mA, 011:140mA, 111:193mA
	SUM[2:0] 	LED pulse setting 000:not allowed, 	001:Ã—2 to 111:Ã—128;   100:x16, 101:*32
	PULSE[1:0] 	LED pulse width setting 		00:9.16us, 01:6.11us, 10:4.58us, 11:3.82us
-------------------------------------------------------------------------------------------------------------------------			
0x04	 TL 		Low threshold setting(Loff) 		16bits counts setting
0x05
-------------------------------------------------------------------------------------------------------------------------			
0x06	 TH 		High threshold setting(Lon) 		16bits counts setting
0x07
-------------------------------------------------------------------------------------------------------------------------			
0x08 	OS_DATA0 	DATA0 offset count(Offset0) 		14bits counts setting
0x09
-------------------------------------------------------------------------------------------------------------------------			
0x0A 	OS_DATA1 	DATA1 offset count(Offset1) 		14bits counts setting
0x0B
-------------------------------------------------------------------------------------------------------------------------			
0x0C 	OS_DATA2 	DATA2 offset count(Offset2) 		14bits counts setting
0x0D
-------------------------------------------------------------------------------------------------------------------------			
0x0E	OS_DATA3 	DATA3 offset count(Offset3) 		14bits counts setting
0x0F
-------------------------------------------------------------------------------------------------------------------------			
0x10	D0 		DATA0 result 				14bits output data of Photodiode0
0x11
-------------------------------------------------------------------------------------------------------------------------			
0x12 	D1 		DATA1 result 				14bits output data of Photodiode1
0x13
-------------------------------------------------------------------------------------------------------------------------			
0x14	D2 		DATA2 result 				14bits output data of Photodiode2
0x15
-------------------------------------------------------------------------------------------------------------------------			
0x16	D3 		DATA3 result 				14bits output data of Photodiode3
0x17
-------------------------------------------------------------------------------------------------------------------------			
0x18	D4 		DATA0-DATA3 sum 			16bits output data of all Photodiode(D4=D0+D1+D2+D3)
0x19
-------------------------------------------------------------------------------------------------------------------------			
I2C slave address is set to 0x45
ADDR terminal setting A6 A5 A4 A3 A2 A1 A0 R/W
Slave address          1  0  0  0  1  0  1  X
R/W: Read:X=1, Write:X=0



*/
/*
Gesture Recognition
If the detected object on the right, 	D0[13:0]+D3[13:0] > D1[13:0]+D2[13:0].
If the detected object on the left, 	D0[13:0]+D3[13:0] < D1[13:0]+D2[13:0].
If the detected object on the top, 	D0[13:0]+D1[13:0] > D2[13:0]+D3[13:0].
If the detected object on the bottom, 	D0[13:0]+D1[13:0] < D2[13:0]+D3[13:0].
*/

// Defines for the bits, to be able to change 
// between bit number and binary definition.
// By using the bit number, programming the sensor 
// is like programming the AVR microcontroller.
// But instead of using "(1<<X)", or "_BV(X)", 
// the Arduino "bit(X)" is used.
#define GP2A_D0 0
#define GP2A_D1 1
#define GP2A_D2 2
#define GP2A_D3 3
#define GP2A_D4 4
#define GP2A_D5 5
#define GP2A_D6 6
#define GP2A_D7 7

// CONFIG Register

// These are the names for the bits.
// Use these only with the bit() macro.
// operation mode
#define GP2A_COMMAND_I_OP3			 GP2A_D7	// [1] 0: shutdown, 1: operation
#define GP2A_COMMAND_I_OP2     			 GP2A_D6	// [1] 0: auto shutdown, 1: continous operation

// intermittent operation
#define GP2A_COMMAND_II_INTVAL1    		 GP2A_D7	// [0] 00: 0msec, 01: 1.56msec, 10: 6.25msec, 11: 25msec
#define GP2A_COMMAND_II_INTVAL0    		 GP2A_D6	// [1] (01 recommended)
/*
Intermittent operating will be done during period set by INTVAL [1:0] register.
For GS mode, in case of INTVAL[1:0]=10(6.25msec), quiescent operation time will be after GS operation.
Although setting a longer intermittent operating period contributes to reduce average consumption current,
it makes update period and response time for detection longer as a result. Need to set it considering your actual
conditions in use
*/
// number of measurement cycles setting
#define GP2A_COMMAND_II_INTSEL2    		 GP2A_D5	// [1] 000:D0[13:0], 001:D1[13:0], 010:D2[13:0], 
#define GP2A_COMMAND_II_INTSEL1    		 GP2A_D4	// [0] 011:D3[13:0], 100:D4[13:0], 101~111:not allowed
#define GP2A_COMMAND_II_INTSEL0    		 GP2A_D3	// [0] (100 recommended)

// INT terminal setting
#define GP2A_COMMAND_II_PIN   		 	 GP2A_D2	// [0] 0:FLAG, 1:PROX(Detection/Non-detection)
#define GP2A_COMMAND_II_INTTYPE		 	 GP2A_D1	// [1] 
/*
0: level interrupt type
In this case, transition from H to L in INT terminal become occurring interrupt signal and INT
terminal will hold L level until interrupt is cleared.
______ event              ________read data
      |__________________|clear
	  
1: pulse interrupt type
In this case, L pulse output in INT terminal become occurring interrupt signal and INT terminal
will not hold L level. Therefore we need not to clear interrupt flag(FLAG). FLAG are cleared
automatically in 1 clock (about 0.39us).
______ event ________read data
      |_____|clear
	  	  
*/
// Software reset
#define GP2A_COMMAND_II_RST  		 	 GP2A_D0	// [1] 
/*
Initialize all registers by writing 1 in RST register. RST register is also initialized automatically and becomes 0.	  
*/

// Register settings for Gesture and Proximity (read only)
// Output value for detection/non-detection. There is a function which clears data by writing 0 in PROX register.
#define GP2A_COMMAND_I_PROX			     GP2A_D3  	// 0: non detection, 1: detection
// Output value for interrupt. There is a function which clears data by writing 0 in FLAG register.
#define GP2A_COMMAND_I_FLAG			     GP2A_D2  	// 0: no interrupt, 1: interrupt

// Number of measurement cycles setting: 1:000, 2:001, 3:010, 4:011, 5:100, 6:101, 7:110, 8:111
#define GP2A_PSI_PRST2 		 	 GP2A_D7			// [0]
#define GP2A_PSI_PRST1 		 	 GP2A_D6			// [0]
#define GP2A_PSI_PRST0 		 	 GP2A_D5			// [0] (recommended for gesture)

/*
Resolution/Measuring duration setting: 
Select measuring resolution and measuring duration by setting RES[1:0] register (Address 02H).
If resolution is low, measuring tolerance becomes large. Please have an adjustment at your system.
00:14bits(6.25msec),01:12bits(1.56msec),10:10bits(0.39msec),11:8bits(0.1msec)
*/
#define GP2A_PSI_RES1 		 	 GP2A_D4			// [0]	(recommended 00 and 01)
#define GP2A_PSI_RES0 		 	 GP2A_D3			// [0]

/*Maximum measurable range: RANGE[2:0]ÂiADDRESS 02HÂj
Select maximum measurable range by setting RANGE [2:0] register (Address 02H).
Detect with a set range. Maximum count value is outputted in case of incident light exceeding maximum
measurable range.
Changing maximum measurable range, detection result count is also change. In case of considering 000: Â~1 setting
as Â~1 time, count would be 1/2 times at 001: Â~2 setting, 1/4 times at 010: Â~4 setting. Adjusting detecting distance by
proximity low threshold TL[15:0] and TH[15:0]. It is necessary to set them considering the condition in the actual
use and evaluating at your system.
*/
#define GP2A_PSI_RANGE2		 	 GP2A_D2			// [0]
#define GP2A_PSI_RANGE1		 	 GP2A_D1			// [0]
#define GP2A_PSI_RANGE0		 	 GP2A_D0			// [1] (x2  recommended)

/*
LED drive peak current setting 
In case of changing this setting, the count will change correspond to the set LED drive peak current.
Please adjust detecting distance with proximity low threshold TL[15:0] and proximity high threshold TH[15:0].
LED drive peak current will depend on Vcc voltage. LED Voltage (VLED) = 2.2 to 5.5V; recommended to have independent 
Voltage supply VCC1 to reduce influence of noise. 
*/

#define GP2A_PSII_IS2 		 	 GP2A_D7				// [0]  000:17.5mA, 001:35.0mA, 010:70mA, 011:140mA, 111:193mA
#define GP2A_PSII_IS1 		 	 GP2A_D6				// [1]
#define GP2A_PSII_IS0 		 	 GP2A_D5				// [1] (recommended: 011 and 111)

/*
LED pulse count setting
If LED pulse setting is low, measuring tolerance becomes large. Please have an adjustment at your system.
Number of LED pulses can be changed from 2times to 128times.
000: NA, 001:Ã—2 times, 010:x4, 011:x8; 100:x16; 101:x32, 110:x64, 111:Ã—128;
*/
#define GP2A_PSII_SUM2 		 	 GP2A_D4				// [1]  100:x16; 101:x32
#define GP2A_PSII_SUM1 		 	 GP2A_D3				// [0]
#define GP2A_PSII_SUM0 		 	 GP2A_D2				// [0] (recommended: 100 and 101)

/*
LED pulse width setting
00: 9.16Âµs, 01: 6.11Âµs, 10: 4.58Âµs, 11: 3.82s
*/
#define GP2A_PSII_PULSE1 	 	 GP2A_D1				// [0]
#define GP2A_PSII_PULSE0	 	 GP2A_D0				// [0] (recommended: 00)

/*
5.9. Gesture and Proximity low threshold (Loff):TL[15:0]ï¼ˆADDRESS 04Hã€05Hï¼‰
Sets proximity low threshold in TL[15:0] register at PS mode.
Please set it with confirming at optical mounting condition in the actual use.
?? no recommendations ??  first just read the raw data and then decide what to do . . . 
*/
#define INT_LT_LSB_TL7  		GP2A_D7
#define INT_LT_LSB_TL6  		GP2A_D6
#define INT_LT_LSB_TL5  		GP2A_D5
#define INT_LT_LSB_TL4  		GP2A_D4
#define INT_LT_LSB_TL3  		GP2A_D3
#define INT_LT_LSB_TL2  		GP2A_D2
#define INT_LT_LSB_TL1  		GP2A_D1
#define INT_LT_LSB_TL0  		GP2A_D0

#define INT_LT_MSB_TL15  		GP2A_D7
#define INT_LT_MSB_TL14 		GP2A_D6
#define INT_LT_MSB_TL13 		GP2A_D5
#define INT_LT_MSB_TL12 		GP2A_D4
#define INT_LT_MSB_TL11 		GP2A_D3
#define INT_LT_MSB_TL10 		GP2A_D2
#define INT_LT_MSB_TL9  		GP2A_D1
#define INT_LT_MSB_TL8  		GP2A_D0

/*
5.10. Gesture and Proximity high threshold (Lon):TH[15:0]ï¼ˆADDRESS 06Hã€07Hï¼‰
Sets proximity high threshold in TH[15:0] register at PS mode.
Please set it with confirming at optical mounting condition in the actual use.
?? no recommendations ??
*/
#define INT_HT_LSB_TH7  		GP2A_D7
#define INT_HT_LSB_TH6  		GP2A_D6
#define INT_HT_LSB_TH5  		GP2A_D5
#define INT_HT_LSB_TH4  		GP2A_D4
#define INT_HT_LSB_TH3  		GP2A_D3
#define INT_HT_LSB_TH2  		GP2A_D2
#define INT_HT_LSB_TH1  		GP2A_D1
#define INT_HT_LSB_TH0  		GP2A_D0

#define INT_HT_MSB_TH15  		GP2A_D7
#define INT_HT_MSB_TH14 		GP2A_D6
#define INT_HT_MSB_TH13 		GP2A_D5
#define INT_HT_MSB_TH12 		GP2A_D4
#define INT_HT_MSB_TH11 		GP2A_D3
#define INT_HT_MSB_TH10 		GP2A_D2
#define INT_HT_MSB_TH9  		GP2A_D1
#define INT_HT_MSB_TH8  		GP2A_D0

/*
5.11. Gesture offset (Offset):OS_D0[13:0],OS_D1[13:0],OS_D2[13:0],OS_D3[13:0]ï¼ˆADDRESS 08H~0FHï¼‰
Sets proximity offset in PO[13:0] register at PS mode.
If there is Panel crosstalk, you will be able to subtract the Panel crosstalk count by using proximity offset.
Please set it with confirming at optical mounting condition in the actual use.
?? no recommendations ??
*/
#define OS_DATA0_LSB_OS_D0_7	GP2A_D7
#define OS_DATA0_LSB_OS_D0_6	GP2A_D6
#define OS_DATA0_LSB_OS_D0_5	GP2A_D5
#define OS_DATA0_LSB_OS_D0_4	GP2A_D4
#define OS_DATA0_LSB_OS_D0_3	GP2A_D3
#define OS_DATA0_LSB_OS_D0_2	GP2A_D2
#define OS_DATA0_LSB_OS_D0_1	GP2A_D1
#define OS_DATA0_LSB_OS_D0_0	GP2A_D0

#define OS_DATA0_MSB_OS_D0_13	GP2A_D5
#define OS_DATA0_MSB_OS_D0_12	GP2A_D4
#define OS_DATA0_MSB_OS_D0_11	GP2A_D3
#define OS_DATA0_MSB_OS_D0_10	GP2A_D2
#define OS_DATA0_MSB_OS_D0_9	GP2A_D1
#define OS_DATA0_MSB_OS_D0_8	GP2A_D0


#define OS_DATA1_LSB_OS_D0_7	GP2A_D7
#define OS_DATA1_LSB_OS_D0_6	GP2A_D6
#define OS_DATA1_LSB_OS_D0_5	GP2A_D5
#define OS_DATA1_LSB_OS_D0_4	GP2A_D4
#define OS_DATA1_LSB_OS_D0_3	GP2A_D3
#define OS_DATA1_LSB_OS_D0_2	GP2A_D2
#define OS_DATA1_LSB_OS_D0_1	GP2A_D1
#define OS_DATA1_LSB_OS_D0_0	GP2A_D0

#define OS_DATA1_MSB_OS_D0_13	GP2A_D5
#define OS_DATA1_MSB_OS_D0_12	GP2A_D4
#define OS_DATA1_MSB_OS_D0_11	GP2A_D3
#define OS_DATA1_MSB_OS_D0_10	GP2A_D2
#define OS_DATA1_MSB_OS_D0_9	GP2A_D1
#define OS_DATA1_MSB_OS_D0_8	GP2A_D0


#define OS_DATA2_LSB_OS_D0_7	GP2A_D7
#define OS_DATA2_LSB_OS_D0_6	GP2A_D6
#define OS_DATA2_LSB_OS_D0_5	GP2A_D5
#define OS_DATA2_LSB_OS_D0_4	GP2A_D4
#define OS_DATA2_LSB_OS_D0_3	GP2A_D3
#define OS_DATA2_LSB_OS_D0_2	GP2A_D2
#define OS_DATA2_LSB_OS_D0_1	GP2A_D1
#define OS_DATA2_LSB_OS_D0_0	GP2A_D0

#define OS_DATA2_MSB_OS_D0_13	GP2A_D5
#define OS_DATA2_MSB_OS_D0_12	GP2A_D4
#define OS_DATA2_MSB_OS_D0_11	GP2A_D3
#define OS_DATA2_MSB_OS_D0_10	GP2A_D2
#define OS_DATA2_MSB_OS_D0_9	GP2A_D1
#define OS_DATA2_MSB_OS_D0_8	GP2A_D0

#define OS_DATA3_LSB_OS_D0_7	GP2A_D7
#define OS_DATA3_LSB_OS_D0_6	GP2A_D6
#define OS_DATA3_LSB_OS_D0_5	GP2A_D5
#define OS_DATA3_LSB_OS_D0_4	GP2A_D4
#define OS_DATA3_LSB_OS_D0_3	GP2A_D3
#define OS_DATA3_LSB_OS_D0_2	GP2A_D2
#define OS_DATA3_LSB_OS_D0_1	GP2A_D1
#define OS_DATA3_LSB_OS_D0_0	GP2A_D0

#define OS_DATA3_MSB_OS_D0_13	GP2A_D5
#define OS_DATA3_MSB_OS_D0_12	GP2A_D4
#define OS_DATA3_MSB_OS_D0_11	GP2A_D3
#define OS_DATA3_MSB_OS_D0_10	GP2A_D2
#define OS_DATA3_MSB_OS_D0_9	GP2A_D1
#define OS_DATA3_MSB_OS_D0_8	GP2A_D0



/*
5.12. GS Detection result: D0[13:0], D1[13:0], D2[13:0], D3[13:0], D4 [15:0]ï¼ˆADDRESS 10H~19Hï¼‰
Detection result of gesture sensing is output to D0[13:0], D1[13:0], D2[13:0], D3[13:0] and D4[15:0] register
(Address 10H~19H).
-----------
| D3 | D2 |		Top view
-----------
| D0 | D1 |
-----------
14 bit resolution is 0 .. 16384 ??

*/
// 14bit output data of Photodiode 0
#define DATA0_LSB_D0_7			GP2A_D7
#define DATA0_LSB_D0_6			GP2A_D6
#define DATA0_LSB_D0_5			GP2A_D5
#define DATA0_LSB_D0_4			GP2A_D4
#define DATA0_LSB_D0_3			GP2A_D3
#define DATA0_LSB_D0_2			GP2A_D2
#define DATA0_LSB_D0_1			GP2A_D1
#define DATA0_LSB_D0_0			GP2A_D0

#define DATA0_MSB_SAT0			GP2A_D7
#define DATA0_MSB_D0_13			GP2A_D5
#define DATA0_MSB_D0_12			GP2A_D4
#define DATA0_MSB_D0_11			GP2A_D3
#define DATA0_MSB_D0_10			GP2A_D2
#define DATA0_MSB_D0_9			GP2A_D1
#define DATA0_MSB_D0_8			GP2A_D0

// 14bit output data of Photodiode 1
#define DATA1_LSB_D1_7			GP2A_D7
#define DATA1_LSB_D1_6			GP2A_D6
#define DATA1_LSB_D1_5			GP2A_D5
#define DATA1_LSB_D1_4			GP2A_D4
#define DATA1_LSB_D1_3			GP2A_D3
#define DATA1_LSB_D1_2			GP2A_D2
#define DATA1_LSB_D1_1			GP2A_D1
#define DATA1_LSB_D1_0			GP2A_D0

#define DATA1_MSB_SAT1			GP2A_D7
#define DATA1_MSB_D1_13			GP2A_D5
#define DATA1_MSB_D1_12			GP2A_D4
#define DATA1_MSB_D1_11			GP2A_D3
#define DATA1_MSB_D1_10			GP2A_D2
#define DATA1_MSB_D1_9			GP2A_D1
#define DATA1_MSB_D1_8			GP2A_D0

// 14bit output data of Photodiode 2
#define DATA2_LSB_D2_7			GP2A_D7
#define DATA2_LSB_D2_6			GP2A_D6
#define DATA2_LSB_D2_5			GP2A_D5
#define DATA2_LSB_D2_4			GP2A_D4
#define DATA2_LSB_D2_3			GP2A_D3
#define DATA2_LSB_D2_2			GP2A_D2
#define DATA2_LSB_D2_1			GP2A_D1
#define DATA2_LSB_D2_0			GP2A_D0

#define DATA2_MSB_SAT2			GP2A_D7
#define DATA2_MSB_D2_13			GP2A_D5
#define DATA2_MSB_D2_12			GP2A_D4
#define DATA2_MSB_D2_11			GP2A_D3
#define DATA2_MSB_D2_10			GP2A_D2
#define DATA2_MSB_D2_9			GP2A_D1
#define DATA2_MSB_D2_8			GP2A_D0

// 14bit output data of Photodiode 3
#define DATA3_LSB_D3_7			GP2A_D7
#define DATA3_LSB_D3_6			GP2A_D6
#define DATA3_LSB_D3_5			GP2A_D5
#define DATA3_LSB_D3_4			GP2A_D4
#define DATA3_LSB_D3_3			GP2A_D3
#define DATA3_LSB_D3_2			GP2A_D2
#define DATA3_LSB_D3_1			GP2A_D1
#define DATA3_LSB_D3_0			GP2A_D0

#define DATA3_MSB_SAT3			GP2A_D7
#define DATA3_MSB_D3_13			GP2A_D5
#define DATA3_MSB_D3_12			GP2A_D4
#define DATA3_MSB_D3_11			GP2A_D3
#define DATA3_MSB_D3_10			GP2A_D2
#define DATA3_MSB_D3_9			GP2A_D1
#define DATA3_MSB_D3_8			GP2A_D0

// 16bit output data of all Photodiode(D4=D0+D1+D2+D3)
#define DATA4_LSB_D4_7			GP2A_D7
#define DATA4_LSB_D4_6			GP2A_D6
#define DATA4_LSB_D4_5			GP2A_D5
#define DATA4_LSB_D4_4			GP2A_D4
#define DATA4_LSB_D4_3			GP2A_D3
#define DATA4_LSB_D4_2			GP2A_D2
#define DATA4_LSB_D4_1			GP2A_D1
#define DATA4_LSB_D4_0			GP2A_D0

#define DATA4_MSB_D4_15			GP2A_D7
#define DATA4_MSB_D4_14			GP2A_D6
#define DATA4_MSB_D4_13			GP2A_D5
#define DATA4_MSB_D4_12			GP2A_D4
#define DATA4_MSB_D4_11			GP2A_D3
#define DATA4_MSB_D4_10			GP2A_D2
#define DATA4_MSB_D4_9			GP2A_D1
#define DATA4_MSB_D4_8			GP2A_D0

/*

Detection result is defined as follows:
Detection result(D0[13:0]) = Raw count(D0[13:0], include panel crosstalk) â€“ Offset(OS_D0[13:0])
Detection result(D1[13:0]) = Raw count(D1[13:0], include panel crosstalk) â€“ Offset(OS_D1[13:0])
Detection result(D2[13:0]) = Raw count(D2[13:0], include panel crosstalk) â€“ Offset(OS_D2[13:0])
Detection result(D3[13:0]) = Raw count(D3[13:0], include panel crosstalk) â€“ Offset(OS_D3[13:0])

Gesture detection:
If the detected object on the right, D0[13:0]+D3[13:0] > D1[13:0]+D2[13:0].
If the detected object on the left, D0[13:0] +D3[13:0] < D1[13:0]+D2[13:0].
If the detected object on the top, D0[13:0]+D1[13:0] > D2[13:0]+D3[13:0].
If the detected object on the bottom, D0[13:0]+D1[13:0] < D2[13:0]+D3[13:0].
*/






void setup()
{      
  int error;
  uint8_t c;


  Serial.begin(9600);
  Serial.println(F("Sharp Gesture & Ambient Light sensor GP2AP052A00F"));
  Serial.println(F("September 2013"));

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();
 
  error = GP2A_read (GP2A_WHO_AM_I, &c, 1);  // doesn't work yet !!!
 /* error = GP2A_read (GP2A_WHO_AM_I, address, 1);  // address is 69 DEC
  byte ERROr, address; 
  Wire.beginTransmission(69);
  delay(10);
    ERROr = Wire.endTransmission();
      if (ERROr == 0)
    {
      Serial.print(F("Sensor found at address 0x"));
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
      	  }
		  */
 
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC); 
  Serial.println();

  Serial.println(F("writing initial settings"));

// *** start-up settings for the sensor 
// Clear the 'sleep' bit to start the sensor.
	GP2A_write_reg (GP2A_COMMAND_I_OP3, 1);

	// select either auto shut down ...
	// GP2A_write_reg (GP2A_COMMAND_I_OP2, 0); 
	// or select continous operation
	GP2A_write_reg (GP2A_COMMAND_I_OP2, 1);
  
// intermittent operation: 01: 1.56msec recommended
	GP2A_write_reg (GP2A_COMMAND_II_INTVAL1, 0);    // [0] 00: 0msec, 01: 1.56msec, 10: 6.25msec, 11: 25msec
	GP2A_write_reg (GP2A_COMMAND_II_INTVAL0, 1);	// [1] (01 recommended) 

// number of measurement cycles setting
	GP2A_write_reg (GP2A_COMMAND_II_INTSEL2, 1); 	// [1] 000:D0[13:0], 001:D1[13:0], 010:D2[13:0], 
	GP2A_write_reg (GP2A_COMMAND_II_INTSEL1, 0); 	// [0] 011:D3[13:0], 100:D4[13:0], 101~111:not allowed
	GP2A_write_reg (GP2A_COMMAND_II_INTSEL0, 0); 	// [0] (100 recommended)

// INT terminal setting
	GP2A_write_reg (GP2A_COMMAND_II_PIN, 0); 		// [0] 0:FLAG, 1:PROX(Detection/Non-detection)
	GP2A_write_reg (GP2A_COMMAND_II_INTTYPE, 1); 	// [1]

// Number of measurement cycles setting: 1:000, 2:001, 3:010, 4:011, 5:100, 6:101, 7:110, 8:111
	GP2A_write_reg (GP2A_PSI_PRST2, 0);				// [0]
	GP2A_write_reg (GP2A_PSI_PRST1, 0);				// [0]
	GP2A_write_reg (GP2A_PSI_PRST0, 0);				// [0] (recommended for gesture) 

//	Resolution/Measuring duration setting: 
//  00:14bits(6.25msec),01:12bits(1.56msec),10:10bits(0.39msec),11:8bits(0.1msec)
	GP2A_write_reg (GP2A_PSI_RES1, 0);				// [0]
	GP2A_write_reg (GP2A_PSI_RES1, 0);				// [0]	(recommended 00 and 01)

//	Maximum measurable range: x1, x2, x4, x8, x16, x32, x64, x128
//  In case of considering 000: Âx1 setting as x1 time, count would be 1/2 times at 001: x2 setting, 
//  1/4 times at 010: Âx4 setting. Adjusting detecting distance by proximity low threshold TL[15:0] and TH[15:0]. 
//  It is necessary to set them considering the condition in the actual use and evaluating at your system.
	GP2A_write_reg (GP2A_PSI_RANGE2, 0);			// [0]
	GP2A_write_reg (GP2A_PSI_RANGE1, 0);			// [0]
	GP2A_write_reg (GP2A_PSI_RANGE0, 1);			// [1] (x2  recommended)

//  LED drive peak current setting 
//  LED drive peak current will depend on Vcc voltage. LED Voltage (VLED) = 2.2 to 5.5V; 
//  recommended to have independent Voltage supply VCC1 to reduce influence of noise. 
	GP2A_write_reg (GP2A_PSII_IS2, 0);				// [0]  000:17.5mA, 001:35.0mA, 010:70mA, 011:140mA, 111:193mA
	GP2A_write_reg (GP2A_PSII_IS1, 1);				// [1] 
	GP2A_write_reg (GP2A_PSII_IS0, 1);				// [1] (recommended: 011 and 111)

//  LED pulse count setting
//  If LED pulse setting is low, measuring tolerance becomes large. Please have an adjustment at your system.
//  Number of LED pulses can be changed from 2times to 128times.
//  000: NA, 001:Ã—2 times, 010:x4, 011:x8; 100:x16; 101:x32, 110:x64, 111:Ã—128;
	GP2A_write_reg (GP2A_PSII_SUM2, 1);				// [1]  100:x16; 101:x32
	GP2A_write_reg (GP2A_PSII_SUM1, 1);				// [0]  
	GP2A_write_reg (GP2A_PSII_SUM0, 1);				// [0] (recommended: 100 and 101)

//  LED pulse width setting
//  00: 9.16Âµs, 01: 6.11Âµs, 10: 4.58Âµs, 11: 3.82s
	GP2A_write_reg (GP2A_PSII_PULSE1, 0);			// [0]
	GP2A_write_reg (GP2A_PSII_PULSE0, 0);			// [0] (recommended: 00)
	
	
    Serial.println(F("Set-up finished"));
	delay (1000);
}









void loop ()
{

I2C_scanner_tool ();
/*
	int prox = GP2A_read_reg (GP2A_COMMAND_I_PROX, 1);
	if (prox == 0 ) {
	Serial.println (F("nothing"));
	}
	else   {
	Serial.println (F("DETECTION"));
	}
	delay(100);

	int flag = GP2A_read_reg (GP2A_COMMAND_I_FLAG, 1);
	if (flag == 0 ) {
	Serial.println (F("nothing"));
	}
	else   {
	Serial.println (F("INTERRUPT"));
	}
	delay(100);	
*/

   
  }


// --------------------------------------------------------
// GP2A_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int GP2A_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(GP2A_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(GP2A_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}
// --------------------------------------------------------
// GP2A_read_reg
//
// An extra function to read a single register.
// It is just a wrapper around the GP2A_read()
// function, and it is only a convenient function
// to make it easier to read a single register.
//
int GP2A_read_reg(int reg, uint8_t data)
{
  int i, n, error, size ;

  Wire.beginTransmission(GP2A_I2C_ADDRESS);
  n = Wire.write(reg);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(GP2A_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
  
  error = GP2A_read(reg, &data, 1);

  return (error);
}


// --------------------------------------------------------
// GP2A_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function GP2A_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   GP2A_write (GP2A_PWR_MGMT_1, &c, 1);
//
int GP2A_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(GP2A_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// GP2A_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int GP2A_write_reg(int reg, uint8_t data)
{
  int error;

  error = GP2A_write(reg, &data, 1);

  return (error);
}

void I2C_scanner_tool() 

{
  byte error, address;
  int nDevices;
  Serial.println("\nI2C Scanner");
  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address ");
      if (address<16) 
        
       
	  Serial.print(address,BIN);
	  Serial.print("\t");
	  Serial.print("0x");
	  Serial.print(address,HEX);
	  Serial.print("\t");
	  Serial.print(address,DEC);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(2000);           // wait 2 seconds for next scan
}  

void setI2C_address ()
{

  Wire.beginTransmission(COMMAND_I);
  Wire.write(2); // set the device slave address to 01 Dec - - or any other desired address
  // ************  THIS DOES NOT WORK YET  ************************************************
/*
ADDR terminal setting A6 A5 A4 A3 A2 A1 A0 R/W
Slave address          1  0  0  0  1  0  1  X     
R/W:   	Read: X=1		// 8Bh   or 0x8B
		Write:X=0		// 8Ah   or 0x8A
		Write slave address 1: 00000010 :  0x02
*/
  delay (200);			  // allow some time to write to the registers
 Wire.endTransmission();    // hold the I2C-bus
Serial.println(F("Slave Address 01 written"));
} 






/* alternative I2C protocol
  byte num;
  
  // set the 24C256 eeprom address to 0
  Wire.beginTransmission(80);
  Wire.send(0);  // address low byte
  Wire.send(0);  // address high byte
  Wire.endTransmission();
  
  // read 1 byte, from address 0
  Wire.requestFrom(80, 1);
  while(Wire.available()) {
    num = Wire.receive();
  }
  Serial.print("num = ");
  Serial.println(num, DEC);
  
  // increment num
  num = num + 1;
  
  // write "num" to 24C256 eeprom at address zero
  Wire.beginTransmission(80);
  Wire.send(0);    // address low byte
  Wire.send(0);    // address high byte
  Wire.send(num);  // any more send starts writing
  Wire.endTransmission();
  
  // next time loop runs, it should retrieve the
  // same number it wrote last time... even if you
  // shut off the power
  delay(5000);
}
*/



/*
Sample code for a MPU-6050 accelerometer
http://playground.arduino.cc//Main/MPU-6050?action=sourceblock&num=1

// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// By arduino.cc user "Krodal".
//
// June 2012
//      first version
// July 2013 
//      The 'int' in the union for the x,y,z
//      changed into int16_t to be compatible
//      with Arduino Due.
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version, 
// since Wire.endTransmission() uses a parameter 
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-6000 and MPU-6050 Product Specification",
//     PS-MPU-6000A.pdf
//   - "MPU-6000 and MPU-6050 Register Map and Descriptions",
//     RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
//   - "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
//     AN-MPU-6000EVB.pdf
// 
// The accuracy is 16-bits.
//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
// 

#include <Wire.h>


// The name of the sensor is "MPU-6050".
// For program code, I omit the '-', 
// therefor I use the name "MPU6050....".


// Register names according to the datasheet.
// According to the InvenSense document 
// "MPU-6000 and MPU-6050 Register Map 
// and Descriptions Revision 3.2", there are no registers
// at 0x02 ... 0x18, but according other information 
// the registers in that unknown area are for gain 
// and offsets.
// 
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R


// Defines for the bits, to be able to change 
// between bit number and binary definition.
// By using the bit number, programming the sensor 
// is like programming the AVR microcontroller.
// But instead of using "(1<<X)", or "_BV(X)", 
// the Arduino "bit(X)" is used.
#define MPU6050_D0 0
#define MPU6050_D1 1
#define MPU6050_D2 2
#define MPU6050_D3 3
#define MPU6050_D4 4
#define MPU6050_D5 5
#define MPU6050_D6 6
#define MPU6050_D7 7

// AUX_VDDIO Register
#define MPU6050_AUX_VDDIO MPU6050_D7  // I2C high: 1=VDD, 0=VLOGIC

// CONFIG Register
// DLPF is Digital Low Pass Filter for both gyro and accelerometers.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DLPF_CFG0     MPU6050_D0
#define MPU6050_DLPF_CFG1     MPU6050_D1
#define MPU6050_DLPF_CFG2     MPU6050_D2
#define MPU6050_EXT_SYNC_SET0 MPU6050_D3
#define MPU6050_EXT_SYNC_SET1 MPU6050_D4
#define MPU6050_EXT_SYNC_SET2 MPU6050_D5

// Combined definitions for the EXT_SYNC_SET values
#define MPU6050_EXT_SYNC_SET_0 (0)
#define MPU6050_EXT_SYNC_SET_1 (bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_2 (bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_3 (bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_4 (bit(MPU6050_EXT_SYNC_SET2))
#define MPU6050_EXT_SYNC_SET_5 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_6 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_7 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))

// Alternative names for the combined definitions.
#define MPU6050_EXT_SYNC_DISABLED     MPU6050_EXT_SYNC_SET_0
#define MPU6050_EXT_SYNC_TEMP_OUT_L   MPU6050_EXT_SYNC_SET_1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L  MPU6050_EXT_SYNC_SET_2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L  MPU6050_EXT_SYNC_SET_3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L  MPU6050_EXT_SYNC_SET_4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L MPU6050_EXT_SYNC_SET_5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L MPU6050_EXT_SYNC_SET_6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L MPU6050_EXT_SYNC_SET_7

// Combined definitions for the DLPF_CFG values
#define MPU6050_DLPF_CFG_0 (0)
#define MPU6050_DLPF_CFG_1 (bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_2 (bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_3 (bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_4 (bit(MPU6050_DLPF_CFG2))
#define MPU6050_DLPF_CFG_5 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_6 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_7 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))

// Alternative names for the combined definitions
// This name uses the bandwidth (Hz) for the accelometer,
// for the gyro the bandwidth is almost the same.
#define MPU6050_DLPF_260HZ    MPU6050_DLPF_CFG_0
#define MPU6050_DLPF_184HZ    MPU6050_DLPF_CFG_1
#define MPU6050_DLPF_94HZ     MPU6050_DLPF_CFG_2
#define MPU6050_DLPF_44HZ     MPU6050_DLPF_CFG_3
#define MPU6050_DLPF_21HZ     MPU6050_DLPF_CFG_4
#define MPU6050_DLPF_10HZ     MPU6050_DLPF_CFG_5
#define MPU6050_DLPF_5HZ      MPU6050_DLPF_CFG_6
#define MPU6050_DLPF_RESERVED MPU6050_DLPF_CFG_7

// GYRO_CONFIG Register
// The XG_ST, YG_ST, ZG_ST are bits for selftest.
// The FS_SEL sets the range for the gyro.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_FS_SEL0 MPU6050_D3
#define MPU6050_FS_SEL1 MPU6050_D4
#define MPU6050_ZG_ST   MPU6050_D5
#define MPU6050_YG_ST   MPU6050_D6
#define MPU6050_XG_ST   MPU6050_D7

// Combined definitions for the FS_SEL values
#define MPU6050_FS_SEL_0 (0)
#define MPU6050_FS_SEL_1 (bit(MPU6050_FS_SEL0))
#define MPU6050_FS_SEL_2 (bit(MPU6050_FS_SEL1))
#define MPU6050_FS_SEL_3 (bit(MPU6050_FS_SEL1)|bit(MPU6050_FS_SEL0))

// Alternative names for the combined definitions
// The name uses the range in degrees per second.
#define MPU6050_FS_SEL_250  MPU6050_FS_SEL_0
#define MPU6050_FS_SEL_500  MPU6050_FS_SEL_1
#define MPU6050_FS_SEL_1000 MPU6050_FS_SEL_2
#define MPU6050_FS_SEL_2000 MPU6050_FS_SEL_3

// ACCEL_CONFIG Register
// The XA_ST, YA_ST, ZA_ST are bits for selftest.
// The AFS_SEL sets the range for the accelerometer.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_ACCEL_HPF0 MPU6050_D0
#define MPU6050_ACCEL_HPF1 MPU6050_D1
#define MPU6050_ACCEL_HPF2 MPU6050_D2
#define MPU6050_AFS_SEL0   MPU6050_D3
#define MPU6050_AFS_SEL1   MPU6050_D4
#define MPU6050_ZA_ST      MPU6050_D5
#define MPU6050_YA_ST      MPU6050_D6
#define MPU6050_XA_ST      MPU6050_D7

// Combined definitions for the ACCEL_HPF values
#define MPU6050_ACCEL_HPF_0 (0)
#define MPU6050_ACCEL_HPF_1 (bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_2 (bit(MPU6050_ACCEL_HPF1))
#define MPU6050_ACCEL_HPF_3 (bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_4 (bit(MPU6050_ACCEL_HPF2))
#define MPU6050_ACCEL_HPF_7 (bit(MPU6050_ACCEL_HPF2)|bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))

// Alternative names for the combined definitions
// The name uses the Cut-off frequency.
#define MPU6050_ACCEL_HPF_RESET  MPU6050_ACCEL_HPF_0
#define MPU6050_ACCEL_HPF_5HZ    MPU6050_ACCEL_HPF_1
#define MPU6050_ACCEL_HPF_2_5HZ  MPU6050_ACCEL_HPF_2
#define MPU6050_ACCEL_HPF_1_25HZ MPU6050_ACCEL_HPF_3
#define MPU6050_ACCEL_HPF_0_63HZ MPU6050_ACCEL_HPF_4
#define MPU6050_ACCEL_HPF_HOLD   MPU6050_ACCEL_HPF_7

// Combined definitions for the AFS_SEL values
#define MPU6050_AFS_SEL_0 (0)
#define MPU6050_AFS_SEL_1 (bit(MPU6050_AFS_SEL0))
#define MPU6050_AFS_SEL_2 (bit(MPU6050_AFS_SEL1))
#define MPU6050_AFS_SEL_3 (bit(MPU6050_AFS_SEL1)|bit(MPU6050_AFS_SEL0))

// Alternative names for the combined definitions
// The name uses the full scale range for the accelerometer.
#define MPU6050_AFS_SEL_2G  MPU6050_AFS_SEL_0
#define MPU6050_AFS_SEL_4G  MPU6050_AFS_SEL_1
#define MPU6050_AFS_SEL_8G  MPU6050_AFS_SEL_2
#define MPU6050_AFS_SEL_16G MPU6050_AFS_SEL_3

// FIFO_EN Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_SLV0_FIFO_EN  MPU6050_D0
#define MPU6050_SLV1_FIFO_EN  MPU6050_D1
#define MPU6050_SLV2_FIFO_EN  MPU6050_D2
#define MPU6050_ACCEL_FIFO_EN MPU6050_D3
#define MPU6050_ZG_FIFO_EN    MPU6050_D4
#define MPU6050_YG_FIFO_EN    MPU6050_D5
#define MPU6050_XG_FIFO_EN    MPU6050_D6
#define MPU6050_TEMP_FIFO_EN  MPU6050_D7

// I2C_MST_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_MST_CLK0  MPU6050_D0
#define MPU6050_I2C_MST_CLK1  MPU6050_D1
#define MPU6050_I2C_MST_CLK2  MPU6050_D2
#define MPU6050_I2C_MST_CLK3  MPU6050_D3
#define MPU6050_I2C_MST_P_NSR MPU6050_D4
#define MPU6050_SLV_3_FIFO_EN MPU6050_D5
#define MPU6050_WAIT_FOR_ES   MPU6050_D6
#define MPU6050_MULT_MST_EN   MPU6050_D7

// Combined definitions for the I2C_MST_CLK
#define MPU6050_I2C_MST_CLK_0 (0)
#define MPU6050_I2C_MST_CLK_1  (bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_2  (bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_3  (bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_4  (bit(MPU6050_I2C_MST_CLK2))
#define MPU6050_I2C_MST_CLK_5  (bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_6  (bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_7  (bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_8  (bit(MPU6050_I2C_MST_CLK3))
#define MPU6050_I2C_MST_CLK_9  (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_10 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_11 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_12 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2))
#define MPU6050_I2C_MST_CLK_13 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK0))
#define MPU6050_I2C_MST_CLK_14 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1))
#define MPU6050_I2C_MST_CLK_15 (bit(MPU6050_I2C_MST_CLK3)|bit(MPU6050_I2C_MST_CLK2)|bit(MPU6050_I2C_MST_CLK1)|bit(MPU6050_I2C_MST_CLK0))

// Alternative names for the combined definitions
// The names uses I2C Master Clock Speed in kHz.
#define MPU6050_I2C_MST_CLK_348KHZ MPU6050_I2C_MST_CLK_0
#define MPU6050_I2C_MST_CLK_333KHZ MPU6050_I2C_MST_CLK_1
#define MPU6050_I2C_MST_CLK_320KHZ MPU6050_I2C_MST_CLK_2
#define MPU6050_I2C_MST_CLK_308KHZ MPU6050_I2C_MST_CLK_3
#define MPU6050_I2C_MST_CLK_296KHZ MPU6050_I2C_MST_CLK_4
#define MPU6050_I2C_MST_CLK_286KHZ MPU6050_I2C_MST_CLK_5
#define MPU6050_I2C_MST_CLK_276KHZ MPU6050_I2C_MST_CLK_6
#define MPU6050_I2C_MST_CLK_267KHZ MPU6050_I2C_MST_CLK_7
#define MPU6050_I2C_MST_CLK_258KHZ MPU6050_I2C_MST_CLK_8
#define MPU6050_I2C_MST_CLK_500KHZ MPU6050_I2C_MST_CLK_9
#define MPU6050_I2C_MST_CLK_471KHZ MPU6050_I2C_MST_CLK_10
#define MPU6050_I2C_MST_CLK_444KHZ MPU6050_I2C_MST_CLK_11
#define MPU6050_I2C_MST_CLK_421KHZ MPU6050_I2C_MST_CLK_12
#define MPU6050_I2C_MST_CLK_400KHZ MPU6050_I2C_MST_CLK_13
#define MPU6050_I2C_MST_CLK_381KHZ MPU6050_I2C_MST_CLK_14
#define MPU6050_I2C_MST_CLK_364KHZ MPU6050_I2C_MST_CLK_15

// I2C_SLV0_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_RW MPU6050_D7

// I2C_SLV0_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV0_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV0_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV0_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV0_GRP     MPU6050_D4
#define MPU6050_I2C_SLV0_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV0_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV0_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV0_LEN_MASK 0x0F

// I2C_SLV1_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV1_RW MPU6050_D7

// I2C_SLV1_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV1_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV1_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV1_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV1_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV1_GRP     MPU6050_D4
#define MPU6050_I2C_SLV1_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV1_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV1_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV1_LEN_MASK 0x0F

// I2C_SLV2_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV2_RW MPU6050_D7

// I2C_SLV2_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV2_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV2_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV2_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV2_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV2_GRP     MPU6050_D4
#define MPU6050_I2C_SLV2_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV2_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV2_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV2_LEN_MASK 0x0F

// I2C_SLV3_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV3_RW MPU6050_D7

// I2C_SLV3_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV3_LEN0    MPU6050_D0
#define MPU6050_I2C_SLV3_LEN1    MPU6050_D1
#define MPU6050_I2C_SLV3_LEN2    MPU6050_D2
#define MPU6050_I2C_SLV3_LEN3    MPU6050_D3
#define MPU6050_I2C_SLV3_GRP     MPU6050_D4
#define MPU6050_I2C_SLV3_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV3_BYTE_SW MPU6050_D6
#define MPU6050_I2C_SLV3_EN      MPU6050_D7

// A mask for the length
#define MPU6050_I2C_SLV3_LEN_MASK 0x0F

// I2C_SLV4_ADDR Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV4_RW MPU6050_D7

// I2C_SLV4_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_MST_DLY0     MPU6050_D0
#define MPU6050_I2C_MST_DLY1     MPU6050_D1
#define MPU6050_I2C_MST_DLY2     MPU6050_D2
#define MPU6050_I2C_MST_DLY3     MPU6050_D3
#define MPU6050_I2C_MST_DLY4     MPU6050_D4
#define MPU6050_I2C_SLV4_REG_DIS MPU6050_D5
#define MPU6050_I2C_SLV4_INT_EN  MPU6050_D6
#define MPU6050_I2C_SLV4_EN      MPU6050_D7

// A mask for the delay
#define MPU6050_I2C_MST_DLY_MASK 0x1F

// I2C_MST_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_NACK MPU6050_D0
#define MPU6050_I2C_SLV1_NACK MPU6050_D1
#define MPU6050_I2C_SLV2_NACK MPU6050_D2
#define MPU6050_I2C_SLV3_NACK MPU6050_D3
#define MPU6050_I2C_SLV4_NACK MPU6050_D4
#define MPU6050_I2C_LOST_ARB  MPU6050_D5
#define MPU6050_I2C_SLV4_DONE MPU6050_D6
#define MPU6050_PASS_THROUGH  MPU6050_D7

// I2C_PIN_CFG Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_CLKOUT_EN       MPU6050_D0
#define MPU6050_I2C_BYPASS_EN   MPU6050_D1
#define MPU6050_FSYNC_INT_EN    MPU6050_D2
#define MPU6050_FSYNC_INT_LEVEL MPU6050_D3
#define MPU6050_INT_RD_CLEAR    MPU6050_D4
#define MPU6050_LATCH_INT_EN    MPU6050_D5
#define MPU6050_INT_OPEN        MPU6050_D6
#define MPU6050_INT_LEVEL       MPU6050_D7

// INT_ENABLE Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DATA_RDY_EN    MPU6050_D0
#define MPU6050_I2C_MST_INT_EN MPU6050_D3
#define MPU6050_FIFO_OFLOW_EN  MPU6050_D4
#define MPU6050_ZMOT_EN        MPU6050_D5
#define MPU6050_MOT_EN         MPU6050_D6
#define MPU6050_FF_EN          MPU6050_D7

// INT_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DATA_RDY_INT   MPU6050_D0
#define MPU6050_I2C_MST_INT    MPU6050_D3
#define MPU6050_FIFO_OFLOW_INT MPU6050_D4
#define MPU6050_ZMOT_INT       MPU6050_D5
#define MPU6050_MOT_INT        MPU6050_D6
#define MPU6050_FF_INT         MPU6050_D7

// MOT_DETECT_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_MOT_ZRMOT MPU6050_D0
#define MPU6050_MOT_ZPOS  MPU6050_D2
#define MPU6050_MOT_ZNEG  MPU6050_D3
#define MPU6050_MOT_YPOS  MPU6050_D4
#define MPU6050_MOT_YNEG  MPU6050_D5
#define MPU6050_MOT_XPOS  MPU6050_D6
#define MPU6050_MOT_XNEG  MPU6050_D7

// IC2_MST_DELAY_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_DLY_EN MPU6050_D0
#define MPU6050_I2C_SLV1_DLY_EN MPU6050_D1
#define MPU6050_I2C_SLV2_DLY_EN MPU6050_D2
#define MPU6050_I2C_SLV3_DLY_EN MPU6050_D3
#define MPU6050_I2C_SLV4_DLY_EN MPU6050_D4
#define MPU6050_DELAY_ES_SHADOW MPU6050_D7

// SIGNAL_PATH_RESET Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_TEMP_RESET  MPU6050_D0
#define MPU6050_ACCEL_RESET MPU6050_D1
#define MPU6050_GYRO_RESET  MPU6050_D2

// MOT_DETECT_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_MOT_COUNT0      MPU6050_D0
#define MPU6050_MOT_COUNT1      MPU6050_D1
#define MPU6050_FF_COUNT0       MPU6050_D2
#define MPU6050_FF_COUNT1       MPU6050_D3
#define MPU6050_ACCEL_ON_DELAY0 MPU6050_D4
#define MPU6050_ACCEL_ON_DELAY1 MPU6050_D5

// Combined definitions for the MOT_COUNT
#define MPU6050_MOT_COUNT_0 (0)
#define MPU6050_MOT_COUNT_1 (bit(MPU6050_MOT_COUNT0))
#define MPU6050_MOT_COUNT_2 (bit(MPU6050_MOT_COUNT1))
#define MPU6050_MOT_COUNT_3 (bit(MPU6050_MOT_COUNT1)|bit(MPU6050_MOT_COUNT0))

// Alternative names for the combined definitions
#define MPU6050_MOT_COUNT_RESET MPU6050_MOT_COUNT_0

// Combined definitions for the FF_COUNT
#define MPU6050_FF_COUNT_0 (0)
#define MPU6050_FF_COUNT_1 (bit(MPU6050_FF_COUNT0))
#define MPU6050_FF_COUNT_2 (bit(MPU6050_FF_COUNT1))
#define MPU6050_FF_COUNT_3 (bit(MPU6050_FF_COUNT1)|bit(MPU6050_FF_COUNT0))

// Alternative names for the combined definitions
#define MPU6050_FF_COUNT_RESET MPU6050_FF_COUNT_0

// Combined definitions for the ACCEL_ON_DELAY
#define MPU6050_ACCEL_ON_DELAY_0 (0)
#define MPU6050_ACCEL_ON_DELAY_1 (bit(MPU6050_ACCEL_ON_DELAY0))
#define MPU6050_ACCEL_ON_DELAY_2 (bit(MPU6050_ACCEL_ON_DELAY1))
#define MPU6050_ACCEL_ON_DELAY_3 (bit(MPU6050_ACCEL_ON_DELAY1)|bit(MPU6050_ACCEL_ON_DELAY0))

// Alternative names for the ACCEL_ON_DELAY
#define MPU6050_ACCEL_ON_DELAY_0MS MPU6050_ACCEL_ON_DELAY_0
#define MPU6050_ACCEL_ON_DELAY_1MS MPU6050_ACCEL_ON_DELAY_1
#define MPU6050_ACCEL_ON_DELAY_2MS MPU6050_ACCEL_ON_DELAY_2
#define MPU6050_ACCEL_ON_DELAY_3MS MPU6050_ACCEL_ON_DELAY_3

// USER_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_SIG_COND_RESET MPU6050_D0
#define MPU6050_I2C_MST_RESET  MPU6050_D1
#define MPU6050_FIFO_RESET     MPU6050_D2
#define MPU6050_I2C_IF_DIS     MPU6050_D4   // must be 0 for MPU-6050
#define MPU6050_I2C_MST_EN     MPU6050_D5
#define MPU6050_FIFO_EN        MPU6050_D6

// PWR_MGMT_1 Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_CLKSEL0      MPU6050_D0
#define MPU6050_CLKSEL1      MPU6050_D1
#define MPU6050_CLKSEL2      MPU6050_D2
#define MPU6050_TEMP_DIS     MPU6050_D3    // 1: disable temperature sensor
#define MPU6050_CYCLE        MPU6050_D5    // 1: sample and sleep
#define MPU6050_SLEEP        MPU6050_D6    // 1: sleep mode
#define MPU6050_DEVICE_RESET MPU6050_D7    // 1: reset to default values

// Combined definitions for the CLKSEL
#define MPU6050_CLKSEL_0 (0)
#define MPU6050_CLKSEL_1 (bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_2 (bit(MPU6050_CLKSEL1))
#define MPU6050_CLKSEL_3 (bit(MPU6050_CLKSEL1)|bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_4 (bit(MPU6050_CLKSEL2))
#define MPU6050_CLKSEL_5 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_6 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL1))
#define MPU6050_CLKSEL_7 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL1)|bit(MPU6050_CLKSEL0))

// Alternative names for the combined definitions
#define MPU6050_CLKSEL_INTERNAL    MPU6050_CLKSEL_0
#define MPU6050_CLKSEL_X           MPU6050_CLKSEL_1
#define MPU6050_CLKSEL_Y           MPU6050_CLKSEL_2
#define MPU6050_CLKSEL_Z           MPU6050_CLKSEL_3
#define MPU6050_CLKSEL_EXT_32KHZ   MPU6050_CLKSEL_4
#define MPU6050_CLKSEL_EXT_19_2MHZ MPU6050_CLKSEL_5
#define MPU6050_CLKSEL_RESERVED    MPU6050_CLKSEL_6
#define MPU6050_CLKSEL_STOP        MPU6050_CLKSEL_7

// PWR_MGMT_2 Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_STBY_ZG       MPU6050_D0
#define MPU6050_STBY_YG       MPU6050_D1
#define MPU6050_STBY_XG       MPU6050_D2
#define MPU6050_STBY_ZA       MPU6050_D3
#define MPU6050_STBY_YA       MPU6050_D4
#define MPU6050_STBY_XA       MPU6050_D5
#define MPU6050_LP_WAKE_CTRL0 MPU6050_D6
#define MPU6050_LP_WAKE_CTRL1 MPU6050_D7

// Combined definitions for the LP_WAKE_CTRL
#define MPU6050_LP_WAKE_CTRL_0 (0)
#define MPU6050_LP_WAKE_CTRL_1 (bit(MPU6050_LP_WAKE_CTRL0))
#define MPU6050_LP_WAKE_CTRL_2 (bit(MPU6050_LP_WAKE_CTRL1))
#define MPU6050_LP_WAKE_CTRL_3 (bit(MPU6050_LP_WAKE_CTRL1)|bit(MPU6050_LP_WAKE_CTRL0))

// Alternative names for the combined definitions
// The names uses the Wake-up Frequency.
#define MPU6050_LP_WAKE_1_25HZ MPU6050_LP_WAKE_CTRL_0
#define MPU6050_LP_WAKE_2_5HZ  MPU6050_LP_WAKE_CTRL_1
#define MPU6050_LP_WAKE_5HZ    MPU6050_LP_WAKE_CTRL_2
#define MPU6050_LP_WAKE_10HZ   MPU6050_LP_WAKE_CTRL_3


// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
#define MPU6050_I2C_ADDRESS 0x68


// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of 
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte 
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally, 
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct 
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};


void setup()
{      
  int error;
  uint8_t c;


  Serial.begin(9600);
  Serial.println(F("InvenSense MPU-6050"));
  Serial.println(F("June 2012"));

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();
                                                                             ***************************************

  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  Serial.print(F("PWR_MGMT_2 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}


void loop()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;


  Serial.println(F(""));
  Serial.println(F("MPU-6050"));

  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  Serial.print(F("Read accel, temp and gyro, error = "));
  Serial.println(error,DEC);


  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


  // Print the raw acceleration values

  Serial.print(F("accel x,y,z: "));
  Serial.print(accel_t_gyro.value.x_accel, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.y_accel, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.z_accel, DEC);
  Serial.println(F(""));


  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet: 
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412

  Serial.print(F("temperature: "));
  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  Serial.print(dT, 3);
  Serial.print(F(" degrees Celsius"));
  Serial.println(F(""));


  // Print the raw gyro values.

  Serial.print(F("gyro x,y,z : "));
  Serial.print(accel_t_gyro.value.x_gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.y_gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.z_gyro, DEC);
  Serial.print(F(", "));
  Serial.println(F(""));

  delay(1000);
}


// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}
*/ 
