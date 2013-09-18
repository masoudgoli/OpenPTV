OpenPTV
=======

The sketches are based on the work of OpenPTV.org for a Segway clone.

The self balancer described here has a few significant changes to the OpenPTV: 
1. a throttle grip allows to control the speed just like a motor cycle
2. the direction is controlled with a standard pot (OpenPTV uses a pot with middle-zero-position)
3. the communication between Display Board and Main Board is SPI

2 Arduino sketches:
MainBoard     -  Arduino Mega
DisplayBoard  -  Wattuino (Arduino Pro Mini with ATmega328P) www.watterott.com

Oszilloscope  -  Processing Sketch to visualize all sensor readings

Note: all credentials for the Open Source sketches and snipplets I have found on the net will be given later on

