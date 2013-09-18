OpenPTV
=======

These Processing sketches visualize all sensor readings of the OpenPTV on a computer screen. 

Copy all files in one directory and create the font Calibri-14.vlw (must be also in the same directory).
Uncomment the Serial.print section for the sensor readings in the Arduino main sketch.

Be aware that Serial.print takes a long time and the PID settings for balancing are different once you 
disconnect (and take out the Serial.print) in the main sketch.
