/*
  This is a sketch for getting and receiving information from a 
  balancing bot and also to control it..
  http://forum.arduino.cc/index.php?PHPSESSID=eqtv86gde852u4nqol2topk5m0&topic=8871.60   
  
  Dont forget to change the com port to the one on your computer
  
  Reads values throw serial communication from a arduino
  the message from the arduino should be like below.
  
  | ACC_angle, actAngle, ACC_X, ACC_Z, GYR_Y, GYR_X, pTerm, iTerm, dTerms, drive \n |
*/

import processing.serial.*;

Serial myPort;

controlManager controls;

graph roll;
graph sensors;
graph pid;
graph drive;

int skip = 0; //Skip the first five messages from the Arduino

boolean dragged = false;
boolean released = false;

void setup(){
  size(1024,580);

  // set inital background:
  background(0);
  // turn on antialiasing:
  smooth();
  
  // A graph showing the acc. values
  sensors = new graph(10, 10, 300, 200, -100, 100, "ACC_X, _Y, GYR_Y");        //Create a new graph    // was -200, 200
  sensors.labels[0] = "ACC_X";                                               //Sets the label for the values
  sensors.labels[1] = "ACC_Z";
  sensors.labels[2] = "GYR_Y";
  sensors.update();                                                 //Shows the graph with the new settings
  
  // A graph showing the gyro values
  pid = new graph(10, 220, 300, 200, -20, 20, "pTerm, iTerm, dTerms");       //Create a new graph           // was -300, 300
  pid.labels[0] = "pTerm";                                                     //Sets the label for the values
  pid.labels[1] = "dTerm";
  pid.labels[2] = "iTerm";
  pid.update();                                                     //Shows the graph with the new settings
  
  // A graph showing the roll angle 
  // of the bot bot unfilterd and filterd
  roll = new graph(320, 10, 690, 200, -50, 50, "actAngle, ACC_angle");       //Create a new graph   // was 0, 1024
  roll.labels[0] = "ACC_angle";                                              //Sets the label for the values
  roll.labels[1] = "actAngle";
  roll.update();                                                    //Shows the graph with the new settings
  
  drive = new graph(320, 220, 300, 200, -255, 255, "Motor");                  //Create a new graph          // was -255, 255
  drive.labels[0] = "drive";                                        //Sets the label for the values
  drive.update();                                                   //Shows the graph with the new settings
 
 
  controls = new controlManager();    //Create a new Control manager
  controls.graphs.add(roll);         //Add the graph roll
  controls.graphs.add(sensors);      //Add the graph sensors
  controls.graphs.add(pid);          //Add the graph pid
  controls.graphs.add(drive);        //Add the graph drive
  
  
  // List all the available serial ports:
  println(Serial.list());
  // I know that the port 7 in the serial list on my pc
  // is always my  Arduino or Wiring module, so I open Serial.list()[7].
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1];
  myPort = new Serial(this, portName, 9600);
  myPort.clear();
  // don't generate a serialEvent() until you get a newline (\n) byte:
  myPort.bufferUntil('\n');
}


void draw(){
  
}


void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  
  //Dont update the controls when dragged and released event are active..
  if(!dragged && !released){
   
    // if it's not empty:
    if (inString != null) {
      // trim off any whitespace:
      inString = trim(inString);
      
      if(skip > 5){
      
        if(split(inString, ",").length >= 9){  //The nr off expected values
        
          // convert to an array of floats:
          float[] temp = float(split(inString, ","));
          
          //Create temp variabels for each graph
          float[] f_roll = new float[2];
          float[] f_sensors = new float[3];
          float[] f_pid = new float[3];
          float[] f_drive = new float[1];
          
          if(temp.length >= 2){
            arrayCopy(temp,0,f_roll,0,2);    //Moves the values for the graph to the graph temp value
            roll.setValues(f_roll);          //Add the values to the graph
            roll.update();                   //Shows the new values
          }
          
          if(temp.length >= 5){
            arrayCopy(temp,2,f_sensors,0,3);
            sensors.setValues(f_sensors);
            sensors.update();
          }
          
          if(temp.length >= 8){
            arrayCopy(temp,5,f_pid,0,3);
            pid.setValues(f_pid);
            pid.update();
          }
          
          if(temp.length >= 9){
            arrayCopy(temp,8,f_drive,0,1);
            drive.setValues(f_drive);
            drive.update();
          }
        }
      }else{
        skip++;
      }
    }
  }
}


void mouseDragged(){
  dragged = true;
  controls.MouseDragged();     //Send the mouseDragged event to the control manager
  dragged = false;
}

void mouseReleased() {
  released = true;
  controls.MouseReleased();    //Send the mouseReleased event to the control manager
  released = false;
}


