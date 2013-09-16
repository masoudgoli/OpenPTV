/*
  A class that draws a graph with max 6 values
*/
class graph{
  
  //Settings
  String label = "";                       // the name and id of the graph
  float[] incomingValues = new float[6];   // array of values
  float[] previousValue = new float[6];    // array of previous values
  String[] labels = {"","","","","",""};                    // the value labels
  boolean view = true;                                      // if the graph should be visible
  boolean maxi = false;                                     // if the graph should be maximized
  int infoHight = 20;                                       // the height of the info and button window
  int infoWidth = 80;                                       // the width of the values window
  
  //Internal variables
  private int maxNumberOfSensors = 6;       // Arduino has 6 analog inputs, so I chose 6
  private int x;
  private int y;
  private int _width;
  private int _hight;
  private int xpos;
  private int x_org;
  private int y_org;
  private int _width_org;
  private int _hight_org;
  private int scale_min;
  private int scale_max;
 
  private PFont myFont;                     // font for writing text to the window
  
  private boolean fontInitialized = false;  // whether the font's been initialized
  
  
  graph(int x_temp, int y_temp, int width_temp, int hight_temp, int scale_min_temp, int scale_max_temp, String label_temp){
     x = x_temp;
     y = y_temp;
     _width = width_temp;
     _hight = hight_temp;
     label = label_temp;
     scale_min = scale_min_temp;
     scale_max = scale_max_temp;
     
     x_org = x_temp;
     y_org = y_temp;
     _width_org = width_temp;
     _hight_org = hight_temp;
    
    fill(0);
    stroke(255);
    //Draw info
    rect(x, y,  _width, infoHight);
    //Sensor info
    rect(x, y + infoHight, infoWidth, (_hight-infoHight));
    //graph window
    rect(x + infoWidth, y + infoHight, (_width-infoWidth), (_hight-infoHight)); 
    
//    myFont = createFont(PFont.list()[3], 14);
     myFont = loadFont("Calibri-14.vlw");
    textFont(myFont);
    fill(255);
    text(label + " | Scale: "+scale_min+" - "+scale_max, x+5, y+13);
    
    fontInitialized = true;
    view = true;
    
    //Set a start value for all values
    previousValue[0] = map(511, 0, 1023, y + infoHight + 1, y + infoHight + _hight - 1);
    previousValue[1] = map(511, 0, 1023, y + infoHight + 1, y + infoHight + _hight - 1);
    previousValue[2] = map(511, 0, 1023, y + infoHight + 1, y + infoHight + _hight - 1);
    previousValue[3] = map(511, 0, 1023, y + infoHight + 1, y + infoHight + _hight - 1);
    previousValue[4] = map(511, 0, 1023, y + infoHight + 1, y + infoHight + _hight - 1);
    previousValue[5] = map(511, 0, 1023, y + infoHight + 1, y + infoHight + _hight - 1);
    
    xpos = x + infoWidth + 1;
  } 
  
  /*
    Redraws the graph
  */
  void update(){
    int r=0;
    int g=0;
    int b=0;
    
    if(view){
      if(incomingValues.length <= maxNumberOfSensors && incomingValues.length > 0){
        for(int i=0; i<incomingValues.length; i++){
          
          // figure out the y position for this particular graph:
          float graphBottom = y + _hight;
          
          float ypos = map(incomingValues[i], scale_min, scale_max, int(graphBottom), y + infoHight + 1);
          
          // make a black block to erase the previous text:
          noStroke();
          fill(0);
          rect(x+1, y + infoHight + (14*i) + 5, infoWidth - 2, 16);
          //println(label+":"+str(x+1)+":"+str(Y + infoHight + (14*i) + 16));
          
          // change colors to draw the graph line:
          switch(i){
            case 0:
              r=0;
              g=0;
              b=255;
              break;
            case 1:
              r=255;
              g=255;
              b=255;
              break;
            case 2:
              r=255;
              g=255;
              b=0;
              break;
            case 3:
             r=0;
             g=255;
             b=250;
             break;
            case 4:
             r=128;
             g=128;
             b=0;
             break;
            case 5:
             r=0;
             g=128;
             b=128;
             break; 
          }
          
          // print the sensor numbers to the screen:
          fill(r, g, b);
          int textPos = int(y + infoHight + (14*i) + 16);
          // sometimes serialEvent() can happen before setup() is done.
          // so you need to make sure the font is initialized before
          // you text():
          if (fontInitialized) {
            //println(label +":"+ labels[i]+":"+str(x+5)+":"+str(textPos));
            text(labels[i] + " :  " + incomingValues[i], x+5, textPos);
          }
          
          stroke(r, g, b);
          if(ypos > graphBottom - 1) { ypos = graphBottom - 1;}
          if(ypos < y + infoHight + 1) { ypos = y + infoHight + 1;}
          
          line(xpos, previousValue[i], xpos+1, ypos);
          // save the current value to be the next time's previous value:
          previousValue[i] = ypos;
        }
        
        if(xpos >= x + _width - 1){
          xpos = x + infoWidth +1;
          fill(0);
          stroke(255);
          rect(x + infoWidth, y + infoHight, (_width-infoWidth), (_hight-infoHight));   
        }else{
          xpos++;
        }
      }
      
      if(mouseOverMove()){
        drawMenuMove(true); 
      } else {
        drawMenuMove(false);
      }
      
      if(mouseOverMax()){
        drawMenuMax(true); 
      } else {
        drawMenuMax(false);
      }
    }//Under view
    
  }
  
  /*
    Draws a the graph in a move state and also recallculates the new position
  */
  void move(){
    x = mouseX - (_width)+20;
    y = mouseY - 9;
    x_org = x;
    y_org = y;
    fill(0);
    stroke(255);
    rect(x,y,_width,_hight);
  }
  
  /*
    Draws the borders of the graph
  */
  void drawBorders(){
    fill(0);
    stroke(255);
    rect(x,y,_width,_hight);
  }
  
  /*
    Draws the MAX button
  */
  private void drawMenuMax(boolean View){
    if(View){
      fill(100);
      noStroke();
      rect(x + (_width) - 80, y+3, 35, 15);
      if (fontInitialized) {
        fill(255);
        if(maxi){
          text("MIN", x + (_width) - 75, y+13);
        } else {
          text("MAX", x + (_width) - 75, y+13);  
        }
      }
    } else {
      fill(0);
      noStroke();
      rect(x + (_width) - 80, y+3, 35, 15);
      if (fontInitialized) {
        fill(255);
        if(maxi){
          text("MIN", x + (_width) - 75, y+13);
        } else {
          text("MAX", x + (_width) - 75, y+13); 
        }
      }
    }
  }
  
  /*
    Draws the MOVE button
  */
  private void drawMenuMove(boolean View){
    if(View){
      fill(100);
      noStroke();
      rect(x + (_width) - 45, y+3, 40, 15);
      if (fontInitialized) {
        fill(255);
        text("MOVE", x + (_width) - 40, y+13);
      }
    } else {
      fill(0);
      noStroke();
      rect(x + (_width) - 45, y+3, 40, 15);
      if (fontInitialized) {
        fill(255);
        text("MOVE", x + (_width) - 40, y+13);
      }
    }
  }
  
  /*
    Redraws the graph in org. size
  */
  void reDrawMin(){
     x = x_org;
     y = y_org;
     _width = _width_org;
     _hight = _hight_org;
     previousValue[0] = y + infoHight + 1;
     previousValue[1] = y + infoHight + 1;
     previousValue[2] = y + infoHight + 1;
     previousValue[3] = y + infoHight + 1;
     previousValue[4] = y + infoHight + 1;
     previousValue[5] = y + infoHight + 1;
     
     fill(0);
     stroke(255);
     //Draw info
     rect(x, y,  _width, infoHight);
     //Sensor info
     rect(x, y + infoHight, infoWidth, (_hight-infoHight));
     //graph window
     rect(x + infoWidth, y + infoHight, (_width-infoWidth), (_hight-infoHight)); 
     fill(255);
     text(label + " | Scale: "+scale_min+" - "+scale_max, x+5, y+13);
     xpos = x + infoWidth + 1;
  }
  
  /*
    Redraws the graph in maximized size
  */
  void reDrawMax(){
     x = 0;
     y = 0;
     _width = width;
     _hight = height; 
     previousValue[0] = y + infoHight + 1;
     previousValue[1] = y + infoHight + 1;
     previousValue[2] = y + infoHight + 1;
     previousValue[3] = y + infoHight + 1;
     previousValue[4] = y + infoHight + 1;
     previousValue[5] = y + infoHight + 1;

     fill(0);
     stroke(255);
     //Draw info
     rect(x, y,  _width, infoHight);
     //Sensor info
     rect(x, y + infoHight, infoWidth, (_hight-infoHight));
     //graph window
     rect(x + infoWidth, y + infoHight, (_width-infoWidth), (_hight-infoHight)); 
     fill(255);
     text(label + "| Scale: "+scale_min+" - "+scale_max, x+5, y+13);
     xpos = x + infoWidth + 1;  
  }
  
  /*
    Update the graph with the new values
  */
  void setValues(float[] values){
    incomingValues = values;
  }
  
  /*
    If the mouse is over the MAX button
  */
  private boolean mouseOverMax(){
    boolean returnValue = false;
    
    if(mouseX > x + (_width)-80 && mouseX < x + (_width) -35)
    {
      if(mouseY > y + 3 && mouseY < y + 15){
        returnValue = true;
      } 
    }
    return returnValue;
  }
  
  /*
    If the mouse is over the MOVE button
  */
  private boolean mouseOverMove(){
    boolean returnValue = false;
    
    if(mouseX > x + (_width)-40 && mouseX < x + (_width) -5)
    {
      if(mouseY > y + 3 && mouseY < y + 15){
        returnValue = true;
      } 
    }
    return returnValue;
  }
}
