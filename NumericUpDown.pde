class numericUpDown{
 //Settings
  String label = "";                       // the name and id of the graph
  float[] incomingValues;   // array of values
  float[] outgoingValues;   // array of values
  String[] labels = {"","","","","",""};                    // the value labels
  boolean view = true;                                      // if the graph should be visible
  int infoHeight = 20;                                       // the height of the info and button window
  int infoWidth = 80;                                       // the width of the values window
  int nrOfValues = 0;
  
  //Internal variables
  private int maxNumberOfSensors = 6;       // Arduino has 6 analog inputs, so I chose 6
  private int x;
  private int y;
  private int _width;
  private int _height;
 
  private PFont myFont;                     // font for writing text to the window
  
  private boolean fontInitialized = false;  // whether the font's been initialized
  
  numericUpDown(int x_temp, int y_temp, int width_temp, int height_temp, int nrOfValues_temp, String label_temp){
    x = x_temp;
    y = y_temp;
    _width = width_temp;
    _height = height_temp;
    label = label_temp;
    nrOfValues = nrOfValues_temp;
     
//    myFont = createFont(PFont.list()[3], 14);
     myFont = loadFont("Calibri-14.vlw");
    textFont(myFont);
    fill(255);
    
    fontInitialized = true;
    view = true;
    
    incomingValues = new float[nrOfValues];
    outgoingValues = new float[nrOfValues];
    
    update();
  } 
  
  /*
    Redraws the numeric up and down
  */
  void update(){
    fill(0);
    stroke(255);
    
    rect(x, y, _width, _height);
    rect(x, y, _width, infoHeight);
    
    fill(255);
    textFont(myFont);
    text(label, x + 5, y + 13);
    
    //Draw the parameters to change
    for(int i=0; i < incomingValues.length; i++){
      drawValue(i); 
    }
    
    if(mouseOverMove()){
      drawMenuMove(true); 
    } else {
      drawMenuMove(false);
    }
  }
  
  /*
    Draws a the graph in a move state and also recallculates the new position
  */
  void move(){
    x = mouseX - (_width)+20;
    y = mouseY - 9;
    fill(0);
    stroke(255);
    rect(x,y,_width,_height);
  }
  
  /*
    Draws the borders of the graph
  */
  void drawBorders(){
    fill(0);
    stroke(255);
    rect(x,y,_width,_height);
  }
  
  /*
    Draws the MOVE button
  */
  private void drawMenuMove(boolean View){
    textFont(myFont);
    textAlign(LEFT);
    
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
    Update the graph with the new values
  */
  void setValues(float[] values){
    incomingValues = values;
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
  
  void drawValue(int id)
  { 
    int valueBoxX = x+10;
    int valueBoxY = y + infoHeight + 30;
    int valueBoxSpacing = 10;
    int valueBoxWidth = 80;
    int valueBoxHeigth = 40;
    int buttonWidth = 20;
    
    fill(0);
    stroke(255);
    rect(valueBoxX, valueBoxY, valueBoxWidth, valueBoxHeigth);
    
    //Buttons
    fill(200);
    
    rect(valueBoxX + valueBoxWidth - buttonWidth, valueBoxY, buttonWidth, valueBoxHeigth / 2); 
    rect(valueBoxX + valueBoxWidth - buttonWidth, valueBoxY + valueBoxHeigth / 2, buttonWidth, valueBoxHeigth / 2); 
    
    fill(0);
    noStroke();
    triangle((valueBoxX + valueBoxWidth - buttonWidth) + 3, valueBoxY + 15, (valueBoxX + valueBoxWidth - buttonWidth) + (buttonWidth/2) , valueBoxY + 3, (valueBoxX + valueBoxWidth - buttonWidth) + buttonWidth - 3 , valueBoxY + 15);
  }
}
