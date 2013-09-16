/*
  This class handels controls
  like if the should be visible or not 
  and mutch more
*/
class controlManager{
 
  ArrayList graphs;
  ArrayList numericUpDowns;
  
  private boolean move = false;
  private String controlToMove = "";
  
  controlManager(){
    graphs = new ArrayList();
    numericUpDowns = new ArrayList();
  } 
  
  /*
    Mouse released event
  */
  void MouseReleased(){
    checkGraphMax();
    checkGraphMoveReleased();
  }
  
  /*
    Mouse dragged event
  */
  void MouseDragged(){
    checkControlMoveDrag();
  }
  
  void checkControlMoveDrag(){
    
    if(move) { background(0); }
    
    if(graphs != null)
    { 
      for(int i=0; i < graphs.size(); i++){
        
        graph temp = (graph) graphs.get(i);

        if(temp.mouseOverMove() || controlToMove == temp.label && move){ 
          hideControls();
          temp.move();
          move = true;
          controlToMove = temp.label;
        }else if(controlToMove != temp.label && move){
          temp.drawBorders(); 
        }
      }
    }
    
    if(numericUpDowns != null)
    { 
      for(int i=0; i < numericUpDowns.size(); i++){
        
        numericUpDown temp = (numericUpDown) numericUpDowns.get(i);

        if(temp.mouseOverMove() || controlToMove == temp.label && move){ 
          hideControls();
          temp.move();
          move = true;
          controlToMove = temp.label;
        }else if(controlToMove != temp.label && move){
          temp.drawBorders(); 
        }
      }
    }
    
  }
  
  /*
  void checkGraphMoveDrag(){
    //println("Dragged: " +mouseX+":"+mouseY);
    if(graphs != null)
    {
      if(move) { background(0); }
      
      for(int i=0; i < graphs.size(); i++){
        
        graph temp = (graph) graphs.get(i);

        if(temp.mouseOverMove() || graphToMove == temp.label && moveGraph){ 
          hideControls();
          temp.move();
          move = true;
          graphToMove = temp.label;
        }else if(graphToMove != temp.label && moveGraph){
          temp.drawBorders(); 
        }
      }
    }
  }
  */
  
  void checkGraphMoveReleased(){
    if(move){
      move = false;
      controlToMove = "";
      background(0);
      showControls();  
    }
  }
  
  void checkNumericUpDownMoveReleased(){
    if(move){
      move = false;
      controlToMove = "";
      background(0);
      showControls(); 
    }
  }
  
  /*
    Function that checks if any of the
    graphs MAX button has bean pressed 
  */
  void checkGraphMax(){
    if(graphs != null)
    {
      for(int i=0; i < graphs.size(); i++){
        
        graph temp = (graph) graphs.get(i);
        
        if(temp.mouseOverMax()){ 
          background(0);
          if(temp.maxi){
            //Show all controls
            showControls();
          }else{
            //Hide other controls
            hideControls();
            temp.view = true;
            temp.maxi = true; 
            temp.reDrawMax();
            temp.update();
          }
        }
      }  
    }  
  }
  
  /*
    Hides all controls
  */
  void hideControls(){
    if(graphs != null){
      for(int i=0; i<graphs.size(); i++){
        graph temp = (graph) graphs.get(i);
        temp.view = false;
      }
    }
    
    if(numericUpDowns != null){
      for(int i=0; i<numericUpDowns.size(); i++){
        numericUpDown temp = (numericUpDown) numericUpDowns.get(i);
        temp.view = false;
      }
    }
  }
  
  /*
    Shows all controls
  */
  void showControls(){
    if(graphs != null){
      for(int i=0; i<graphs.size(); i++){
       graph temp = (graph) graphs.get(i);
       temp.maxi = false;
       temp.view = true;
       temp.reDrawMin();
       temp.update();
      }
    }
    
    if(numericUpDowns != null){
      for(int i=0; i<numericUpDowns.size(); i++){
        numericUpDown temp = (numericUpDown) numericUpDowns.get(i);
        temp.view = true;
        temp.update();
      }
    }
  }
  
}



