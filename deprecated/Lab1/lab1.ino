
#include <Sparki.h>

// States
#define ROTATE 1
#define APPROACH 2
#define CLOSE 3
#define TURNAROUND 4 
#define FINDLINE 5
#define FOLLOW 6
#define DROP 7
#define CELEBRATE 8

// Globals
#define OBJECT_DISTANCE 3

int current_state = ROTATE;

void setup() {
  // put your setup code here, to run once:
  sparki.RGB(RGB_BLUE);
  
  sparki.servo(SERVO_CENTER);
  sparki.gripperOpen();
  
  delay(3000);
  
  sparki.gripperStop();
  sparki.RGB(RGB_GREEN);
  
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Measurements
  int dist = sparki.ping();
  int threshold = 700;
  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor

  switch (current_state) {
    
    case ROTATE:
      sparki.RGB(RGB_BLUE);
      
      if (dist <= 30 && dist != -1) {
        current_state = APPROACH;
      }
      else {
        sparki.moveRight();
      }
      
      break;
      
    case APPROACH:
      sparki.RGB(RGB_RED);
      if (dist <= OBJECT_DISTANCE && dist != -1){
        delay(100);
        
        sparki.moveStop();        
        sparki.RGB(RGB_BLUE);
        
        delay(1000);
        
        current_state = CLOSE;
      }
      else {
        sparki.moveForward();
      }
      break;
      
    case CLOSE:
      sparki.RGB(RGB_GREEN);
      sparki.gripperClose();
      delay(5000);
      
      current_state = FINDLINE;
      break;

    case TURNAROUND:
      sparki.RGB(RGB_RED);
      
      delay(1000);
      
      sparki.moveRight(180);
      
      delay(3000);

      current_state = FINDLINE;
      break;

    case FINDLINE:
      sparki.RGB(RGB_BLUE);
      
      if ( lineCenter >= threshold ) // if line is below left line sensor
      {  
        sparki.moveForward(); // move forward
      }
      else{
        
        if ( lineLeft > threshold ) // if line is below left line sensor
        {  
          sparki.moveLeft(); // turn left
        }
      
        if ( lineRight > threshold ) // if line is below right line sensor
        {  
          sparki.moveRight(); // turn right
        }

        if (lineCenter < threshold && lineLeft < threshold && lineRight < threshold){
          sparki.moveRight();
          delay(1000);
        }
        else {
          if (lineCenter < threshold ){
            sparki.moveStop();
            delay(1000);
            current_state = FOLLOW;
          }
        }
        
      }
    
      break;
    case FOLLOW:
      sparki.RGB(RGB_GREEN);

      if (lineCenter < threshold && lineLeft < threshold && lineRight < threshold){
        delay(1000);
        sparki.moveStop();

        current_state = DROP;
        break;
      }

      if ( lineCenter < threshold ) // if line is below left line sensor
      {  
        sparki.moveForward(); // move forward
      }
      else{
        if ( lineLeft < threshold ) // if line is below left line sensor
        {  
          sparki.moveLeft(); // turn left
        }
      
        if ( lineRight < threshold ) // if line is below right line sensor
        {  
          sparki.moveRight(); // turn right
        }
      }
      break;

    case DROP:
      sparki.RGB(RGB_RED);
      sparki.beep();
      sparki.gripperOpen();
      
      delay(3000);

      sparki.gripperStop();
      
      current_state = CELEBRATE;
      
      break;

    case CELEBRATE:
      sparki.RGB(RGB_BLUE);
      delay(700);
      sparki.RGB(RGB_GREEN);
      delay(700);
      sparki.RGB(RGB_RED);
      delay(700);
      break;

    default:
      sparki.beep();
      break;

  }
  
  
}
