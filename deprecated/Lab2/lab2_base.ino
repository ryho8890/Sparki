#include <Sparki.h>
#include <math.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2

// Constant for pi
#define PI 3.14159265

// Sparki Measurement Constants
#define WHEEL_BASE 8.59 // cm's
#define WHEEL_RADIUS 2.51 // cm's
#define PHI_PER_LOOP 0.111 // assuming 100ms loops
#define ROT_PER_WHEEL (WHEEL_RADIUS * PHI_PER_LOOP) / 2.0
#define D_THETA_PER_LOOP (2 * WHEEL_RADIUS * PHI_PER_LOOP) / WHEEL_BASE 
#define CM_PER_MS = 0.0028 // measured from CONTROLLER_DISTANCE_MEASURE

int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
bool reset = false;

unsigned long time;
//unsigned long start_time;
//unsigned long end_time;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup() {

  sparki.clearLCD();

  sparki.servo(SERVO_CENTER);
  
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;

  sparki.RGB(RGB_RED);
  delay(3000);
  sparki.RGB(RGB_GREEN);
  delay(3000);
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  // distance = sparki.ping();
}

void measure_30cm_speed() {
  sparki.clearLCD();
  delay(5000);
  
  sparki.print("Start Time: ");
  sparki.print(millis());

  sparki.moveRight(360);

  sparki.print("\n\nEnd Time: ");
  sparki.print(millis());

  sparki.updateLCD();

  current_state = -1;
}


void updateOdometry() {
  // TODO
}

void displayOdometry() {
  // TODO
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  float delta_x = 0.;
  float delta_y = 0.;
  float delta_th = 0.;
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:

      time = millis();
    
      readSensors();

      
      if (line_center < threshold && line_right < threshold && line_left < threshold && !reset){
        // defintely detecting start line
        reset = true;
      }
      else if (reset && !(line_center < threshold && line_right < threshold && line_left < threshold)){
        sparki.RGB(RGB_BLUE);
        sparki.moveStop();
        reset = false;
        current_state = -1;
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      else if ( line_center < threshold ) // if line is below left line sensor
      {  
        sparki.moveForward(); // move forward

        delta_x += 2 * ROT_PER_WHEEL * cos((pose_theta * PI) / 180.0);
        delta_y += 2 * ROT_PER_WHEEL * sin((pose_theta * PI) / 180.0);
        
      }
      else{
        if ( line_left < threshold ) // if line is below left line sensor
        {  
          sparki.moveLeft(); // turn left
          //
          //delta_th += THETA_PER_MS * 100;
          //delta_x += ROT_PER_WHEEL * cos((pose_theta * PI) / 180.0);
          //delta_y += ROT_PER_WHEEL * sin((pose_theta * PI) / 180.0);
          delta_th += D_THETA_PER_LOOP;
        }
      
        if ( line_right < threshold ) // if line is below right line sensor
        {  
          sparki.moveRight();// turn right
          //
          //delta_th += -1 * THETA_PER_MS * 100;

          //delta_x += ROT_PER_WHEEL * cos((pose_theta * PI) / 180.0);
          //delta_y += ROT_PER_WHEEL * sin((pose_theta * PI) / 180.0);
          delta_th -= D_THETA_PER_LOOP;
        }

      }
      
      pose_x += delta_x;
      pose_y += delta_y;
      pose_theta += (delta_th * 180 / PI);

      if (pose_theta >= 360) {
          pose_theta -= 360;
      }

      else if (pose_theta < 0) {
          pose_theta += 360;
      }

      sparki.clearLCD();
      sparki.print("X: ");
      sparki.print(pose_x);
      
      sparki.print("\n\nY: ");
      sparki.print(pose_y);

      sparki.print("\n\nTheta: ");
      sparki.print(pose_theta);

      sparki.updateLCD();

      time = millis() - time;
      delay(100 - time);
         
      break;
      
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;

    default:
      while(true){}
  }


  //delay(1000*CYCLE_TIME);
}
