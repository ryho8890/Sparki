#include <Sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
//#define CYCLE_TIME .050 // Default 50ms cycle time
#define CYCLE_TIME .100
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3
#define TEST 4

// self measured
#define PHI_PER_SEC (ROBOT_SPEED) / (WHEEL_RADIUS) 
#define ROT_PER_WHEEL_PER_SEC (WHEEL_RADIUS * PHI_PER_SEC) / 2.0 
#define D_THETA_PER_SEC (2 * WHEEL_RADIUS * PHI_PER_SEC) / AXLE_DIAMETER 

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000; 

// delta pose
float delta_x = 0.;
float delta_y = 0.;
float delta_th = 0.;

// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART3; // CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void updateErrors() {
  d_err = sqrt(pow((pose_x - dest_pose_x),2) + pow((pose_y - dest_pose_y),2));
  
  float adj_x = dest_pose_x - pose_x;
  float adj_y = dest_pose_y - pose_y;
  float adj_theta = pose_theta;
  
  if (pose_theta < 0) {
    adj_theta = (pose_theta + 2 * M_PI);
  }

  float diff = (adj_y) / (adj_x);
  b_err = atan(diff) - adj_theta;

  // when goal is in quads II or II relative to sparki
  if (adj_x < 0) {
    b_err = (abs(b_err + M_PI) < abs(b_err - M_PI)) ? b_err + M_PI : b_err - M_PI;
  }

  if (abs(b_err) > M_PI) {
    b_err = (abs(b_err - 2*M_PI) < abs(b_err + 2*M_PI)) ? b_err - 2*M_PI : b_err + 2*M_PI;
  }

  h_err = dest_pose_theta - pose_theta;

  if (h_err > M_PI) {
    h_err -= 2*M_PI;
  }
  else if(h_err < -1 * M_PI){
    h_err += 2*M_PI;
  }

}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = to_radians(0);
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(-1, 1, to_radians(45));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians

  //delay(3000);
  sparki.RGB(RGB_GREEN);
  delay(3000);
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -1. * M_PI) dest_pose_theta += 2*M_PI;
  updateErrors();
  orig_dist_to_goal = d_err; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void resetDeltas() {
  delta_x = 0.;
  delta_y = 0.;
  delta_th = 0.;
}

void updateOdometry() {
  // TODO: Update pose_x, pose_y, pose_theta

  pose_x += delta_x;
  pose_y += delta_y;
  pose_theta += delta_th;

  // Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta < -1* M_PI) pose_theta += 2.*M_PI;
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.println(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));
  sparki.println(d_err);
  
  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  float max_motor = 0.;

  // gains for (h)eading and (b)earing
  float b_const = 5. * M_PI;
  float h_const = 2.5 * M_PI;

  resetDeltas();

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        delta_x += 2 * ROT_PER_WHEEL_PER_SEC * cos(pose_theta) * 0.1;
        delta_y += 2 * ROT_PER_WHEEL_PER_SEC * sin(pose_theta) * 0.1;
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        delta_th += D_THETA_PER_SEC * 0.1;
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        delta_th -= D_THETA_PER_SEC * 0.1;
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }


      updateOdometry();

      
      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = M_PI;
      }
      
      
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination

      updateErrors();
      
      if (abs(b_err) > 0) {
        sparki.clearLCD();
        sparki.print(pose_theta);
        sparki.updateLCD();
        delay(5000);
        if (b_err < 0) {
          // negative -> turn right abs err
          sparki.moveRight(to_degrees(abs(b_err)));
          delta_th = b_err;
        }
        else if (b_err > 0){
          // positive -> turn left err
          sparki.moveLeft(to_degrees(b_err));
          delta_th = b_err;
        }
        
      }
      delay(5000);
      updateOdometry();
      updateErrors();
      resetDeltas();
      
      if (abs(d_err) > 0) {
          sparki.moveForward(100 * d_err);
          delta_x += d_err * cos(pose_theta);
          delta_y += d_err * sin(pose_theta);
        
      }
      
      delay(5000);
      updateOdometry();
      updateErrors();
      resetDeltas();
      
      if (abs(h_err) > 0) {
        sparki.clearLCD();
        sparki.print(pose_theta);
        sparki.updateLCD();
        delay(5000);
        if (h_err < 0) {
          // negative -> turn right abs err
          sparki.moveRight(to_degrees(abs(h_err)));
          delta_th = h_err;
        }
        else if (h_err > 0){
          // positive -> turn left err
          sparki.moveLeft(to_degrees(h_err));
          delta_th = h_err;
        }
      }
      
      updateOdometry();
      updateErrors();
      resetDeltas();


      sparki.moveStop();

      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));
      
      updateErrors();

      max_motor = 0.;
      phi_l = 0.;
      phi_r = 0.;
      dX = 0.;
      dTheta = 0.;

      if (d_err < 0.03){
        // withing 3 cm and 8 degrees ... No. I DO NOT think thats too lenient.
        if (abs(h_err) < 0.1396){
          sparki.moveStop();
          while(true){}
        }
      }

      dX = d_err / orig_dist_to_goal;
      dX = min(dX, 1.);
      
      dTheta += dX * (b_const*b_err + h_const*h_err);
 
      phi_l = (dX - ((dTheta * AXLE_DIAMETER) / 2.0)) / WHEEL_RADIUS;
      phi_r = (dX + ((dTheta * AXLE_DIAMETER) / 2.0)) / WHEEL_RADIUS;
      
      max_motor = max(abs(phi_l), abs(phi_r));
      right_speed_pct = phi_r / (max_motor);
      left_speed_pct = phi_l / (max_motor);

      if (right_speed_pct < 0){
        sparki.motorRotate(MOTOR_RIGHT, left_dir, abs(int(right_speed_pct*100.)));
      }
      else {
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));
      }

      if (left_speed_pct < 0) {
        sparki.motorRotate(MOTOR_LEFT, right_dir, abs(int(left_speed_pct*100.)));
      }
      else {
        sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      }
      
      delta_x += left_speed_pct * ROT_PER_WHEEL_PER_SEC * cos(pose_theta) * 0.1;
      delta_x += right_speed_pct * ROT_PER_WHEEL_PER_SEC * cos(pose_theta) * 0.1;
      
      delta_y += left_speed_pct * ROT_PER_WHEEL_PER_SEC * sin(pose_theta) * 0.1;
      delta_y += right_speed_pct * ROT_PER_WHEEL_PER_SEC * sin(pose_theta) * 0.1;

      delta_th += right_speed_pct * D_THETA_PER_SEC * 0.1 * 0.5;
      delta_th -= left_speed_pct * D_THETA_PER_SEC * 0.1 * 0.5;

      updateOdometry();

      break;
    
  }

  
  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();
  

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
