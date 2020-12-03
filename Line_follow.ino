#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"

// Constants
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define DIR_FWD LOW
#define DIR_BKD HIGH

#define BAUD_RATE 9600
#define BUZZER_PIN 6

#define LINE_LEFT_PIN   A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A2 //Pin for the right line sensor

#define STATE_INITIAL        0
#define STATE_DRIVE_FORWARDS 1
#define STATE_FOUND_LINE     2
#define STATE_FOLLOW_LINE    3
#define STATE_LOST_LINE      4
#define STATE_REFIND_LINE    5
#define STATE_TURN_TO_HOME   6
#define STATE_DRIVE_HOME     7
#define STATE_STOP           10

// PID Values
#define heading_kp 50.0
#define heading_ki 0.01
#define heading_kd 1.0

#define line_kp 0.1
#define line_ki 0.0
#define line_kd 0.0

#define wheel_kp 0.08
#define wheel_ki 0.0
#define wheel_kd 0.0

// Initialise line sensors
LineSensor line_left(LINE_LEFT_PIN); 
LineSensor line_centre(LINE_CENTRE_PIN);
LineSensor line_right(LINE_RIGHT_PIN);

// Initialise pose
Kinematics pose;

// Initialise PIDs
PID heading_pid(heading_kp, heading_ki, heading_kd);
PID line_pid(line_kp, line_ki, line_kd);

// Timestamps
float update_ts;
float init_ts;

// States
bool on_line;
bool first_line_found;
bool buzzer_on;
bool beep_complete;

float distance_to_home;
float last_distance;

bool heading_target_found = false;

// Line Confidence
float left_line_confidence;
float centre_line_confidence;
float right_line_confidence;

// Thresholds
float line_threshold;
float confidence_threshold;

float line_demand;
float heading_demand;
float heading_measurement;
float forward_speed;

float recorded_mean_count;

// Refind line
bool left_checked;
bool right_checked;

int STATE;

// Remember, setup only runs once.
void setup()
{

  setupMotors();
  setupLeftEncoder();
  setupRightEncoder();
  setupTimestamps(); 

  // Calibrate and setup line sensors
  setupLineSensors();

  pinMode(BUZZER_PIN, OUTPUT);

  buzzer_on = false;
  beep_complete = false;
  heading_target_found = false;

  line_threshold = 300.0;
  confidence_threshold = 60.0;

  line_demand = 0.0;
  heading_demand = 0.0;
  heading_measurement = 0.0;
  forward_speed = 18.0;

  last_distance = 0.0;
  distance_to_home = 0.0;

  recorded_mean_count = 0;

  left_checked = false;
  right_checked = false;

  // Delay for start of run
  delay(1000);

  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");

  STATE = 0;
}


// Remmeber, loop is called again and again.
void loop() 
{


  // Update kinematics
  pose.update();

  // Check if on a line
  updateLineConfidence();
  on_line = checkForLine();

  float elapsed_time = millis() - update_ts;

  switch( STATE ) {
    case STATE_INITIAL:
      initialisingBeeps();
      break;
    case STATE_DRIVE_FORWARDS:
      driveForwards();     
      break;
    case STATE_FOLLOW_LINE:
      if (elapsed_time > 10) {
        update_ts = millis();
        followLine();
      } 
      break;
    case STATE_LOST_LINE:
      lostLine();
      break;
    case STATE_REFIND_LINE:
      if (elapsed_time > 10) {
        update_ts = millis();
        refindLine();
      }
      break;
    case STATE_TURN_TO_HOME:
      turnToHome();
      break;
    case STATE_DRIVE_HOME:
      if (elapsed_time > 5) {
        update_ts = millis();
        driveHome();
      }
      break;
    case STATE_STOP:
      updateSpeed(0,0);
      break;
    default:
      Serial.println("System Error, Unknown state!");
      break;
  }

}

void driveHome() {

  Serial.println(distance_to_home);

  heading_demand = pose.getHeadingTarget();


  updateHeadingPID(25.0);
  if (abs(pose.getDistanceTarget()) < 75) {
    if (distance_to_home >= last_distance){
      STATE = STATE_STOP;
    } 
  }

  last_distance = distance_to_home;
}


void turnToHome() {

  heading_demand = pose.getHeadingTarget();

  if (heading_demand >= pose.getTheta()) {
    updateSpeed(35.0, -35.0);
  } else {
    updateSpeed(-35.0, 35.0);
  }

  float angleDiff = heading_demand - pose.getTheta();
  if (abs(angleDiff) < 0.016) {
    updateSpeed(0, 0);
    last_distance = pose.getDistanceTarget();
    STATE = STATE_DRIVE_HOME;
    return;
  }
}

void refindLine() {

  // If line found restart line follow PID
  if (on_line) {
    STATE = STATE_FOLLOW_LINE;
    return;
  }
  
  // Turn left and then right to refind line
  if (!left_checked && !right_checked) {
    heading_demand = heading_measurement - M_PI/8;
    updateSpeed(-20, 20);
    
    if (pose.getTheta() <= heading_demand) {
      left_checked = true;
    }
  }

  if (left_checked && !right_checked) {
    heading_demand = heading_measurement + M_PI/8;
    updateSpeed(20, -20);

    if (pose.getTheta() >= heading_demand) {
      right_checked = true;
    }
  }

  if (left_checked && right_checked) {
    heading_demand = heading_measurement;
    updateSpeed(-20, 20);

    if (pose.getTheta() <= heading_demand) {
      updateSpeed(0, 0);
      playDelayTone(10, 200);
      delay(1000);
      STATE = STATE_TURN_TO_HOME;
      return;
    }
  }
}


void lostLine() {
  if (first_line_found) {
    updateSpeed(0.0, 0.0);
    heading_measurement = pose.getTheta();
    STATE = STATE_REFIND_LINE;
  }
}

void followLine() {
  if (on_line) {
    updateLinePID();
  }
  else { 
    STATE = STATE_LOST_LINE;
  }
}


void driveForwards() {
  if (!on_line) {
    updateHeadingPID(30.0);
  } else {
    first_line_found = true;
    STATE = STATE_FOLLOW_LINE;
  }
}

void updateSpeed(float left_new_speed, float right_new_speed) {
 
  if (left_new_speed < 0) {
    left_new_speed = left_new_speed * -1;
    digitalWrite( L_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( L_DIR_PIN, DIR_FWD );
  }

  if (right_new_speed < 0) {
    right_new_speed = right_new_speed * -1;
    digitalWrite( R_DIR_PIN, DIR_BKD );
  } else {
    digitalWrite( R_DIR_PIN, DIR_FWD );
  }

  analogWrite( L_PWM_PIN, left_new_speed );
  analogWrite( R_PWM_PIN, right_new_speed );
}

void updateLineConfidence() {
  // If sensor is above threshold, increase confidence
  if (line_left.readCalibrated() > line_threshold) {
    left_line_confidence += 1.0;
  }
  else {
    left_line_confidence -= 1.0;
  }

  if (line_centre.readCalibrated() > line_threshold) {
    centre_line_confidence += 1.0;
  }
  else {
    centre_line_confidence -= 1.0;
  }

  if (line_right.readCalibrated() > line_threshold) {
    right_line_confidence += 1.0;
  }
  else {
    right_line_confidence -= 1.0;
  }

  left_line_confidence = constrain(left_line_confidence, 0.0, 100.0);
  centre_line_confidence = constrain(centre_line_confidence, 0.0, 100.0);
  right_line_confidence = constrain(right_line_confidence, 0.0, 100.0);
  
  return;
}

bool checkForLine() {
  bool left_on_line = left_line_confidence > confidence_threshold;
  bool centre_on_line = centre_line_confidence > confidence_threshold;
  bool right_on_line = right_line_confidence > confidence_threshold;
  
  if (left_on_line || centre_on_line || right_on_line) {
    if (!first_line_found) {
      left_line_confidence = 100.0;
      centre_line_confidence = 100.0;
      right_line_confidence = 100.0;
      first_line_found = true;
    }
    return true;
  }

  return false;
}

void updateLinePID() {
  // Get weighted line sensing ratio
  float line_centre_value = getLineCentre();
  float pid_output = line_pid.update(line_demand, line_centre_value);

  float left_speed = forward_speed - pid_output;
  float right_speed = forward_speed + pid_output;

  left_speed = constrain(left_speed, -254, 254);
  right_speed = constrain(right_speed, -254, 254);

  updateSpeed(left_speed, right_speed);
}

void updateHeadingPID(float forward_bias) {
  float heading_output = heading_pid.update(heading_demand, pose.getTheta());

  float left_speed = forward_bias + heading_output;
  float right_speed = forward_bias - heading_output;

  left_speed = constrain(left_speed, -254, 254);
  right_speed = constrain(right_speed, -254, 254);

  updateSpeed(left_speed, right_speed);
}

float getLineCentre() {
  float left_value = line_left.readCalibrated();
  float centre_value = line_centre.readCalibrated();
  float right_value = line_right.readCalibrated();

  float I_total = left_value + centre_value + right_value;

  float p_1 = left_value / I_total;
  float p_2 = centre_value / I_total;
  float p_3 = right_value / I_total;

  float line_centre = (p_1 * 1000) + (p_2 * 2000) + (p_3 * 3000);

  line_centre = line_centre - 2000;
  
  line_centre = constrain(line_centre, -2000, 2000);

  return line_centre;
}

void initialisingBeeps() {
  // Update state
  STATE = STATE_DRIVE_FORWARDS;
}

void playDelayTone(int volume, int duration) {
  analogWrite(BUZZER_PIN, volume);
  delay(duration);
  analogWrite(BUZZER_PIN, 0);
}

void setupMotors() {
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r wheels
  digitalWrite( L_DIR_PIN, DIR_FWD );
  digitalWrite( R_DIR_PIN, DIR_FWD );
}

void setupTimestamps() {
  update_ts = millis();
  init_ts = millis();
}

void setupLineSensors() {
  // Calibrate line sensors
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();

  on_line = false;
  first_line_found = false;

  left_line_confidence = 0.0;
  centre_line_confidence = 0.0;
  right_line_confidence = 0.0;
}
