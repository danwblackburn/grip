///--------------------------------------------------------------------------
// 2023 Spring ME 327 Project
// Blake Jones, Danny Blackburn, Ashwin Vangipuram, Ankitha Durvasula
//
// Forked by Danny Blackburn 5.24.2023 
// From code by Allison Okamura 1.17.2018, Nathan Kau 9.9.2022
//--------------------------------------------------------------------------

//#define HALL_EFFECT_SENSOR
#define MR_SENSOR

#include <math.h>
#include "helpers.h"
#include "GripperState.h"

#define BAUDRATE 115200


/*
*   TODO: Could probably add scaled_bilateral_leader, scaled_bilateral_follower.
*   Switch case between modes in calc_force function so that .ino's can
*   be merged. Header files wouldn't need to be copied in two folders
*/
enum class Mode {
    bilateral,
    scaled_bilateral
};

// Set current mode
Mode current_mode = Mode::bilateral;

// Create new state
GripperState current_state;

/*
* Map from mr counts reading to sector angle in radians.
*/
double MRToSectorAngle (double mr_counts)
{
 // double m = -0.0116;
// double b = -3.8115;
// double deg = m * mr_counts + b;
  double deg = mr_counts * .01295897 - 12.5226847 + 6;


 return deg * M_PI / 180.0; //deg to rad
}

/*
 * Specifies a desired force to render on the hapkit
 */
float calculate_force(float handle_position, float handle_velocity, float handle_position_remote) {
  float handle_velocity_remote = 0;
  float kp = 200;
  float kd = 1;
  float force = 0;
  
  if(current_mode == Mode::bilateral)
  {
    force = kp * (handle_position_remote - handle_position) + kd * (handle_velocity_remote - handle_velocity);
  } 
  else //else in scaled bilateral mode
  {
    float scaling = 3;
    force = kp * (handle_position_remote - scaling * handle_position) + kd * (handle_velocity_remote - scaling * handle_velocity);
  } 

  return force;
}

void setup() 
{
  Serial.begin(BAUDRATE);
  initialize_sensor();
  initialize_motor();
  initialize_loop_checker();

  pinMode(A0, OUTPUT);
}

int count = 0;
void loop()
{
  // Compute position in counts
  float mr_counts = read_sensor();

  // Update state with newest reading
  current_state.UpdateState(MRToSectorAngle(mr_counts), /*DT=*/0.001);
  
  // Receive leader position
  float remote_position = send_receive_remote_arduino(current_state.GetFingerPos());

  // Calculate and set force
  float force = calculate_force(current_state.GetFingerPos(), smoothed_velocity, remote_position);
  current_state.SetDesiredForce(force);

  if(abs(force) >  .6)
  {
    digitalWrite(A0, HIGH);
  }
  else
  {
    digitalWrite(A0, LOW);
  }

  // Command motor torque
  command_motor(current_state.GetTorqueCmd());

  // Check if loop speed is too slow, and if so, print an error message.
  check_loop_speed();
}
