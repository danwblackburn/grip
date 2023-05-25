///--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018, Nathan Kau 9.9.2022
//--------------------------------------------------------------------------

//#define HALL_EFFECT_SENSOR
#define MR_SENSOR

#include <math.h>
#include "helpers.h"

#define BAUDRATE 115200

enum class Mode {
    bilateral,
    scaled_bilateral
};

Mode current_mode = Mode::bilateral;

/*
* Map from position reading to sector angle in radians.
*/
double pos_to_angle (double pos)
{
  // Linear fit in degrees
 double m = -0.011283;
 double b = 7.749591;
 double deg = m * pos + b;
 return deg * M_PI / 180.0; //deg to rad
}


/*
 * Calculates the position of the hapkit handle given the sensor reading
 */
float calculate_handle_position(float updated_position) {
  
  double rh = .075;                       // m 
  double sector_angle = pos_to_angle(updated_position);
  float xh = rh * sector_angle;                        // x = r * theta

  
  //convert from sector angle to gripper finger location
  //return gripper position
  
  return xh;
}

/*
 * Calculates the pulley torque necessary to achieve the desired force
 */
float calculate_pulley_torque(float force) {
  double rh = 0.075;   //[m]
  double rs = 0.075;   //[m]
  double rp = 0.0095; //[m]
  float Tp = rh * rp / rs * force;
  
  //update torque calcs to be accurate for gripper fingers
  return Tp;
 }

/*
 * Specifies a desired force to render on the hapkit
 */
float student_specified_force(float handle_position, float handle_velocity, float handle_position_remote) {
  // STUDENT CODE HERE

  float handle_velocity_remote = 0;
  float kp = 50;
  float kd = 1;
  float force = 0;
  
  if(current_mode == Mode::bilateral)
  {
    force = kp * (handle_position_remote - handle_position) + kd * (handle_velocity_remote - handle_velocity);
  } 
  else //else in scale bilateral mode
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
}

int count = 0;
void loop()
{
  // Compute position in counts
  float updated_position = read_sensor();

  // Compute position [m] and velocity [m/s]
  float handle_position = calculate_handle_position(updated_position);
  float smoothed_velocity = calculate_smoothed_velocity(handle_position, /*DT=*/0.001);
  
  float remote_position = send_receive_remote_arduino(handle_position);
  
  // calculate 
  float force = student_specified_force(handle_position, smoothed_velocity, remote_position);
  float pulley_torque = calculate_pulley_torque(force);
  
  // Command the motor
  command_motor(pulley_torque);

  // Check if your loop speed is too slow, and if so, print an error message.
  check_loop_speed();
}
