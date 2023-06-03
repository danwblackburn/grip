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

#define BAUDRATE 115200

/*
* Map from mr counts reading to sector angle in radians.
*/
double MRToSectorAngle (double mr_counts)
{
  // Linear fit in degrees
 double m = -0.0116;
 double b = -3.8115;
 double deg = m * mr_counts + b;

 return - deg * M_PI / 180.0; //deg to rad
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
  float mr_counts = read_sensor();

  Serial.println(MRToSectorAngle(mr_counts));

  // Check if loop speed is too slow, and if so, print an error message.
  check_loop_speed();
}
