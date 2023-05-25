#include "me327_AS5048A.h"

#ifdef HALL_EFFECT_SENSOR
#define initialize_sensor initialize_hall_effect_sensor
#define read_sensor read_hall_effect_sensor
#endif
#ifdef MR_SENSOR
#define initialize_sensor initialize_mr_sensor
#define read_sensor read_mr_sensor
#endif

// Pin declares
const int MOTOR_PWM_PIN = 5; // PWM output pin for motor 1
const int MOTOR_DIR_PIN = 8; // direction output pin for motor 1
const int POSITION_SENSOR_PIN = A2; // input pin for MR sensor
const int HALL_EFFECT_SENSOR_PIN = 10;

const int PWM_DIVIDER = 1;
const int PRESCALE_FACTOR = 64 / PWM_DIVIDER;

const float COUNTS_PER_ROTATION = 920;
const float HALL_EFFECT_COUNTS_PER_ROTATION = 16384;
const int COMMON_SPEED = 200;
const int FLIP_THRESHOLD = 700;     // threshold to determine whether or not a flip over the 180 degree mark occurred
const int HALL_EFFECT_FLIP_THRESHOLD = 8000;

// Position tracking variables
boolean flipped = false;        // Whether we've flipped
int raw_position = 0;           // current raw reading from MR sensor
int last_raw_position = 0;      // last raw reading from MR sensor
int last_last_raw_position = 0; // last last raw reading from MR sensor
int flips = 0;                  // keeps track of the number of flips over the 180deg mark
int raw_difference = 0;
int last_raw_difference = 0;
int raw_offset = 0;
int last_raw_offset = 0;

// Velocity tracking variables
float last_handle_position = 0.0;
float smoothed_velocity = 0.0;

// Loop speed tracking
const int SLOW_LOOP_THRESHOLD_MICROS = 5000; // Longest loop time allowed in microseconds
long last_loop = 0;

// Hall effect sensor
ME327_AS5048A hall_effect_sensor(HALL_EFFECT_SENSOR_PIN);

void initialize_mr_sensor();
void initialize_motor();
float read_mr_sensor();
void command_motor(float pulley_torque);
void set_pwm_frequency(int pin, int divisor);

void initialize_mr_sensor() {
  // Input pins
  pinMode(POSITION_SENSOR_PIN, INPUT); // set MR sensor pin to be an input
  
  // Initialize position valiables
  last_raw_position = analogRead(POSITION_SENSOR_PIN);
  last_last_raw_position = last_raw_position;
  flips = 0;
}

void initialize_hall_effect_sensor() {
  hall_effect_sensor.init();

  hall_effect_sensor.getRawRotation();
  last_raw_position = hall_effect_sensor.getRawRotation();
  last_last_raw_position = last_raw_position;
}

void initialize_motor() {
  // Set PWM frequency 
  set_pwm_frequency(MOTOR_PWM_PIN, PWM_DIVIDER); 
  
  // Output pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);  // PWM pin for motor A
  pinMode(MOTOR_DIR_PIN, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(MOTOR_PWM_PIN, 0);     // set to not be spinning (0/255)
  digitalWrite(MOTOR_DIR_PIN, LOW);  // set direction
}

float read_mr_sensor() {
  raw_position = analogRead(POSITION_SENSOR_PIN);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  raw_difference = raw_position - last_raw_position;              //difference btwn current raw position and last raw position
  last_raw_difference = raw_position - last_last_raw_position;    //difference btwn current raw position and last last raw position
  raw_offset = abs(raw_difference);
  last_raw_offset = abs(last_raw_difference);
  
  // Update position record-keeping vairables
  last_last_raw_position = last_raw_position;
  last_raw_position = raw_position;
  
  // Keep track of flips over 180 degrees
  if((last_raw_offset > FLIP_THRESHOLD) && (!flipped)) {  // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(last_raw_difference > 0) {                         // check to see which direction the drive wheel was turning
      flips--;                                            // cw rotation 
    } else {                                              
      flips++;                                            // ccw rotation
    }
    flipped = true;                                       // set boolean so that the next time through the loop won't trigger a flip
  } else {                                                // anytime no flip has occurred
    flipped = false;
  }
  return raw_position + flips * COUNTS_PER_ROTATION;
}

float read_hall_effect_sensor() {
  raw_position = int(hall_effect_sensor.getRawRotation());

  if(raw_position - last_raw_position > HALL_EFFECT_FLIP_THRESHOLD) {
    flips--;
  }
  if(raw_position - last_raw_position < -HALL_EFFECT_FLIP_THRESHOLD) {
    flips++;
  }
  last_raw_position = raw_position;
  return raw_position + flips * HALL_EFFECT_COUNTS_PER_ROTATION;
}

float calculate_smoothed_velocity(float handle_position, float dt) {
  float velocity_estimate = (handle_position - last_handle_position) / dt;
  smoothed_velocity = 0.9 * smoothed_velocity + 0.1 * velocity_estimate;
  last_handle_position = handle_position;
  return smoothed_velocity;
}

void command_motor(float motor_torque) {
  // Determine correct direction for motor torque
  if(motor_torque > 0) { 
    digitalWrite(MOTOR_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_DIR_PIN, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  float duty = sqrt(abs(motor_torque)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  unsigned int output = (int)(duty* 255);     // convert duty cycle to output signal
  analogWrite(MOTOR_PWM_PIN, output);                // output the signal
}


typedef union {
 int integer;
 byte binary[2];
} BinaryIntUnion; 

BinaryIntUnion handle_position_binary;
BinaryIntUnion handle_position_remote_binary;

bool sending = true;
bool receiving = false;

float previous_handle_position_remote = 0.0;
float handle_position_remote = 0.0;

/* 
 *  Sends handle position to remote arduino.
 *  Returns handle position of remote hapkit.
 *  
 *  Be careful taking the difference between subsequent calls to this function
 *  because it will return the same value if a new data has not been received
 */
float send_receive_remote_arduino(float handle_position) {  
  if (sending && Serial.availableForWrite() > sizeof(int)){             //check that we have space in the serial buffer to write
    handle_position_binary.integer = int(handle_position * 100000.0);   // save space by using a integer representation
    Serial.write(handle_position_binary.binary, 2);                     // write the integer to serial
    sending = false; 
    receiving = true;
    Serial.flush(); // flush the serial for good measure
  }
  
  // read our follower position
  if (receiving && Serial.available() > 1){                                           // if there is at least 2 bytes of data to read from the serial
    previous_handle_position_remote = handle_position_remote;                         // backup old follower position
    Serial.readBytes(handle_position_remote_binary.binary, 2);                        // read the bytes in
    handle_position_remote = (float)handle_position_remote_binary.integer/100000.0;   // convert the integer back into a float
    if (isnan(handle_position_remote))                                                // if xh_2 is corrupt, just use the old value
      handle_position_remote = previous_handle_position_remote;
    sending = true;                                                 
    receiving = false;
    Serial.flush(); // flush the serial for good measure
  }
  return handle_position_remote;
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void set_pwm_frequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

// Returns the actual number of elapsed microseconds since start. 
// Because we are changing the TIM1 prescaler, just using micros()
// will give an inaccurate time. Similarly, millis() will also be wrong.
long accurate_micros() {
  return micros() / PRESCALE_FACTOR;
}

long accurate_delay(float delay_millis) {
  delay(delay_millis * PRESCALE_FACTOR);
}

void initialize_loop_checker() {
  last_loop = accurate_micros();
}

void check_loop_speed() {
  long now = accurate_micros();
  if(now - last_loop > SLOW_LOOP_THRESHOLD_MICROS) {
    command_motor(0.0);
    for(int i = 0 ;i < 100; i++) {
      Serial.println("DETECTED SLOW LOOP RATE. STOPPING PROGRAM. PLEASE FIX YOUR CODE.");
      accurate_delay(5);
    }
  }
  last_loop = now;
}
