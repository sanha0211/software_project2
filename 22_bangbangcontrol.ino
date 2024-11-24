#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_VAR A3

// Event interval parameters
#define _INTERVAL_DIST 20  // Distance sensor interval (unit: ms)
#define _INTERVAL_SERVO 20 // Servo interval (unit: ms)
#define _INTERVAL_SERIAL 20 // Serial interval (unit: ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.5 // EMA weight (range: 0 to 1)
// Setting _EMA_ALPHA to 0 effectively disables EMA filter.

// Servo adjustment
#define _DUTY_MAX 2000 // Servo angle: D degree
#define _DUTY_NEU 1500 // Servo angle: 0 degree
#define _DUTY_MIN 1000 // Servo angle: E degree
#define _SERVO_ANGLE_DIFF 85 // Replace with |D - E| degree
#define _SERVO_SPEED 500 // Servo speed limit (unit: degree/second)
#define _BANGBANG_RANGE 120

// Global variables
float dist_filtered, dist_ema, dist_target; // Unit: mm
Servo myservo;
int duty_change_per_interval; // Maximum duty difference per interval
int duty_target; // Target duty
int duty_curr; // Current duty
unsigned long last_sampling_time_dist; // Unit: msec
unsigned long last_sampling_time_servo; // Unit: msec
unsigned long last_sampling_time_serial; // Unit: msec
bool event_dist, event_servo, event_serial; // Event triggered?

void setup() {
  // Initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;

  // Convert angular speed into duty change per interval
  duty_change_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (float)_SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0);

  // Initialize serial port
  Serial.begin(1000000);

  // Set a target distance
  dist_target = 155; // 15.5 cm, the center of the rail
}

void loop() {
  unsigned long time_curr = millis();

  // Wait until next event time
  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  if (event_dist) {
    event_dist = false;

    // Get a distance reading from the distance sensor
    dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5)); // Replace n with your desired value
    dist_ema = _EMA_ALPHA * dist_ema + (1.0 - _EMA_ALPHA) * dist_filtered;

    // Bang-bang control
    if (dist_target <= dist_ema) { // Replace with a proper test expression
      duty_target = _DUTY_MIN; // Minimum range
      digitalWrite(PIN_LED, 0);
    } else {
      duty_target = _DUTY_MAX; // Maximum range
      digitalWrite(PIN_LED, 1);
    }
  }

  if (event_servo) {
    event_servo = false;

    // Adjust duty_curr toward duty_target by duty_change_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_change_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_change_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }

    // Update servo position
    if (duty_curr > _DUTY_MAX) duty_curr = _DUTY_MAX; // For servo arm protection
    if (duty_curr < _DUTY_MIN) duty_curr = _DUTY_MIN;
    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;

    // Output the read value to the serial port
    Serial.print("Min:0,TARGET:");
    Serial.print(dist_target);
    Serial.print(",DIST:");
    Serial.println(dist_ema);
  }
}

float volt_to_distance(int a_value) {
  // Replace the below line with the equation obtained from nonlinear regression analysis
  return 1466 + (-8.28 * a_value) + 0.0165 * pow(a_value, 2) + (-1.13 * pow(10, -5)) * pow(a_value, 3);
}

unsigned int ir_sensor_filtered(unsigned int n, float position) {
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  unsigned int *ir_val, tmp, ret_idx, ret_val;
  unsigned int start_time;
  ret_idx = (unsigned int)ceil(n * position);

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * (ret_idx + 2));
  ir_val[0] = analogRead(PIN_IR);
  for (int i = 1; i < n; i++) {
    int j;
    if (i < ret_idx + 1) {
      ir_val[i] = analogRead(PIN_IR);
      j = i - 1;
    } else {
      ir_val[ret_idx + 1] = analogRead(PIN_IR);
      j = ret_idx;
    }
    for (; j >= 0; j--) {
      if (ir_val[j] > ir_val[j + 1]) {
        tmp = ir_val[j];
        ir_val[j] = ir_val[j + 1];
        ir_val[j + 1] = tmp;
      }
    }
  }

  if (position > 0.0)
    ret_val = ir_val[ret_idx];
  else
    ret_val = ir_val[0];

  free(ir_val);
  return ret_val;
}
