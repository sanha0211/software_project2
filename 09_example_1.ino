// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

#define _EMA_ALPHA 0.1    // EMA weight of new sample (range: 0 to 1)

// global variables
unsigned long last_sampling_time = 0;   // unit: msec
float dist_prev = _DIST_MAX;            // Distance last-measured
float dist_ema = _DIST_MAX;             // EMA distance initialized

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;

  // wait until next sampling time
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the ultrasonic sensor (USS)
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  
  // Filter the raw distance to ensure it's within the min and max range
  dist_filtered = constrain(dist_raw, _DIST_MIN, _DIST_MAX);

  // Implement the EMA equation
  dist_ema = _EMA_ALPHA * dist_filtered + (1 - _EMA_ALPHA) * dist_ema;

  // Output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(", raw:");  Serial.print(dist_raw);
  Serial.print(", ema:");  Serial.print(dist_ema);
  Serial.print(", Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // Update last sampling time
  last_sampling_time += INTERVAL;
}

// Get a distance reading from USS. Return value is in millimeters.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // Convert pulse duration to distance
}
