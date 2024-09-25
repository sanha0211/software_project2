// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 100      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar
 
  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // Check distance and handle LED logic
  if ((distance == 0.0) || (distance > _DIST_MAX)) {
      distance = _DIST_MAX + 10.0;    // Set Higher Value
      Serial.println("Distance out of range: Too Far");
      blinkLED(3, 100);               // Blink LED 3 times for out-of-range (too far)
  } else if (distance < _DIST_MIN) {
      distance = _DIST_MIN - 10.0;    // Set Lower Value
      Serial.println("Distance out of range: Too Close");
      blinkLED(3, 100);               // Blink LED 3 times for out-of-range (too close)
  } else {    // In desired Range
      Serial.println("Distance in range.");
      digitalWrite(PIN_LED, HIGH);    // LED ON      
  }

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(", distance:");  Serial.print(distance);
  Serial.print(", Max:");       Serial.print(_DIST_MAX);
  Serial.println("");
 
  // wait until next sampling time.
  delay(INTERVAL);
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
 
  float duration = pulseIn(ECHO, HIGH, TIMEOUT);
  if (duration == 0) {
    // Timeout occurred, meaning no object was detected within the range
    return 0.0;
  }
  
  return duration * SCALE; // convert time to distance (mm)
}

// Function to blink the LED a specified number of times
void blinkLED(int times, int delayTime) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED, HIGH); // LED ON
    delay(delayTime);
    digitalWrite(PIN_LED, LOW);  // LED OFF
    delay(delayTime);
  }
}
