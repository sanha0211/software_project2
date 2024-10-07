#include <Servo.h>

#define PIN_SERVO 10

Servo myServo;
unsigned long MOVING_TIME = 3000; // moving time is 3 seconds
unsigned long moveStartTime;
int startAngle = 30; // 30°
int stopAngle  = 90; // 90°
bool isMoving = false;

void setup() {
  myServo.attach(PIN_SERVO);
  myServo.write(startAngle); // Set initial position
  delay(500); // Wait for the servo to reach the initial position
  moveStartTime = millis(); // Initialize the movement start time
  isMoving = true; // Start moving the servo
}

void loop() {
  if (isMoving) {
    unsigned long progress = millis() - moveStartTime;

    if (progress <= MOVING_TIME) {
      // Use float for accurate movement control
      float angle = map(progress, 0, MOVING_TIME, startAngle, stopAngle);
      myServo.write(angle); 
    } else {
      myServo.write(stopAngle); // Ensure the servo reaches the final angle
      isMoving = false; // Stop moving after reaching the final position
    }
  }
}
