#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- CONFIGURATION ---
#define SERVO_FREQ 50
#define SERVO_MID_PULSE 375 // Your 90-degree "Zero"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);
  

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Safety delay on startup
  delay(1000);
}

void loop() {
  
  // =================================================
  // POSITION 1 (All Middle / Assembly Pose)
  // =================================================
  Serial.println("Moving to Position 1...");
  
  // Front Left
  pwm.setPWM(8, 0, 360);
  pwm.setPWM(9, 0, 355);
  pwm.setPWM(10, 0, 390);
  
  // Front Right
  pwm.setPWM(0, 0, 430);
  pwm.setPWM(1, 0, 360);
  pwm.setPWM(2, 0, 380);
  
  // Back Lef
  pwm.setPWM(12, 0, 360);
  pwm.setPWM(13, 0, 375);
  pwm.setPWM(14, 0, 360);

  // Back Right
  pwm.setPWM(4, 0, 375);
  pwm.setPWM(5, 0, 340);
  pwm.setPWM(6, 0, 390);

  // HOLD Position 1 for 1 second
  delay(1000);


 }