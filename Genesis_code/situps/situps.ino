#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =================================================================
// 1. ROBOT CONFIGURATION
// =================================================================

// --- ROBOT DIMENSIONS ---
float L1 = 115.0; // Femur
float L2 = 137.0; // Tibia

// --- PINS ---
int LEG_PINS[4][3] = {
  {8, 9, 10},  // Leg 0: Front-Left
  {0, 1, 2},    // Leg 1: Front-Right
  {12, 13, 14}, // Leg 2: Back-Left
  {4, 5, 6}     // Leg 3: Back-Right
};

// --- HOME POSITIONS ---
int LEG_HOME[4][3] = {
  {360, 355, 390}, // FL
  {430, 360, 380}, // FR
  {360, 375, 360}, // BL
  {375, 340, 390}  // BR
};

// --- MIRROR LOGIC ---
bool IS_RIGHT_LEG[4] = {false, true, false, true};

// --- PUSH-UP SETTINGS ---
float STAND_Z = -160.0; // Normal standing height
float DOWN_Z  = -110.0; // Push-up down position (lower body)
int SPEED     = 5;      // Speed of movement

// =================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50
#define MIN_PULSE 150 
#define MAX_PULSE 600 
#define MID 375 

int currentPos[16]; 
float thigh_angle_zero, knee_angle_zero; 

// =================================================================
// 2. MATH HELPERS
// =================================================================

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculateNeutralAngles() {
  float x = 0; float z = STAND_Z;
  float D = sqrt(x*x + z*z);
  float a1 = atan2(x, -z);
  float a2 = acos((sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D));
  float b  = acos((sq(L1) + sq(L2) - sq(D)) / (2 * L1 * L2));
  thigh_angle_zero = (a1 + a2) * 180 / PI;
  knee_angle_zero  = b * 180 / PI;
}

void moveLegRelative(int legNum, float x, float z) {
  // No forward/backward movement for pushups, so x is typically 0
  // But we keep the function generic
  
  // Mirror logic for X if needed (though X is 0 for simple pushup)
  // if (IS_RIGHT_LEG[legNum]) x = -x; 

  float D = sqrt(x*x + z*z);
  if (D > (L1 + L2)) return; 

  float a1 = atan2(x, -z);
  float a2 = acos((sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D));
  float b  = acos((sq(L1) + sq(L2) - sq(D)) / (2 * L1 * L2));

  float thigh_deg = (a1 + a2) * 180 / PI;
  float knee_deg  = b * 180 / PI;

  float thigh_diff = thigh_deg - thigh_angle_zero;
  float knee_diff  = knee_deg  - knee_angle_zero;

  float pulses_per_degree = (MAX_PULSE - MIN_PULSE) / 180.0;
  
  int thigh_change = thigh_diff * pulses_per_degree;
  int knee_change  = knee_diff  * pulses_per_degree;

  if (IS_RIGHT_LEG[legNum]) {
      thigh_change = -thigh_change; 
      knee_change = -knee_change;
  }

  // Update Global Position Tracking
  currentPos[LEG_PINS[legNum][0]] = LEG_HOME[legNum][0]; // Hip stays home
  currentPos[LEG_PINS[legNum][1]] = LEG_HOME[legNum][1] + thigh_change;
  currentPos[LEG_PINS[legNum][2]] = LEG_HOME[legNum][2] + knee_change;

  // Send Command
  pwm.setPWM(LEG_PINS[legNum][0], 0, currentPos[LEG_PINS[legNum][0]]);
  pwm.setPWM(LEG_PINS[legNum][1], 0, currentPos[LEG_PINS[legNum][1]]);
  pwm.setPWM(LEG_PINS[legNum][2], 0, currentPos[LEG_PINS[legNum][2]]);
}

void smoothMoveTo(int targetPos[16], int speedDelay) {
  bool moving = true;
  while(moving) {
    moving = false; 
    for(int i=0; i<16; i++) {
      if(i==3 || i==7 || i==11 || i==15) continue;
      if (currentPos[i] < targetPos[i]) {
        currentPos[i]++; pwm.setPWM(i, 0, currentPos[i]); moving = true; 
      } else if (currentPos[i] > targetPos[i]) {
        currentPos[i]--; pwm.setPWM(i, 0, currentPos[i]); moving = true;
      }
    }
    delay(speedDelay); 
  }
}

void standUp() {
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i];
  for(int leg=0; leg<4; leg++) {
    target[LEG_PINS[leg][0]] = LEG_HOME[leg][0];
    target[LEG_PINS[leg][1]] = LEG_HOME[leg][1];
    target[LEG_PINS[leg][2]] = LEG_HOME[leg][2];
  }
  smoothMoveTo(target, 10); 
}


// =================================================================
// 3. PUSH-UP FUNCTION
// =================================================================

void doPushUps() {
  // Loop for multiple pushups
  for (int k = 0; k < 3; k++) { // Do 3 pushups
    
    // --- DOWN Phase ---
    for (int i = 0; i <= 20; i++) {
      float z = mapFloat(i, 0, 20, STAND_Z, DOWN_Z);
      
      // Move all 4 legs to new Z
      moveLegRelative(0, 0, z);
      moveLegRelative(1, 0, z);
      moveLegRelative(2, 0, z);
      moveLegRelative(3, 0, z);
      
      delay(SPEED);
    }
    delay(200); // Pause at bottom

    // --- UP Phase ---
    for (int i = 0; i <= 20; i++) {
      float z = mapFloat(i, 0, 20, DOWN_Z, STAND_Z);
      
      moveLegRelative(0, 0, z);
      moveLegRelative(1, 0, z);
      moveLegRelative(2, 0, z);
      moveLegRelative(3, 0, z);
      
      delay(SPEED);
    }
    delay(500); // Pause at top
  }
  
  // Return to stable stand
  standUp();
}

// =================================================================
// 4. SETUP & LOOP
// =================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("--- Push-Up Demo ---");
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  calculateNeutralAngles();
  
  // Initialize to MID
  for(int i=0; i<16; i++) {
     pwm.setPWM(i, 0, MID);
     currentPos[i] = MID;
  }
  delay(1000);

  standUp();
  delay(2000);
}

void loop() {
  // Perform Pushups
  doPushUps();
  
  delay(2000); // Wait before repeating
}