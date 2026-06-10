/*
 * ULTIMATE BLUETOOTH CONTROLLER
 * -----------------------------
 * 1. COMMANDS:
 * - 'W' -> Walk Forward (Continuous)
 * - 'D' -> Happy Dance
 * - 'H' -> Hello Wave
 * - 'P' -> Push Ups
 * - 'S' -> Stop / Stand Up
 *
 * 2. FEATURES:
 * - Direction Calibration Fixed
 * - Inverse Kinematics for Walking
 * - Interrupts (Stop immediately when S is pressed)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =================================================================
// 1. DIRECTION TUNING (From your Calibration)
// =================================================================

int HIP_DIR[4]   = {-1, -1, 1, 1};  
int THIGH_DIR[4] = {1, -1, 1, -1};  
int KNEE_DIR[4]  = {1, -1, 1, -1};  

// =================================================================
// 2. WALKING SETTINGS
// =================================================================

float STEP_HEIGHT = 40.0; 
float STEP_LEN    = 40.0; 
int   STEP_SPEED  = 10;   

// =================================================================
// 3. CONFIGURATION
// =================================================================

int LEG_PINS[4][3] = {
  {0, 1, 2},  // FL
  {9, 10, 11},    // FR
  {4,5,6}, // BL
  {12, 13, 14}     // BR
};

int LEG_HOME[4][3] = {
  {350, 355, 400}, // FL
  {400, 375, 375}, // FR
  {365, 340, 375}, // BL
  {400, 375, 375}  // BR
};

// Robot Geometry
float L1 = 115.0; 
float L2 = 137.0; 
float GROUND_Z  = -160.0;
bool INVERT_FORWARD = true; 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50
#define MIN_PULSE 150 
#define MAX_PULSE 600 
#define MID 375 

int currentPos[16]; 
float thigh_angle_zero, knee_angle_zero; 

// =================================================================
// 4. MATH HELPERS (Inverse Kinematics)
// =================================================================

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculateNeutralAngles() {
  float x = 0; float z = GROUND_Z;
  float D = sqrt(x*x + z*z);
  float a1 = atan2(x, -z);
  float a2 = acos((sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D));
  float b  = acos((sq(L1) + sq(L2) - sq(D)) / (2 * L1 * L2));
  thigh_angle_zero = (a1 + a2) * 180 / PI;
  knee_angle_zero  = b * 180 / PI;
}

void moveLegRelative(int legNum, float x, float y, float z) {
  if (INVERT_FORWARD) x = -x;       
  
  // Hip Calculation
  float hip_deg_change = y * 0.5;
  hip_deg_change *= HIP_DIR[legNum]; 

  // IK Calculation
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
  
  // Apply Direction Multipliers
  int hip_change   = hip_deg_change * pulses_per_degree;
  int thigh_change = thigh_diff * pulses_per_degree * THIGH_DIR[legNum];
  int knee_change  = knee_diff  * pulses_per_degree * KNEE_DIR[legNum];

  currentPos[LEG_PINS[legNum][0]] = LEG_HOME[legNum][0] + hip_change;
  currentPos[LEG_PINS[legNum][1]] = LEG_HOME[legNum][1] + thigh_change;
  currentPos[LEG_PINS[legNum][2]] = LEG_HOME[legNum][2] + knee_change;

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
// 5. INTERRUPT CHECKER (Stop Button)
// =================================================================

bool checkInterrupt() {
  if (Serial1.available() > 0) {
    char cmd = Serial1.read();
    if (cmd == 'S') {
      Serial.println("--- INTERRUPT: STOP ---");
      standUp();
      return true; // Stop triggered
    }
  }
  return false; // Continue
}

// =================================================================
// 6. ACTIONS: WALK, WAVE, DANCE, PUSHUP
// =================================================================

// --- WALKING (Continuous Loop) ---
void doWalkTick() {
  static float phase = 0;
  phase += 0.05; // Speed
  if(phase > 2*PI) phase -= 2*PI;
  
  for(int leg=0; leg<4; leg++) {
    float leg_phase = phase;
    if(leg == 1 || leg == 2) leg_phase += PI; 
    
    // Stance/Swing Logic
    float this_x = 0;
    float this_z = GROUND_Z;
    
    if (leg_phase < PI) {
      // SWING (Move Forward)
      this_x = mapFloat(leg_phase, 0, PI, -STEP_LEN/2, STEP_LEN/2);
      this_z = GROUND_Z + sin(leg_phase) * STEP_HEIGHT; 
    } else {
      // STANCE (Move Backward)
      this_x = mapFloat(leg_phase, PI, 2*PI, STEP_LEN/2, -STEP_LEN/2);
      this_z = GROUND_Z; 
    }
    
    moveLegRelative(leg, this_x, 0, this_z);
  }
  delay(STEP_SPEED);
}

void startWalkingLoop() {
  Serial.println("Action: Walking (Press 'S' to Stop)");
  while(true) {
    if (checkInterrupt()) break; // Exit loop if S is pressed
    doWalkTick(); // Perform one small movement step
  }
}

// --- HELLO WAVE ---
void sitDownForWave() {
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i]; 

  // Front legs Home
  target[LEG_PINS[0][0]] = LEG_HOME[0][0]; target[LEG_PINS[0][1]] = LEG_HOME[0][1]; target[LEG_PINS[0][2]] = LEG_HOME[0][2];
  target[LEG_PINS[1][0]] = LEG_HOME[1][0]; target[LEG_PINS[1][1]] = LEG_HOME[1][1]; target[LEG_PINS[1][2]] = LEG_HOME[1][2];

  // Back Legs Sit
  target[LEG_PINS[2][1]] = LEG_HOME[2][1] + 100; target[LEG_PINS[2][2]] = LEG_HOME[2][2] - 150; 
  target[LEG_PINS[3][1]] = LEG_HOME[3][1] - 100; target[LEG_PINS[3][2]] = LEG_HOME[3][2] + 150; 

  smoothMoveTo(target, 10); 
}

void wavePaw() {
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i]; 

  // Lift
  target[LEG_PINS[1][1]] += 150; 
  target[LEG_PINS[1][2]] += 50;  
  smoothMoveTo(target, 4);

  // Wave
  int centerHip = LEG_HOME[1][0];
  for(int i=0; i<3; i++) { 
    if(checkInterrupt()) return;
    target[LEG_PINS[1][0]] = centerHip - 60; smoothMoveTo(target, 2); 
    target[LEG_PINS[1][0]] = centerHip + 60; smoothMoveTo(target, 2); 
  }

  // Return
  target[LEG_PINS[1][0]] = LEG_HOME[1][0];
  target[LEG_PINS[1][1]] = LEG_HOME[1][1];
  target[LEG_PINS[1][2]] = LEG_HOME[1][2];
  smoothMoveTo(target, 5);
}

void doHelloWave() {
  if(checkInterrupt()) return;
  sitDownForWave();
  delay(500);
  if(checkInterrupt()) return;
  wavePaw();
  delay(500);
  standUp();
}

// --- PUSH UPS (Balanced) ---
void doPushUps() {
  Serial.println("Action: Push Ups");
  int count = 5; // Do 5 pushups
  
  // Settings to prevent falling backward
  float shift_x = 40.0;  // Forward shift when down
  float up_z = -110.0;   // Standing height
  float down_z = -160.0; // Ground height
  float shift_y = -25.0; // Stability width adjustment
  
  for(int k=0; k<count; k++) {
    if(checkInterrupt()) return;
    
    // DOWN (Crouch + Shift Forward)
    for(int i=0; i<=20; i++) {
        float percent = (float)i/20.0;
        float z = (up_z * (1-percent)) + (down_z * percent);
        float x = (0 * (1-percent)) + (shift_x * percent); 
        
        moveLegRelative(0, x, shift_y, z);
        moveLegRelative(1, x, shift_y, z);
        moveLegRelative(2, x, shift_y, z);
        moveLegRelative(3, x, shift_y, z);
        delay(15);
    }
    delay(200);
    
    if(checkInterrupt()) return;

    // UP (Stand + Shift Back)
    for(int i=0; i<=20; i++) {
        float percent = (float)i/20.0;
        float z = (down_z * (1-percent)) + (up_z * percent);
        float x = (shift_x * (1-percent)) + (0 * percent);
        
        moveLegRelative(0, x, shift_y, z);
        moveLegRelative(1, x, shift_y, z);
        moveLegRelative(2, x, shift_y, z);
        moveLegRelative(3, x, shift_y, z);
        delay(15);
    }
    delay(500);
  }
  standUp();
}


// --- HAPPY DANCE ---
void doDance() {
  int target[16];
  int WIGGLE = 40;
  int BOUNCE_DEPTH = 60;
  
  // Wiggle
  for(int k=0; k<4; k++) {
    if(checkInterrupt()) return;
    for(int i=0; i<16; i++) target[i] = currentPos[i];
    
    // Left
    target[LEG_PINS[0][0]] = LEG_HOME[0][0] - WIGGLE; target[LEG_PINS[2][0]] = LEG_HOME[2][0] + WIGGLE; 
    target[LEG_PINS[1][0]] = LEG_HOME[1][0] - WIGGLE; target[LEG_PINS[3][0]] = LEG_HOME[3][0] + WIGGLE; 
    smoothMoveTo(target, 4);

    if(checkInterrupt()) return;
    
    // Right
    target[LEG_PINS[0][0]] = LEG_HOME[0][0] + WIGGLE; target[LEG_PINS[2][0]] = LEG_HOME[2][0] - WIGGLE; 
    target[LEG_PINS[1][0]] = LEG_HOME[1][0] + WIGGLE; target[LEG_PINS[3][0]] = LEG_HOME[3][0] - WIGGLE; 
    smoothMoveTo(target, 4);
  }
  
  standUp();
  delay(500); 

  // Bounce
  for(int k=0; k<3; k++) {
    if(checkInterrupt()) return;
    for(int i=0; i<16; i++) target[i] = currentPos[i];

    // Crouch (Shift Back)
    target[LEG_PINS[0][1]] = LEG_HOME[0][1] + (BOUNCE_DEPTH * 0.7); target[LEG_PINS[1][1]] = LEG_HOME[1][1] - (BOUNCE_DEPTH * 0.7); 
    target[LEG_PINS[2][1]] = LEG_HOME[2][1] + BOUNCE_DEPTH; target[LEG_PINS[3][1]] = LEG_HOME[3][1] - BOUNCE_DEPTH; 
    
    smoothMoveTo(target, 3); 
    standUp(); 
  }
}

// =================================================================
// 7. MAIN SETUP & LOOP
// =================================================================

void setup() {
  Serial.begin(9600);    // USB
  Serial1.begin(9600);   // Bluetooth (Pin 18/19)
  Serial.println("--- Ultimate Robot Ready ---");
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  calculateNeutralAngles();
  
  // Safe Start
  for(int i=0; i<16; i++) {
     pwm.setPWM(i, 0, MID);
     currentPos[i] = MID;
  }
  delay(1000);

  standUp();
}

void loop() {
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    Serial.print("Received: "); Serial.println(command);
    
    if (command == 'W') {
      startWalkingLoop(); // Runs until 'S' is pressed
    }
    else if (command == 'D') {
      doDance();
    }
    else if (command == 'H') {
      doHelloWave();
    }
    else if (command == 'P') {
      doPushUps();
    }
    else if (command == 'S') {
      standUp();
    }
  }
}