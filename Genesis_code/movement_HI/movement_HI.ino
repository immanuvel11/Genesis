#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- CONFIGURATION ---
#define SERVO_FREQ 50
// Your specific 90-degree "Zero"
#define MID 375 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// We track current position to move smoothly
int currentPos[16]; 

// --- YOUR TUNED HOME VALUES ---
// FL, FR, BL, BR
int HIP_HOME[4]   = {360, 430, 360, 375};
int THIGH_HOME[4] = {355, 360, 375, 340};
int KNEE_HOME[4]  = {390, 380, 360, 390};

// Map leg ID to Pin Index
// Leg 0=FL, 1=FR, 2=BL, 3=BR
int LEG_PINS[4][3] = {
  {9, 10, 11},  // FL
  {0, 1, 2},    // FR```
  {12, 13, 14}, // BL
  {4, 5, 6}     // BR
};

void setup() {
  Serial.begin(9600);
  Serial.println("--- Robot Hello Wave (Direction Fixed) ---");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  // 1. Initialize System
  for(int i=0; i<16; i++) {
     pwm.setPWM(i, 0, MID);
     currentPos[i] = MID;
  }
  delay(1000);

  // 2. Move to Stand Slowly
  Serial.println("Standing Up...");
  standUp();
  delay(1000);
}

void loop() {
  
  // --- STEP 1: SIT DOWN ---
  Serial.println("Sitting...");
  sitDown();
  delay(500);

  // --- STEP 2: WAVE HELLO ---
  Serial.println("Waving!");
  wavePaw();
  delay(500);

  // --- STEP 3: STAND UP ---
  Serial.println("Standing...");
  standUp();
  delay(2000); 
}

// ==========================================
// POSES
// ==========================================

void standUp() {
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i]; // Init with current

  for(int leg=0; leg<4; leg++) {
    target[LEG_PINS[leg][0]] = HIP_HOME[leg];
    target[LEG_PINS[leg][1]] = THIGH_HOME[leg];
    target[LEG_PINS[leg][2]] = KNEE_HOME[leg];
  }
  smoothMoveTo(target, 5); 
}

void sitDown() {
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i]; // Init with current

  // Front legs straight
  target[LEG_PINS[0][0]] = HIP_HOME[0];
  target[LEG_PINS[0][1]] = THIGH_HOME[0];
  target[LEG_PINS[0][2]] = KNEE_HOME[0];

  target[LEG_PINS[1][0]] = HIP_HOME[1];
  target[LEG_PINS[1][1]] = THIGH_HOME[1];
  target[LEG_PINS[1][2]] = KNEE_HOME[1];

  // MODIFY BACK LEGS TO SIT
  // Left Back (Leg 2): Thigh UP (+)
  target[LEG_PINS[2][1]] = THIGH_HOME[2] + 100; 
  target[LEG_PINS[2][2]] = KNEE_HOME[2] - 150; 

  // Right Back (Leg 3): Thigh DOWN (-)
  target[LEG_PINS[3][1]] = THIGH_HOME[3] - 100; 
  target[LEG_PINS[3][2]] = KNEE_HOME[3] + 150; 

  smoothMoveTo(target, 10); 
}

void wavePaw() {
  // We will move ONLY the Front Right Leg (Leg 1)
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i]; // Freeze others

  // 1. LIFT PAW (FIXED DIRECTION)
  // Previous attempt: -= 150 (Moved Back/In)
  // New attempt:      += 150 (Should Move Front/Out)
  
  target[LEG_PINS[1][1]] += 150; // Lift Thigh (Flipped sign)
  target[LEG_PINS[1][2]] += 50;  // Straighten Knee (Flipped sign)
  smoothMoveTo(target, 4);

  // 2. WAVE (Wiggle Hip Left/Right)
  int centerHip = HIP_HOME[1];
  
  for(int i=0; i<3; i++) { 
    // Wave Out (Tried to guess direction, swapping to be safe)
    target[LEG_PINS[1][0]] = centerHip - 60; // Try Negative first
    smoothMoveTo(target, 2); 
    
    // Wave In
    target[LEG_PINS[1][0]] = centerHip + 60; // Try Positive second
    smoothMoveTo(target, 2); 
  }

  // 3. RETURN PAW
  target[LEG_PINS[1][0]] = HIP_HOME[1];
  target[LEG_PINS[1][1]] = THIGH_HOME[1];
  target[LEG_PINS[1][2]] = KNEE_HOME[1];
  smoothMoveTo(target, 5);
}

// ==========================================
// SMOOTH MOVEMENT ENGINE
// ==========================================
void smoothMoveTo(int targetPos[16], int speedDelay) {
  bool moving = true;
  while(moving) {
    moving = false; 
    for(int i=0; i<16; i++) {
      if(i==3 || i==7 || i==11 || i==15) continue; 

      if (currentPos[i] < targetPos[i]) {
        currentPos[i]++;
        pwm.setPWM(i, 0, currentPos[i]);
        moving = true; 
      } 
      else if (currentPos[i] > targetPos[i]) {
        currentPos[i]--;
        pwm.setPWM(i, 0, currentPos[i]);
        moving = true;
      }
    }
    delay(speedDelay); 
  }
}