#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =================================================================
// 1. CONFIGURATION
// =================================================================

// --- PINS ---
int LEG_PINS[4][3] = {
  {9, 10, 11},  // FL
  {0, 1, 2},    // FR
  {12, 13, 14}, // BL
  {4, 5, 7}     // BR
};

// --- HOME POSITIONS ---
int LEG_HOME[4][3] = {
  {350, 355, 400}, // FL
  {400, 375, 375}, // FR
  {365, 340, 375}, // BL
  {400, 375, 375}  // BR
};

// --- SETTINGS ---
float L1 = 115.0; 
float L2 = 137.0; 
bool IS_RIGHT_LEG[4] = {false, true, false, true};

// --- WALKING SETTINGS ---
float GROUND_Z  = -160.0;
float STEP_LIFT = 40.0;
float STEP_LEN  = 50.0;   
int STEP_SPEED  = 5;
bool INVERT_FORWARD = true; 

// --- SIT UP SETTINGS ---
float SIT_MIN_Z = -110.0; 
float SIT_MAX_Z = -160.0;
float SIT_SHIFT_X = 35.0; 
float SIT_SHIFT_Y = -25.0;

// --- HIP DIRECTION FIX ---
int HIP_DIR[4] = {-1, -1, 1, 1}; 

// =================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50
#define MIN_PULSE 150 
#define MAX_PULSE 600 
#define MID 375 

int currentPos[16]; 
float thigh_angle_zero, knee_angle_zero; 
float body_offset_x = 0;
float body_offset_y = 0;

// =================================================================
// 2. MATH HELPERS
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
  if (IS_RIGHT_LEG[legNum]) x = -x; 

  // Hip Math
  float hip_deg_change = y * 0.5;
  if (IS_RIGHT_LEG[legNum]) hip_deg_change = -hip_deg_change; 
  
  // Apply Fix
  bool isFront = (legNum == 0 || legNum == 1);
  if (isFront) hip_deg_change *= HIP_DIR[0]; 
  else         hip_deg_change *= HIP_DIR[2]; 

  // IK Math
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
  
  int hip_change   = hip_deg_change * pulses_per_degree;
  int thigh_change = thigh_diff * pulses_per_degree;
  int knee_change  = knee_diff  * pulses_per_degree;

  if (IS_RIGHT_LEG[legNum]) {
      thigh_change = -thigh_change; 
      knee_change = -knee_change;
  }

  // Update Global Position Tracking
  currentPos[LEG_PINS[legNum][0]] = LEG_HOME[legNum][0] + hip_change;
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
// 3. DANCE LOGIC (UPDATED)
// =================================================================

void doDance() {
  int target[16];
  int WIGGLE = 40;
  int BOUNCE_DEPTH = 60;
  int BOUNCE_BACK_SHIFT = 30; // SHIFT WEIGHT BACKWARD during bounce!

  // 1. WIGGLE LOOP
  for(int k=0; k<4; k++) {
    for(int i=0; i<16; i++) target[i] = currentPos[i];
    
    // Twist Left
    target[LEG_PINS[0][0]] = LEG_HOME[0][0] - WIGGLE; 
    target[LEG_PINS[2][0]] = LEG_HOME[2][0] + WIGGLE; 
    target[LEG_PINS[1][0]] = LEG_HOME[1][0] - WIGGLE; 
    target[LEG_PINS[3][0]] = LEG_HOME[3][0] + WIGGLE; 
    smoothMoveTo(target, 4);

    // Twist Right
    target[LEG_PINS[0][0]] = LEG_HOME[0][0] + WIGGLE; 
    target[LEG_PINS[2][0]] = LEG_HOME[2][0] - WIGGLE; 
    target[LEG_PINS[1][0]] = LEG_HOME[1][0] + WIGGLE; 
    target[LEG_PINS[3][0]] = LEG_HOME[3][0] - WIGGLE; 
    smoothMoveTo(target, 4);
  }
  
  // Center Hips before bounce
  standUp();

  // --- ADDED DELAY HERE (0.5 Seconds) ---
  delay(500); 

  // 2. BOUNCE LOOP (Modified for Stability)
  for(int k=0; k<3; k++) {
    for(int i=0; i<16; i++) target[i] = currentPos[i];

    // Crouch (Bounce Down) + SHIFT BACK
    // To shift body BACK, we do the opposite of Forward.
    // Forward usually means Front Thighs Down, Back Thighs Up?
    // Or we can use moveLegRelative logic but we are in direct pulse mode here.
    // Let's assume:
    // Left Legs: Thigh + = Up/Back?  Right Legs: Thigh - = Up/Back?
    // Let's keep it simple: Just modify the Thigh targets to lean back.
    
    // Front Legs (0, 1): Move Thighs more "Up" (relative to crouching) to keep chest high?
    // No, let's just crouch but maybe not as deep on front?
    // Actually, shifting CoM back means moving FEET forward.
    // Moving feet forward (Thigh down/forward) might be tricky here with raw pulses.
    
    // Easier Fix: Crouch LESS deep on Front, MORE deep on Back.
    // This tilts body up/back.
    
    // Crouch target calculation
    // Front: Go down only 70% of depth
    target[LEG_PINS[0][1]] = LEG_HOME[0][1] + (BOUNCE_DEPTH * 0.7); 
    target[LEG_PINS[1][1]] = LEG_HOME[1][1] - (BOUNCE_DEPTH * 0.7); 
    
    // Back: Go down full depth
    target[LEG_PINS[2][1]] = LEG_HOME[2][1] + BOUNCE_DEPTH; 
    target[LEG_PINS[3][1]] = LEG_HOME[3][1] - BOUNCE_DEPTH; 
    
    smoothMoveTo(target, 3); // Slightly slower (3 vs 2) for safety

    // Stand (Bounce Up)
    standUp(); 
  }
}

// =================================================================
// 4. SETUP & LOOP
// =================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("--- Dance Calibration ---");
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  calculateNeutralAngles();
  
  for(int i=0; i<16; i++) {
     pwm.setPWM(i, 0, MID);
     currentPos[i] = MID;
  }
  delay(1000);

  standUp();
  delay(2000);
}

void loop() {
  Serial.println("Dancing...");
  doDance();
  delay(2000); // Wait before repeating
}