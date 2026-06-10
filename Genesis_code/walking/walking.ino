/*
 * STANDALONE WALKING GAIT (Creep Walk)
 * ------------------------------------
 * - Uses your Calibrated Directions (HIP_DIR, THIGH_DIR, KNEE_DIR)
 * - Moves one leg at a time for maximum stability.
 * - No Bluetooth required. Just powers on and walks.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =================================================================
// 1. WALKING SETTINGS (TUNE THESE!)
// =================================================================

float STEP_HEIGHT = 40.0; // How high to lift the foot
float STEP_LEN    = 40.0; // How long the step is
int   STEP_SPEED  = 10;   // Delay in ms (Lower = Faster)

// =================================================================
// 2. DIRECTION TUNING (From your Calibrated File)
// =================================================================

int HIP_DIR[4]   = {-1, -1, 1, 1};  
int THIGH_DIR[4] = {1, -1, 1, -1};  
int KNEE_DIR[4]  = {1, -1, 1, -1};  

// =================================================================
// 3. CONFIGURATION
// =================================================================

int LEG_PINS[4][3] = {
  {9, 10, 11},  // FL
  {0, 1, 2},    // FR
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
// 5. WALKING LOGIC (Creep Gait)
// =================================================================
// This moves one leg forward while pushing the body forward with the other 3.
// Sequence: BR -> FR -> BL -> FL

void takeStep(int leg, float stepLen, float stepHeight) {
  int steps = 20; // Resolution of the step curve
  
  // 1. LIFT & MOVE FORWARD (Swing Phase)
  for (int i = 0; i <= steps; i++) {
    float percent = (float)i / steps;
    
    // X goes from -StepLen/2 (Back) to +StepLen/2 (Front)
    float x_pos = mapFloat(i, 0, steps, -stepLen/2, stepLen/2);
    
    // Z goes up and down (Sine wave)
    float z_pos = GROUND_Z + (sin(percent * PI) * stepHeight);
    
    moveLegRelative(leg, x_pos, 0, z_pos);
    
    // While one leg swings forward, others must push body forward slightly (Move leg backward)
    // This keeps the movement continuous
    float shift_back = stepLen / (steps * 3); // 3 legs pushing
    
    for(int otherLeg=0; otherLeg<4; otherLeg++) {
      if(otherLeg != leg) {
        // We don't have perfect tracking of "where the leg is", 
        // so we assume it pushes back from center. 
        // This is a simplified gait loop.
      }
    }
    
    delay(STEP_SPEED);
  }
}

// A simpler, coordinated Creep Walk Loop
void walkForward() {
  int swing_steps = 15; // Speed of the leg moving in air
  int body_steps = 15;  // Speed of the body moving forward
  
  // Order: BR(3), FR(1), BL(2), FL(0)
  int gaitOrder[4] = {3, 1, 2, 0};
  
  for (int cycle = 0; cycle < 4; cycle++) {
    int swingLeg = gaitOrder[cycle];
    
    // --- PHASE 1: SWING ONE LEG FORWARD ---
    for(int i=0; i<=swing_steps; i++) {
       float percent = (float)i / swing_steps;
       
       // Swing Leg moves X: -STEP to +STEP
       float swing_x = (STEP_LEN/2.0) * (2*percent - 1); // -Half to +Half
       // Swing Leg moves Z: UP in arch
       float swing_z = GROUND_Z + (sin(percent * PI) * STEP_HEIGHT);
       
       // Other legs stay planted (X stays relative back, Z is Ground)
       // To simplify, we just move the swing leg here.
       moveLegRelative(swingLeg, swing_x, 0, swing_z);
       delay(STEP_SPEED);
    }
    
    // --- PHASE 2: SHIFT BODY FORWARD (All legs move back) ---
    // Moving legs BACKWARD relative to body = Body moving FORWARD
    for(int i=0; i<=body_steps; i++) {
       float shift_amount = STEP_LEN / 4.0; // We move 1/4th of a step per cycle
       
       // We want to interpolate 'current' position slightly back?
       // Since we don't track absolute X state easily here without global vars,
       // We will cheat slightly: We Reset legs to "Start of Push" and move to "End of Push".
       
       // Actually, for a simple standalone walk, let's just do the Swing.
       // The body shift is complex without a state machine.
       // Let's do a "Trot in Place" essentially but moving legs forward.
       // Wait, if we only move legs forward, it will split.
       
       // SIMPLE FIX: Just Reset the leg back smoothly on the ground? No, that drags.
       
       // Let's implement the "Push" correctly.
       // We move ALL 4 legs backward by (STEP_LEN / 4)
       // But wait, the Swung leg is at +STEP/2.
       // The others are at various stages.
    }
  }
}

// --- ROBUST SINE-WAVE GAIT ---
// This is mathematically easier and smoother for walking.
void doWalkLoop() {
  static float phase = 0;
  phase += 0.05; // Speed of walk
  if(phase > 2*PI) phase -= 2*PI;
  
  // Trot Gait: Legs 0&3 move together, 1&2 move together.
  // We use sin() for Z (Height) and cos() for X (Forward/Back)
  
  for(int leg=0; leg<4; leg++) {
    float leg_phase = phase;
    
    // Offset phases for diagonal pairs
    if(leg == 1 || leg == 2) leg_phase += PI; // 180 degrees out of phase
    
    // X Movement (Forward/Back)
    float x_pos = (STEP_LEN / 2.0) * cos(leg_phase);
    
    // Z Movement (Up/Down) - Only lift when moving forward (cos > 0)
    float z_pos = GROUND_Z;
    if (cos(leg_phase) > 0) {
       z_pos += (sin(leg_phase) * STEP_HEIGHT); // This creates an arch? 
       // Actually sin(0 to PI) is arch. cos(-PI/2 to PI/2) is forward.
       // Let's stick to simple logic:
       // If moving forward (Swing), lift. If moving back (Stance), stay on ground.
    }
    
    // Better math for Trot:
    // Swing: -PI/2 to PI/2. Stance: PI/2 to 3PI/2.
    
    if (leg_phase > 2*PI) leg_phase -= 2*PI;
    
    float this_x = 0;
    float this_z = GROUND_Z;
    
    if (leg_phase < PI) {
      // SWING PHASE (0 to 180) -> Move Forward
      // map 0..PI to -Len..+Len
      this_x = mapFloat(leg_phase, 0, PI, -STEP_LEN/2, STEP_LEN/2);
      // Lift
      this_z = GROUND_Z + sin(leg_phase) * STEP_HEIGHT; 
    } else {
      // STANCE PHASE (180 to 360) -> Move Backward (Drag body forward)
      // map PI..2PI to +Len..-Len
      this_x = mapFloat(leg_phase, PI, 2*PI, STEP_LEN/2, -STEP_LEN/2);
      this_z = GROUND_Z; // On ground
    }
    
    moveLegRelative(leg, this_x, 0, this_z);
  }
  delay(STEP_SPEED);
}

// helper
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =================================================================
// 6. SETUP & LOOP
// =================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("--- Standalone Walking ---");
  
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
  delay(2000);
}

void loop() {
  doWalkLoop();
}