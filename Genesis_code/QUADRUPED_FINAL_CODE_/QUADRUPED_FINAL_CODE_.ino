/*
 * ULTIMATE BLUETOOTH ROBOT (Separate Wave vs Handshake)
 * -----------------------------------------------------
 * COMMANDS:
 * - 'F': Walk Forward
 * - 'B': Walk Backward
 * - 'D': Happy Dance
 * - 'P': Pushups
 * - 'H': HELLO WAVE (Wiggles Left/Right)
 * - 'O': Shutdown
 * - 'S': Stop/Stand -> Enters Watch Mode
 *
 * SENSOR TRIGGER:
 * - When hand is detected -> HANDSHAKE (Shakes Up/Down)
 *
 * WIRING:
 * - Bluetooth: TX->19, RX->18 (Serial1)
 * - Sensor: Trig->17, Echo->16
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =================================================================
// 1. SETTINGS & PINS
// =================================================================

#define TRIG_PIN 17
#define ECHO_PIN 16
#define SHAKE_DIST 15 

int LEG_PINS[4][3] = {
  {8, 9, 10},   // FL
  {0, 1, 2},    // FR
  {12, 13, 14}, // BL
  {4, 5, 6}     // BR
};

int LEG_HOME[4][3] = {
  {360, 355, 390}, // FL
  {430, 360, 380}, // FR
  {360, 375, 360}, // BL
  {375, 340, 390}  // BR
};

int HIP_DIR[4]   = {-1, -1, 1, 1};  
int THIGH_DIR[4] = {1, -1, 1, -1};  
int KNEE_DIR[4]  = {1, -1, 1, -1};  

float STEP_HEIGHT = 40.0; 
float STEP_LEN    = 40.0; 
int   STEP_SPEED  = 10;   

float L1 = 115.0; 
float L2 = 137.0; 
float GROUND_Z  = -160.0;
bool INVERT_FORWARD = true; 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50
#define MID 375 

int currentPos[16]; 
float thigh_angle_zero, knee_angle_zero; 
bool isWatchMode = true; 

// =================================================================  
// 2. HELPERS
// =================================================================

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
  float hip_deg_change = y * 0.5 * HIP_DIR[legNum]; 
  float D = sqrt(x*x + z*z);
  if (D > (L1 + L2)) return; 
  float a1 = atan2(x, -z);
  float a2 = acos((sq(L1) + sq(D) - sq(L2)) / (2 * L1 * D));
  float b  = acos((sq(L1) + sq(L2) - sq(D)) / (2 * L1 * L2));
  float thigh_deg = (a1 + a2) * 180 / PI;
  float knee_deg  = b * 180 / PI;
  float thigh_diff = thigh_deg - thigh_angle_zero;
  float knee_diff  = knee_deg  - knee_angle_zero;
  float pulses_per_degree = (600 - 150) / 180.0;
  
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

void standUp() {
  int target[16];
  for(int i=0; i<16; i++) target[i] = currentPos[i]; 
  for(int leg=0; leg<4; leg++) {
    target[LEG_PINS[leg][0]] = LEG_HOME[leg][0];
    target[LEG_PINS[leg][1]] = LEG_HOME[leg][1];
    target[LEG_PINS[leg][2]] = LEG_HOME[leg][2];
  }
  smoothMoveTo(target, 5); 
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

bool checkInterrupt() {
  if (Serial1.available() > 0) {
    char cmd = Serial1.read();
    if (cmd == 'S') { isWatchMode = true; standUp(); return true; }
    if (cmd == 'O') { isWatchMode = false; return true; }
    if (cmd == 'F') INVERT_FORWARD = true;
    if (cmd == 'B') INVERT_FORWARD = false;
  }
  return false; 
}

// =================================================================
// 3. MOVEMENT SEQUENCES
// =================================================================

// --- 1. HELLO WAVE (Wiggle Hip Left/Right) ---
// Triggered by: 'H' Command
void doHelloWave() {
  Serial.println("Action: Hello Wave (Wiggle)!");
  int target[16]; for(int i=0; i<16; i++) target[i] = currentPos[i]; 

  // Sit Gentle
  target[LEG_PINS[0][0]] = LEG_HOME[0][0]; target[LEG_PINS[0][1]] = LEG_HOME[0][1]; target[LEG_PINS[0][2]] = LEG_HOME[0][2];
  target[LEG_PINS[1][0]] = LEG_HOME[1][0]; target[LEG_PINS[1][1]] = LEG_HOME[1][1]; target[LEG_PINS[1][2]] = LEG_HOME[1][2];
  target[LEG_PINS[2][1]] = LEG_HOME[2][1] + 60; target[LEG_PINS[2][2]] = LEG_HOME[2][2] - 90; 
  target[LEG_PINS[3][1]] = LEG_HOME[3][1] - 60; target[LEG_PINS[3][2]] = LEG_HOME[3][2] + 90; 
  if(checkInterrupt()) return;
  smoothMoveTo(target, 15); 
  
  // Lift Paw
  target[LEG_PINS[1][1]] += 150; target[LEG_PINS[1][2]] += 50;  
  smoothMoveTo(target, 4);

  // WIGGLE (Left/Right)
  int centerHip = LEG_HOME[1][0];
  for(int i=0; i<3; i++) { 
    if(checkInterrupt()) return;
    target[LEG_PINS[1][0]] = centerHip - 60; smoothMoveTo(target, 2); 
    target[LEG_PINS[1][0]] = centerHip + 60; smoothMoveTo(target, 2); 
  }
  
  standUp(); delay(1000); 
}

// --- 2. HANDSHAKE (Shake Up/Down) ---
// Triggered by: SENSOR
void doHandshake() {
  Serial.println("Action: Handshake (Up/Down)!");
  int target[16]; for(int i=0; i<16; i++) target[i] = currentPos[i]; 

  // Sit Gentle
  target[LEG_PINS[0][0]] = LEG_HOME[0][0]; target[LEG_PINS[0][1]] = LEG_HOME[0][1]; target[LEG_PINS[0][2]] = LEG_HOME[0][2];
  target[LEG_PINS[1][0]] = LEG_HOME[1][0]; target[LEG_PINS[1][1]] = LEG_HOME[1][1]; target[LEG_PINS[1][2]] = LEG_HOME[1][2];
  target[LEG_PINS[2][1]] = LEG_HOME[2][1] + 60; target[LEG_PINS[2][2]] = LEG_HOME[2][2] - 90; 
  target[LEG_PINS[3][1]] = LEG_HOME[3][1] - 60; target[LEG_PINS[3][2]] = LEG_HOME[3][2] + 90; 
  if(checkInterrupt()) return;
  smoothMoveTo(target, 15); 
  
  // Lift Paw
  target[LEG_PINS[1][1]] += 150; target[LEG_PINS[1][2]] += 50;  
  smoothMoveTo(target, 4);

  // SHAKE (Up/Down)
  int high_pos = target[LEG_PINS[1][1]]; 
  int low_pos = high_pos - 40; 

  for(int i=0; i<3; i++) {
    if(checkInterrupt()) return;
    target[LEG_PINS[1][1]] = low_pos; smoothMoveTo(target, 3);
    target[LEG_PINS[1][1]] = high_pos; smoothMoveTo(target, 3);
  }
  
  standUp(); delay(1000); 
}

void doShutdown() {
  Serial.println("Shutting Down..."); isWatchMode = false; 
  float start_z = GROUND_Z; float end_z = -60.0; 
  for(int i=0; i<=30; i++) {
    float percent = (float)i/30.0;
    float current_z = (start_z * (1-percent)) + (end_z * percent);
    moveLegRelative(0, 0, 0, current_z); moveLegRelative(1, 0, 0, current_z);
    moveLegRelative(2, 0, 0, current_z); moveLegRelative(3, 0, 0, current_z);
    delay(20);
  }
}

void startWalkingLoop() {
  if (INVERT_FORWARD) Serial.println("Fwd..."); else Serial.println("Bwd...");
  static float phase = 0;
  while(true) {
    if (checkInterrupt()) break; 
    phase += 0.05; if(phase > 2*PI) phase -= 2*PI;
    for(int leg=0; leg<4; leg++) {
      float leg_phase = phase; if(leg == 1 || leg == 2) leg_phase += PI; 
      if(leg_phase > 2*PI) leg_phase -= 2*PI;
      float this_x = 0, this_z = GROUND_Z;
      if (leg_phase < PI) {
        this_x = mapFloat(leg_phase, 0, PI, -STEP_LEN/2, STEP_LEN/2);
        this_z = GROUND_Z + sin(leg_phase) * STEP_HEIGHT; 
      } else {
        this_x = mapFloat(leg_phase, PI, 2*PI, STEP_LEN/2, -STEP_LEN/2);
        this_z = GROUND_Z; 
      }
      moveLegRelative(leg, this_x, 0, this_z);
    }
    delay(STEP_SPEED);
  }
}

void doPushUps() {
  Serial.println("Push Ups...");
  float up_z = -110.0; float down_z = -140.0; float shift_x = 40.0; float shift_y = 0.0; 
  for(int k=0; k<5; k++) {
    if(checkInterrupt()) return;
    for(int i=0; i<=20; i++) {
        float percent = (float)i/20.0;
        float z = (up_z * (1-percent)) + (down_z * percent);
        float x = (0 * (1-percent)) + (shift_x * percent); 
        moveLegRelative(0, x, shift_y, z); moveLegRelative(1, x, shift_y, z); moveLegRelative(2, x, shift_y, z); moveLegRelative(3, x, shift_y, z);
        delay(25);
    }
    delay(200);
    for(int i=0; i<=20; i++) {
        float percent = (float)i/20.0;
        float z = (down_z * (1-percent)) + (up_z * percent);
        float x = (shift_x * (1-percent)) + (0 * percent);
        moveLegRelative(0, x, shift_y, z); moveLegRelative(1, x, shift_y, z); moveLegRelative(2, x, shift_y, z); moveLegRelative(3, x, shift_y, z);
        delay(25);
    }
    delay(500);
  }
  standUp();
}

void doDance() {
  int target[16]; int WIGGLE = 40; int BOUNCE_DEPTH = 60;
  for(int k=0; k<4; k++) {
    if(checkInterrupt()) return;
    for(int i=0; i<16; i++) target[i] = currentPos[i];
    target[LEG_PINS[0][0]] = LEG_HOME[0][0] - WIGGLE; target[LEG_PINS[2][0]] = LEG_HOME[2][0] + WIGGLE; 
    target[LEG_PINS[1][0]] = LEG_HOME[1][0] - WIGGLE; target[LEG_PINS[3][0]] = LEG_HOME[3][0] + WIGGLE; 
    smoothMoveTo(target, 4);
    if(checkInterrupt()) return;
    target[LEG_PINS[0][0]] = LEG_HOME[0][0] + WIGGLE; target[LEG_PINS[2][0]] = LEG_HOME[2][0] - WIGGLE; 
    target[LEG_PINS[1][0]] = LEG_HOME[1][0] + WIGGLE; target[LEG_PINS[3][0]] = LEG_HOME[3][0] - WIGGLE; 
    smoothMoveTo(target, 4);
  }
  standUp(); delay(500); 
  for(int k=0; k<3; k++) {
    if(checkInterrupt()) return;
    for(int i=0; i<16; i++) target[i] = currentPos[i];
    target[LEG_PINS[0][1]] = LEG_HOME[0][1] + (BOUNCE_DEPTH * 0.7); target[LEG_PINS[1][1]] = LEG_HOME[1][1] - (BOUNCE_DEPTH * 0.7); 
    target[LEG_PINS[2][1]] = LEG_HOME[2][1] + BOUNCE_DEPTH; target[LEG_PINS[3][1]] = LEG_HOME[3][1] - BOUNCE_DEPTH; 
    smoothMoveTo(target, 3); 
    standUp(); 
  }
}

// =================================================================
// 4. MAIN LOOP
// =================================================================

void setup() {
  Serial.begin(9600);    
  Serial1.begin(9600); 
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pwm.begin(); pwm.setPWMFreq(SERVO_FREQ);
  calculateNeutralAngles();
  for(int i=0; i<16; i++) { pwm.setPWM(i, 0, MID); currentPos[i] = MID; }
  delay(1000); standUp();
}

void loop() {
  // Bluetooth Handler
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    
    if (command == 'S') { isWatchMode = true; standUp(); }
    else if (command == 'O') { doShutdown(); }
    else {
      isWatchMode = false; 
      if (command == 'F') { INVERT_FORWARD = true; startWalkingLoop(); }
      else if (command == 'B') { INVERT_FORWARD = false; startWalkingLoop(); }
      else if (command == 'D') doDance();
      else if (command == 'P') doPushUps();
      else if (command == 'H') doHelloWave(); // MANUAL -> WIGGLE
    }
  }

  // Sensor Handler
  if (isWatchMode) {
    static long lastCheck = 0;
    if (millis() - lastCheck > 150) {
      lastCheck = millis();
      long dist = getDistance();
      if (dist > 0 && dist < SHAKE_DIST) {
        doHandshake(); // SENSOR -> SHAKE UP/DOWN
      }
    }
  }
}