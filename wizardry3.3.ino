#include <math.h>

// --- HARDWARE PINS ---
#define EN_PIN      8
#define X_STEP_PIN  2
#define X_DIR_PIN   5
#define Y_STEP_PIN  3
#define Y_DIR_PIN   6
#define Z_STEP_PIN  4    
#define Z_DIR_PIN   7    
#define Z_STOP_PIN  10  

#define L_LIMIT_PIN 9   
#define R_LIMIT_PIN 13  

// --- MOTOR & SENSOR LOGIC ---
const bool INVERT_LEFT_MOTOR  = true;  
const bool INVERT_RIGHT_MOTOR = false;
const bool INVERT_Z_MOTOR     = true;  

#define Z_PRESSED HIGH
#define Z_OPEN    LOW

// --- MACHINE CONSTANTS ---
const float STEPS_PER_INCH   = 2032.0;
const float Z_STEPS_PER_INCH = 2000.0; 
const float motorDist        = 42.0;
const float pulleyRadius     = 0.3183;
const float resolution       = 0.05;   
const float Z_LIFT_DISTANCE  = 0.25;   

// --- YOUR PHYSICAL MEASUREMENTS ---
const float CAL_L_MEASURED = 12.0;
const float CAL_R_MEASURED = 11.5625; 

// --- GLOBAL STATE ---
float curX = 0;
float curY = 0;
long stepsL = 0;
long stepsR = 0;
int stepDelay = 800;
bool penIsDown = false;
bool isCalibrated = false;

// --- STEPPING HELPERS ---
void stepZ(int speed) {
  digitalWrite(Z_STEP_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(Z_STEP_PIN, LOW);
  delayMicroseconds(speed);
}

void safetyRetract(int durationMs) {
  digitalWrite(Z_DIR_PIN, INVERT_Z_MOTOR); 
  unsigned long startWait = millis();
  while (millis() - startWait < durationMs) {
    stepZ(800); 
  }
}

// --- Z-AXIS CONTROL (THE PLUNGER LOGIC) ---
void moveZ(bool down) {
  if (penIsDown == down) return; 
  
  if (down) {
    digitalWrite(Z_DIR_PIN, !INVERT_Z_MOTOR); 
    // PLUNGE: Drive until the limit switch hits the board/paper
    while (digitalRead(Z_STOP_PIN) == Z_OPEN) {
      stepZ(1200); 
    }
    // Small extra nudge to ensure bit engagement
    for(int i=0; i<150; i++) stepZ(1500); 
    penIsDown = true;
  } 
  else {
    digitalWrite(Z_DIR_PIN, INVERT_Z_MOTOR); 
    // RETRACT: Move up by the specified clearance distance
    long liftSteps = (long)(Z_LIFT_DISTANCE * Z_STEPS_PER_INCH);
    for (long i = 0; i < liftSteps; i++) {
      stepZ(800);
    }
    penIsDown = false;
  }
  delay(300); // Settle vibrations before X/Y movement
}

// --- CORE KINEMATICS ---
long getStepsForCoord(float x, float y, bool isLeft) {
  float mx = isLeft ? 0 : motorDist;
  float dx = x - mx;
  float dy = y;
  float d2 = (dx * dx) + (dy * dy);
  float d = sqrt(d2);
  if (d < (pulleyRadius + 0.1)) return isLeft ? stepsL : stepsR;
  float l_straight = sqrt(max(0.0f, d2 - (pulleyRadius * pulleyRadius)));
  float theta = atan2(dy, dx);
  float phi = acos(constrain(pulleyRadius / d, -1.0, 1.0));
  float wrapAngle = isLeft ? (M_PI/2.0 - (theta + phi)) : (theta - phi - M_PI/2.0);
  return (long)((l_straight + (pulleyRadius * fabs(wrapAngle))) * STEPS_PER_INCH);
}

void moveRelative(float dx, float dy) {
  float absTargetX = curX + dx;
  float absTargetY = curY + dy;
  float totalDist = sqrt((dx * dx) + (dy * dy));
  int segments = max(1, (int)(totalDist / resolution));
  
  for (int s = 1; s <= segments; s++) {
    float subTargetX = curX + (dx * ((float)s / segments));
    float subTargetY = curY + (dy * ((float)s / segments));
    long goalL = getStepsForCoord(subTargetX, subTargetY, true);
    long goalR = getStepsForCoord(subTargetX, subTargetY, false);
    
    long deltaL = abs(goalL - stepsL);
    long deltaR = abs(goalR - stepsR);
    long maxDelta = max(deltaL, deltaR);
    
    digitalWrite(X_DIR_PIN, INVERT_LEFT_MOTOR ? (goalL < stepsL) : (goalL > stepsL));
    digitalWrite(Y_DIR_PIN, INVERT_RIGHT_MOTOR ? (goalR < stepsR) : (goalR > stepsR));
    
    long errorL = maxDelta / 2;
    long errorR = maxDelta / 2;
    
    for (long i = 0; i < maxDelta; i++) {
      bool pulse = false;
      errorL -= deltaL;
      if (errorL < 0) { digitalWrite(X_STEP_PIN, HIGH); stepsL += (goalL > stepsL) ? 1 : -1; errorL += maxDelta; pulse = true; }
      errorR -= deltaR;
      if (errorR < 0) { digitalWrite(Y_STEP_PIN, HIGH); stepsR += (goalR > stepsR) ? 1 : -1; errorR += maxDelta; pulse = true; }
      if (pulse) { delayMicroseconds(10); digitalWrite(X_STEP_PIN, LOW); digitalWrite(Y_STEP_PIN, LOW); }
      delayMicroseconds(stepDelay);
    }
  }
  curX = absTargetX; curY = absTargetY;
  Serial.println("DONE"); // Signal Python that this step is finished
}

// --- CALIBRATION ROUTINE ---
void runAutoCalibration() {
  Serial.println("Homing Sequence Starting...");
  int calSpeed = 500; 
  long secondarySlackCount = 0;

  // Step 1: Home Left & Slack Right
  digitalWrite(X_DIR_PIN, INVERT_LEFT_MOTOR);  
  digitalWrite(Y_DIR_PIN, !INVERT_RIGHT_MOTOR); 
  while(digitalRead(L_LIMIT_PIN) == LOW) {
    digitalWrite(X_STEP_PIN, HIGH); digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(10); digitalWrite(X_STEP_PIN, LOW); digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(calSpeed);
  }
  delay(200);

  // Step 2: Home Right & Slack Left
  digitalWrite(Y_DIR_PIN, INVERT_RIGHT_MOTOR);  
  digitalWrite(X_DIR_PIN, !INVERT_LEFT_MOTOR); 
  while(digitalRead(R_LIMIT_PIN) == LOW) {
    digitalWrite(X_STEP_PIN, HIGH); digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(10); digitalWrite(X_STEP_PIN, LOW); digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(calSpeed);
    secondarySlackCount++; 
  }

  stepsR = (long)(CAL_R_MEASURED * STEPS_PER_INCH);
  stepsL = (long)(CAL_L_MEASURED * STEPS_PER_INCH) + secondarySlackCount;

  float L = (float)stepsL / STEPS_PER_INCH;
  float R = (float)stepsR / STEPS_PER_INCH;
  curX = ((L * L) - (R * R) + (motorDist * motorDist)) / (2.0 * motorDist);
  curY = sqrt(max(0.0f, (L * L) - (curX * curX)));

  Serial.println("Calibration complete. Centering Gondola...");
  moveRelative(21.0 - curX, 15.0 - curY);
  isCalibrated = true;
  Serial.println("CNC_READY");
}

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT); pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT); pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT); pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(L_LIMIT_PIN, INPUT_PULLUP);
  pinMode(R_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Z_STOP_PIN, INPUT_PULLUP);
  
  digitalWrite(EN_PIN, LOW); 
  Serial.println("WIZARD_ONLINE: Send 'H' to home.");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'H' || cmd == 'h') {
      safetyRetract(2000); 
      runAutoCalibration();
    }
    
    if ((cmd == 'G' || cmd == 'g') && isCalibrated) {
      int type = Serial.parseInt(); 
      float dx = Serial.parseFloat();
      float dy = Serial.parseFloat();
      
      // Execute Z-Axis first, wait for it to finish
      if (type == 0) moveZ(true);  
      else moveZ(false);           
      
      // Now execute the X/Y movement
      moveRelative(dx, dy);
    }
  }
}