// Arduino.ino
#include <Servo.h>
//test
// ── Pin definitions ────────────────────────────────────────────────────────────
#define BASE_STEP       3
#define BASE_DIR        2
#define SHOULDER_STEP   5
#define SHOULDER_DIR    4
#define PIN_ELBOW       10
#define PIN_GRIPPER     9

// Roll-about-link1 servo — attached and locked at 0°; servo PID resists any external torque
#define PIN_ROLL        11           // PWM pin for roll servo — verify pin with hardware

// ── Stepper parameters ─────────────────────────────────────────────────────────
#define STEPS_PER_REV   200         // full steps per motor revolution
#define MICROSTEP       16          // microstepping setting on the driver

// Calibrated from hardware test (sketch_mar28a.ino):
//   rotateStepper(12) -> ~180° physical base rotation -> motor-deg/physical-deg = 12/180
//   rotateStepper(25) -> ~90° physical shoulder rotation  -> motor-deg/physical-deg = 25/90
#define BASE_STEPS_PER_DEG      ((STEPS_PER_REV * MICROSTEP) / 360.0f * (12.0f / 180.0f))
#define SHOULDER_STEPS_PER_DEG  ((STEPS_PER_REV * MICROSTEP) / 360.0f * (25.0f / 90.0f))

#define STEP_DELAY_US   15000       // microseconds between step pulses; controls speed

// ── Angle limits (physical degrees) ───────────────────────────────────────────
#define BASE_MAX_DEG        180.0f  
#define SHOULDER_MAX_DEG    144.0f  // max = 40 input units ≈ 144°
#define ELBOW_NEUTRAL_DEG   90      // servo neutral
#define GRIPPER_CLOSED_DEG  30      // servo closed
#define GRIPPER_OPEN_DEG    0       // servo fully open

// ── Serial configuration ───────────────────────────────────────────────────────
#define BAUD            9600
#define MAX_MSG_LEN     64          // maximum length of one CSV line

#define NB_ANGLES       4           // base, shoulder, elbow, gripper

Servo servo_roll;                   // roll servo — locked at 0°, never commanded again
Servo servo_elbow;
Servo servo_gripper;

float  target_angles[NB_ANGLES];    // joint angles from the last command
bool   letter_sorted = false;       // true when the letter has been placed

long base_steps_current     = 0;    // current base position in steps
long shoulder_steps_current = 0;    // current shoulder position in steps


// ── Homing ────────────────────────────────────────────────────────────────────
// Drives axis into its hard-stop (3× slower), then backs off by backoff_deg.
// stepsPerDeg is the joint-specific calibrated conversion.
void homeAxis(int stepPin, int dirPin, long &currentSteps, int dirToStop,
              float homeAngle_deg, float backoff_deg, float stepsPerDeg) {

  digitalWrite(dirPin, dirToStop);
  long maxSteps = (long)(200.0f * stepsPerDeg); // 270° physical as safe sweep limit

  for (long i = 0; i < maxSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US * 3);       // 3× slower to reduce impact force
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US * 3);
  }

  currentSteps = (long)(homeAngle_deg * stepsPerDeg);

  int  backoffDir   = (dirToStop == HIGH) ? LOW : HIGH;
  long backoffSteps = (long)(backoff_deg * stepsPerDeg);

  digitalWrite(dirPin, backoffDir);
  for (long i = 0; i < backoffSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US * 2);       // 2× slower than normal
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US * 2);
  }

  if (backoffDir == HIGH) currentSteps += backoffSteps;
  else currentSteps -= backoffSteps;
}


// ── Serial parser ─────────────────────────────────────────────────────────────
bool parseSerial() {
  static char  buf[MAX_MSG_LEN];
  static int   idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;

      char* token = strtok(buf, ",");         // first token: letter_sorted (0 or 1)
      if (token == NULL) return false;
      letter_sorted = atoi(token);

      for (int i = 0; i < NB_ANGLES; i++) {  // next 4 tokens: joint angles
        token = strtok(NULL, ",");
        if (token == NULL) return false;
        target_angles[i] = atof(token);
      }
      return true;
    }

    if (idx < MAX_MSG_LEN - 1) buf[idx++] = c;
  }
  return false;
}


// ── Stepper move (absolute, physical degrees) ─────────────────────────────────
void moveStepper(int stepPin, int dirPin, long &currentSteps,
                 float targetDeg, float stepsPerDeg) {
  long targetSteps = (long)(targetDeg * stepsPerDeg);
  long delta       = targetSteps - currentSteps;

  if (delta == 0) return;

  digitalWrite(dirPin, delta > 0 ? HIGH : LOW);

  long n = abs(delta);
  for (long i = 0; i < n; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  currentSteps = targetSteps;
}


// Apply received angles to all joints 
// Order: Base → Shoulder → Elbow → Gripper  (avoids collisions)
void applyAngles() {
  float baseDeg     = constrain(target_angles[0], -BASE_MAX_DEG, BASE_MAX_DEG);
  float shoulderDeg = constrain(target_angles[1], 0.0f, SHOULDER_MAX_DEG);

  moveStepper(BASE_STEP, BASE_DIR, base_steps_current, baseDeg, BASE_STEPS_PER_DEG);
  delay(3000);

  moveStepper(SHOULDER_STEP, SHOULDER_DIR, shoulder_steps_current, shoulderDeg, SHOULDER_STEPS_PER_DEG);
  delay(4000);

  servo_elbow.write(constrain((int)target_angles[2], 0, 180));
  delay(500);

  // Gripper: 0° = closed, 30° = open
  servo_gripper.write(constrain((int)target_angles[3], GRIPPER_CLOSED_DEG, GRIPPER_OPEN_DEG));
  delay(200);
}


void setup() {
  Serial.begin(BAUD);

  pinMode(BASE_STEP,     OUTPUT);
  pinMode(BASE_DIR,      OUTPUT);
  pinMode(SHOULDER_STEP, OUTPUT);
  pinMode(SHOULDER_DIR,  OUTPUT);

  // Roll-about-link1: attach servo and lock at 0° — servo PID actively holds the position
  servo_roll.attach(PIN_ROLL);
  servo_roll.write(0);              // locked at 0°; never written again

  servo_elbow.attach(PIN_ELBOW);
  servo_gripper.attach(PIN_GRIPPER);

  servo_elbow.write(ELBOW_NEUTRAL_DEG);   // neutral = 90°
  servo_gripper.write(GRIPPER_CLOSED_DEG);

  Serial.println("[HOMING] Starting...");

  // Home base axis (θ1) — confirm dirToStop on hardware
  /*
  homeAxis(BASE_STEP, BASE_DIR, base_steps_current,
           HIGH,           // direction toward hard stop — confirm with hardware
           0.0f,           // home = 0° (arm facing tray)
           5.0f,           // back off 5° from stop
           BASE_STEPS_PER_DEG);
  Serial.println("[HOMING] Base done.");
*/
  // Home shoulder axis (θ2)
  homeAxis(SHOULDER_STEP, SHOULDER_DIR, shoulder_steps_current,
           LOW,            // direction of the camera
           18.0f,           // IK angle at hard stop (measured)
           5.0f,           // back off 5° → settles at ~23°
           SHOULDER_STEPS_PER_DEG);
  Serial.println("[HOMING] Shoulder done.");

  servo_elbow.write(ELBOW_NEUTRAL_DEG);
  servo_gripper.write(GRIPPER_OPEN_DEG);

  Serial.print("base: ");    Serial.print(base_steps_current / BASE_STEPS_PER_DEG);    Serial.println(" deg");
  Serial.print("shoulder: "); Serial.print(shoulder_steps_current / SHOULDER_STEPS_PER_DEG); Serial.println(" deg");
  Serial.print("elbow: ");   Serial.print(ELBOW_NEUTRAL_DEG);  Serial.println(" deg");
  Serial.print("gripper: "); Serial.print(GRIPPER_OPEN_DEG);   Serial.println(" deg");

  Serial.println("READY");
}


void loop() {
  if (parseSerial()) {
    Serial.print("letter_sorted: "); Serial.println(letter_sorted);
    const char* labels[] = {"base", "shoulder", "elbow", "gripper"};
    for (int i = 0; i < NB_ANGLES; i++) {
      Serial.print(labels[i]);
      Serial.print(": ");
      Serial.print(target_angles[i]);
      Serial.println(" deg");
    }

    applyAngles();

    Serial.println("READY");
  }
}
