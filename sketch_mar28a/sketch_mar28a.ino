// stepper_test.ino — tests every motor one by one, 30° each
// Order: Base → Shoulder → Elbow → Gripper
// 5 second delay at start, 3 seconds between each joint

#include <Servo.h>                  // servo library for elbow and gripper

// ── Pin definitions ────────────────────────────────────────────────────────────
#define BASE_STEP       3           // step pulse pin for base stepper
#define BASE_DIR        2           // direction pin for base stepper
#define SHOULDER_STEP   5           // step pulse pin for shoulder stepper
#define SHOULDER_DIR    4           // direction pin for shoulder stepper
#define PIN_ELBOW       10           // PWM pin for elbow servo
#define PIN_GRIPPER     9          // PWM pin for gripper servo
#define PIN_ROLL     11

// ── Stepper parameters ─────────────────────────────────────────────────────────
#define STEPS_PER_REV   200         // full steps per motor revolution — confirm with datasheet
#define MICROSTEP       16          // microstepping setting on the driver — confirm with hardware
#define STEPS_PER_DEG   ((STEPS_PER_REV * MICROSTEP) / 360.0f)  // steps per degree
#define STEP_DELAY_US   15000         // microseconds between step pulses — controls speed

// ── Test parameters ────────────────────────────────────────────────────────────
#define TEST_ANGLE      30          // degrees each motor will rotate
#define DELAY_START_MS  5000        // wait 5 seconds before starting — time to stand clear
#define DELAY_STEP_MS   3000        // wait 3 seconds between each joint

Servo servo_elbow;                  // servo object for elbow
Servo servo_gripper;                // servo object for gripper
Servo servo_roll;

// ── Helper: rotate one stepper by a given number of degrees ───────────────────
// Positive degrees = HIGH direction, negative = LOW direction
void rotateStepper(int stepPin, int dirPin, float degrees) {
  long steps = (long)(abs(degrees) * STEPS_PER_DEG);   // convert degrees to step count
  digitalWrite(dirPin, degrees > 0 ? HIGH : LOW);      // set direction based on sign

  for (long i = 0; i < steps; i++) {   // pulse loop
    digitalWrite(stepPin, HIGH);        // rising edge — one microstep
    delayMicroseconds(STEP_DELAY_US);  // half-period
    digitalWrite(stepPin, LOW);         // falling edge
    delayMicroseconds(STEP_DELAY_US);  // half-period
  }
}


void setup() {
  Serial.begin(9600);                 // open serial for status messages

  pinMode(BASE_STEP,     OUTPUT);     // configure stepper pins as outputs
  pinMode(BASE_DIR,      OUTPUT);
  delay(100);
  pinMode(SHOULDER_STEP, OUTPUT);
  pinMode(SHOULDER_DIR,  OUTPUT);
  digitalWrite(BASE_DIR, HIGH);
  digitalWrite(SHOULDER_DIR, LOW);
  digitalWrite(BASE_STEP, LOW);
  digitalWrite(SHOULDER_STEP, LOW);


  servo_gripper.attach(PIN_GRIPPER);  // attach gripper servo
  servo_gripper.write(0);            // start gripper at neutral position
  

  Serial.println("Motor test starting in 5 seconds...");
  delay(DELAY_START_MS);              // 5 second safety delay before any movement


  // ── 1. Base stepper ─────────────────────────────────────────────────────────
  Serial.println("Base: rotating 180 deg");
  rotateStepper(BASE_STEP, BASE_DIR, 12);   //12 correspond to a 180° rotation of the base; rotate base 90° (=6)
  delay(DELAY_STEP_MS);                             // 3 second pause


  // ── 2. Shoulder stepper ─────────────────────────────────────────────────────
  Serial.println("Shoulder: rotating max rotation");
  rotateStepper(SHOULDER_STEP, SHOULDER_DIR, 40);   // rotate shoulder 30° (25 = ~90°, max rotation = 40 (~144°))
  delay(DELAY_STEP_MS);                                     // 3 second pause


  // ── 3. Elbow servo ──────────────────────────────────────────────────────────
  Serial.println("Elbow: rotating 30 deg");
  servo_elbow.attach(PIN_ROLL);      
  servo_elbow.write(0); 
  servo_elbow.attach(PIN_ELBOW);      // attach elbow servo
  servo_elbow.write(90 + TEST_ANGLE);   // move from neutral (90°) by +30°
  delay(DELAY_STEP_MS);                 // 3 second pause


  // ── 4. Gripper servo ────────────────────────────────────────────────────────
  Serial.println("Gripper: rotating 30 deg");
  servo_gripper.write(TEST_ANGLE); // move from neutral  by +30°
  delay(DELAY_STEP_MS);                 // 3 second pause


  Serial.println("Test complete.");
}


void loop() {
  // nothing — test runs once in setup() only
}
