// Arduino
#include <Wire.h>
#include <Servo.h>

// Pin definitions — confirm with hardware
#define PIN_SIGNAL      7   // Pulse to Raspberry Pi



// Stepper drivers: Base (θ1) and Shoulder (θ2)
#define BASE_STEP       2
#define BASE_DIR        4
#define SHOULDER_STEP   3
#define SHOULDER_DIR    5

// Servo pins: Elbow (θ3), Wrist (θ4), Gripper
#define PIN_ELBOW       6
#define PIN_WRIST       9
#define PIN_GRIPPER     10

// Stepper parameters — confirm with hardware
#define STEPS_PER_REV   200     // Full steps per motor revolution
#define MICROSTEP       16      // Driver microstepping setting (e.g. DRV8825 at 1/16)
#define STEPS_PER_DEG   ((STEPS_PER_REV * MICROSTEP) / 360.0f)
#define STEP_DELAY_US   300     // Microseconds between pulses (controls speed)


// I2C
#define ADRESSE_I2C     8
#define NB_JOINTS       5
#define NB_BOOL         1
#define PACKET_SIZE     (NB_BOOL + NB_JOINTS * 4)

Servo servo_elbow;
Servo servo_wrist;
Servo servo_gripper;

volatile float  target_angles[NB_JOINTS];
volatile bool   letter_sorted    = false; // State: says if the letter has been sorted and if we can go to the next one
volatile bool   newData = false; // New I2C data flag

long base_steps_current     = 0;  // Current base position tracked in steps (needed for direction and delta)
long shoulder_steps_current = 0;  // Current shoulder position tracked in steps


// ─────────────────────────────────────────────────────────────────
// Stall-based homing for one stepper axis
//
// Strategy: drive slowly toward the hard-stop for up to maxSteps.
// The motor will stall against the stop. We don't detect the stall
// electrically — we just drive a safe over-travel distance and rely
// on the physical stop to absorb it (low speed = low force = safe).
// After homing, zero the step counter and back off by backoff_deg
// so the arm is not pressing against the stop during operation.
//
// IMPORTANT: dirToStop must be confirmed on hardware.
//   - HIGH or LOW depending on which direction reaches the stop.
// ─────────────────────────────────────────────────────────────────

// homeAngle_deg : IK angle (degrees) that the physical hardstop corresponds to.
//                 The operational zero is NOT the stop — it is wherever the IK defines 0.
//                 After homing, currentSteps reflects the true IK angle so moveStepper
//                 can use absolute IK targets directly.
void homeAxis(int stepPin, int dirPin, long &currentSteps,
              int dirToStop, float homeAngle_deg, float backoff_deg) {

  // Drive toward the stop at reduced speed (3× slower than normal)
  digitalWrite(dirPin, dirToStop);
  long maxSteps = (long)(270.0f * STEPS_PER_DEG); // 270° max travel — covers full range

  for (long i = 0; i < maxSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US * 3);  // Slow: less force on stop, less lost steps
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US * 3);
  }

  // We are now at the hard-stop (IK angle = homeAngle_deg).
  currentSteps = (long)(homeAngle_deg * STEPS_PER_DEG);

  // Back off so the arm isn't pressing against the stop during operation.
  int backoffDir = (dirToStop == HIGH) ? LOW : HIGH;
  long backoffSteps = (long)(backoff_deg * STEPS_PER_DEG);

  digitalWrite(dirPin, backoffDir);
  for (long i = 0; i < backoffSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US * 2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US * 2);
  }

  // Update currentSteps: backoffDir HIGH = positive direction, LOW = negative
  currentSteps += (backoffDir == HIGH) ? backoffSteps : -backoffSteps;
}


// I2C receive
void receiveData(int nbBytes) {
  if (nbBytes < PACKET_SIZE) return;

  byte buffer[PACKET_SIZE];
  for (int i = 0; i < PACKET_SIZE; i++) {
    buffer[i] = Wire.read();
  }

  // Extract process state
  memcpy((void*)&letter_sorted, buffer, 1);

  // Extract joint angles
  for (int i = 0; i < NB_JOINTS; i++) {
    memcpy((void*)&target_angles[i], buffer + NB_BOOL + i * 4, 4);
  }
  newData = true;
}

// Move one stepper to a target angle (blocking); check direction of the rotation with hardware
void moveStepper(int stepPin, int dirPin, long &currentSteps, float targetDeg) {
  long targetSteps = (long)(targetDeg * STEPS_PER_DEG);
  long delta = targetSteps - currentSteps;
  if (delta == 0) return;
  else if(delta > 0){
      digitalWrite(dirPin, HIGH);
  }
  else{
      digitalWrite(dirPin, LOW);
  }

  long n = abs(delta);
  for (long i = 0; i < n; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  currentSteps = targetSteps;
}


// Sequential movement to avoid collisions
// Order: [Base + Wrist] → [Shoulder] → [Elbow] → [Gripper]
void applyAngles() {
  // Step 1: Base (stepper) + Wrist (servo) simultaneously
  servo_wrist.write(constrain((int)target_angles[3], 0, 180));// Wrist servo is written first (non-blocking) 
  moveStepper(BASE_STEP, BASE_DIR, base_steps_current, target_angles[0]); //Then base stepper runs
  delay(700);  // Wait for wrist to reach position; adjust to worst-case travel angle in hardware

  // Step 2: Shoulder (stepper)
  moveStepper(SHOULDER_STEP, SHOULDER_DIR, shoulder_steps_current, target_angles[1]);
  delay(300);// Wait for shoulder to reach position; adjust to worst-case travel angle in hardware

  // Step 3: Elbow (servo)
  servo_elbow.write(constrain((int)target_angles[2], 0, 180));
  delay(1000);  // Wait for elbow to reach position; adjust to worst-case travel angle in hardware

  // Step 4: Gripper (servo)
  servo_gripper.write(constrain((int)target_angles[4], 0, 180));
  delay(500);// Wait for gripper to reach position; adjust to hardware
}

void Readysignal() { // Pulse to inform of the state change
  digitalWrite(PIN_SIGNAL, HIGH);
  delay(100);
  digitalWrite(PIN_SIGNAL, LOW);
}


void setup() {
  Serial.begin(9600);  // Must be first so homing messages are visible

  pinMode(PIN_SIGNAL, OUTPUT);
  digitalWrite(PIN_SIGNAL, LOW);

  // Stepper pins
  pinMode(BASE_STEP,OUTPUT);
  pinMode(BASE_DIR,OUTPUT);
  pinMode(SHOULDER_STEP, OUTPUT);
  pinMode(SHOULDER_DIR,OUTPUT);

  // Servos
  servo_elbow.attach(PIN_ELBOW);
  servo_wrist.attach(PIN_WRIST);
  servo_gripper.attach(PIN_GRIPPER);

  // Neutral positions — confirm with hardware
  servo_elbow.write(90);
  servo_wrist.write(90);
  servo_gripper.write(90);

  Serial.println("[HOMING] Starting...");

  // HOME BASE (θ1)
  homeAxis(BASE_STEP, BASE_DIR, base_steps_current,
           HIGH,   // ← confirm: which direction reaches the physical stop?
           0.0f,   // ← confirm: IK angle (deg) of the base hardstop
           5.0f);  // back off 5° from the stop

  Serial.println("[HOMING] Base done.");

  // HOME SHOULDER (θ2): hardstop at 85° IK, approached from the negative direction
  homeAxis(SHOULDER_STEP, SHOULDER_DIR, shoulder_steps_current,
           LOW,    // negative direction reaches the stop
           85.0f,  // IK angle of the shoulder hardstop
           5.0f);  // back off 5° → arm will settle at 90° after homing

  Serial.println("[HOMING] Shoulder done. System ready.");

  // Servos to neutral
  servo_elbow.write(90);
  servo_wrist.write(90);
  servo_gripper.write(90);

  Wire.begin(ADRESSE_I2C);
  Wire.onReceive(receiveData);
}

void loop() {
  if (newData) {
    newData = false;

    Serial.print("Ready for the next letter? "); Serial.println(letter_sorted);
    Serial.print("Angles → ");
    const char* labels[] = {"base", "shoulder", "elbow", "wrist", "gripper"};
    for (int i = 0; i < NB_JOINTS; i++) {
      Serial.print(labels[i]); Serial.print(": ");
      Serial.print(target_angles[i]); Serial.print("°  ");
    }
    Serial.println();

    applyAngles();
    Readysignal();
  }
}
