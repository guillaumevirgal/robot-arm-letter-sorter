#include <Servo.h>

Servo gripper;
Servo elbow;

const int SERVO_GRIP = 9;
const int SERVO_elbow = 10;


const int GRIP_OPEN   = 10;   // degrees ; gripper open
const int GRIP_CLOSED = 0;  // degrees ; gripper close

const int SWEEP_DELAY_MS = 100;  // time to hold each position

void setup() {
  Serial.begin(9600);
  gripper.attach(SERVO_GRIP);
  elbow.attach(SERVO_elbow);

  Serial.println("Gripper test starting...");
  Serial.println("Sweeping between open and closed positions.");

  // Start in open position
  gripper.write(GRIP_OPEN);
  elbow.write(-90);
  delay(1000);
}

void loop() {
  // Close gripper
  gripper.write(0);
  
  for(int i = -0; i< 90; i++){
    Serial.print("Opening -> ");
    Serial.print(i);
    Serial.println(" deg");
    //gripper.write(i);
    elbow.write(i);
    delay(SWEEP_DELAY_MS);
  }

  for(int i = 90; i >0; i--){
    Serial.print("Closing -> ");
    Serial.print(i);
    Serial.println(" deg");
    //gripper.write(i);
    elbow.write(i);
    delay(SWEEP_DELAY_MS);
  }
  

}
