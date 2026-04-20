#include <Servo.h>
#include <Wire.h>

#define ADRESSE_I2C  8
#define PACKET_SIZE  4

Servo gripper;

const int SERVO_PIN = 9;

const int GRIP_OPEN   = 55;   // degrees ; gripper open
const int GRIP_CLOSED = 110;  // degrees ; gripper close

const int SWEEP_DELAY_MS = 1000;  // time to hold each position

volatile float target_angles[1];
volatile bool  newData = false;

void receiveData(int nbBytes) {
    if (nbBytes < PACKET_SIZE) {
      Serial.print("[WARN] Short packet: "); Serial.println(nbBytes);
      return;
    }
    byte buffer[PACKET_SIZE];
    for (int i = 0; i < PACKET_SIZE; i++) buffer[i] = Wire.read();

    for (int i = 0; i < 1; i++){
      memcpy((void*)&target_angles[i], buffer + 0, 4);  // Extract joint angles
    }
    newData = true;
}
void applyAngles() {
  // Gripper (servo)
  gripper.write(constrain((int)target_angles[0], 0, 180));
  delay(500);// Wait for gripper to reach position; adjust to hardware
}

void setup() {
  Serial.begin(9600);
  gripper.attach(SERVO_PIN);
  
  // Start in open position
  gripper.write(GRIP_OPEN);

  Wire.begin(ADRESSE_I2C);
  Wire.onReceive(receiveData);
  Serial.println("Gripper test starting...");
  Serial.println("[READY] Waiting for I2C...");

  
}

void loop() {
  if (newData) {
    newData = false;
    const char* labels[] = {"gripper"}; //Get data
    for (int i = 0; i < 1; i++) {
        Serial.print(labels[i]); 
        Serial.print(": ");
        Serial.println(target_angles[i]);
        Serial.print("°  ");
    }
  //applyAngles();  
  }
}


