#include <Wire.h>
#include <Servo.h>

#define ADRESSE_I2C   8
#define NB_SERVOS     4
#define NB_FLOATS     4  //  4 angles (4 octets)
#define NB_BOOL       1  //  1 state (1 octets)

// Servos
Servo servos[NB_SERVOS];
const int PINS_SERVOS[] = {3, 5, 6, 9}; //check with hardware

volatile bool letter_sorted = true ;// # state says if the letter has been letter_sorted and if we can go to the next one 
volatile float angles[NB_SERVOS];
volatile bool nouvellesDonnees = false;

void setup() {
  Wire.begin(ADRESSE_I2C);
  Wire.onReceive(receiveData);
  Serial.begin(9600);

  // Initialise servos
  for (int i = 0; i < NB_SERVOS; i++) {
    servos[i].attach(PINS_SERVOS[i]);
    servos[i].write(90);  // neutral position check with hardware
  }
}

void receiveData(int nbBytes) {
  if (nbBytes < NB_FLOATS * 4 + NB_BOOL ) return;  // incomplete package

  byte buffer[NB_FLOATS * 4 + NB_BOOL];
  for (int i = 0; i < NB_FLOATS * 4 + NB_BOOL; i++) {
    buffer[i] = Wire.read();
  }

  // Extract state of the process
  memcpy((void*)&letter_sorted, buffer,     1);

  // Extract angles
  for (int i = 0; i < NB_SERVOS; i++) {
    memcpy((void*)&angles[i], buffer + NB_BOOL + (i * 4), 4);
  }

  nouvellesDonnees = true;
}

void appliquerAngles() {
  for (int i = 0; i < NB_SERVOS; i++) {
    int angle = constrain((int)angles[i], 0, 180);  // Limit of rotation of the motors check with hardware
    servos[i].write(angle);
  }
}

void loop() {
  if (nouvellesDonnees) {
    nouvellesDonnees = false;

    Serial.print("Ready for the next letter ? :");
    Serial.println(letter_sorted);

    Serial.print("Angles → ");
    for (int i = 0; i < NB_SERVOS; i++) {
      Serial.print(angles[i]); Serial.print("° ");
    }
    Serial.println();

    appliquerAngles();
  }
}