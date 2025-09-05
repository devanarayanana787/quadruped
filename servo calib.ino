#include <Servo.h>

// Servo pin definitions for each leg (Coxa, Femur, Tibia)
#define FR_COXA_PIN 4
#define FR_FEMUR_PIN 2
#define FR_TIBIA_PIN 3

#define FL_COXA_PIN 10
#define FL_FEMUR_PIN 8
#define FL_TIBIA_PIN 9

#define BR_COXA_PIN 7
#define BR_FEMUR_PIN 5
#define BR_TIBIA_PIN 6

#define BL_COXA_PIN 13
#define BL_FEMUR_PIN 11
#define BL_TIBIA_PIN 12

// Create Servo objects for each joint
Servo frCoxa, frFemur, frTibia;
Servo flCoxa, flFemur, flTibia;
Servo brCoxa, brFemur, brTibia;
Servo blCoxa, blFemur, blTibia;

// Function to attach all servos to their respective pins
void attachServos() {
  frCoxa.attach(FR_COXA_PIN);
  frFemur.attach(FR_FEMUR_PIN);
  frTibia.attach(FR_TIBIA_PIN);

  flCoxa.attach(FL_COXA_PIN);
  flFemur.attach(FL_FEMUR_PIN);
  flTibia.attach(FL_TIBIA_PIN);

  brCoxa.attach(BR_COXA_PIN);
  brFemur.attach(BR_FEMUR_PIN);
  brTibia.attach(BR_TIBIA_PIN);

  blCoxa.attach(BL_COXA_PIN);
  blFemur.attach(BL_FEMUR_PIN);
  blTibia.attach(BL_TIBIA_PIN);
}

// Function to detach all servos
void detachServos() {
  frCoxa.detach();
  frFemur.detach();
  frTibia.detach();

  flCoxa.detach();
  flFemur.detach();
  flTibia.detach();

  brCoxa.detach();
  brFemur.detach();
  brTibia.detach();

  blCoxa.detach();
  blFemur.detach();
  blTibia.detach();
}

// ======================= SERVO CALIBRATION FUNCTIONS =======================

// Function to calculate and write servo angles for a single leg
void setFrLegAngles(float gamma, float alpha, float beta) {
  // Front Right (FR) Leg
  int frCoxaAngle = gamma + 90;
  int frFemurAngle = 90 - alpha;
  int frTibiaAngle = 180 - beta;

  // Write the calculated angles to the servos
  frCoxa.write(frCoxaAngle);
  frFemur.write(frFemurAngle);
  frTibia.write(frTibiaAngle);
}

void setFlLegAngles(float gamma, float alpha, float beta) {
  // Front Left (FL) Leg
  int flCoxaAngle = 90 - gamma;
  int flFemurAngle;
  // Conditional logic for the femur angle based on the sign of alpha
  if (alpha >= 0) {
    flFemurAngle = 90 + alpha;
  } else {
    flFemurAngle = 90 - alpha;
  }
  int flTibiaAngle = beta;

  flCoxa.write(flCoxaAngle);
  flFemur.write(flFemurAngle);
  flTibia.write(flTibiaAngle);
}

void setBlLegAngles(float gamma, float alpha, float beta) {
  // Back Left (BL) Leg
  int blCoxaAngle = gamma;
  int blFemurAngle = 90 - alpha;
  int blTibiaAngle = 180 - beta;

  blCoxa.write(blCoxaAngle);
  blFemur.write(blFemurAngle);
  blTibia.write(blTibiaAngle);
}

void setBrLegAngles(float gamma, float alpha, float beta) {
  // Back Right (BR) Leg
  int brCoxaAngle = 180 - gamma;
  int brFemurAngle = 90 + alpha;
  int brTibiaAngle = beta;

  brCoxa.write(brCoxaAngle);
  brFemur.write(brFemurAngle);
  brTibia.write(brTibiaAngle);
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Servo Calibration Code Ready.");

  // Attach all servos to their pins
  attachServos();

  // You can set initial angles here if needed, for example, to a "stand" position
  // Example:
  // setAllLegs(90, 90, 90); // a placeholder function for setting all to a neutral angle
  // delay(1000);
}

void loop() {
  // Your main control logic would go here.
  // This loop can be used to read sensor data, receive commands,
  // and then call the servo angle functions with the appropriate values.

  // Example of setting angles for all legs:
  // float targetGamma = 45;
  // float targetAlpha = 15;
  // float targetBeta = 60;

  // setFrLegAngles(targetGamma, targetAlpha, targetBeta);
  // setFlLegAngles(targetGamma, targetAlpha, targetBeta);
  // setBlLegAngles(targetGamma, targetAlpha, targetBeta);
  // setBrLegAngles(targetGamma, targetAlpha, targetBeta);

  // delay(50); // Small delay to avoid jerky movement

  // You can comment out the example code and integrate your main code here.
}
