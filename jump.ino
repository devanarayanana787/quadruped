#include <Servo.h>

// -------- Pin mapping --------
Servo kneeFR, hipFR;  // D3, D2
Servo kneeFL, hipFL;  // D5, D4
Servo kneeBL, hipBL;  // D7, D6
Servo kneeBR, hipBR;  // D9, D8

// -------- Neutral Angles --------
const int KNEE_FL_NEUTRAL = 160;
const int KNEE_FR_NEUTRAL = 20;
const int KNEE_BL_NEUTRAL = 20;
const int KNEE_BR_NEUTRAL = 160;

const int HIP_NEUTRAL = 90;

// Jump bend amount
const int KNEE_BEND = 40;   // how much to bend for crouch

void setup() {
  // Attach pins
  kneeFR.attach(3); hipFR.attach(2);
  kneeFL.attach(5); hipFL.attach(4);
  kneeBL.attach(7); hipBL.attach(6);
  kneeBR.attach(9); hipBR.attach(8);

  // Neutral hip position
  hipFL.write(HIP_NEUTRAL);
  hipFR.write(HIP_NEUTRAL);
  hipBL.write(HIP_NEUTRAL);
  hipBR.write(HIP_NEUTRAL);

  // Start in neutral standing
  kneeFL.write(KNEE_FL_NEUTRAL);
  kneeFR.write(KNEE_FR_NEUTRAL);
  kneeBL.write(KNEE_BL_NEUTRAL);
  kneeBR.write(KNEE_BR_NEUTRAL);

  delay(2000); // wait before continuous jumping
}

void loop() {
  // ----- Step 1: Crouch -----
  kneeFL.write(KNEE_FL_NEUTRAL - KNEE_BEND);
  kneeFR.write(KNEE_FR_NEUTRAL + KNEE_BEND);
  kneeBL.write(KNEE_BL_NEUTRAL + KNEE_BEND);
  kneeBR.write(KNEE_BR_NEUTRAL - KNEE_BEND);
  delay(150); // short crouch

  // ----- Step 2: Jump (extend fast) -----
  kneeFL.write(KNEE_FL_NEUTRAL);
  kneeFR.write(KNEE_FR_NEUTRAL);
  kneeBL.write(KNEE_BL_NEUTRAL);
  kneeBR.write(KNEE_BR_NEUTRAL);
  delay(150); // quick push

  // ----- Step 3: Small pause before next jump -----
  delay(100);
}
