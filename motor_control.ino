/* =========================================================
   Tempest AMR — Motor Control Module
   Matches wiring: 2x L298N, 4x DC motor, GND-tied 5V logic
   =========================================================

   IMPORTANT: On each L298N, IN1+IN3 are tied together and
   IN2+IN4 are tied together, and ENA+ENB are tied together.
   That means each side (left / right) only has ONE direction
   signal and ONE speed signal shared by both motors on that
   side. This is a true 2-motor-equivalent differential drive
   (left group + right group), not 4 independently steerable
   wheels. Do not try to give FL/RL or FR/RR separate speeds —
   the hardware physically won't allow it as wired.

   Pin map (from your wiring doc):
     LEFT  side: IN1/IN3 -> GPIO25   IN2/IN4 -> GPIO26   ENA/ENB -> GPIO27
     RIGHT side: IN1/IN3 -> GPIO21   IN2/IN4 -> GPIO22   ENA/ENB -> GPIO13

   Left  L298N -> OUT1/OUT2 -> Motor-FL, OUT3/OUT4 -> Motor-RL
   Right L298N -> OUT1/OUT2 -> Motor-FR, OUT3/OUT4 -> Motor-RR
   ========================================================= */

// ---- Pin definitions ----
#define LEFT_IN1   25   // drives IN1 + IN3 on L298N-Left
#define LEFT_IN2   26   // drives IN2 + IN4 on L298N-Left
#define LEFT_EN    27   // drives ENA + ENB on L298N-Left (PWM)

#define RIGHT_IN1  21   // drives IN1 + IN3 on L298N-Right
#define RIGHT_IN2  22   // drives IN2 + IN4 on L298N-Right
#define RIGHT_EN   13   // drives ENA + ENB on L298N-Right (PWM)

// ---- PWM config (ESP32 Arduino core 3.x style, pin-based API) ----
const int PWM_FREQ_HZ   = 20000;  // 20kHz, above audible range
const int PWM_RES_BITS  = 8;      // 0-255 duty
const int PWM_MAX_DUTY  = 255;

void setupMotors() {
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Attach PWM directly to the EN pins (core 3.x: no channel numbers needed)
  ledcAttach(LEFT_EN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(RIGHT_EN, PWM_FREQ_HZ, PWM_RES_BITS);

  // Start stopped
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  ledcWrite(LEFT_EN, 0);
  ledcWrite(RIGHT_EN, 0);
}

/* If your installed esp32 board package is older (core 2.x),
   ledcAttach()/ledcWrite(pin,...) won't exist. Use this instead
   in setupMotors(), and swap ledcWrite(pin,...) calls below for
   ledcWrite(channel,...):

     ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);
     ledcAttachPin(LEFT_EN, 0);
     ledcSetup(1, PWM_FREQ_HZ, PWM_RES_BITS);
     ledcAttachPin(RIGHT_EN, 1);
*/

// ---- Low-level: set one side's direction + speed ----
// dir:  1 = forward, -1 = backward, 0 = stop (coast)
// speed: 0-255
void setLeftSide(int dir, int speed) {
  speed = constrain(speed, 0, PWM_MAX_DUTY);
  if (dir > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    speed = 0;
  }
  ledcWrite(LEFT_EN, speed);
}

void setRightSide(int dir, int speed) {
  speed = constrain(speed, 0, PWM_MAX_DUTY);
  if (dir > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else if (dir < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    speed = 0;
  }
  ledcWrite(RIGHT_EN, speed);
}

// ---- High-level movement helpers ----
// These are the ones you'd call from your route handlers /
// state machine (e.g. from esp32_web_remote.ino or the serial version).

void stopMotors() {
  setLeftSide(0, 0);
  setRightSide(0, 0);
}

void driveForward(int speed = 150) {
  setLeftSide(1, speed);
  setRightSide(1, speed);
}

void driveBackward(int speed = 150) {
  setLeftSide(-1, speed);
  setRightSide(-1, speed);
}

// In-place turns (one side forward, other backward)
void turnLeftInPlace(int speed = 130) {
  setLeftSide(-1, speed);
  setRightSide(1, speed);
}

void turnRightInPlace(int speed = 130) {
  setLeftSide(1, speed);
  setRightSide(-1, speed);
}

// Pivot turns (one side stopped, other drives) — gentler, good for tight aisles
void pivotLeft(int speed = 150) {
  setLeftSide(0, 0);
  setRightSide(1, speed);
}

void pivotRight(int speed = 150) {
  setLeftSide(1, speed);
  setRightSide(0, 0);
}
