// ---------------- Robot/encoder constants ----------------
const float WHEEL_DIAMETER_CM = 6.5;
const int ENCODER_TICKS_PER_REV = 20;
const float CM_PER_TICK = (PI * WHEEL_DIAMETER_CM) / ENCODER_TICKS_PER_REV;

// Preset bin positions: {forward_cm, turn_deg} — simple point-to-point
// moves. In a full deployment these come from the Python-side path planner
// (grid/A*); the ESP32 just executes primitive forward/turn commands.
struct BinPose { float forward_cm; float turn_deg; };
const int NUM_BINS = 4;
BinPose binTable[NUM_BINS] = {
  {0,   0},    // bin 0 unused / home
  {60,  0},    // bin 1: straight ahead
  {60,  90},   // bin 2: right
  {60, -90},   // bin 3: left
};

// ---------------- State ----------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;
HX711 scale;
Servo actuator;

void IRAM_ATTR onLeftEncoder()  { leftTicks  += digitalRead(L_ENC_B) ? 1 : -1; }
void IRAM_ATTR onRightEncoder() { rightTicks += digitalRead(R_ENC_B) ? 1 : -1; }

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(R_PWM, OUTPUT);
  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), onRightEncoder, RISING);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(DROP_CONFIRM_PIN, INPUT_PULLUP);
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(420.0);   // calibration factor — tune per load cell
  scale.tare();
  actuator.attach(SERVO_PIN);
  actuator.write(SERVO_DROP_ANGLE);
  stopMotors();
  Serial.println("READY");
}
// ---------------- Main loop ----------------
String inputLine;
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}

// ---------------- Command dispatch ----------------
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  Serial.println("ACK");

  if (cmd.startsWith("MOVE_BIN_")) {
    int bin = cmd.substring(9).toInt();
    moveToBin(bin);
  } else if (cmd == "PICK") {
    actuator.write(SERVO_PICK_ANGLE);
    delay(400);
    Serial.println("DONE");
  } else if (cmd == "DROP") {
    doDrop();
  } else if (cmd == "STOP") {
    stopMotors();
    Serial.println("DONE");
  } else if (cmd == "GET_STATUS") {
    sendStatus();
  } else {
    Serial.println("ERROR,UNKNOWN_CMD");
  }
}

// ---------------- Motion primitives ----------------
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(L_IN1, leftSpeed >= 0);
  digitalWrite(L_IN2, leftSpeed < 0);
  analogWrite(L_PWM, abs(leftSpeed));

  digitalWrite(R_IN1, rightSpeed >= 0);
  digitalWrite(R_IN2, rightSpeed < 0);
  analogWrite(R_PWM, abs(rightSpeed));
}

void stopMotors() { setMotors(0, 0); }

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20ms timeout
  if (duration == 0) return 999.0;
  return duration * 0.0343 / 2.0;
}

// Drive forward a set distance, with obstacle safety abort.
bool driveForwardCm(float distanceCm, int speed = 150) {
  long targetTicks = distanceCm / CM_PER_TICK;
  leftTicks = 0; rightTicks = 0;
  setMotors(speed, speed);

  while (abs((leftTicks + rightTicks) / 2) < targetTicks) {
    if (readDistanceCm() < OBSTACLE_STOP_CM) {
      stopMotors();
      Serial.println("ERROR,OBSTACLE");
      return false;
    }
    delay(10);
  }
  stopMotors();
  return true;
}

// Turn in place by a target angle (deg); simplistic tick-based estimate —
// tune TICKS_PER_DEGREE for your wheelbase during calibration.
bool turnDeg(float angleDeg, int speed = 130) {
  const float TICKS_PER_DEGREE = 1.1; // calibrate on real hardware
  long targetTicks = abs(angleDeg) * TICKS_PER_DEGREE;
  leftTicks = 0; rightTicks = 0;

  int dir = (angleDeg >= 0) ? 1 : -1;
  setMotors(dir * speed, -dir * speed);

  while (abs((abs(leftTicks) + abs(rightTicks)) / 2) < targetTicks) {
    delay(10);
  }
  stopMotors();
  return true;
}

void moveToBin(int bin) {
  if (bin < 0 || bin >= NUM_BINS) {
    Serial.println("ERROR,INVALID_BIN");
    return;
  }
  BinPose pose = binTable[bin];
  if (pose.turn_deg != 0 && !turnDeg(pose.turn_deg)) return;
  if (pose.forward_cm != 0 && !driveForwardCm(pose.forward_cm)) return;
  Serial.println("DONE");
}

void doDrop() {
  actuator.write(SERVO_DROP_ANGLE);
  unsigned long start = millis();
  bool confirmed = false;
  while (millis() - start < 1500) {           // 1.5s confirmation window
    if (digitalRead(DROP_CONFIRM_PIN) == LOW) { // break-beam triggered
      confirmed = true;
      break;
    }
    delay(20);
  }
  Serial.println(confirmed ? "DONE" : "ERROR,DROP_NOT_CONFIRMED");
}

// ---------------- Sensing ----------------
int readAttachmentId() {
  int raw = analogRead(ATTACH_ID_PIN); // 0-4095 on ESP32 ADC
  // Map voltage bands to attachment IDs — calibrate per attachment resistor.
  if (raw < 800)  return 0;  // no attachment
  if (raw < 1600) return 1;  // e.g. gripper module
  if (raw < 2400) return 2;  // e.g. scanner module
  if (raw < 3200) return 3;  // e.g. lift module
  return 4;                  // reserved
}

void sendStatus() {
  float dist = readDistanceCm();
  float load = scale.is_ready() ? scale.get_units(3) : -1.0;
  int attach = readAttachmentId();
  Serial.print("STATUS,DIST:");
  Serial.print(dist, 1);
  Serial.print(",LOAD:");
  Serial.print(load, 1);
  Serial.print(",ATTACH:");
  Serial.println(attach);
}
