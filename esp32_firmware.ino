#include <WiFi.h>
#include <WebServer.h>
#include <HX711.h>
#include <ESP32Servo.h>

// ---------------- WiFi Access Point settings ----------------
const char* AP_SSID = "Tempest_AMR";
const char* AP_PASSWORD = "tempest123";  // must be 8+ characters

WebServer server(80);

// ---------------- Pin map (same as main firmware) ----------------
#define L_IN1 25
#define L_IN2 26
#define L_PWM 27
#define R_IN1 21   // moved off strapping pin 12
#define R_IN2 22
#define R_PWM 13
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 32
#define R_ENC_B 33
#define TRIG_PIN 5
#define ECHO_PIN 18
#define OBSTACLE_STOP_CM 15.0
#define HX711_DT 19
#define HX711_SCK 23
#define ATTACH_ID_PIN 36
#define SERVO_PIN 2   // moved off strapping pin 15
#define SERVO_PICK_ANGLE 120
#define SERVO_DROP_ANGLE 20
#define DROP_CONFIRM_PIN 4

const int DRIVE_SPEED = 160;
const int TURN_SPEED = 140;
const int CURVE_INNER_SPEED = 70;   // slower wheel on a diagonal arc

volatile long leftTicks = 0;
volatile long rightTicks = 0;
HX711 scale;
Servo actuator;

void IRAM_ATTR onLeftEncoder()  { leftTicks  += digitalRead(L_ENC_B) ? 1 : -1; }
void IRAM_ATTR onRightEncoder() { rightTicks += digitalRead(R_ENC_B) ? 1 : -1; }
bool driveForwardCm(float distanceCm, int speed = 150);
bool turnDeg(float angleDeg, int speed = 130);

// ---------------- The web page (HTML + CSS + JS, sent as one string) ----------------
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Tempest AMR Remote</title>
  <style>
    body { font-family: sans-serif; text-align: center; background:#111; color:#eee; }
    h2 { margin-top: 20px; }
    .pad { display: grid; grid-template-columns: 80px 80px 80px; gap: 10px; justify-content: center; margin: 20px auto; }
    button { font-size: 20px; padding: 20px; border-radius: 10px; border: none; background:#2b7a2b; color:white; }
    button:active { background:#1e561e; }
    .stop { background:#a01d1d; }
    .actions { display:flex; gap:10px; justify-content:center; margin-top:20px; flex-wrap:wrap; }
    .actions button { background:#2b5a7a; }
    #status { margin-top:20px; font-size:14px; color:#aaa; white-space:pre; }
  </style>
</head>
<body>
  <h2>Tempest AMR Remote</h2>

  <div class="pad">
  <button onclick="send('/forward_left')">&#8598;</button>
  <button onclick="send('/forward')">&uarr;</button>
  <button onclick="send('/forward_right')">&#8599;</button>

  <button onclick="send('/left')">&larr;</button>
  <button class="stop" onclick="send('/stop')">STOP</button>
  <button onclick="send('/right')">&rarr;</button>

  <button onclick="send('/backward_left')">&#8601;</button>
  <button onclick="send('/backward')">&darr;</button>
  <button onclick="send('/backward_right')">&#8600;</button>
</div>

  <div class="actions">
    <button onclick="send('/pick')">PICK</button>
    <button onclick="send('/drop')">DROP</button>
    <button onclick="send('/move?bin=1')">BIN 1</button>
    <button onclick="send('/move?bin=2')">BIN 2</button>
    <button onclick="send('/move?bin=3')">BIN 3</button>
  </div>

  <div id="status">Loading status...</div>

  <script>
    function send(path) {
      fetch(path).then(r => r.text()).then(t => console.log(t));
    }
    function refreshStatus() {
      fetch('/status').then(r => r.text()).then(t => {
        document.getElementById('status').innerText = t;
      });
    }
    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";

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
  scale.set_scale(420.0);
  if (scale.is_ready()){scale.tare();}
  else{ Serial.println("HX711 not detected, skipping tare");}
  actuator.attach(SERVO_PIN);
  actuator.write(SERVO_DROP_ANGLE);

  stopMotors();

  // Start WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println("Access Point started");
  Serial.print("Connect to WiFi '");
  Serial.print(AP_SSID);
  Serial.println("' then open this IP in a browser:");
  Serial.println(WiFi.softAPIP());

  // Route setup
  server.on("/", handleRoot);
  server.on("/forward", [](){ driveWhilePressed(DRIVE_SPEED, DRIVE_SPEED); });
  server.on("/backward", [](){ driveWhilePressed(-DRIVE_SPEED, -DRIVE_SPEED); });
  server.on("/left", [](){ driveWhilePressed(-TURN_SPEED, TURN_SPEED); });
  server.on("/right", [](){ driveWhilePressed(TURN_SPEED, -TURN_SPEED); });
server.on("/forward_left",  [](){ driveWhilePressed(CURVE_INNER_SPEED, DRIVE_SPEED); });
server.on("/forward_right", [](){ driveWhilePressed(DRIVE_SPEED, CURVE_INNER_SPEED); });
server.on("/backward_left", [](){ driveWhilePressed(-CURVE_INNER_SPEED, -DRIVE_SPEED); });
server.on("/backward_right",[](){ driveWhilePressed(-DRIVE_SPEED, -CURVE_INNER_SPEED); });
  server.on("/stop", [](){ stopMotors(); server.send(200, "text/plain", "stopped"); });
  server.on("/pick", handlePick);
  server.on("/drop", handleDrop);
  server.on("/move", handleMoveToBin);
  server.on("/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();
}

// ---------------- Web handlers ----------------
void handleRoot() {
  server.send_P(200, "text/html", PAGE_HTML);
}

// Simple "press and it moves briefly" behavior — safer for a browser button
// than a true press-and-hold, and avoids needing WebSockets for a first version.
void driveWhilePressed(int leftSpeed, int rightSpeed) {
  float dist = readDistanceCm();
  if (dist < OBSTACLE_STOP_CM && (leftSpeed > 0 || rightSpeed > 0)) {
    stopMotors();
    server.send(200, "text/plain", "blocked: obstacle ahead");
    return;
  }
  setMotors(leftSpeed, rightSpeed);
  delay(400);   // move for 0.4s per tap — tap repeatedly to keep going
  stopMotors();
  server.send(200, "text/plain", "ok");
}

void handlePick() {
  actuator.write(SERVO_PICK_ANGLE);
  delay(400);
  server.send(200, "text/plain", "picked");
}

void handleDrop() {
  actuator.write(SERVO_DROP_ANGLE);
  unsigned long start = millis();
  bool confirmed = false;
  while (millis() - start < 1500) {
    if (digitalRead(DROP_CONFIRM_PIN) == LOW) { confirmed = true; break; }
    delay(20);}
  server.send(200, "text/plain", confirmed ? "dropped" : "drop not confirmed");
}

void handleMoveToBin() {
  if (!server.hasArg("bin")) {
    server.send(400, "text/plain", "missing bin arg");
    return;
  }
  int bin = server.arg("bin").toInt();
  // Same simple preset positions as the serial version — tune per bin layout.
  float forward_cm = 60;
  float turn_deg = (bin == 2) ? 90 : (bin == 3) ? -90 : 0;

  if (turn_deg != 0) turnDeg(turn_deg);
  bool ok = driveForwardCm(forward_cm);
  server.send(200, "text/plain", ok ? ("arrived at bin " + String(bin)) : "blocked by obstacle");
}

void handleStatus() {
  float dist = readDistanceCm();
  float load = scale.is_ready() ? scale.get_units(3) : -1.0;
  int attach = readAttachmentId();
  String msg = "Distance: " + String(dist, 1) + " cm\n"
             + "Load: " + String(load, 1) + " g\n"
             + "Attachment ID: " + String(attach);
  server.send(200, "text/plain", msg);
}

// ---------------- Motion + sensing (same logic as main firmware) ----------------
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(L_IN1, leftSpeed >= 0);
  digitalWrite(L_IN2, leftSpeed < 0);
  analogWrite(L_PWM, abs(leftSpeed));

  digitalWrite(R_IN1, rightSpeed >= 0);
  digitalWrite(R_IN2, rightSpeed < 0);
  analogWrite(R_PWM, abs(rightSpeed));}
void stopMotors() { setMotors(0, 0); }

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  if (duration == 0) return 999.0;
  return duration * 0.0343 / 2.0;
}

bool driveForwardCm(float distanceCm, int speed) {
  const float WHEEL_DIAMETER_CM = 6.5;
  const int ENCODER_TICKS_PER_REV = 20;
  const float CM_PER_TICK = (PI * WHEEL_DIAMETER_CM) / ENCODER_TICKS_PER_REV;

  long targetTicks = distanceCm / CM_PER_TICK;
  leftTicks = 0; rightTicks = 0;
  setMotors(speed, speed);

  while (abs((leftTicks + rightTicks) / 2) < targetTicks) {
    if (readDistanceCm() < OBSTACLE_STOP_CM) {
      stopMotors();
      return false;
    }
    delay(10);
  }
  stopMotors();
  return true;
}

bool turnDeg(float angleDeg, int speed) {
  const float TICKS_PER_DEGREE = 1.1;
  long targetTicks = abs(angleDeg) * TICKS_PER_DEGREE;
  leftTicks = 0; rightTicks = 0;

  int dir = (angleDeg >= 0) ? 1 : -1;
  setMotors(dir * speed, -dir * speed);
  while (abs((abs(leftTicks) + abs(rightTicks)) / 2) < targetTicks) {
    delay(10);}
  stopMotors();
  return true;}
int readAttachmentId() {
  int raw = analogRead(ATTACH_ID_PIN);
  if (raw < 800)  return 0;
  if (raw < 1600) return 1;
  if (raw < 2400) return 2;
  if (raw < 3200) return 3;
  return 4;}
