# Tempest AMR for Sorting Logic Reference Implementation
SIH26112  Modular AMR Platform for Smart Warehouse Automation

This matches the architecture:

```
Camera/Sensor -> [Python: classify box]
              -> send command over serial
              -> [C/C++: control motors/arm]
              -> sensor confirms drop
              -> send status back to Python
              -> loop
```

## Layout
```
esp32_firmware/
  esp32_firmware.ino     # C++ (Arduino) motors, encoders, sensors, actuator, serial protocol
python/
  serial_bridge.py       # UART bridge to the ESP32
  vision_classifier.py   # OpenCV: color / size / barcode detection
  decision_engine.py     # Rules engine: attributes -> destination bin
  sorting_controller.py  # State machine: IDLE -> SCAN -> CLASSIFY -> MOVE -> DROP -> IDLE
  requirements.txt
```

## Setup
1. **ESP32**: open `esp32_firmware/esp32_firmware.ino` in Arduino IDE, install the
   `HX711` and `ESP32Servo` libraries, adjust the pin map and `binTable` to your
   wiring, then flash it.
2. **Python**: `pip install -r python/requirements.txt`
3. Connect the ESP32, find its serial port, then run:
   ```
   python python/sorting_controller.py --port /dev/ttyUSB0
   ```

## What's real vs. what needs calibration
This is a working reference implementation of the *logic and protocol*, not
a plug and play drop-in before it runs on your actual robot you'll need to:
- Confirm the pin map against your motor driver / sensor wiring.
- Calibrate `CM_PER_TICK`, `TICKS_PER_DEGREE`, and the HX711 `set_scale()` factor
  on the real hardware.
- Tune the HSV color ranges and size thresholds in `vision_classifier.py` to your
  camera, lighting, and actual boxes.
- Replace the simple point-to-point `binTable` moves with real grid/A* path
  planning (ROS 2 Nav2) once you move from single-line paths to a full warehouse map.
- Calibrate the `ATTACH_ID_PIN` voltage bands to whatever resistor values you
  put on each physical attachment.




## FOR THE ESP32 FILMWARE LOGIC MAP

/*
 * Responsibilities (real-time / hardware layer):
 *   - Differential-drive motor control (PWM)
 *   - Quadrature encoder reading (odometry / distance-based moves)
 *   - Obstacle sensor (ultrasonic) — safety stop
 *   - Load cell (HX711) — weight feedback
 *   - Attachment identification (analog ID pin on the attachment connector)
 *   - Actuator control (servo gripper / diverter) with drop confirmation
 *   - Serial command protocol to talk to the Python "brain"
 *
 * Protocol (line-based, newline terminated):
 *   Host -> ESP32:
 *     MOVE_BIN_<n>      e.g. MOVE_BIN_3   -> drive to preset bin position n
 *     PICK                                -> close gripper / engage attachment
 *     DROP                                -> open gripper / release load
 *     STOP                                -> emergency stop
 *     GET_STATUS                          -> request one status line
 *
 *   ESP32 -> Host:
 *     ACK                                 -> command received
 *     DONE                                -> action completed successfully
 *     ERROR,<reason>                      -> action failed (e.g. obstacle, timeout)
 *     STATUS,DIST:<cm>,LOAD:<g>,ATTACH:<id>
 *
 * Install libraries (Arduino Library Manager):
 *   - HX711 (bogde/HX711) for the load cell
 *   - ESP32Servo for the actuator
 */

#include <HX711.h>
#include <ESP32Servo.h>

// ---------------- Pin map ----------------
// Motor driver (e.g. L298N / TB6612) — left & right
#define L_IN1 25
#define L_IN2 26
#define L_PWM 27
#define R_IN1 14
#define R_IN2 12
#define R_PWM 13

// Quadrature encoders (interrupt-capable pins)
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 32
#define R_ENC_B 33

// Ultrasonic obstacle sensor (HC-SR04)
#define TRIG_PIN 5
#define ECHO_PIN 18
#define OBSTACLE_STOP_CM 15.0

// Load cell (HX711)
#define HX711_DT 19
#define HX711_SCK 23

// Attachment ID line (each attachment presents a distinct voltage via
// a resistor divider on its connector; ADC read maps to an ID)
#define ATTACH_ID_PIN 36

// Actuator (gripper / diverter servo)
#define SERVO_PIN 15
#define SERVO_PICK_ANGLE 120
#define SERVO_DROP_ANGLE 20

// Drop confirmation sensor (IR break-beam or microswitch at bin)
#define DROP_CONFIRM_PIN 4
