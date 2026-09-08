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
 * The ESP32 creates its OWN WiFi network (no router needed) and serves a
 * simple control page with buttons. Connect your phone/laptop to that
 * network, open a browser, and press buttons to drive the robot.
 *
 * HOW TO USE:
 *   1. Flash this file to your ESP32 (same steps as before — Arduino IDE,
 *      select board + port, Upload).
 *   2. Open Serial Monitor briefly (115200 baud) just to confirm it booted —
 *      you'll see "Access Point started" and an IP address (usually 192.168.4.1).
 *   3. On your phone or laptop, open WiFi settings and connect to the
 *      network named "Tempest_AMR" (password: "tempest123").
 *   4. Open a browser and go to:  http://192.168.4.1
 *   5. You'll see buttons — tap them to control the robot.
 *
 * Install libraries (Arduino Library Manager):
 *   - HX711 (bogde/HX711)
 *   - ESP32Servo
 *   (WiFi.h and WebServer.h come built-in with the ESP32 board package)  define SERVO_PIN 15
#define SERVO_PICK_ANGLE 120
#define SERVO_DROP_ANGLE 20

// Drop confirmation sensor (IR break-beam or microswitch at bin)
#define DROP_CONFIRM_PIN 4

<img width="4096" height="3072" alt="image" src="https://github.com/user-attachments/assets/d77a138c-ef9d-49fd-baca-f8a5fd816da4" />
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/0cc8fab2-ab96-4b42-abff-5484d146f2ff" />
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/841f7901-fc11-4ca0-a982-dcba31952a3b" />

