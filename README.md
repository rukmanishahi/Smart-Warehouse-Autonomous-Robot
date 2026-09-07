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
