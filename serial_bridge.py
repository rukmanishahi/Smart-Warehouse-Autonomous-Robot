"""
serial_bridge.py — UART bridge between the Python "brain" and the ESP32.

Sends line-based commands (MOVE_BIN_3, PICK, DROP, STOP, GET_STATUS) and
parses the ESP32's ACK / DONE / ERROR / STATUS responses.

Install: pip install pyserial
"""

import serial
import time
from dataclasses import dataclass


@dataclass
class RobotStatus:
    distance_cm: float
    load_g: float
    attachment_id: int


class SerialBridge:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # allow ESP32 to reset after port opens
        self._flush_boot_message()

    def _flush_boot_message(self):
        deadline = time.time() + 3
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line == "READY":
                return
            if not line:
                continue

    def send_command(self, command: str, wait_for: str = "DONE", timeout: float = 8.0) -> str:
        """
        Send a command and block until we see the ESP32's ACK, then the
        terminal response (DONE / ERROR,...). Returns the terminal line.
        """
        self.ser.reset_input_buffer()
        self.ser.write((command + "\n").encode())

        deadline = time.time() + timeout
        got_ack = False
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line == "ACK":
                got_ack = True
                continue
            if line.startswith(wait_for) or line.startswith("ERROR"):
                return line
        raise TimeoutError(f"No response to '{command}' (ack_received={got_ack})")

    def get_status(self) -> RobotStatus:
        line = self.send_command("GET_STATUS", wait_for="STATUS")
        # STATUS,DIST:12.3,LOAD:450.0,ATTACH:2
        parts = dict(p.split(":") for p in line.split(",")[1:])
        return RobotStatus(
            distance_cm=float(parts["DIST"]),
            load_g=float(parts["LOAD"]),
            attachment_id=int(parts["ATTACH"]),
        )

    def move_to_bin(self, bin_id: int) -> bool:
        result = self.send_command(f"MOVE_BIN_{bin_id}")
        return result == "DONE"

    def pick(self) -> bool:
        return self.send_command("PICK") == "DONE"

    def drop(self) -> bool:
        return self.send_command("DROP") == "DONE"

    def stop(self) -> bool:
        return self.send_command("STOP") == "DONE"

    def close(self):
        self.ser.close()