"""
sorting_controller.py — Top-level state machine for the Tempest AMR.

    IDLE -> SCAN -> CLASSIFY -> MOVE -> DROP -> IDLE (loop)

This is the Python "brain": it owns perception (vision_classifier) and the
decision engine, and drives the ESP32 (serial_bridge) for everything that
needs real-time hardware timing (motors, sensors, actuation).

Run:
    python sorting_controller.py --port /dev/ttyUSB0
"""

import argparse
import time
import logging
from enum import Enum, auto

from serial_bridge import SerialBridge
from vision_classifier import BoxClassifier, BoxAttributes
from decision_engine import classify_to_bin

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("sorting_controller")


class State(Enum):
    IDLE = auto()
    SCAN = auto()
    CLASSIFY = auto()
    MOVE = auto()
    DROP = auto()
    ERROR = auto()


class SortingController:
    def __init__(self, port: str):
        self.bridge = SerialBridge(port)
        self.classifier = BoxClassifier()
        self.state = State.IDLE
        self.current_attrs: BoxAttributes | None = None
        self.current_bin: int | None = None

    def run_forever(self):
        log.info("Sorting controller started.")
        while True:
            try:
                self.step()
            except KeyboardInterrupt:
                log.info("Stopping (user interrupt).")
                self.bridge.stop()
                break
            except TimeoutError as e:
                log.error(f"Hardware timeout: {e}")
                self.state = State.ERROR

            if self.state == State.ERROR:
                self._handle_error()

    def step(self):
        if self.state == State.IDLE:
            log.info("IDLE -> waiting for a box on the scan station.")
            time.sleep(0.5)
            self.state = State.SCAN

        elif self.state == State.SCAN:
            attrs = self.classifier.capture_and_classify()
            if attrs.detected:
                log.info(f"Box detected: color={attrs.color}, size={attrs.size}, "
                         f"barcode={attrs.barcode}")
                self.current_attrs = attrs
                self.state = State.CLASSIFY
            else:
                time.sleep(0.3)  # keep polling for a box

        elif self.state == State.CLASSIFY:
            status = self.bridge.get_status()  # includes load cell reading
            bin_id = classify_to_bin(self.current_attrs, status.load_g)
            log.info(f"Classified -> bin {bin_id} (load={status.load_g}g, "
                     f"attachment={status.attachment_id})")
            self.current_bin = bin_id
            self.state = State.MOVE

        elif self.state == State.MOVE:
            if not self.bridge.pick():
                raise RuntimeError("Pick failed")
            ok = self.bridge.move_to_bin(self.current_bin)
            if ok:
                self.state = State.DROP
            else:
                log.warning("Move blocked (likely obstacle) — retrying shortly.")
                time.sleep(1.0)

        elif self.state == State.DROP:
            ok = self.bridge.drop()
            if ok:
                log.info(f"Box delivered to bin {self.current_bin}.")
            else:
                log.warning("Drop not confirmed — flagging for manual check.")
            self.bridge.move_to_bin(0)  # return to home/scan station
            self.current_attrs = None
            self.current_bin = None
            self.state = State.IDLE

    def _handle_error(self):
        log.info("Recovering from error: stopping motors, returning to IDLE.")
        try:
            self.bridge.stop()
        except Exception:
            pass
        time.sleep(1.0)
        self.state = State.IDLE

    def shutdown(self):
        self.classifier.release()
        self.bridge.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0 or COM5")
    args = parser.parse_args()

    controller = SortingController(args.port)
    try:
        controller.run_forever()
    finally:
        controller.shutdown()