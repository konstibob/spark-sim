"""Portable reader for the proposed ESP32-C6 VL53L5CX JSONL bridge.
Copy this file to the real Zumo only if the ESP firmware uses docs/VL53L5CX.md's protocol.
This is a serial reader, not an ST/Pololu sensor driver.
"""
import json
from machine import UART


class ToFSerial:
    def __init__(self, uart=None):
        self.uart = uart if uart is not None else UART(0, 115200, timeout=0, rxbuf=4096)
        self._buffer = b""

    def read(self):
        """Return the next complete frame dict, or None. Retain incomplete UART lines."""
        if self.uart.any():
            self._buffer += self.uart.read() or b""
        if len(self._buffer) > 16384:
            self._buffer = b""
            raise ValueError("ESP depth frame exceeds receive buffer")
        if b"\n" not in self._buffer:
            return None
        line, self._buffer = self._buffer.split(b"\n", 1)
        data = json.loads(line)
        n = data.get("resolution")
        if data.get("type") != "vl53l5cx" or n not in (16, 64):
            raise ValueError("unexpected ESP depth frame")
        for key in ("distance_mm", "target_status", "nb_target_detected"):
            if len(data.get(key, [])) != n:
                raise ValueError("incomplete ESP depth array")
        return data
