#!/usr/bin/env python3
"""SLAM 3.0 serial protocol helpers for Lepu car."""

from __future__ import annotations

import re
import threading
from typing import Callable, Iterable, List, Optional, Tuple

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise ImportError('pyserial is required: pip3 install pyserial') from exc

FRAME_HEADER = b'\xAA\x54'


def calc_checksum(data_bytes: bytes) -> int:
    result = len(data_bytes) & 0xFF
    for value in data_bytes:
        result ^= value
    return result


def build_frame(cmd_str: str) -> bytes:
    data_bytes = cmd_str.encode('ascii')
    length = len(data_bytes)
    checksum = calc_checksum(data_bytes)
    return FRAME_HEADER + bytes([length]) + data_bytes + bytes([checksum])


def parse_frame(data: bytes) -> Optional[str]:
    if len(data) < 4:
        return None
    if data[0] != 0xAA or data[1] != 0x54:
        return None

    length = data[2]
    if len(data) < length + 4:
        return None

    payload = data[3:3 + length]
    checksum = data[3 + length]
    expected = calc_checksum(payload)
    if expected != checksum:
        return None
    return payload.decode('ascii', errors='ignore')


class SerialFrameReader:
    """Incremental parser for AA 54 framed ASCII payloads."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._lock = threading.Lock()

    def feed(self, chunk: bytes) -> List[str]:
        messages: List[str] = []
        with self._lock:
            self._buffer.extend(chunk)
            while True:
                frame_start = self._buffer.find(FRAME_HEADER)
                if frame_start < 0:
                    self._buffer.clear()
                    break
                if frame_start > 0:
                    del self._buffer[:frame_start]

                if len(self._buffer) < 3:
                    break

                length = self._buffer[2]
                frame_len = length + 4
                if len(self._buffer) < frame_len:
                    break

                frame = bytes(self._buffer[:frame_len])
                del self._buffer[:frame_len]
                parsed = parse_frame(frame)
                if parsed is not None:
                    messages.append(parsed)
        return messages


NAV_TIME_POSE_RE = re.compile(
    r'nav:time_pose\[([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\]'
)
NAV_POSE_RE = re.compile(r'nav:pose\[([-\d.]+),([-\d.]+),([-\d.]+)\]')
BASE_VEL_RE = re.compile(r'base_vel\[([-\d.]+)\s+([-\d.]+)\]')
CORE_DATA_RE = re.compile(r'core_data\{([^}]+)\}')
WHEEL_STATUS_RE = re.compile(r'wheel_status\{([^}]+)\}')
MODEL_RE = re.compile(r'model:(\d+)')
HFLS_VERSION_RE = re.compile(
    r'hfls_version:([^,]+),([^,]+),([^,]+),(.+)'
)


def parse_nav_pose(message: str) -> Optional[Tuple[float, float, float, Optional[int]]]:
    match = NAV_TIME_POSE_RE.search(message)
    if match:
        x, y, yaw, stamp = match.groups()
        return float(x), float(y), float(yaw), float(stamp)

    match = NAV_POSE_RE.search(message)
    if match:
        x, y, yaw = match.groups()
        return float(x), float(y), float(yaw), None
    return None


def parse_base_vel(message: str) -> Optional[Tuple[float, float]]:
    match = BASE_VEL_RE.search(message)
    if not match:
        return None
    return float(match.group(1)), float(match.group(2))


def parse_core_data(message: str) -> Optional[List[int]]:
    match = CORE_DATA_RE.search(message)
    if not match:
        return None
    parts = match.group(1).strip().split()
    if len(parts) < 5:
        return None
    values = [int(float(value)) for value in parts]
    # Some firmware builds report 5 fields; full API reports 7 with encoders.
    while len(values) < 7:
        values.append(0)
    return values[:7]


def parse_wheel_status(message: str) -> Optional[List[int]]:
    match = WHEEL_STATUS_RE.search(message)
    if not match:
        return None
    values: List[int] = []
    for token in match.group(1).split():
        try:
            values.append(int(float(token)))
        except ValueError:
            break
    return values if len(values) >= 7 else None


def parse_wheel_encoders(message: str) -> Optional[Tuple[int, int]]:
    """Return (left, right) encoder ticks from core_data or wheel_status."""
    core_data = parse_core_data(message)
    if core_data is not None:
        return core_data[5], core_data[6]

    wheel_status = parse_wheel_status(message)
    if wheel_status is not None:
        return wheel_status[5], wheel_status[6]
    return None


class LepuSerialLink:
    """Thread-safe serial link with framed read/write helpers."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.05,
        on_message: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.on_message = on_message
        self._serial: Optional[serial.Serial] = None
        self._reader = SerialFrameReader()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    def open(self) -> None:
        self._serial = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def send_command(self, command: str) -> None:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError('Serial port is not open')
        frame = build_frame(command)
        with self._write_lock:
            self._serial.write(frame)

    def transact(self, command: str, wait_sec: float = 0.5) -> List[str]:
        self.send_command(command)
        return self.collect_messages(wait_sec)

    def collect_messages(self, wait_sec: float) -> List[str]:
        import time

        collected: List[str] = []
        end_time = time.time() + wait_sec
        while time.time() < end_time:
            time.sleep(0.05)
        return collected

    def _read_loop(self) -> None:
        import time

        while not self._stop.is_set():
            if self._serial is None or not self._serial.is_open:
                break
            try:
                chunk = self._serial.read(256)
            except serial.SerialException:
                break
            if not chunk:
                time.sleep(0.01)
                continue
            for message in self._reader.feed(chunk):
                if self.on_message is not None:
                    self.on_message(message)
