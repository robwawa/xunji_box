#!/usr/bin/env python3
import sys
import time

import serial


def build_frame(command):
    payload = command.encode("ascii")
    checksum = len(payload)
    for byte in payload:
        checksum ^= byte
    return bytes((0xAA, 0x54, len(payload))) + payload + bytes((checksum,))


def read_messages(port, duration):
    deadline = time.monotonic() + duration
    buffer = bytearray()
    messages = []
    while time.monotonic() < deadline:
        buffer.extend(port.read(256))
        while True:
            header = buffer.find(b"\xaa\x54")
            if header < 0:
                buffer.clear()
                break
            if header:
                del buffer[:header]
            if len(buffer) < 4:
                break
            length = buffer[2]
            frame_size = length + 4
            if len(buffer) < frame_size:
                break
            payload = bytes(buffer[3:3 + length])
            received_checksum = buffer[3 + length]
            del buffer[:frame_size]
            checksum = length
            for byte in payload:
                checksum ^= byte
            if checksum == received_checksum:
                messages.append(payload.decode("ascii", errors="replace"))
    return messages


def send(port, command):
    port.write(build_frame(command))
    port.flush()
    print("TX", command)


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/lepu_chassis"
    with serial.Serial(device, 115200, timeout=0.05) as port:
        port.reset_input_buffer()
        port.reset_output_buffer()
        send(port, "app_vel[0,0]")
        send(port, "model:request")
        before = read_messages(port, 1.0)
        send(port, "model:mapping")
        time.sleep(0.5)
        send(port, "model:request")
        after = read_messages(port, 2.0)

    for phase, messages in (("BEFORE", before), ("AFTER", after)):
        mode_messages = [msg for msg in messages if msg.startswith("model:")]
        print(phase, "frames=", len(messages), "mode=", mode_messages)


if __name__ == "__main__":
    main()
