#!/usr/bin/env python3
"""
uart_control_test.py -- Linux/Mac host test script for hoverboard-firmware-hack-FOC
===================================================================================
Sends SerialCommand frames to the hoverboard over a UART-USB adapter and
optionally decodes the SerialFeedback frames that the board transmits back.

Firmware configuration required (Inc/config.h, VARIANT_USART):
    #define CONTROL_SERIAL_USART2  0    // enable binary command input on LEFT cable
    #define FEEDBACK_SERIAL_USART2      // enable binary feedback output on LEFT cable
    // DEBUG_SERIAL_USART2 must be disabled (conflicts with FEEDBACK_SERIAL_USART2)

Wire-up (LEFT sensor cable, 3.3 V logic only – NOT 5 V tolerant!):
    GND  → GND of USB-UART adapter
    TX2  → RX of USB-UART adapter
    RX2  → TX of USB-UART adapter

Protocol (little-endian, all fields 2 bytes):
    SerialCommand  (host → board):
        uint16  start      = 0xABCD
        int16   steer      steering command  [-1000, 1000]
        int16   speed      speed   command  [-1000, 1000]
        uint16  checksum   = start ^ steer ^ speed  (XOR, treating int16 as uint16)

    SerialFeedback (board → host, sent every ~10 ms):
        uint16  start      = 0xABCD
        int16   cmd1       echoed steer command
        int16   cmd2       echoed speed command
        int16   speedR_meas  right motor speed  [rpm units]
        int16   speedL_meas  left  motor speed  [rpm units]
        int16   batVoltage   battery voltage × 100  [V×100]
        int16   boardTemp    board temperature      [°C]
        uint16  cmdLed       LED command flags
        uint16  checksum     XOR of all preceding fields

Usage:
    python3 uart_control_test.py [--port /dev/ttyUSB0] [--speed 100] [--steer 0]

    --port   Serial device (default: /dev/ttyUSB0)
    --speed  Fixed speed command to hold after ramp  (default: 0 = safe bench test)
    --steer  Fixed steer command to hold             (default: 0)

Press Ctrl+C to stop and send speed=0 before exiting.
"""

import argparse
import struct
import threading
import time
import sys

try:
    import serial
except ImportError:
    sys.exit(
        "pyserial is not installed. Install it with:  pip install pyserial"
    )

# ── Protocol constants ────────────────────────────────────────────────────────
START_FRAME         = 0xABCD
BAUD_RATE           = 115200
SEND_INTERVAL_S     = 0.1       # send a command every 100 ms
RAMP_STEP           = 10        # increment speed by this much per send interval
FEEDBACK_FRAME_SIZE = 18        # bytes: 9 × uint16/int16

# Struct formats (little-endian)
CMD_FMT  = "<HhhH"   # start(u16) steer(i16) speed(i16) checksum(u16)
FB_FMT   = "<HhhhhhhHH"  # start cmd1 cmd2 speedR speedL batV temp cmdLed checksum


def build_command(steer: int, speed: int) -> bytes:
    """Pack a SerialCommand frame with XOR checksum."""
    start = START_FRAME
    checksum = start ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    return struct.pack(CMD_FMT, start, steer, speed, checksum)


def parse_feedback(buf: bytes):
    """
    Return a dict of decoded feedback fields, or None if the frame is invalid.
    Expects exactly FEEDBACK_FRAME_SIZE bytes starting at offset 0.
    """
    if len(buf) < FEEDBACK_FRAME_SIZE:
        return None
    fields = struct.unpack(FB_FMT, buf[:FEEDBACK_FRAME_SIZE])
    start, cmd1, cmd2, speedR, speedL, batV, temp, cmdLed, checksum = fields

    if start != START_FRAME:
        return None

    computed = (
        start
        ^ (cmd1  & 0xFFFF)
        ^ (cmd2  & 0xFFFF)
        ^ (speedR & 0xFFFF)
        ^ (speedL & 0xFFFF)
        ^ (batV  & 0xFFFF)
        ^ (temp  & 0xFFFF)
        ^ cmdLed
    )
    if computed != checksum:
        return None

    return {
        "cmd1":      cmd1,
        "cmd2":      cmd2,
        "speedR":    speedR,
        "speedL":    speedL,
        "batV_x100": batV,
        "temp_C":    temp,
        "cmdLed":    cmdLed,
    }


# ── Feedback reader thread ────────────────────────────────────────────────────
_stop_event = threading.Event()
_rx_buf = bytearray()


def feedback_reader(ser: serial.Serial) -> None:
    """Background thread: read bytes, find 0xABCD, decode full frames."""
    global _rx_buf
    while not _stop_event.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            break
        if not chunk:
            continue
        _rx_buf.extend(chunk)

        # Scan for valid start frames and consume them
        while len(_rx_buf) >= FEEDBACK_FRAME_SIZE:
            # Find START_FRAME (little-endian: 0xCD 0xAB)
            idx = 0
            found = False
            while idx <= len(_rx_buf) - 2:
                if _rx_buf[idx] == 0xCD and _rx_buf[idx + 1] == 0xAB:
                    found = True
                    break
                idx += 1
            if not found:
                # No start frame at all; keep the last byte (might be 0xCD)
                _rx_buf = _rx_buf[-1:]
                break
            if idx > 0:
                # Discard garbage before start frame
                _rx_buf = _rx_buf[idx:]
            if len(_rx_buf) < FEEDBACK_FRAME_SIZE:
                break
            frame = parse_feedback(bytes(_rx_buf[:FEEDBACK_FRAME_SIZE]))
            if frame is not None:
                bat_v = frame["batV_x100"] / 100.0
                print(
                    f"[FB] cmd1={frame['cmd1']:5d}  cmd2={frame['cmd2']:5d}  "
                    f"speedR={frame['speedR']:5d}  speedL={frame['speedL']:5d}  "
                    f"bat={bat_v:.2f}V  temp={frame['temp_C']}°C  "
                    f"led=0x{frame['cmdLed']:04X}",
                    flush=True,
                )
                _rx_buf = _rx_buf[FEEDBACK_FRAME_SIZE:]
            else:
                # Bad frame – skip 1 byte and retry
                _rx_buf = _rx_buf[1:]


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hoverboard UART control+feedback test (VARIANT_USART / USART2)"
    )
    parser.add_argument("--port",  default="/dev/ttyUSB0", help="Serial device")
    parser.add_argument("--speed", type=int, default=0,
                        help="Target speed command [-1000, 1000] (default 0)")
    parser.add_argument("--steer", type=int, default=0,
                        help="Steer command       [-1000, 1000] (default 0)")
    args = parser.parse_args()

    target_speed = max(-1000, min(1000, args.speed))
    target_steer = max(-1000, min(1000, args.steer))

    print(f"Opening {args.port} at {BAUD_RATE} baud (8N1)…")
    try:
        ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open serial port: {exc}")

    print(
        "Connected. Starting with speed=0, steer=0.\n"
        f"Ramping toward speed={target_speed}, steer={target_steer}.\n"
        "Press Ctrl+C to stop safely.\n"
    )

    # Start background reader
    reader = threading.Thread(target=feedback_reader, args=(ser,), daemon=True)
    reader.start()

    current_speed = 0
    try:
        while True:
            # Gentle linear ramp toward target speed
            if current_speed < target_speed:
                current_speed = min(current_speed + RAMP_STEP, target_speed)
            elif current_speed > target_speed:
                current_speed = max(current_speed - RAMP_STEP, target_speed)

            frame = build_command(target_steer, current_speed)
            ser.write(frame)
            print(
                f"[TX] steer={target_steer:5d}  speed={current_speed:5d}",
                flush=True,
            )
            time.sleep(SEND_INTERVAL_S)

    except KeyboardInterrupt:
        print("\nStopping… sending speed=0")
    finally:
        _stop_event.set()
        # Send a few zero-speed frames before closing
        for _ in range(5):
            ser.write(build_command(0, 0))
            time.sleep(0.05)
        ser.close()
        print("Port closed. Goodbye.")


if __name__ == "__main__":
    main()
