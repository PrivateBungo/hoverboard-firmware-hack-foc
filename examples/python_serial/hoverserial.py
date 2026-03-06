#!/usr/bin/env python3
"""
hoverserial.py – Python host example for hoverboard-firmware-hack-FOC
======================================================================

Sends SerialCommand frames to the board and reads back:
  • SerialFeedback frames  (always available when FEEDBACK_SERIAL_USARTx is defined)
  • SerialAck frames       (available when CONTROL_SERIAL_ACK is defined in firmware)

Frame layouts (little-endian, packed):
  SerialCommand  : <HhhH   = start(u16) steer(i16) speed(i16) checksum(u16)              [8 bytes]
  SerialFeedback : <HhhhhhhHH = start cmd1 cmd2 speedR speedL bat temp led checksum      [18 bytes]
  SerialAck      : <HHhhH  = start(u16) seq(u16) steer(i16) speed(i16) checksum(u16)    [10 bytes]

Usage
-----
  pip install pyserial
  python hoverserial.py --port /dev/ttyUSB0 [--baud 115200] [--ack] [--speed-max 300]

Quick bench-test procedure (wheels off ground)
-----------------------------------------------
1. Flash firmware with CONTROL_SERIAL_USART3 + FEEDBACK_SERIAL_USART3
   (+ optionally CONTROL_SERIAL_ACK) enabled in Inc/config.h.
2. Connect USB-UART adapter to right (short) sensor cable:
     GND <-> GND,  TX(adapter) <-> RX(board),  RX(adapter) <-> TX(board)
   USART3 is 5 V tolerant, so a standard 5 V adapter is safe.
3. Run:
     python hoverserial.py --port /dev/ttyUSB0 --ack
4. Expected output (with --ack):
     Sent: steer=0    speed=50
     ACK  seq=1   steer=0    speed=50    ✓
     FB   cmd1=0  cmd2=50  speedR=45  speedL=45  bat=3980  temp=28  led=0
5. Increase --speed-max to spin wheels faster; verify speedR/speedL rise.
6. Press Ctrl-C to stop (speed ramps back to 0 before exit).
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial not found. Install it with:  pip install pyserial", file=sys.stderr)
    sys.exit(1)

# ── Frame constants ────────────────────────────────────────────────────────────
START_FRAME     = 0xABCD   # SerialCommand / SerialFeedback start word
ACK_FRAME       = 0xCDAB   # SerialAck start word (SERIAL_ACK_FRAME in firmware)

# start(u16), steer(i16), speed(i16), checksum(u16)
CMD_STRUCT      = struct.Struct("<HhhH")   # 8 bytes
FB_STRUCT       = struct.Struct("<HhhhhhhHH")  # 18 bytes
ACK_STRUCT      = struct.Struct("<HHhhH")  # 10 bytes

CMD_LEN         = CMD_STRUCT.size     # 8
FB_LEN          = FB_STRUCT.size      # 18
ACK_LEN         = ACK_STRUCT.size     # 10

SEND_INTERVAL   = 0.1   # seconds between command frames


# ── Helper: build a command frame ─────────────────────────────────────────────
def build_command(steer: int, speed: int) -> bytes:
    checksum = (START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
    return CMD_STRUCT.pack(START_FRAME, steer, speed, checksum)


# ── Helper: validate feedback checksum ────────────────────────────────────────
def validate_feedback(fields) -> bool:
    start, cmd1, cmd2, speedR, speedL, bat, temp, led, checksum = fields
    calc = (start ^ (cmd1 & 0xFFFF) ^ (cmd2 & 0xFFFF) ^
            (speedR & 0xFFFF) ^ (speedL & 0xFFFF) ^
            (bat & 0xFFFF) ^ (temp & 0xFFFF) ^ led) & 0xFFFF
    return calc == checksum


# ── Helper: validate ACK checksum ─────────────────────────────────────────────
def validate_ack(fields) -> bool:
    start, seq, steer, speed, checksum = fields
    calc = (start ^ seq ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
    return calc == checksum


# ── Receiver: incremental byte-stream parser ──────────────────────────────────
class FrameParser:
    """Pulls complete SerialFeedback and SerialAck frames out of a raw byte stream."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf += data

    def _try_parse_at(self, offset: int):
        """Try to parse a frame starting at buf[offset]. Returns (type, fields, end) or None."""
        buf = self._buf
        if offset + 2 > len(buf):
            return None
        sw = struct.unpack_from("<H", buf, offset)[0]
        if sw == START_FRAME and offset + FB_LEN <= len(buf):
            fields = FB_STRUCT.unpack_from(buf, offset)
            if validate_feedback(fields):
                return ("FB", fields, offset + FB_LEN)
        if sw == ACK_FRAME and offset + ACK_LEN <= len(buf):
            fields = ACK_STRUCT.unpack_from(buf, offset)
            if validate_ack(fields):
                return ("ACK", fields, offset + ACK_LEN)
        return None

    def frames(self):
        """Yield (type, fields) for each complete validated frame; discard leading garbage."""
        while len(self._buf) >= 2:
            result = self._try_parse_at(0)
            if result:
                ftype, fields, end = result
                self._buf = self._buf[end:]
                yield ftype, fields
            else:
                # Slide forward one byte to re-sync
                self._buf = self._buf[1:]


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Hoverboard serial control + telemetry example")
    parser.add_argument("--port",      default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud",      type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--ack",       action="store_true", help="Expect SerialAck frames (requires CONTROL_SERIAL_ACK in firmware)")
    parser.add_argument("--speed-max", type=int, default=300, help="Maximum speed value for ramp test (default: 300)")
    parser.add_argument("--steer",     type=int, default=0,   help="Fixed steer value (default: 0)")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud …")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02)
    except serial.SerialException as exc:
        print(f"Error opening port: {exc}", file=sys.stderr)
        sys.exit(1)

    parser_obj = FrameParser()
    speed      = 0
    step       = 20
    last_send  = 0.0
    ack_seq_expected = None   # track expected ACK seq

    print("Running ramp test. Press Ctrl-C to stop.\n")
    try:
        while True:
            now = time.monotonic()

            # ── Send command at fixed interval ──────────────────────────────
            if now - last_send >= SEND_INTERVAL:
                frame = build_command(args.steer, speed)
                ser.write(frame)
                print(f"Sent: steer={args.steer:<6} speed={speed:<6}", end="")
                if args.ack:
                    print("  (waiting for ACK…)", end="")
                print()
                last_send = now

                # Advance ramp
                speed += step
                if speed >= args.speed_max or speed <= -args.speed_max:
                    step = -step

            # ── Read available bytes ─────────────────────────────────────────
            waiting = ser.in_waiting
            if waiting:
                parser_obj.feed(ser.read(waiting))

            # ── Dispatch complete frames ─────────────────────────────────────
            for ftype, fields in parser_obj.frames():
                if ftype == "FB":
                    _, cmd1, cmd2, speedR, speedL, bat, temp, led, _ = fields
                    print(f"  FB  cmd1={cmd1:<5} cmd2={cmd2:<5} "
                          f"speedR={speedR:<6} speedL={speedL:<6} "
                          f"bat={bat:<5} temp={temp:<4} led={led}")
                elif ftype == "ACK" and args.ack:
                    _, seq, steer_echo, speed_echo, _ = fields
                    ok = "✓" if (ack_seq_expected is None or seq == ack_seq_expected) else "✗ (gap!)"
                    ack_seq_expected = (seq + 1) & 0xFFFF
                    print(f"  ACK seq={seq:<4} steer={steer_echo:<6} speed={speed_echo:<6} {ok}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping …")
        # Ramp speed back to zero
        while speed != 0:
            speed = max(0, speed - abs(step)) if speed > 0 else min(0, speed + abs(step))
            ser.write(build_command(args.steer, speed))
            time.sleep(SEND_INTERVAL)
        ser.write(build_command(0, 0))
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
