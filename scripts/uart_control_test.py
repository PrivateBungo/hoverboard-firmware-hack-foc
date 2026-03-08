#!/usr/bin/env python3
"""
uart_control_test.py -- Linux/Mac host test script for hoverboard-firmware-hack-FOC
===================================================================================
Sends SerialCommand frames to the hoverboard over one UART-USB adapter (control
port) and optionally captures the CSV debug stream from a second adapter (debug
port) into a timestamped log file.

Firmware configuration required (Inc/config.h, VARIANT_USART):
    #define CONTROL_SERIAL_USART2  0    // enable binary command input on LEFT cable
    #define FEEDBACK_SERIAL_USART2      // enable binary feedback output on LEFT cable
    #define DEBUG_SERIAL_USART3         // enable CSV debug output on RIGHT cable
    // DEBUG_SERIAL_USART2 must be disabled (conflicts with FEEDBACK_SERIAL_USART2)

Wire-up:
    Control port (LEFT sensor cable, USART2 – 3.3 V logic only, NOT 5 V tolerant):
        GND  → GND of USB-UART adapter
        TX2  → RX of USB-UART adapter
        RX2  → TX of USB-UART adapter

    Debug port (RIGHT sensor cable, USART3 – 3.3 V logic only):
        GND  → GND of second USB-UART adapter
        TX3  → RX of USB-UART adapter
        (RX3 is TX-only in debug mode; nothing to connect)

Protocol (little-endian, all fields 2 bytes):
    SerialCommand  (host → board, via control port):
        uint16  start      = 0xABCD
        int16   steer      steering command  [-1000, 1000]
        int16   speed      speed   command  [-1000, 1000]
        uint16  checksum   = start ^ steer ^ speed  (XOR, treating int16 as uint16)

    SerialFeedback (board → host, sent every ~10 ms, via control port):
        uint16  start      = 0xABCD
        int16   cmd1       echoed steer command
        int16   cmd2       echoed speed command
        int16   speedR_meas  right motor speed  [rpm units]
        int16   speedL_meas  left  motor speed  [rpm units]
        int16   batVoltage   battery voltage × 100  [V×100]
        int16   boardTemp    board temperature      [°C]
        uint16  cmdLed       LED command flags
        uint16  checksum     XOR of all preceding fields

    CSV debug stream (board → host, sent every 50 ms (20 Hz), via debug port):
        One header row (column names), then data rows.
        Lines starting with '#' are diagnostic comments and can be ignored.
        Columns:
          t_ms          uptime in ms
          cmdL,cmdR     motor commands [-1000,1000]
          cModReq       requested control mode (0=OPEN,1=VLT,2=SPD,3=TRQ)
          cModActL      actual applied mode for left  motor (may differ from cModReq, e.g. forced to OPEN on fault)
          cModActR      actual applied mode for right motor
          focL          left  commutation relay (0=6-step active, 1=FOC active)
          focR          right commutation relay
          hallL         left Hall sector (1-6; 0/7 = fault)
          angL          left electrical angle (FOC estimate, fixdt units)
          spdL          left speed [RPM]
          phAL..phCL    left BLDC controller phase duty values (DC_phaA/B/C);
                        NOTE: actual motor drive may differ when OPENLOOP override is active
          iqL,idL       left torque/flux currents (raw; divide by 50 for Amps)
          cABL,cBCL     left phase AB/BC currents (raw ADC; divide by 50 for Amps)
          dcL           left DC-link current (raw ADC)
          errL          left error code (0=OK)
          olPhL         left open-loop phase (0=inactive, 1=align, 2=rotate); always 0 if OPENLOOP_ENABLE not defined
          olThL         left open-loop synthetic theta [0..23039] (electrical angle units)
          olDthL        left open-loop delta_theta per ISR (signed)
          olVL          left open-loop voltage amplitude
          uL,vL,wL      left final applied PWM commands written to timer CCRs (post OPENLOOP override)
          (hallR..wR)   same set for right motor

Usage:
    python3 uart_control_test.py --control_port /dev/ttyUSB0 \\
                                  --debug_port   /dev/ttyUSB1 \\
                                  --speed 100 --steering 0

    --control_port  Serial device for control+feedback (default: /dev/ttyUSB0)
    --debug_port    Serial device for CSV debug output (optional).
                    When given, all debug lines are appended to a new
                    debug_YYYYMMDD_HHMMSS.csv file in the current directory.
    --speed         Fixed speed command to hold after ramp  (default: 0)
    --steering      Fixed steering command to hold          (default: 0)

Press Ctrl+C to stop and send speed=0 before exiting.
When --duration_s is set the script stops automatically after that many seconds.
"""

import argparse
import datetime
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


# ── Feedback reader thread (control port) ────────────────────────────────────
_stop_event = threading.Event()
_rx_buf = bytearray()


def feedback_reader(ser: serial.Serial) -> None:
    """Background thread: read bytes from control port, decode SerialFeedback frames."""
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


# ── Debug reader thread (debug port) ─────────────────────────────────────────
def debug_reader(ser: serial.Serial, log_path: str) -> None:
    """
    Background thread: read ASCII lines from the debug port (USART3) and write
    them to *log_path*.  Lines starting with '#' are diagnostic comments and are
    also printed to stdout so they are visible during a live run.

    The logger captures the firmware's CSV header dynamically (line starting
    with ``t_ms,``). If data lines arrive before header, it synthesizes a
    fallback header from the detected column count so captures stay analyzable.
    """
    fallback_headers = {
        # Current firmware format with soft/hard stall telemetry.
        53: (
            "t_ms,cmdL,cmdR,cmdLRaw,cmdRRaw,cModReq,cModActL,cModActR,focL,focR,"
            "hStall,softCondL,softActL,softCondR,softActR,"
            "hallL,angL,spdL,phAL,phBL,phCL,iqL,idL,cABL,cBCL,dcL,errL,"
            "olPhL,olThL,olDthL,olVL,uL,vL,wL,"
            "hallR,angR,spdR,phAR,phBR,phCR,iqR,idR,cABR,cBCR,dcR,errR,"
            "olPhR,olThR,olDthR,olVR,uR,vR,wR"
        ),
        # Legacy firmware format before stall telemetry columns were added.
        46: (
            "t_ms,cmdL,cmdR,cModReq,cModActL,cModActR,focL,focR,"
            "hallL,angL,spdL,phAL,phBL,phCL,iqL,idL,cABL,cBCL,dcL,errL,"
            "olPhL,olThL,olDthL,olVL,uL,vL,wL,"
            "hallR,angR,spdR,phAR,phBR,phCR,iqR,idR,cABR,cBCR,dcR,errR,"
            "olPhR,olThR,olDthR,olVR,uR,vR,wR"
        ),
    }

    line_buf = b""
    header_written = False
    with open(log_path, "w", buffering=1) as log_file:
        print(f"[DBG] Logging CSV debug output to: {log_path}", flush=True)
        while not _stop_event.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            line_buf += chunk
            while b"\n" in line_buf:
                line, line_buf = line_buf.split(b"\n", 1)
                decoded = line.rstrip(b"\r").decode("ascii", errors="replace")
                if decoded.startswith("t_ms,"):
                    if not header_written:
                        log_file.write(decoded + "\n")
                        header_written = True
                    print(f"[DBG] {decoded}", flush=True)
                    continue
                if (not header_written) and decoded and (not decoded.startswith("#")):
                    col_count = len(decoded.split(","))
                    fallback = fallback_headers.get(col_count)
                    if fallback is None:
                        fallback = ",".join(f"col{i}" for i in range(col_count))
                    log_file.write(fallback + "\n")
                    header_written = True
                    print(
                        f"[DBG] Warning: CSV header missing; synthesized fallback header for {col_count} columns",
                        flush=True,
                    )
                log_file.write(decoded + "\n")
                # Echo comment lines to stdout for visibility.
                if decoded.startswith("#"):
                    print(f"[DBG] {decoded}", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hoverboard UART control+feedback test with optional CSV debug capture"
    )
    parser.add_argument(
        "--control_port", default="/dev/ttyUSB0",
        help="Serial device for control+feedback / USART2 (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--debug_port", default=None,
        help="Serial device for CSV debug output / USART3 (optional). "
             "When given, output is saved to debug_YYYYMMDD_HHMMSS.csv",
    )
    parser.add_argument("--speed", type=int, default=0,
                        help="Target speed command [-1000, 1000] (default 0)")
    parser.add_argument("--steering", type=int, default=0,
                        help="Steering command [-1000, 1000] (default 0)")
    parser.add_argument("--steer", type=int, default=None,
                        help="Alias for --steering (deprecated; use --steering)")
    parser.add_argument("--duration_s", type=float, default=None,
                        help="Auto-stop after this many seconds (e.g. 2.0). "
                             "Sends speed=0 and exits cleanly. Default: run until Ctrl+C.")
    args = parser.parse_args()

    target_speed = max(-1000, min(1000, args.speed))
    # --steer is kept as a deprecated alias; --steering takes precedence when both given
    steering_val = args.steer if (args.steer is not None and args.steering == 0) else args.steering
    target_steer = max(-1000, min(1000, steering_val))

    # ── Open control port ─────────────────────────────────────────────────────
    print(f"Opening control port {args.control_port} at {BAUD_RATE} baud (8N1)…")
    try:
        ctrl_ser = serial.Serial(args.control_port, BAUD_RATE, timeout=0.05)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open control port: {exc}")

    print(
        "Connected. Starting with speed=0, steer=0.\n"
        f"Ramping toward speed={target_speed}, steering={target_steer}.\n"
        "Press Ctrl+C to stop safely.\n"
    )

    # ── Start feedback reader on control port ─────────────────────────────────
    reader = threading.Thread(target=feedback_reader, args=(ctrl_ser,), daemon=True)
    reader.start()

    # ── Open debug port if requested ──────────────────────────────────────────
    debug_ser = None
    if args.debug_port:
        print(f"Opening debug port  {args.debug_port} at {BAUD_RATE} baud (8N1)…")
        try:
            debug_ser = serial.Serial(args.debug_port, BAUD_RATE, timeout=0.05)
        except serial.SerialException as exc:
            sys.exit(f"Cannot open debug port: {exc}")

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"debug_{timestamp}.csv"
        dbg_reader = threading.Thread(
            target=debug_reader, args=(debug_ser, log_path), daemon=True
        )
        dbg_reader.start()

    # ── Main control loop ─────────────────────────────────────────────────────
    t0 = time.monotonic()
    current_speed = 0
    try:
        while True:
            # Auto-stop when duration_s has elapsed
            if args.duration_s is not None and (time.monotonic() - t0) >= args.duration_s:
                print(f"\nDuration {args.duration_s} s elapsed — stopping…")
                break

            # Gentle linear ramp toward target speed
            if current_speed < target_speed:
                current_speed = min(current_speed + RAMP_STEP, target_speed)
            elif current_speed > target_speed:
                current_speed = max(current_speed - RAMP_STEP, target_speed)

            frame = build_command(target_steer, current_speed)
            ctrl_ser.write(frame)
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
            ctrl_ser.write(build_command(0, 0))
            time.sleep(0.05)
        ctrl_ser.close()
        if debug_ser is not None:
            debug_ser.close()
        print("Port(s) closed. Goodbye.")


if __name__ == "__main__":
    main()
