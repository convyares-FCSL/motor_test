#!/usr/bin/env python3
import argparse
import time

import serial


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def build_ping_frame(servo_id: int) -> bytes:
    # Same frame format used by local ST/SC Python library ping:
    # FF FF ID 02 01 CHK, CHK = ~(ID + 2 + 1) & 0xFF
    chk = (~(servo_id + 0x02 + 0x01)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id & 0xFF, 0x02, 0x01, chk])


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Open COM first, then trigger serial forwarding from UI, then raw ping."
    )
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--timeout-ms", type=int, default=500)
    parser.add_argument("--tries", type=int, default=5)
    args = parser.parse_args()

    frame = build_ping_frame(args.id)
    print(f"[STEP] Opening {args.port} @ {args.baud}")

    # Keep control lines low as early as possible.
    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.timeout = 0
    ser.bytesize = serial.EIGHTBITS
    ser.rtscts = False
    ser.dsrdtr = False
    ser.rts = False
    ser.dtr = False

    try:
        ser.open()
    except Exception as exc:
        print(f"[ERROR] open failed: {exc}")
        return 1

    print("[OK] Port open and held.")
    print("[ACTION] Now click 'Start Serial Forwarding' in the phone UI.")
    input("Press Enter here after you clicked Start...")

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"[INFO] TX ping frame: {hex_bytes(frame)}")

    for i in range(1, args.tries + 1):
        ser.write(frame)
        t0 = time.time()
        deadline = t0 + args.timeout_ms / 1000.0
        rx = bytearray()
        while time.time() < deadline:
            n = ser.in_waiting
            if n:
                rx.extend(ser.read(n))
            time.sleep(0.005)

        if rx:
            print(f"[TRY {i}] RX[{len(rx)}]: {hex_bytes(bytes(rx))}")
        else:
            print(f"[TRY {i}] RX timeout")
        time.sleep(0.1)

    ser.close()
    print("[STEP] Port closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
