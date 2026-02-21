#!/usr/bin/env python3
import argparse
import binascii
import sys
import time

import serial


def hex_bytes(data: bytes) -> str:
    if not data:
        return ""
    return " ".join(f"{b:02X}" for b in data)


def build_ping_frame(servo_id: int) -> bytes:
    # Library-defined packet fields:
    # [0xFF, 0xFF, ID, LENGTH=2, INST_PING=1, CHECKSUM]
    length = 0x02
    inst_ping = 0x01
    checksum = (~(servo_id + length + inst_ping)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id & 0xFF, length, inst_ping, checksum])


def passive_sniff(port: str, baud: int, duration_s: float) -> None:
    print(f"\n[sniff] port={port} baud={baud} duration={duration_s}s")
    with serial.Serial(port=port, baudrate=baud, timeout=0, bytesize=serial.EIGHTBITS) as ser:
        ser.setRTS(False)
        ser.setDTR(False)
        ser.reset_input_buffer()
        t0 = time.time()
        total = 0
        while (time.time() - t0) < duration_s:
            n = ser.in_waiting
            if n:
                data = ser.read(n)
                total += len(data)
                ts = time.time() - t0
                print(f"  t={ts:0.3f}s rx[{len(data)}]: {hex_bytes(data)}")
            time.sleep(0.01)
        if total == 0:
            print("  no bytes captured")
        else:
            print(f"  total captured bytes: {total}")


def ping_once(port: str, baud: int, servo_id: int, timeout_ms: int) -> None:
    frame = build_ping_frame(servo_id)
    expected_reply = "FF FF <ID> <LEN> <ERR> <CHK> (minimum 6 bytes; LEN may increase with params)"
    print(f"\n[ping] port={port} baud={baud} id={servo_id}")
    print(f"  tx frame: {hex_bytes(frame)}")
    print(f"  expected reply format: {expected_reply}")

    with serial.Serial(port=port, baudrate=baud, timeout=0, bytesize=serial.EIGHTBITS) as ser:
        ser.setRTS(False)
        ser.setDTR(False)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        t_tx = time.time()
        written = ser.write(frame)
        print(f"  wrote {written} bytes")

        deadline = t_tx + (timeout_ms / 1000.0)
        rx = bytearray()
        while time.time() < deadline:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                rx.extend(chunk)
                dt = (time.time() - t_tx) * 1000.0
                print(f"  t+{dt:0.1f}ms rx[{len(chunk)}]: {hex_bytes(chunk)}")
            time.sleep(0.005)

        if not rx:
            print(f"  timeout: no bytes received within {timeout_ms} ms")
        else:
            print(f"  total rx bytes: {len(rx)}")
            print(f"  rx aggregate: {hex_bytes(bytes(rx))}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Raw byte-level diagnostics for ST/SC serial forwarding.")
    parser.add_argument("--port", default="COM13", help="Serial port (default: COM13)")
    parser.add_argument("--id", type=int, default=1, help="Servo ID for ping (default: 1)")
    parser.add_argument("--timeout-ms", type=int, default=300, help="Ping receive timeout in ms (default: 300)")
    parser.add_argument("--sniff-seconds", type=float, default=3.0, help="Passive sniff duration per baud")
    parser.add_argument(
        "--mode",
        choices=["all", "ping", "sniff"],
        default="all",
        help="Run raw ping tests, passive sniffer tests, or both",
    )
    args = parser.parse_args()

    if args.id < 0 or args.id > 253:
        print("id must be in range 0..253")
        return 2

    bauds = [115200, 1000000]
    print("Library packet baseline:")
    print("  ping request format: FF FF ID 02 01 CHK")
    print("  checksum: CHK = ~(ID + 0x02 + 0x01) & 0xFF")
    print("  library default servo baud: 1000000")
    print("  transport type: raw UART bytes (no encapsulation layer in library)")

    try:
        if args.mode in ("all", "sniff"):
            for baud in bauds:
                passive_sniff(args.port, baud, args.sniff_seconds)

        if args.mode in ("all", "ping"):
            for baud in bauds:
                ping_once(args.port, baud, args.id, args.timeout_ms)
    except serial.SerialException as exc:
        print(f"serial error: {exc}")
        return 1
    except KeyboardInterrupt:
        print("interrupted")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
