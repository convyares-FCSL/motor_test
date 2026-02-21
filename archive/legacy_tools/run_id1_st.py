#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path


def add_local_library_path() -> None:
    project_root = Path(__file__).resolve().parent
    lib_root = project_root / "stservo-env"
    if not lib_root.exists():
        raise FileNotFoundError(f"Library folder not found: {lib_root}")
    sys.path.insert(0, str(lib_root))


add_local_library_path()

from scservo_sdk import (  # noqa: E402
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    SMS_STS_TORQUE_ENABLE,
    PortHandler,
    sms_sts,
)


def log(msg: str) -> None:
    print(msg, flush=True)


def comm_ok(packet_handler: sms_sts, comm_result: int, error: int, step: str) -> bool:
    ok = True
    if comm_result != COMM_SUCCESS:
        log(f"[ERROR] {step}: {packet_handler.getTxRxResult(comm_result)}")
        ok = False
    if error != 0:
        log(f"[ERROR] {step}: {packet_handler.getRxPacketError(error)}")
        ok = False
    return ok


def wait_until_stopped(
    packet_handler: sms_sts,
    servo_id: int,
    timeout_s: float,
    verbose: bool,
) -> bool:
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        moving, comm_result, error = packet_handler.ReadMoving(servo_id)
        if not comm_ok(packet_handler, comm_result, error, "ReadMoving"):
            return False

        pos, comm_result, error = packet_handler.ReadPos(servo_id)
        if not comm_ok(packet_handler, comm_result, error, "ReadPos"):
            return False

        if verbose:
            log(f"[INFO] moving={moving} pos={pos}")
        if moving == 0:
            return True
        time.sleep(0.1)

    log("[WARN] Motion wait timeout reached.")
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Move ST-series serial bus servo on a given ID using Waveshare ST/SC Python library."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help=f"Baud rate (default: {DEFAULT_BAUDRATE})")
    parser.add_argument("--id", type=int, default=1, help="Servo ID (default: 1)")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose readback during moves")
    args = parser.parse_args()

    # Safe-ish defaults for a standard 0..4095 ST position range.
    center = 2047
    left = 1024
    right = 3072
    speed = 300
    acc = 20
    dwell_s = 0.3

    port_handler = PortHandler(args.port)
    packet_handler = sms_sts(port_handler)
    torque_enabled = False
    port_opened = False

    try:
        log(f"[STEP] Opening port {args.port}")
        if not port_handler.openPort():
            log("[ERROR] Failed to open serial port.")
            return 1
        port_opened = True
        log("[OK] Port opened.")

        log(f"[STEP] Setting baud rate to {args.baud}")
        if not port_handler.setBaudRate(args.baud):
            log("[ERROR] Failed to set baud rate.")
            return 1
        log("[OK] Baud rate configured.")

        log(f"[STEP] Pinging servo ID {args.id}")
        model_number, comm_result, error = packet_handler.ping(args.id)
        if not comm_ok(packet_handler, comm_result, error, "Ping"):
            return 1
        log(f"[OK] Ping success. Model number: {model_number}")

        log(f"[STEP] Enabling torque on ID {args.id}")
        comm_result, error = packet_handler.write1ByteTxRx(args.id, SMS_STS_TORQUE_ENABLE, 1)
        if not comm_ok(packet_handler, comm_result, error, "Enable torque"):
            return 1
        torque_enabled = True
        log("[OK] Torque enabled.")

        pos, comm_result, error = packet_handler.ReadPos(args.id)
        if comm_ok(packet_handler, comm_result, error, "Initial ReadPos"):
            log(f"[INFO] Initial position: {pos}")

        sequence = [center, left, right, center]
        for idx, target in enumerate(sequence, start=1):
            log(f"[STEP] Move {idx}/{len(sequence)} -> target position {target}, speed={speed}, acc={acc}")
            comm_result, error = packet_handler.WritePosEx(args.id, target, speed, acc)
            if not comm_ok(packet_handler, comm_result, error, f"WritePosEx target={target}"):
                return 1

            if not wait_until_stopped(packet_handler, args.id, timeout_s=8.0, verbose=args.verbose):
                log("[ERROR] Could not confirm motion completion.")
                return 1
            time.sleep(dwell_s)

            pos, comm_result, error = packet_handler.ReadPos(args.id)
            if not comm_ok(packet_handler, comm_result, error, "ReadPos after move"):
                return 1
            log(f"[INFO] Position readback: {pos}")

        log("[OK] Motion sequence complete.")
        return 0

    except KeyboardInterrupt:
        log("[WARN] Interrupted by user (Ctrl+C).")
        return 130
    except Exception as exc:
        log(f"[ERROR] Serial setup/communication failed: {exc}")
        log("[HINT] On Linux/WSL, use /dev/tty* (for example /dev/ttyS5), not COM*.")
        return 1
    finally:
        if torque_enabled:
            log(f"[STEP] Disabling torque on ID {args.id}")
            comm_result, error = packet_handler.write1ByteTxRx(args.id, SMS_STS_TORQUE_ENABLE, 0)
            if comm_ok(packet_handler, comm_result, error, "Disable torque"):
                log("[OK] Torque disabled.")
        if port_opened:
            log("[STEP] Closing serial port")
            port_handler.closePort()
            log("[OK] Port closed.")


if __name__ == "__main__":
    raise SystemExit(main())
