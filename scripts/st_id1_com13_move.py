#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path


def add_local_library_path() -> None:
    root = Path(__file__).resolve().parent
    lib_root = root.parent / "library" / "stservo-env"
    if not lib_root.exists():
        raise FileNotFoundError(f"Missing library folder: {lib_root}")
    sys.path.insert(0, str(lib_root))


add_local_library_path()

from STservo_sdk import COMM_SUCCESS, PortHandler, STS_TORQUE_ENABLE, sts  # noqa: E402


def hx(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def wrap_port_io_for_hex(port_handler: PortHandler, enabled: bool) -> None:
    if not enabled:
        return

    original_write = port_handler.writePort
    original_read = port_handler.readPort

    def write_wrapper(packet):
        out = original_write(packet)
        try:
            raw = bytes(packet)
        except Exception:
            raw = b""
        if raw:
            print(f"[RAW TX] {hx(raw)}", flush=True)
        return out

    def read_wrapper(length):
        data = original_read(length)
        raw = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        if raw:
            print(f"[RAW RX] {hx(bytes(raw))}", flush=True)
        return data

    port_handler.writePort = write_wrapper
    port_handler.readPort = read_wrapper


def comm_ok(handler: sts, comm_result: int, error: int, step: str) -> bool:
    ok = True
    if comm_result != COMM_SUCCESS:
        print(f"[ERROR] {step}: {handler.getTxRxResult(comm_result)}", flush=True)
        ok = False
    if error != 0:
        print(f"[ERROR] {step}: {handler.getRxPacketError(error)}", flush=True)
        ok = False
    return ok


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def pct_to_pos(pct: float) -> int:
    return clamp(int(round((pct / 100.0) * 4095.0)), 0, 4095)


def main() -> int:
    parser = argparse.ArgumentParser(description="ST servo move test (defaults: COM13, 115200).")
    parser.add_argument("id_pos", nargs="?", type=int, help="Servo ID (positional shortcut).")
    parser.add_argument("delta_pos", nargs="?", type=int, help="Move delta (positional shortcut).")
    parser.add_argument("speed_pos", nargs="?", type=int, help="Move speed (positional shortcut).")
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--id", "-i", type=int, default=None)
    parser.add_argument("--delta", "-d", type=int, default=None)
    parser.add_argument("--speed", "-s", type=int, default=None)
    parser.add_argument(
        "--position",
        "-p",
        type=int,
        default=None,
        help="Absolute target position (0..4095). If set, move once to this position and do not auto-return.",
    )
    parser.add_argument(
        "--percent",
        type=float,
        default=None,
        help="Absolute target as percent (0..100). Converted to 0..4095. If set, move once and do not auto-return.",
    )
    parser.add_argument("--acc", type=int, default=10)
    parser.add_argument("--wait", type=float, default=0.8)
    parser.add_argument("--hex", action="store_true", help="Hex dump raw serial TX/RX used by library.")
    args = parser.parse_args()

    servo_id = args.id if args.id is not None else (args.id_pos if args.id_pos is not None else 1)
    delta = args.delta if args.delta is not None else (args.delta_pos if args.delta_pos is not None else 120)
    speed = args.speed if args.speed is not None else (args.speed_pos if args.speed_pos is not None else 120)

    if args.position is not None and args.percent is not None:
        print("[ERROR] Use either --position or --percent, not both.", flush=True)
        return 2

    print(f"[STEP] Using ST stack (STservo_sdk.sts), port={args.port}, baud={args.baud}, id={servo_id}", flush=True)
    port_handler = PortHandler(args.port)
    wrap_port_io_for_hex(port_handler, args.hex)
    handler = sts(port_handler)

    print("[STEP] Opening COM port", flush=True)
    if not port_handler.openPort():
        print("[ERROR] openPort failed", flush=True)
        return 1

    try:
        # Flush buffers right after open.
        if hasattr(port_handler, "ser") and port_handler.ser is not None:
            port_handler.ser.reset_input_buffer()
            port_handler.ser.reset_output_buffer()
            port_handler.ser.setRTS(False)
            port_handler.ser.setDTR(False)

        print(f"[STEP] Setting baud={args.baud}", flush=True)
        if not port_handler.setBaudRate(args.baud):
            print("[ERROR] setBaudRate failed", flush=True)
            return 1

        print(f"[STEP] Pinging ST servo ID={servo_id}", flush=True)
        model, comm, err = handler.ping(servo_id)
        if not comm_ok(handler, comm, err, "ping"):
            return 1
        print(f"[OK] Ping success, model={model}", flush=True)

        print("[STEP] Enabling torque", flush=True)
        comm, err = handler.write1ByteTxRx(servo_id, STS_TORQUE_ENABLE, 1)
        if not comm_ok(handler, comm, err, "torque enable"):
            return 1

        print("[STEP] Reading current position", flush=True)
        current_pos, comm, err = handler.ReadPos(servo_id)
        if not comm_ok(handler, comm, err, "ReadPos"):
            return 1
        print(f"[OK] Current position={current_pos}", flush=True)

        if args.position is not None or args.percent is not None:
            if args.percent is not None:
                pct = clamp(args.percent, 0.0, 100.0)
                target = pct_to_pos(pct)
                print(
                    f"[STEP] Move to absolute target={target} ({pct:.2f}%) (speed={speed}, acc={args.acc})",
                    flush=True,
                )
            else:
                target = clamp(args.position, 0, 4095)
                print(f"[STEP] Move to absolute target={target} (speed={speed}, acc={args.acc})", flush=True)
            comm, err = handler.WritePosEx(servo_id, target, speed, args.acc)
            if not comm_ok(handler, comm, err, "WritePosEx absolute"):
                return 1
            time.sleep(args.wait)

            pos_after, comm, err = handler.ReadPos(servo_id)
            if comm_ok(handler, comm, err, "ReadPos after absolute move"):
                print(f"[OK] Position after absolute move={pos_after}", flush=True)
            else:
                return 1
            print("[DONE] Absolute move completed", flush=True)
        else:
            plus = clamp(current_pos + delta, 0, 4095)
            minus = clamp(current_pos - delta, 0, 4095)
            target = plus if plus != current_pos else minus

            print(f"[STEP] Move to target={target} (delta={delta}, speed={speed}, acc={args.acc})", flush=True)
            comm, err = handler.WritePosEx(servo_id, target, speed, args.acc)
            if not comm_ok(handler, comm, err, "WritePosEx target"):
                return 1
            time.sleep(args.wait)

            pos_after_1, comm, err = handler.ReadPos(servo_id)
            if comm_ok(handler, comm, err, "ReadPos after move1"):
                print(f"[OK] Position after move1={pos_after_1}", flush=True)
            else:
                return 1

            print(f"[STEP] Move back to original={current_pos}", flush=True)
            comm, err = handler.WritePosEx(servo_id, current_pos, speed, args.acc)
            if not comm_ok(handler, comm, err, "WritePosEx return"):
                return 1
            time.sleep(args.wait)

            pos_after_2, comm, err = handler.ReadPos(servo_id)
            if comm_ok(handler, comm, err, "ReadPos after move2"):
                print(f"[OK] Position after move2={pos_after_2}", flush=True)
            else:
                return 1

            print("[DONE] Movement test completed", flush=True)
        return 0
    finally:
        print("[STEP] Closing COM port", flush=True)
        port_handler.closePort()


if __name__ == "__main__":
    raise SystemExit(main())
