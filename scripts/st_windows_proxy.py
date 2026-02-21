#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path


def add_library_path(path: str) -> None:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f'library path not found: {p}')
    sys.path.insert(0, str(p))


def comm_result(handler, comm_success: int, comm: int, err: int) -> tuple[bool, str]:
    if comm != comm_success:
        return False, str(handler.getTxRxResult(comm))
    if err != 0:
        return False, str(handler.getRxPacketError(err))
    return True, 'ok'


def main() -> int:
    parser = argparse.ArgumentParser(description='Windows-side ST servo proxy helper.')
    parser.add_argument('--library-path', required=True)
    parser.add_argument('--port', default='COM13')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--id', type=int, required=True)
    parser.add_argument('--op', choices=['ping', 'read', 'write'], required=True)
    parser.add_argument('--position', type=int, default=0)
    parser.add_argument('--speed', type=int, default=120)
    parser.add_argument('--acc', type=int, default=10)
    parser.add_argument('--settle', type=float, default=0.2)
    args = parser.parse_args()

    try:
        add_library_path(args.library_path)
        from STservo_sdk import COMM_SUCCESS, PortHandler, STS_TORQUE_ENABLE, sts
    except Exception as exc:
        print(json.dumps({'success': False, 'message': f'import failed: {exc}'}))
        return 1

    port = PortHandler(args.port)
    handler = sts(port)

    try:
        if not port.openPort():
            print(json.dumps({'success': False, 'message': f'openPort failed on {args.port}'}))
            return 1

        if hasattr(port, 'ser') and port.ser is not None:
            try:
                port.ser.reset_input_buffer()
                port.ser.reset_output_buffer()
                port.ser.setRTS(False)
                port.ser.setDTR(False)
            except Exception:
                pass

        if not port.setBaudRate(args.baud):
            print(json.dumps({'success': False, 'message': f'setBaudRate({args.baud}) failed'}))
            return 1

        if hasattr(port, 'ser') and port.ser is not None:
            try:
                port.ser.reset_input_buffer()
            except Exception:
                pass

        if args.op == 'ping':
            model, comm, err = handler.ping(int(args.id))
            ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
            print(json.dumps({
                'success': ok,
                'model': int(model) if ok else 0,
                'message': msg,
            }))
            return 0 if ok else 1

        if args.op == 'read':
            pos, comm, err = handler.ReadPos(int(args.id))
            ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
            print(json.dumps({
                'success': ok,
                'position': int(pos) if ok else 0,
                'message': msg,
            }))
            return 0 if ok else 1

        comm, err = handler.write1ByteTxRx(int(args.id), STS_TORQUE_ENABLE, 1)
        ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
        if not ok:
            print(json.dumps({'success': False, 'present_position': 0, 'message': f'torque enable failed: {msg}'}))
            return 1

        target = max(0, min(4095, int(args.position)))
        comm, err = handler.WritePosEx(int(args.id), target, int(args.speed), int(args.acc))
        ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
        if not ok:
            print(json.dumps({'success': False, 'present_position': 0, 'message': f'command failed: {msg}'}))
            return 1

        if args.settle > 0.0:
            time.sleep(float(args.settle))

        pos, comm, err = handler.ReadPos(int(args.id))
        ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
        print(json.dumps({
            'success': ok,
            'present_position': int(pos) if ok else 0,
            'message': msg,
        }))
        return 0 if ok else 1
    except Exception as exc:
        print(json.dumps({'success': False, 'message': f'exception: {exc}'}))
        return 1
    finally:
        try:
            port.closePort()
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
