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


def emit(payload: dict) -> None:
    print(json.dumps(payload), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description='Persistent Windows-side ST servo proxy daemon.')
    parser.add_argument('--library-path', required=True)
    parser.add_argument('--port', default='COM13')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--startup-timeout', type=float, default=8.0)
    args = parser.parse_args()

    try:
        add_library_path(args.library_path)
        from STservo_sdk import COMM_SUCCESS, PortHandler, STS_TORQUE_ENABLE, sts
    except Exception as exc:
        emit({'success': False, 'message': f'import failed: {exc}'})
        return 1

    port = PortHandler(args.port)
    handler = sts(port)

    try:
        if not port.openPort():
            emit({'success': False, 'message': f'openPort failed on {args.port}'})
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
            emit({'success': False, 'message': f'setBaudRate({args.baud}) failed'})
            return 1

        if hasattr(port, 'ser') and port.ser is not None:
            try:
                port.ser.reset_input_buffer()
            except Exception:
                pass

        # Ready marker for the parent process.
        emit({'success': True, 'message': 'proxy ready'})

        while True:
            line = sys.stdin.readline()
            if line == '':
                break

            line = line.strip()
            if not line:
                continue

            try:
                req = json.loads(line)
            except Exception as exc:
                emit({'success': False, 'message': f'invalid request json: {exc}'})
                continue

            op = str(req.get('op', '')).strip().lower()
            if op == 'exit':
                emit({'success': True, 'message': 'bye'})
                break

            try:
                servo_id = int(req.get('id', 0))
            except Exception:
                emit({'success': False, 'message': 'id must be int'})
                continue

            if op == 'ping':
                try:
                    model, comm, err = handler.ping(servo_id)
                    ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                    emit({'success': ok, 'model': int(model) if ok else 0, 'message': msg})
                except Exception as exc:
                    emit({'success': False, 'message': f'ping exception: {exc}'})
                continue

            if op == 'read':
                try:
                    pos, comm, err = handler.ReadPos(servo_id)
                    ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                    emit({'success': ok, 'position': int(pos) if ok else 0, 'message': msg})
                except Exception as exc:
                    emit({'success': False, 'message': f'read exception: {exc}'})
                continue

            if op == 'write':
                try:
                    position = int(req.get('position', 0))
                    speed = int(req.get('speed', 120))
                    acc = int(req.get('acc', 10))
                except Exception:
                    emit({'success': False, 'message': 'position/speed/acc must be int'})
                    continue

                try:
                    comm, err = handler.write1ByteTxRx(servo_id, STS_TORQUE_ENABLE, 1)
                    ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                    if not ok:
                        emit({'success': False, 'present_position': 0, 'message': f'torque enable failed: {msg}'})
                        continue

                    target = max(0, min(4095, int(position)))
                    comm, err = handler.WritePosEx(servo_id, target, speed, acc)
                    ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                    if not ok:
                        emit({'success': False, 'present_position': 0, 'message': f'command failed: {msg}'})
                        continue

                    time.sleep(0.2)
                    pos, comm, err = handler.ReadPos(servo_id)
                    ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                    emit({
                        'success': ok,
                        'present_position': int(pos) if ok else 0,
                        'message': msg,
                    })
                except Exception as exc:
                    emit({'success': False, 'message': f'write exception: {exc}'})
                continue

            emit({'success': False, 'message': f'unknown op: {op}'})

        return 0
    finally:
        try:
            port.closePort()
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
