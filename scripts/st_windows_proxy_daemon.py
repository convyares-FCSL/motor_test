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


def read_position(handler, comm_success: int, servo_id: int) -> tuple[bool, int, str]:
    pos, comm, err = handler.ReadPos(servo_id)
    ok, msg = comm_result(handler, comm_success, comm, err)
    return ok, int(pos if ok else 0), msg


def set_torque(
    handler,
    comm_success: int,
    servo_id: int,
    torque_reg: int,
    enable: bool,
) -> tuple[bool, int, str]:
    expected = 1 if bool(enable) else 0
    comm, err = handler.write1ByteTxRx(int(servo_id), int(torque_reg), int(expected))
    ok, msg = comm_result(handler, comm_success, comm, err)
    if not ok:
        return False, -1, msg

    # Readback helps verify that torque state actually changed on hardware.
    try:
        value, comm_rb, err_rb = handler.read1ByteTxRx(int(servo_id), int(torque_reg))
        ok_rb, msg_rb = comm_result(handler, comm_success, comm_rb, err_rb)
        if not ok_rb:
            return True, -1, f'write ok; readback unavailable: {msg_rb}'
        readback = int(value)
        if readback != expected:
            return False, readback, f'readback mismatch expected={expected} got={readback}'
        return True, readback, 'ok'
    except Exception as exc:
        return True, -1, f'write ok; readback exception: {exc}'


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
    middle_positions: dict[int, int] = {}

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
                    ok, pos, msg = read_position(handler, COMM_SUCCESS, servo_id)
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
                    ok, _readback, msg = set_torque(
                        handler,
                        COMM_SUCCESS,
                        servo_id,
                        STS_TORQUE_ENABLE,
                        enable=True,
                    )
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
                    ok, pos, msg = read_position(handler, COMM_SUCCESS, servo_id)
                    emit({
                        'success': ok,
                        'present_position': int(pos) if ok else 0,
                        'message': msg,
                    })
                except Exception as exc:
                    emit({'success': False, 'message': f'write exception: {exc}'})
                continue

            if op == 'control':
                command = str(req.get('command', '')).strip().lower().replace(' ', '_')
                setup_speed = max(1, int(req.get('speed', 300)))
                setup_acc = max(1, int(req.get('acc', 20)))

                try:
                    if command == 'torque_on':
                        ok, readback, msg = set_torque(
                            handler,
                            COMM_SUCCESS,
                            servo_id,
                            STS_TORQUE_ENABLE,
                            enable=True,
                        )
                        emit({
                            'success': ok,
                            'present_position': 0,
                            'message': (msg if readback < 0 else f'{msg}; readback={readback}') if ok else f'torque on failed: {msg}',
                        })
                        continue

                    if command in ('release', 'torque_off'):
                        ok, readback, msg = set_torque(
                            handler,
                            COMM_SUCCESS,
                            servo_id,
                            STS_TORQUE_ENABLE,
                            enable=False,
                        )
                        emit({
                            'success': ok,
                            'present_position': 0,
                            'message': (msg if readback < 0 else f'{msg}; readback={readback}') if ok else f'torque off failed: {msg}',
                        })
                        continue

                    if command == 'set_middle':
                        ok, pos, msg = read_position(handler, COMM_SUCCESS, servo_id)
                        if not ok:
                            emit({'success': False, 'present_position': 0, 'message': f'read failed: {msg}'})
                            continue
                        middle_positions[servo_id] = int(pos)
                        emit({'success': True, 'present_position': int(pos), 'message': f'middle set to {int(pos)}'})
                        continue

                    if command == 'middle':
                        target = int(middle_positions.get(servo_id, 2048))
                        ok_torque, _readback, torque_msg = set_torque(
                            handler,
                            COMM_SUCCESS,
                            servo_id,
                            STS_TORQUE_ENABLE,
                            enable=True,
                        )
                        if not ok_torque:
                            emit({
                                'success': False,
                                'present_position': 0,
                                'message': f'torque on failed before middle move: {torque_msg}',
                            })
                            continue
                        comm, err = handler.WritePosEx(servo_id, target, setup_speed, setup_acc)
                        ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                        if not ok:
                            emit({'success': False, 'present_position': 0, 'message': f'middle move failed: {msg}'})
                            continue
                        time.sleep(0.2)
                        ok, pos, msg = read_position(handler, COMM_SUCCESS, servo_id)
                        emit({'success': ok, 'present_position': int(pos) if ok else 0, 'message': msg})
                        continue

                    if command == 'stop':
                        ok_pos, pos, msg_pos = read_position(handler, COMM_SUCCESS, servo_id)
                        if not ok_pos:
                            emit({'success': False, 'present_position': 0, 'message': f'stop read failed: {msg_pos}'})
                            continue

                        ok_torque, _readback, torque_msg = set_torque(
                            handler,
                            COMM_SUCCESS,
                            servo_id,
                            STS_TORQUE_ENABLE,
                            enable=True,
                        )
                        if not ok_torque:
                            emit({'success': False, 'present_position': 0, 'message': f'stop torque-on failed: {torque_msg}'})
                            continue

                        comm, err = handler.WritePosEx(servo_id, int(pos), setup_speed, setup_acc)
                        ok, msg = comm_result(handler, COMM_SUCCESS, comm, err)
                        emit({
                            'success': ok,
                            'present_position': int(pos if ok else 0),
                            'message': 'stop-hold latched' if ok else f'stop hold failed: {msg}',
                        })
                        continue

                    if command == 'recover':
                        ok_off, readback_off, msg_off = set_torque(
                            handler,
                            COMM_SUCCESS,
                            servo_id,
                            STS_TORQUE_ENABLE,
                            enable=False,
                        )
                        if not ok_off:
                            emit({'success': False, 'present_position': 0, 'message': f'recover torque off failed: {msg_off}'})
                            continue
                        time.sleep(0.25)
                        ok_on, readback_on, msg_on = set_torque(
                            handler,
                            COMM_SUCCESS,
                            servo_id,
                            STS_TORQUE_ENABLE,
                            enable=True,
                        )
                        if not ok_on:
                            emit({'success': False, 'present_position': 0, 'message': f'recover torque on failed: {msg_on}'})
                            continue
                        ok_pos, pos, msg_pos = read_position(handler, COMM_SUCCESS, servo_id)
                        if not ok_pos:
                            emit({'success': False, 'present_position': 0, 'message': f'recover done but read failed: {msg_pos}'})
                            continue
                        off_text = str(readback_off) if readback_off >= 0 else '?'
                        on_text = str(readback_on) if readback_on >= 0 else '?'
                        emit({
                            'success': True,
                            'present_position': int(pos),
                            'message': f'recover complete; torque {off_text}->{on_text}; {msg_on}',
                        })
                        continue
                except Exception as exc:
                    emit({'success': False, 'present_position': 0, 'message': f'control exception: {exc}'})
                    continue

                emit({'success': False, 'present_position': 0, 'message': f'unknown control command: {command}'})
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
