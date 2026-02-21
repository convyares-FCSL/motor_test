#!/usr/bin/env python3
import importlib
import json
import shlex
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path
from typing import Optional

import rclpy
from lifecycle_msgs.msg import Transition
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from servo_interfaces.srv import ServoCommand, ServoRead


class ServoSerialNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('servo_serial')

        self.declare_parameter('port', 'COM13')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('library_path', '')
        self.declare_parameter('known_ids', [1, 2])
        self.declare_parameter('auto_torque_enable', True)
        self.declare_parameter('command_settle_s', 0.15)
        self.declare_parameter('wait_for_known_ids', True)
        self.declare_parameter('require_all_known_ids', True)
        self.declare_parameter('known_ids_wait_timeout_s', 8.0)
        self.declare_parameter('known_ids_retry_period_s', 0.4)
        self.declare_parameter('startup_settle_s', 0.4)
        self.declare_parameter('simulation_enabled', False)
        self.declare_parameter('simulation_default_position', 2048)
        self.declare_parameter('simulation_command_delay_s', 0.02)
        self.declare_parameter('windows_proxy_enabled', True)
        self.declare_parameter('windows_proxy_timeout_s', 8.0)
        self.declare_parameter('windows_proxy_python', 'py -3')
        self.declare_parameter('windows_proxy_script', '')

        self._sdk = None
        self._port_handler = None
        self._packet_handler = None
        self._comm_success = None
        self._torque_reg = None
        self._transport_mode = 'native'
        self._proxy_port = ''
        self._proxy_baud = 115200
        self._proxy_library_path = ''
        self._proxy_script_path = ''
        self._proxy_python = 'py -3'
        self._proxy_timeout_s = 8.0
        self._proxy_proc: Optional[subprocess.Popen] = None
        self._proxy_proc_lock = threading.Lock()
        self._proxy_stderr_tail: deque[str] = deque(maxlen=20)
        self._proxy_stderr_thread: Optional[threading.Thread] = None
        self._simulation_enabled = False
        self._sim_positions: dict[int, int] = {}
        self._service_command = None
        self._service_read = None
        self._lock = threading.Lock()

    def _resolve_library_path(self) -> str:
        configured = self.get_parameter('library_path').get_parameter_value().string_value
        if configured:
            return configured

        # Try common source/build/install roots and return the first existing SDK folder.
        candidates = []
        here = Path(__file__).resolve()
        candidates.append(Path.cwd() / 'library' / 'stservo-env')
        for parent in [here] + list(here.parents):
            candidates.append(parent / 'library' / 'stservo-env')

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        # Final fallback keeps previous relative pattern.
        return str(Path.cwd() / 'library' / 'stservo-env')

    def _resolve_proxy_script_path(self) -> str:
        configured = self.get_parameter('windows_proxy_script').get_parameter_value().string_value
        if configured:
            return configured

        candidates = []
        here = Path(__file__).resolve()
        candidates.append(Path.cwd() / 'scripts' / 'st_windows_proxy_daemon.py')
        for parent in [here] + list(here.parents):
            candidates.append(parent / 'scripts' / 'st_windows_proxy_daemon.py')
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(Path.cwd() / 'scripts' / 'st_windows_proxy_daemon.py')

    @staticmethod
    def _to_windows_path(path: str) -> str:
        try:
            out = subprocess.check_output(['wslpath', '-w', str(path)], text=True).strip()
            if out:
                return out
        except Exception:
            pass
        return str(path)

    @staticmethod
    def _ps_quote(value: str) -> str:
        return "'" + str(value).replace("'", "''") + "'"

    def _proxy_stderr_reader(self, proc: subprocess.Popen) -> None:
        if proc.stderr is None:
            return
        while True:
            line = proc.stderr.readline()
            if not line:
                break
            text = line.strip()
            if text:
                self._proxy_stderr_tail.append(text)

    def _proxy_readline_with_timeout(self, timeout_s: float) -> tuple[Optional[str], Optional[str]]:
        proc = self._proxy_proc
        if proc is None or proc.stdout is None:
            return None, 'proxy stdout unavailable'

        holder: dict[str, object] = {}

        def _reader() -> None:
            try:
                holder['line'] = proc.stdout.readline()
            except Exception as exc:
                holder['err'] = exc

        reader_thread = threading.Thread(target=_reader, daemon=True)
        reader_thread.start()
        reader_thread.join(timeout=max(0.1, timeout_s))
        if reader_thread.is_alive():
            return None, 'proxy read timeout'
        if 'err' in holder:
            return None, f'proxy read error: {holder["err"]}'
        return str(holder.get('line', '')), None

    def _proxy_request(self, payload: dict) -> dict:
        with self._proxy_proc_lock:
            proc = self._proxy_proc
            if proc is None:
                return {'success': False, 'message': 'proxy not running'}
            if proc.poll() is not None:
                tail = '; '.join(list(self._proxy_stderr_tail)[-3:])
                return {
                    'success': False,
                    'message': f'proxy exited rc={proc.returncode}; stderr_tail={tail or "<empty>"}',
                }
            if proc.stdin is None:
                return {'success': False, 'message': 'proxy stdin unavailable'}

            try:
                proc.stdin.write(json.dumps(payload) + '\n')
                proc.stdin.flush()
            except Exception as exc:
                return {'success': False, 'message': f'proxy write failed: {exc}'}

            timeout_s = float(self._proxy_timeout_s)
            non_json_lines: list[str] = []
            for _ in range(6):
                line, err = self._proxy_readline_with_timeout(timeout_s=timeout_s)
                if err:
                    return {'success': False, 'message': err}
                if line is None:
                    return {'success': False, 'message': 'proxy returned no data'}
                text = line.strip()
                if not text:
                    continue
                try:
                    return json.loads(text)
                except Exception:
                    non_json_lines.append(text)

            return {
                'success': False,
                'message': f'proxy returned non-JSON lines: {" | ".join(non_json_lines) or "<none>"}',
            }

    def _start_proxy_daemon(self) -> bool:
        py_parts = shlex.split(self._proxy_python)
        if not py_parts:
            py_parts = ['py', '-3']

        cmd_parts = ['&'] + [self._ps_quote(v) for v in py_parts]
        cmd_parts.append(self._ps_quote(self._proxy_script_path))
        cmd_parts += [
            '--library-path', self._ps_quote(self._proxy_library_path),
            '--port', self._ps_quote(self._proxy_port),
            '--baud', self._ps_quote(str(self._proxy_baud)),
            '--startup-timeout', self._ps_quote(str(max(1.0, self._proxy_timeout_s))),
        ]
        ps_command = ' '.join(cmd_parts)

        try:
            proc = subprocess.Popen(
                ['powershell.exe', '-NoProfile', '-Command', ps_command],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to start windows proxy process: {exc}')
            return False

        if proc.stdin is None or proc.stdout is None:
            self.get_logger().error('Failed to start windows proxy process: stdin/stdout unavailable')
            return False

        self._proxy_proc = proc
        self._proxy_stderr_tail.clear()
        self._proxy_stderr_thread = threading.Thread(target=self._proxy_stderr_reader, args=(proc,), daemon=True)
        self._proxy_stderr_thread.start()

        line, err = self._proxy_readline_with_timeout(timeout_s=self._proxy_timeout_s)
        if err or line is None:
            self.get_logger().error(f'Windows proxy startup timeout/error: {err or "no output"}')
            self._stop_proxy_daemon()
            return False
        try:
            ready = json.loads(line.strip())
        except Exception:
            self.get_logger().error(f'Windows proxy startup returned non-JSON: {line.strip()}')
            self._stop_proxy_daemon()
            return False
        if not bool(ready.get('success', False)):
            self.get_logger().error(f'Windows proxy startup failed: {ready}')
            self._stop_proxy_daemon()
            return False
        return True

    def _stop_proxy_daemon(self) -> None:
        with self._proxy_proc_lock:
            proc = self._proxy_proc
            self._proxy_proc = None

        if proc is None:
            return

        try:
            if proc.poll() is None and proc.stdin is not None:
                proc.stdin.write(json.dumps({'op': 'exit'}) + '\n')
                proc.stdin.flush()
        except Exception:
            pass

        try:
            proc.wait(timeout=1.5)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
            try:
                proc.wait(timeout=1.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _run_proxy(self, op: str, servo_id: int, position: int = 0, speed: int = 120, acc: int = 10) -> dict:
        payload = {'op': str(op), 'id': int(servo_id)}
        if op == 'write':
            payload['position'] = int(position)
            payload['speed'] = int(speed)
            payload['acc'] = int(acc)
        return self._proxy_request(payload)

    def _init_windows_proxy(self, port: str, baud: int, library_path: str) -> bool:
        script_path = self._resolve_proxy_script_path()
        if not Path(script_path).exists():
            self.get_logger().error(f'windows proxy script not found: {script_path}')
            return False

        self._proxy_script_path = self._to_windows_path(script_path)
        self._proxy_library_path = self._to_windows_path(library_path)
        self._proxy_port = str(port)
        self._proxy_baud = int(baud)
        self._proxy_timeout_s = float(self.get_parameter('windows_proxy_timeout_s').value)
        self._proxy_python = str(self.get_parameter('windows_proxy_python').value)

        if not self._start_proxy_daemon():
            return False

        self._transport_mode = 'windows_proxy'
        self.get_logger().info(
            f'Using Windows proxy transport for {port} via {self._proxy_python} {self._proxy_script_path}'
        )
        return True

    def _init_sdk(self) -> bool:
        self._simulation_enabled = bool(self.get_parameter('simulation_enabled').value)
        known_ids = [int(v) for v in list(self.get_parameter('known_ids').value)]

        if self._simulation_enabled:
            default_pos = int(self.get_parameter('simulation_default_position').value)
            self._sim_positions = {sid: default_pos for sid in known_ids}
            self.get_logger().warn(
                f'Simulation mode enabled. No hardware will be used. IDs={known_ids}, default_pos={default_pos}'
            )
            return True

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        lib_path = self._resolve_library_path()
        is_windows_com = isinstance(port, str) and port.upper().startswith('COM')
        proxy_enabled = bool(self.get_parameter('windows_proxy_enabled').value)
        if sys.platform != 'win32' and is_windows_com and proxy_enabled:
            if not self._init_windows_proxy(port=str(port), baud=baud, library_path=lib_path):
                return False
        else:
            self._transport_mode = 'native'
            if lib_path not in sys.path:
                sys.path.insert(0, lib_path)

            try:
                self._sdk = importlib.import_module('STservo_sdk')
            except Exception as exc:
                self.get_logger().error(f'Failed importing STservo_sdk from {lib_path}: {exc}')
                return False

            self._comm_success = self._sdk.COMM_SUCCESS
            self._torque_reg = self._sdk.STS_TORQUE_ENABLE

            self.get_logger().info(f'Initializing serial transport on port={port}, baud={baud}')

            self._port_handler = self._sdk.PortHandler(port)
            self._packet_handler = self._sdk.sts(self._port_handler)

            try:
                if not self._port_handler.openPort():
                    self.get_logger().error(f'openPort failed on {port}')
                    return False
                if not self._port_handler.setBaudRate(baud):
                    self.get_logger().error(f'setBaudRate({baud}) failed on {port}')
                    return False
            except Exception as exc:
                port_hint = ''
                if isinstance(port, str) and port.upper().startswith('COM') and sys.platform != 'win32':
                    port_hint = (
                        ' Hint: COM* ports are Windows-only. '
                        'Enable windows_proxy_enabled or use /dev/tty*.'
                    )
                self.get_logger().error(f'Serial open/configure exception on {port}: {exc}.{port_hint}')
                return False

            # Keep ESP32 auto-reset lines low if supported by pyserial backend.
            if hasattr(self._port_handler, 'ser') and self._port_handler.ser is not None:
                try:
                    self._port_handler.ser.setRTS(False)
                    self._port_handler.ser.setDTR(False)
                    self._port_handler.ser.reset_input_buffer()
                    self._port_handler.ser.reset_output_buffer()
                except Exception:
                    pass

            startup_settle = float(self.get_parameter('startup_settle_s').value)
            if startup_settle > 0.0:
                time.sleep(startup_settle)
                try:
                    self._port_handler.ser.reset_input_buffer()
                except Exception:
                    pass

        wait_for_ids = bool(self.get_parameter('wait_for_known_ids').value)
        require_all = bool(self.get_parameter('require_all_known_ids').value)
        wait_timeout_s = float(self.get_parameter('known_ids_wait_timeout_s').value)
        retry_s = float(self.get_parameter('known_ids_retry_period_s').value)

        found: dict[int, int] = {}
        started = time.time()
        deadline = started + max(0.0, wait_timeout_s)

        while True:
            for sid in known_ids:
                if sid in found:
                    continue
                try:
                    if self._transport_mode == 'windows_proxy':
                        result = self._run_proxy(op='ping', servo_id=int(sid))
                        ok = bool(result.get('success', False))
                        if ok:
                            model = int(result.get('model', 0))
                            found[sid] = model
                            self.get_logger().info(f'Ping OK id={sid}, model={model}')
                        else:
                            msg = str(result.get('message', 'ping failed'))
                            self.get_logger().debug(f'Ping miss id={sid}: {msg}')
                        continue

                    model, comm, err = self._packet_handler.ping(int(sid))
                except Exception as exc:
                    self.get_logger().error(f'Ping exception while probing id={sid}: {exc}')
                    return False
                if comm == self._comm_success and err == 0:
                    found[sid] = int(model)
                    self.get_logger().info(f'Ping OK id={sid}, model={model}')

            missing = [sid for sid in known_ids if sid not in found]
            if not missing:
                break

            if not wait_for_ids:
                break

            if time.time() >= deadline:
                break
            time.sleep(max(0.05, retry_s))

        missing = [sid for sid in known_ids if sid not in found]
        if missing:
            self.get_logger().warn(f'Missing IDs after scan: {missing}')
            if require_all:
                self.get_logger().error('Required known IDs were not detected during configure')
                return False

        self.get_logger().info(f'Serial link ready on {port} @ {baud}')
        return True

    def _shutdown_sdk(self) -> None:
        self._sim_positions.clear()
        self._transport_mode = 'native'
        self._stop_proxy_daemon()

        if self._port_handler is not None:
            try:
                self._port_handler.closePort()
            except Exception:
                pass

        self._packet_handler = None
        self._port_handler = None
        self._sdk = None

    def _comm_ok(self, comm: int, err: int) -> tuple[bool, str]:
        if comm != self._comm_success:
            return False, self._packet_handler.getTxRxResult(comm)
        if err != 0:
            return False, self._packet_handler.getRxPacketError(err)
        return True, 'ok'

    def _srv_read(self, request: ServoRead.Request, response: ServoRead.Response) -> ServoRead.Response:
        if self._simulation_enabled:
            sid = int(request.id)
            with self._lock:
                pos = self._sim_positions.get(sid)
            if pos is None:
                response.success = False
                response.position = 0
                response.message = f'unknown simulated id {sid}'
            else:
                response.success = True
                response.position = int(pos)
                response.message = 'simulated read'
            return response

        if self._transport_mode == 'windows_proxy':
            result = self._run_proxy(op='read', servo_id=int(request.id))
            response.success = bool(result.get('success', False))
            response.position = int(result.get('position', 0)) if response.success else 0
            response.message = str(result.get('message', 'ok' if response.success else 'read failed'))
            return response

        if self._packet_handler is None:
            response.success = False
            response.position = 0
            response.message = 'serial node is not active'
            return response

        with self._lock:
            pos, comm, err = self._packet_handler.ReadPos(int(request.id))

        ok, msg = self._comm_ok(comm, err)
        response.success = ok
        response.position = int(pos if ok else 0)
        response.message = msg
        return response

    def _srv_command(self, request: ServoCommand.Request, response: ServoCommand.Response) -> ServoCommand.Response:
        if self._simulation_enabled:
            sid = int(request.id)
            pos = int(request.position)
            delay_s = float(self.get_parameter('simulation_command_delay_s').value)
            with self._lock:
                if sid not in self._sim_positions:
                    response.success = False
                    response.present_position = 0
                    response.message = f'unknown simulated id {sid}'
                    return response
                self._sim_positions[sid] = max(0, min(4095, pos))
                present = self._sim_positions[sid]
            if delay_s > 0.0:
                time.sleep(delay_s)
            response.success = True
            response.present_position = int(present)
            response.message = 'simulated command'
            return response

        if self._transport_mode == 'windows_proxy':
            result = self._run_proxy(
                op='write',
                servo_id=int(request.id),
                position=int(request.position),
                speed=int(request.speed),
                acc=int(request.acc),
            )
            response.success = bool(result.get('success', False))
            response.present_position = int(result.get('present_position', 0)) if response.success else 0
            response.message = str(result.get('message', 'ok' if response.success else 'command failed'))
            return response

        if self._packet_handler is None:
            response.success = False
            response.present_position = 0
            response.message = 'serial node is not active'
            return response

        sid = int(request.id)
        pos = int(request.position)
        speed = int(request.speed)
        acc = int(request.acc)
        settle = float(self.get_parameter('command_settle_s').value)
        auto_torque = bool(self.get_parameter('auto_torque_enable').value)

        with self._lock:
            if auto_torque:
                comm, err = self._packet_handler.write1ByteTxRx(sid, self._torque_reg, 1)
                ok, msg = self._comm_ok(comm, err)
                if not ok:
                    response.success = False
                    response.present_position = 0
                    response.message = f'torque enable failed: {msg}'
                    return response

            comm, err = self._packet_handler.WritePosEx(sid, pos, speed, acc)
            ok, msg = self._comm_ok(comm, err)
            if not ok:
                response.success = False
                response.present_position = 0
                response.message = f'command failed: {msg}'
                return response

            if settle > 0.0:
                time.sleep(settle)

            read_pos, comm, err = self._packet_handler.ReadPos(sid)
            ok, msg = self._comm_ok(comm, err)
            response.success = ok
            response.present_position = int(read_pos if ok else 0)
            response.message = msg
            return response

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            if not self._init_sdk():
                return TransitionCallbackReturn.FAILURE
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            self.get_logger().error(f'Unhandled exception in on_configure:\n{traceback.format_exc()}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._service_command = self.create_service(ServoCommand, 'servo_serial/command', self._srv_command)
        self._service_read = self.create_service(ServoRead, 'servo_serial/read', self._srv_read)
        self.get_logger().info('servo_serial services activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._service_command is not None:
            self.destroy_service(self._service_command)
            self._service_command = None
        if self._service_read is not None:
            self.destroy_service(self._service_read)
            self._service_read = None
        self.get_logger().info('servo_serial services deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._shutdown_sdk()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._shutdown_sdk()
        return TransitionCallbackReturn.SUCCESS



def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ServoSerialNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
