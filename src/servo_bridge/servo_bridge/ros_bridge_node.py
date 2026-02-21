#!/usr/bin/env python3
import asyncio
import json
import threading
from pathlib import Path
from typing import Optional

from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from servo_interfaces.action import MotorMove
from servo_interfaces.msg import MotorState
from servo_interfaces.srv import ServoRead


class RosBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('ros_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('state_topic', 'servo/state')
        self.declare_parameter('motor_action_name', 'servo_motor/move')
        self.declare_parameter('known_ids', [1, 2])
        self.declare_parameter('motor_ids', [1, 2])
        self.declare_parameter('motor_min', [0, 0])
        self.declare_parameter('motor_max', [4095, 4095])
        self.declare_parameter('names_file', '')
        self.declare_parameter('positions_file', '')
        self.declare_parameter('serial_read_service', 'servo_serial/read')
        self.declare_parameter('live_poll_period_s', 1.0)

        state_topic = str(self.get_parameter('state_topic').value)
        action_name = str(self.get_parameter('motor_action_name').value)
        read_service = str(self.get_parameter('serial_read_service').value)
        live_poll_period_s = float(self.get_parameter('live_poll_period_s').value)

        self._sub = self.create_subscription(MotorState, state_topic, self._on_state, 10)
        self._action_client = ActionClient(self, MotorMove, action_name)
        self._read_client = self.create_client(ServoRead, read_service)
        self._live_limits = self._build_live_limits()
        self._live_poll_pending: set[int] = set()
        self._live_poll_lock = threading.Lock()
        self._live_timer = None
        if live_poll_period_s > 0.0:
            self._live_timer = self.create_timer(live_poll_period_s, self._poll_live_positions)

        self._ws_clients = set()
        self._clients_lock = threading.Lock()

        self._names_lock = threading.Lock()
        self._names_file = self._resolve_names_file()
        self._motor_names = self._load_names()

        self._positions_lock = threading.Lock()
        self._positions_file = self._resolve_positions_file()
        self._saved_positions, self._dual_presets = self._load_positions()

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_web_loop, daemon=True)
        self._thread.start()

    def _submit_coro(self, coro) -> None:
        if self._loop.is_closed():
            try:
                coro.close()
            except Exception:
                pass
            return
        try:
            asyncio.run_coroutine_threadsafe(coro, self._loop)
        except Exception:
            try:
                coro.close()
            except Exception:
                pass

    def _known_ids(self) -> list[int]:
        return [int(x) for x in list(self.get_parameter('known_ids').value)]

    def _build_live_limits(self) -> dict[int, tuple[int, int]]:
        ids = [int(x) for x in list(self.get_parameter('motor_ids').value)]
        mins = [int(x) for x in list(self.get_parameter('motor_min').value)]
        maxs = [int(x) for x in list(self.get_parameter('motor_max').value)]

        limits: dict[int, tuple[int, int]] = {}
        if len(ids) == len(mins) and len(ids) == len(maxs):
            for idx, motor_id in enumerate(ids):
                lo = int(mins[idx])
                hi = int(maxs[idx])
                if hi > lo:
                    limits[int(motor_id)] = (lo, hi)

        for sid in self._known_ids():
            if sid not in limits:
                limits[sid] = (0, 4095)

        self.get_logger().info(f'Live scaling limits: {limits}')
        return limits

    def _position_to_percent(self, motor_id: int, position: int) -> float:
        lo, hi = self._live_limits.get(int(motor_id), (0, 4095))
        if hi <= lo:
            return 0.0
        ratio = (float(position) - float(lo)) / float(hi - lo)
        return float(max(0.0, min(100.0, ratio * 100.0)))

    def _resolve_names_file(self) -> Path:
        configured = str(self.get_parameter('names_file').value).strip()
        if configured:
            return Path(configured)
        return Path.cwd() / 'data' / 'motor_names.json'

    def _resolve_positions_file(self) -> Path:
        configured = str(self.get_parameter('positions_file').value).strip()
        if configured:
            return Path(configured)
        return Path.cwd() / 'data' / 'motor_positions.json'

    def _default_names(self) -> dict[int, str]:
        return {sid: f'Motor {sid}' for sid in self._known_ids()}

    def _default_positions(self) -> dict[int, list[float]]:
        defaults = [0.0, 12.5, 25.0, 37.5, 50.0, 62.5, 75.0, 87.5, 100.0]
        return {sid: list(defaults) for sid in self._known_ids()}

    @staticmethod
    def _default_dual_presets() -> list[dict]:
        return [
            {'name': f'Pose {idx + 1}', 'p1': 50.0, 'p2': 50.0}
            for idx in range(9)
        ]

    def _load_names(self) -> dict[int, str]:
        defaults = self._default_names()
        path = self._names_file
        try:
            if path.exists():
                raw = json.loads(path.read_text(encoding='utf-8'))
                loaded: dict[int, str] = {}
                for k, v in raw.items():
                    sid = int(k)
                    if sid in defaults:
                        name = str(v).strip()
                        if name:
                            loaded[sid] = name
                return defaults | loaded
        except Exception as exc:
            self.get_logger().warn(f'Failed loading names file {path}: {exc}')
        return defaults

    def _load_positions(self) -> tuple[dict[int, list[float]], list[dict]]:
        defaults_single = self._default_positions()
        defaults_dual = self._default_dual_presets()
        path = self._positions_file
        try:
            if path.exists():
                raw = json.loads(path.read_text(encoding='utf-8'))
                single_source = raw.get('single', raw) if isinstance(raw, dict) else {}
                dual_source = raw.get('dual', {}) if isinstance(raw, dict) else {}

                merged_single = dict(defaults_single)
                for k, arr in single_source.items():
                    sid = int(k)
                    if sid not in defaults_single:
                        continue
                    if not isinstance(arr, list):
                        continue
                    vals: list[float] = []
                    for v in arr[:9]:
                        try:
                            vals.append(float(max(0.0, min(100.0, float(v)))))
                        except Exception:
                            vals.append(50.0)
                    while len(vals) < 9:
                        vals.append(50.0)
                    merged_single[sid] = vals

                merged_dual = list(defaults_dual)
                if isinstance(dual_source, dict):
                    for slot_str, preset in dual_source.items():
                        try:
                            slot = int(slot_str)
                        except Exception:
                            continue
                        if slot < 1 or slot > 9 or not isinstance(preset, dict):
                            continue
                        name = str(preset.get('name', f'Pose {slot}')).strip() or f'Pose {slot}'
                        p1 = float(max(0.0, min(100.0, float(preset.get('p1', 50.0)))))
                        p2 = float(max(0.0, min(100.0, float(preset.get('p2', 50.0)))))
                        merged_dual[slot - 1] = {'name': name[:40], 'p1': p1, 'p2': p2}

                return merged_single, merged_dual
        except Exception as exc:
            self.get_logger().warn(f'Failed loading positions file {path}: {exc}')
        return defaults_single, defaults_dual

    def _save_names(self) -> None:
        path = self._names_file
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._names_lock:
            payload = {str(k): v for k, v in self._motor_names.items()}
        path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')

    def _save_positions(self) -> None:
        path = self._positions_file
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._positions_lock:
            payload = {
                'single': {str(k): v for k, v in self._saved_positions.items()},
                'dual': {
                    str(idx + 1): {
                        'name': str(preset['name']),
                        'p1': float(preset['p1']),
                        'p2': float(preset['p2']),
                    }
                    for idx, preset in enumerate(self._dual_presets)
                },
            }
        path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')

    def _name_for(self, motor_id: int) -> str:
        with self._names_lock:
            return self._motor_names.get(motor_id, f'Motor {motor_id}')

    def _positions_payload(self) -> dict[str, list[float]]:
        with self._positions_lock:
            return {str(k): list(v) for k, v in self._saved_positions.items()}

    def _dual_payload(self) -> dict[str, dict]:
        with self._positions_lock:
            return {
                str(idx + 1): {
                    'name': str(preset['name']),
                    'p1': float(preset['p1']),
                    'p2': float(preset['p2']),
                }
                for idx, preset in enumerate(self._dual_presets)
            }

    def _static_root(self) -> Path:
        try:
            return Path(get_package_share_directory('servo_bridge')) / 'web'
        except Exception:
            return Path(__file__).resolve().parent / 'web'

    async def _index(self, request: web.Request) -> web.StreamResponse:
        return web.FileResponse(self._static_root() / 'index.html')

    async def _app_js(self, request: web.Request) -> web.StreamResponse:
        return web.FileResponse(self._static_root() / 'app.js')

    async def _styles_css(self, request: web.Request) -> web.StreamResponse:
        return web.FileResponse(self._static_root() / 'styles.css')

    async def _ws(self, request: web.Request) -> web.StreamResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        with self._clients_lock:
            self._ws_clients.add(ws)

        try:
            with self._names_lock:
                names_payload = {str(k): v for k, v in self._motor_names.items()}
            await ws.send_str(json.dumps({
                'type': 'hello',
                'message': 'connected',
                'names': names_payload,
                'positions': self._positions_payload(),
                'dual_presets': self._dual_payload(),
                'modes': ['manual', 'saved', 'dual'],
            }))
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    await self._handle_ws_text(msg.data, ws)
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            with self._clients_lock:
                self._ws_clients.discard(ws)
        return ws

    async def _ws_send(self, ws: web.WebSocketResponse, payload: dict) -> None:
        try:
            await ws.send_str(json.dumps(payload))
        except Exception:
            with self._clients_lock:
                self._ws_clients.discard(ws)

    def _schedule_ws_send(self, ws: web.WebSocketResponse, payload: dict) -> None:
        self._submit_coro(self._ws_send(ws, payload))

    async def _handle_ws_text(self, payload: str, ws: web.WebSocketResponse) -> None:
        try:
            data = json.loads(payload)
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'invalid json'})
            return

        msg_type = data.get('type')
        if msg_type == 'set_names':
            await self._handle_ws_set_names(data, ws)
            return
        if msg_type == 'set_name':
            await self._handle_ws_set_name(data, ws)
            return
        if msg_type == 'set_dual_slot':
            await self._handle_ws_set_dual_slot(data, ws)
            return
        if msg_type == 'move_dual':
            await self._handle_ws_move_dual(data, ws)
            return
        if msg_type == 'set_slot':
            await self._handle_ws_set_slot(data, ws)
            return
        if msg_type == 'set':
            # Backward-compatible alias from older UI.
            data = {
                'type': 'move',
                'id': data.get('id'),
                'percent': data.get('percent'),
                'speed': data.get('speed', 0.0),
                'acc': data.get('acc', 0.0),
                'mode': 'manual',
            }
            msg_type = 'move'

        if msg_type != 'move':
            await self._ws_send(ws, {'type': 'error', 'message': 'unknown message type'})
            return

        await self._handle_ws_move(data, ws)

    async def _handle_ws_set_name(self, data: dict, ws: web.WebSocketResponse) -> None:
        try:
            motor_id = int(data['id'])
            name = str(data['name']).strip()
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'bad set_name schema'})
            return

        if not name:
            await self._ws_send(ws, {'type': 'error', 'message': 'name cannot be empty'})
            return

        if len(name) > 40:
            name = name[:40]

        with self._names_lock:
            self._motor_names[motor_id] = name
        try:
            self._save_names()
        except Exception as exc:
            await self._ws_send(ws, {'type': 'error', 'message': f'failed to save name: {exc}'})
            return

        await self._broadcast({'type': 'name', 'id': motor_id, 'name': name})

    async def _handle_ws_set_names(self, data: dict, ws: web.WebSocketResponse) -> None:
        try:
            incoming = dict(data['names'])
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'bad set_names schema'})
            return

        updated: dict[int, str] = {}
        for sid in self._known_ids():
            key = str(sid)
            if key not in incoming:
                await self._ws_send(ws, {'type': 'error', 'message': f'missing name for id={sid}'})
                return
            name = str(incoming[key]).strip()
            if not name:
                await self._ws_send(ws, {'type': 'error', 'message': f'name cannot be empty for id={sid}'})
                return
            if len(name) > 40:
                name = name[:40]
            updated[sid] = name

        with self._names_lock:
            self._motor_names.update(updated)
        try:
            self._save_names()
        except Exception as exc:
            await self._ws_send(ws, {'type': 'error', 'message': f'failed to save names: {exc}'})
            return

        payload = {str(k): v for k, v in updated.items()}
        await self._broadcast({'type': 'names', 'names': payload})

    async def _handle_ws_set_slot(self, data: dict, ws: web.WebSocketResponse) -> None:
        try:
            motor_id = int(data['id'])
            slot = int(data['slot'])
            percent = float(data['percent'])
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'bad set_slot schema'})
            return

        if motor_id not in self._known_ids():
            await self._ws_send(ws, {'type': 'error', 'message': f'unknown id={motor_id}'})
            return
        if slot < 1 or slot > 9:
            await self._ws_send(ws, {'type': 'error', 'message': 'slot must be 1..9'})
            return

        clamped = float(max(0.0, min(100.0, percent)))
        with self._positions_lock:
            if motor_id not in self._saved_positions:
                self._saved_positions[motor_id] = [50.0] * 9
            self._saved_positions[motor_id][slot - 1] = clamped
        try:
            self._save_positions()
        except Exception as exc:
            await self._ws_send(ws, {'type': 'error', 'message': f'failed to save slot: {exc}'})
            return

        await self._broadcast({'type': 'slot', 'id': motor_id, 'slot': slot, 'percent': clamped})

    async def _handle_ws_set_dual_slot(self, data: dict, ws: web.WebSocketResponse) -> None:
        try:
            slot = int(data['slot'])
            name = str(data['name']).strip()
            p1 = float(data['p1'])
            p2 = float(data['p2'])
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'bad set_dual_slot schema'})
            return

        if slot < 1 or slot > 9:
            await self._ws_send(ws, {'type': 'error', 'message': 'slot must be 1..9'})
            return

        if not name:
            name = f'Pose {slot}'
        if len(name) > 40:
            name = name[:40]
        p1 = float(max(0.0, min(100.0, p1)))
        p2 = float(max(0.0, min(100.0, p2)))

        with self._positions_lock:
            self._dual_presets[slot - 1] = {'name': name, 'p1': p1, 'p2': p2}
        try:
            self._save_positions()
        except Exception as exc:
            await self._ws_send(ws, {'type': 'error', 'message': f'failed to save dual slot: {exc}'})
            return

        await self._broadcast({'type': 'dual_slot', 'slot': slot, 'name': name, 'p1': p1, 'p2': p2})

    async def _handle_ws_move_dual(self, data: dict, ws: web.WebSocketResponse) -> None:
        try:
            slot = int(data['slot'])
            speed = int(float(data.get('speed', 0.0)))
            acc = int(float(data.get('acc', 0.0)))
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'bad move_dual schema'})
            return

        if slot < 1 or slot > 9:
            await self._ws_send(ws, {'type': 'error', 'message': 'slot must be 1..9'})
            return

        with self._positions_lock:
            preset = dict(self._dual_presets[slot - 1])

        context = {
            'ws': ws,
            'slot': slot,
            'name': str(preset.get('name', f'Pose {slot}')),
            'p1': float(preset.get('p1', 50.0)),
            'p2': float(preset.get('p2', 50.0)),
            'speed': speed,
            'acc': acc,
            'results': {},
        }
        self._schedule_ws_send(
            ws,
            {
                'type': 'queued_dual',
                'slot': slot,
                'name': context['name'],
                'p1': context['p1'],
                'p2': context['p2'],
            },
        )
        self._send_dual_stage(context, stage=1)

    def _send_dual_stage(self, context: dict, stage: int) -> None:
        if not self._action_client.wait_for_server(timeout_sec=0.4):
            self._schedule_ws_send(context['ws'], {'type': 'error', 'message': 'motor action server unavailable'})
            return

        motor_id = 1 if stage == 1 else 2
        percent = float(context['p1']) if stage == 1 else float(context['p2'])

        goal = MotorMove.Goal()
        goal.id = int(motor_id)
        goal.percent = float(percent)
        goal.speed = int(context['speed'])
        goal.acc = int(context['acc'])
        goal.mode = f'dual:{int(context["slot"])}:{stage}'

        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._on_action_feedback)
        send_future.add_done_callback(lambda fut: self._on_dual_goal_response(fut, context, stage))

    def _on_dual_goal_response(self, future, context: dict, stage: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._schedule_ws_send(context['ws'], {'type': 'error', 'message': f'dual goal send failed: {exc}'})
            return

        if not goal_handle.accepted:
            self._schedule_ws_send(
                context['ws'],
                {'type': 'rejected_dual', 'slot': int(context['slot']), 'stage': stage},
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_dual_goal_result(fut, context, stage))

    def _on_dual_goal_result(self, future, context: dict, stage: int) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            success = bool(result.success)
            payload = {
                'success': success,
                'id': int(result.id),
                'percent': float(result.percent),
                'target_position': int(result.target_position),
                'actual_position': int(result.actual_position),
                'message': str(result.message),
                'status': int(wrapped.status),
            }
        except Exception as exc:
            success = False
            payload = {'success': False, 'message': f'dual stage result error: {exc}'}

        context['results'][stage] = payload
        if not success:
            self._submit_coro(
                self._broadcast(
                    {
                        'type': 'dual_result',
                        'slot': int(context['slot']),
                        'name': context['name'],
                        'success': False,
                        'results': context['results'],
                    }
                )
            )
            return

        if stage == 1:
            self._send_dual_stage(context, stage=2)
            return

        self._submit_coro(
            self._broadcast(
                {
                    'type': 'dual_result',
                    'slot': int(context['slot']),
                    'name': context['name'],
                    'success': True,
                    'results': context['results'],
                }
            )
        )

    async def _handle_ws_move(self, data: dict, ws: web.WebSocketResponse) -> None:
        try:
            motor_id = int(data['id'])
            speed = int(float(data.get('speed', 0.0)))
            acc = int(float(data.get('acc', 0.0)))
            mode = str(data.get('mode', 'manual')).strip().lower() or 'manual'
        except Exception:
            await self._ws_send(ws, {'type': 'error', 'message': 'bad move schema'})
            return

        if mode == 'dual':
            await self._handle_ws_move_dual(data, ws)
            return

        if motor_id not in self._known_ids():
            await self._ws_send(ws, {'type': 'error', 'message': f'unknown id={motor_id}'})
            return

        percent = 0.0
        slot = None
        if mode == 'saved':
            try:
                slot = int(data['slot'])
            except Exception:
                await self._ws_send(ws, {'type': 'error', 'message': 'saved mode requires slot=1..9'})
                return
            if slot < 1 or slot > 9:
                await self._ws_send(ws, {'type': 'error', 'message': 'slot must be 1..9'})
                return
            with self._positions_lock:
                percent = float(self._saved_positions.get(motor_id, [50.0] * 9)[slot - 1])
        else:
            try:
                percent = float(data['percent'])
            except Exception:
                await self._ws_send(ws, {'type': 'error', 'message': 'manual mode requires percent'})
                return
            percent = float(max(0.0, min(100.0, percent)))

        self._send_move_goal(ws, motor_id, percent, speed, acc, mode, slot)

    def _send_move_goal(
        self,
        ws: web.WebSocketResponse,
        motor_id: int,
        percent: float,
        speed: int,
        acc: int,
        mode: str,
        slot: Optional[int],
    ) -> None:
        if not self._action_client.wait_for_server(timeout_sec=0.4):
            self._schedule_ws_send(ws, {'type': 'error', 'message': 'motor action server unavailable'})
            return

        goal = MotorMove.Goal()
        goal.id = int(motor_id)
        goal.percent = float(percent)
        goal.speed = int(speed)
        goal.acc = int(acc)
        goal.mode = str(mode if slot is None else f'{mode}:{slot}')

        send_future = self._action_client.send_goal_async(goal, feedback_callback=self._on_action_feedback)
        send_future.add_done_callback(
            lambda fut: self._on_action_goal_response(fut, ws, motor_id, percent, mode, slot)
        )

        payload = {'type': 'queued', 'id': motor_id, 'percent': percent, 'mode': mode}
        if slot is not None:
            payload['slot'] = slot
        self._schedule_ws_send(ws, payload)

    def _on_action_goal_response(
        self,
        future,
        ws: web.WebSocketResponse,
        motor_id: int,
        percent: float,
        mode: str,
        slot: Optional[int],
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._schedule_ws_send(ws, {'type': 'error', 'message': f'goal send failed: {exc}'})
            return

        if not goal_handle.accepted:
            payload = {'type': 'rejected', 'id': motor_id, 'percent': percent, 'mode': mode}
            if slot is not None:
                payload['slot'] = slot
            self._schedule_ws_send(ws, payload)
            return

        accepted = {'type': 'accepted', 'id': motor_id, 'percent': percent, 'mode': mode}
        if slot is not None:
            accepted['slot'] = slot
        self._schedule_ws_send(ws, accepted)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        payload = {
            'type': 'feedback',
            'id': int(fb.id),
            'percent': float(fb.percent),
            'target_position': int(fb.target_position),
            'stage': str(fb.stage),
            'message': str(fb.message),
        }
        self._submit_coro(self._broadcast(payload))

    def _on_action_result(self, future) -> None:
        try:
            wrapped = future.result()
            res = wrapped.result
            payload = {
                'type': 'result',
                'status': int(wrapped.status),
                'success': bool(res.success),
                'id': int(res.id),
                'percent': float(res.percent),
                'target_position': int(res.target_position),
                'actual_position': int(res.actual_position),
                'mode': str(res.mode),
                'message': str(res.message),
            }
        except Exception as exc:
            payload = {'type': 'result', 'success': False, 'message': f'result error: {exc}'}
        self._submit_coro(self._broadcast(payload))

    async def _broadcast(self, payload: dict) -> None:
        text = json.dumps(payload)
        with self._clients_lock:
            clients = list(self._ws_clients)

        stale = []
        for client in clients:
            try:
                await client.send_str(text)
            except Exception:
                stale.append(client)

        if stale:
            with self._clients_lock:
                for client in stale:
                    self._ws_clients.discard(client)

    def _on_state(self, msg: MotorState) -> None:
        payload = {
            'type': 'state',
            'id': int(msg.id),
            'name': self._name_for(int(msg.id)),
            'percent': float(msg.percent),
            'target_position': int(msg.target_position),
            'actual_position': int(msg.actual_position),
            'success': bool(msg.success),
            'message': str(msg.message),
        }
        self._submit_coro(self._broadcast(payload))

    def _poll_live_positions(self) -> None:
        if self._read_client is None or not self._read_client.service_is_ready():
            return
        for sid in self._known_ids():
            with self._live_poll_lock:
                if sid in self._live_poll_pending:
                    continue
                self._live_poll_pending.add(sid)

            req = ServoRead.Request()
            req.id = int(sid)
            fut = self._read_client.call_async(req)
            fut.add_done_callback(lambda future, servo_id=sid: self._on_live_read_done(servo_id, future))

    def _on_live_read_done(self, servo_id: int, future) -> None:
        with self._live_poll_lock:
            self._live_poll_pending.discard(int(servo_id))

        try:
            res = future.result()
            position = int(res.position) if bool(res.success) else 0
            percent = self._position_to_percent(int(servo_id), position) if bool(res.success) else 0.0
            payload = {
                'type': 'live',
                'id': int(servo_id),
                'position': position,
                'percent': percent,
                'success': bool(res.success),
                'message': str(res.message),
            }
        except Exception as exc:
            payload = {
                'type': 'live',
                'id': int(servo_id),
                'position': 0,
                'percent': 0.0,
                'success': False,
                'message': f'poll failed: {exc}',
            }
        self._submit_coro(self._broadcast(payload))

    def _run_web_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        app = web.Application()
        app.router.add_get('/', self._index)
        app.router.add_get('/app.js', self._app_js)
        app.router.add_get('/styles.css', self._styles_css)
        app.router.add_get('/ws', self._ws)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)

        self.get_logger().info(f'Web UI available at http://{host}:{port}')
        try:
            web.run_app(app, host=host, port=port, loop=self._loop, print=None, handle_signals=False)
        except Exception as exc:
            self.get_logger().error(f'Web loop stopped: {exc}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RosBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
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
