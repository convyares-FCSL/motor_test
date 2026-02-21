#!/usr/bin/env python3
import time
from dataclasses import dataclass
from threading import Lock
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from servo_interfaces.action import MotorMove
from servo_interfaces.msg import MotorState
from servo_interfaces.srv import ServoCommand


@dataclass
class MotorLimit:
    min_pos: int
    max_pos: int


class MotorNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('servo_motor')

        self.declare_parameter('motor_ids', [1, 2])
        self.declare_parameter('motor_min', [1800, 1800])
        self.declare_parameter('motor_max', [2800, 2800])
        self.declare_parameter('default_speed', 120)
        self.declare_parameter('default_acc', 10)
        self.declare_parameter('command_topic', 'servo/command_percent')
        self.declare_parameter('state_topic', 'servo/state')
        self.declare_parameter('serial_command_service', 'servo_serial/command')
        self.declare_parameter('move_action_name', 'servo_motor/move')

        self._limits: dict[int, MotorLimit] = {}
        self._goal_lock = Lock()
        self._goal_in_progress = False

        self._pub_state = None
        self._client_command = None
        self._action_server = None

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(value, max_value))

    def _rebuild_limits(self) -> bool:
        ids = list(self.get_parameter('motor_ids').value)
        mins = list(self.get_parameter('motor_min').value)
        maxs = list(self.get_parameter('motor_max').value)

        if len(ids) != len(mins) or len(ids) != len(maxs):
            self.get_logger().error('motor_ids, motor_min, motor_max must have equal lengths')
            return False

        rebuilt: dict[int, MotorLimit] = {}
        for idx, motor_id in enumerate(ids):
            min_pos = int(mins[idx])
            max_pos = int(maxs[idx])
            if min_pos >= max_pos:
                self.get_logger().error(f'Invalid limits for id={motor_id}: min >= max')
                return False
            rebuilt[int(motor_id)] = MotorLimit(min_pos=min_pos, max_pos=max_pos)

        self._limits = rebuilt
        self.get_logger().info(f'Loaded limits: {self._limits}')
        return True

    def _percent_to_position(self, motor_id: int, percent: float) -> int:
        limit = self._limits[motor_id]
        p = self._clamp(percent, 0.0, 100.0)
        span = limit.max_pos - limit.min_pos
        return int(round(limit.min_pos + (span * p / 100.0)))

    def _publish_state(
        self,
        motor_id: int,
        percent: float,
        target_position: int,
        success: bool,
        actual_position: int,
        message: str,
    ) -> None:
        if self._pub_state is None:
            return
        state = MotorState()
        state.id = int(motor_id)
        state.percent = float(percent)
        state.target_position = int(target_position)
        state.actual_position = int(actual_position)
        state.success = bool(success)
        state.message = str(message)
        self._pub_state.publish(state)

    @staticmethod
    def _feedback(
        goal_handle,
        motor_id: int,
        percent: float,
        target_position: int,
        stage: str,
        message: str,
    ) -> None:
        fb = MotorMove.Feedback()
        fb.id = int(motor_id)
        fb.percent = float(percent)
        fb.target_position = int(target_position)
        fb.stage = str(stage)
        fb.message = str(message)
        goal_handle.publish_feedback(fb)

    def _goal_callback(self, goal_request: MotorMove.Goal) -> GoalResponse:
        motor_id = int(goal_request.id)
        if motor_id not in self._limits:
            self.get_logger().warn(f'Rejecting move goal: unknown id={motor_id}')
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._goal_in_progress:
                self.get_logger().warn(f'Rejecting move goal for id={motor_id}: another axis is active')
                return GoalResponse.REJECT
            self._goal_in_progress = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> MotorMove.Result:
        goal = goal_handle.request

        motor_id = int(goal.id)
        percent = self._clamp(float(goal.percent), 0.0, 100.0)
        speed = int(goal.speed) if int(goal.speed) > 0 else int(self.get_parameter('default_speed').value)
        acc = int(goal.acc) if int(goal.acc) > 0 else int(self.get_parameter('default_acc').value)
        mode = str(goal.mode).strip() or 'manual'
        target = self._percent_to_position(motor_id, percent)

        result = MotorMove.Result()
        result.id = motor_id
        result.percent = float(percent)
        result.target_position = int(target)
        result.mode = mode
        result.actual_position = 0
        result.success = False
        result.message = 'not executed'

        try:
            if self._client_command is None or not self._client_command.service_is_ready():
                result.message = 'serial command service unavailable'
                self.get_logger().error(result.message)
                self._publish_state(motor_id, percent, target, False, 0, result.message)
                goal_handle.abort()
                return result

            self.get_logger().info(
                f'Executing action goal id={motor_id} mode={mode} percent={percent:.2f} '
                f'target={target} speed={speed} acc={acc}'
            )
            self._feedback(goal_handle, motor_id, percent, target, 'dispatch', 'sending command')

            request = ServoCommand.Request()
            request.id = motor_id
            request.position = target
            request.speed = speed
            request.acc = acc

            future = self._client_command.call_async(request)
            while rclpy.ok() and not future.done():
                if goal_handle.is_cancel_requested:
                    result.message = 'cancel requested'
                    self._feedback(goal_handle, motor_id, percent, target, 'cancel', result.message)
                    self._publish_state(motor_id, percent, target, False, 0, result.message)
                    goal_handle.canceled()
                    return result
                time.sleep(0.01)

            if not future.done():
                result.message = 'service call timed out'
                self._publish_state(motor_id, percent, target, False, 0, result.message)
                goal_handle.abort()
                return result

            try:
                response = future.result()
            except Exception as exc:
                result.message = f'service call failed: {exc}'
                self._publish_state(motor_id, percent, target, False, 0, result.message)
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                result.message = 'cancel requested'
                self._feedback(goal_handle, motor_id, percent, target, 'cancel', result.message)
                self._publish_state(motor_id, percent, target, False, 0, result.message)
                goal_handle.canceled()
                return result

            result.success = bool(response.success)
            result.actual_position = int(response.present_position)
            result.message = str(response.message)

            self._feedback(goal_handle, motor_id, percent, target, 'result', result.message)
            self._publish_state(
                motor_id,
                percent,
                target,
                result.success,
                result.actual_position,
                result.message,
            )

            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            with self._goal_lock:
                self._goal_in_progress = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if not self._rebuild_limits():
            return TransitionCallbackReturn.FAILURE

        service_name = str(self.get_parameter('serial_command_service').value)
        self._client_command = self.create_client(ServoCommand, service_name)
        self.get_logger().info(f'Waiting for {service_name} ...')
        if not self._client_command.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service unavailable: {service_name}')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        topic_out = str(self.get_parameter('state_topic').value)
        action_name = str(self.get_parameter('move_action_name').value)

        self._pub_state = self.create_publisher(MotorState, topic_out, 10)
        self._action_server = ActionServer(
            self,
            MotorMove,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(f'servo_motor active (action={action_name})')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._pub_state is not None:
            self.destroy_publisher(self._pub_state)
            self._pub_state = None

        with self._goal_lock:
            self._goal_in_progress = False

        self.get_logger().info('servo_motor deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._client_command is not None:
            self.destroy_client(self._client_command)
            self._client_command = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotorNode()
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
