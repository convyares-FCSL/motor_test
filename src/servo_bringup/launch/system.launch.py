#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def _load_motor_limits(config_path: str) -> tuple[list[int], list[int], list[int]]:
    default_ids = [1, 2]
    default_min = [0, 0]
    default_max = [4095, 4095]

    try:
        import yaml

        with open(config_path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}
        params = (raw.get('servo_motor') or {}).get('ros__parameters') or {}

        ids = [int(x) for x in list(params.get('motor_ids', default_ids))]
        mins = [int(x) for x in list(params.get('motor_min', default_min))]
        maxs = [int(x) for x in list(params.get('motor_max', default_max))]
        if len(ids) == len(mins) and len(ids) == len(maxs):
            all_valid = all(hi > lo for lo, hi in zip(mins, maxs))
            if all_valid and len(ids) > 0:
                return ids, mins, maxs
    except Exception:
        pass

    return default_ids, default_min, default_max


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('servo_bringup')
    motor_config = os.path.join(bringup_share, 'config', 'motors.yaml')
    known_ids, motor_min, motor_max = _load_motor_limits(motor_config)

    serial_port = os.getenv('STS_PORT', 'COM13')
    serial_baud = int(os.getenv('STS_USB_BAUD', '115200'))
    library_path = os.getenv('STS_LIBRARY_PATH', '')
    bridge_host = os.getenv('ROS_BRIDGE_HOST', '0.0.0.0')
    bridge_port = int(os.getenv('ROS_BRIDGE_PORT', '8080'))
    bridge_names_file = os.getenv('ROS_BRIDGE_NAMES_FILE', '')
    bridge_positions_file = os.getenv('ROS_BRIDGE_POSITIONS_FILE', '')
    bridge_live_poll_s = float(os.getenv('ROS_BRIDGE_LIVE_POLL_S', '1.0'))
    motor_type = os.getenv('SERVO_MOTOR_TYPE', 'waveshare_st3215').strip().lower()
    simulation_enabled = os.getenv('STS_SIMULATION', '0').strip().lower() in ('1', 'true', 'yes', 'on')
    simulation_default_position = int(os.getenv('STS_SIM_DEFAULT_POSITION', '2048'))
    windows_proxy_enabled = os.getenv('STS_WINDOWS_PROXY', '1').strip().lower() in ('1', 'true', 'yes', 'on')
    windows_proxy_timeout = float(os.getenv('STS_WINDOWS_PROXY_TIMEOUT_S', '8.0'))
    windows_proxy_python = os.getenv('STS_WINDOWS_PROXY_PYTHON', 'py -3')
    windows_proxy_script = os.getenv('STS_WINDOWS_PROXY_SCRIPT', '')
    wait_for_ids = os.getenv('STS_WAIT_FOR_IDS', '1').strip().lower() in ('1', 'true', 'yes', 'on')
    require_all_ids = os.getenv('STS_REQUIRE_ALL_IDS', '1').strip().lower() in ('1', 'true', 'yes', 'on')
    wait_timeout = float(os.getenv('STS_WAIT_IDS_TIMEOUT_S', '8.0'))
    retry_period = float(os.getenv('STS_WAIT_IDS_RETRY_S', '0.4'))
    startup_settle = float(os.getenv('STS_STARTUP_SETTLE_S', '0.4'))
    recover_cycle_delay = float(os.getenv('STS_RECOVER_CYCLE_DELAY_S', '0.25'))

    serial_node = LifecycleNode(
        package='servo_serial',
        executable='serial_node',
        namespace='',
        name='servo_serial',
        output='screen',
        parameters=[{
            'port': serial_port,
            'baud': serial_baud,
            'library_path': library_path,
            'known_ids': known_ids,
            'auto_torque_enable': True,
            'command_settle_s': 0.2,
            'wait_for_known_ids': wait_for_ids,
            'require_all_known_ids': require_all_ids,
            'known_ids_wait_timeout_s': wait_timeout,
            'known_ids_retry_period_s': retry_period,
            'startup_settle_s': startup_settle,
            'simulation_enabled': simulation_enabled,
            'simulation_default_position': simulation_default_position,
            'simulation_command_delay_s': 0.02,
            'recover_torque_cycle_delay_s': recover_cycle_delay,
            'windows_proxy_enabled': windows_proxy_enabled,
            'windows_proxy_timeout_s': windows_proxy_timeout,
            'windows_proxy_python': windows_proxy_python,
            'windows_proxy_script': windows_proxy_script,
        }],
    )

    motor_node = LifecycleNode(
        package='servo_motor',
        executable='motor_node',
        namespace='',
        name='servo_motor',
        output='screen',
        parameters=[motor_config],
    )

    bridge_node = Node(
        package='servo_bridge',
        executable='ros_bridge_node',
        name='ros_bridge',
        output='screen',
        parameters=[{
            'host': bridge_host,
            'port': bridge_port,
            'state_topic': 'servo/state',
            'motor_action_name': 'servo_motor/move',
            'serial_read_service': 'servo_serial/read',
            'serial_control_service': 'servo_serial/control',
            'live_poll_period_s': bridge_live_poll_s,
            'known_ids': known_ids,
            'motor_ids': known_ids,
            'motor_min': motor_min,
            'motor_max': motor_max,
            'motor_type': motor_type,
            'names_file': bridge_names_file,
            'positions_file': bridge_positions_file,
        }],
    )

    configure_serial = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(serial_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_serial = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(serial_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    configure_motor = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(motor_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_motor = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(motor_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription([
        serial_node,
        motor_node,
        bridge_node,
        TimerAction(period=1.0, actions=[configure_serial]),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=serial_node,
                goal_state='inactive',
                entities=[activate_serial],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=serial_node,
                goal_state='active',
                entities=[configure_motor],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=motor_node,
                goal_state='inactive',
                entities=[activate_motor],
            )
        ),
    ])
