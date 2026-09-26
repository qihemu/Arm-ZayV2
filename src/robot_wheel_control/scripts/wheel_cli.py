#!/usr/bin/env python3
"""Bounded suspended wheel commands. Never writes registers or automatically migrates IDs."""
import argparse
import time
import uuid
import math
import signal
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robot_interfaces.msg import WheelVelocityCommand, WheelMotionHeartbeat
from robot_interfaces.srv import GetWheelBaseState, GetWheelControlResult, SetWheelBaseEnabled, StopWheelBase, ClearWheelBaseFault
from robot_interfaces.srv import MoveWheelBaseRelative, GetWheelConfiguration


class Client(Node):
    def __init__(self):
        super().__init__('wheel_operator')
        self.configuration_client = self.create_client(GetWheelConfiguration, '/base/get_configuration')
        self.config = None
        self.config_session = None
        self.state_client = self.create_client(GetWheelBaseState, '/base/get_state')
        self.result_client = self.create_client(GetWheelControlResult, '/base/get_control_result')
        self.enable_client = self.create_client(SetWheelBaseEnabled, '/base/set_enabled')
        self.stop_client = self.create_client(StopWheelBase, '/base/stop')
        self.clear_client = self.create_client(ClearWheelBaseFault, '/base/clear_fault')
        self.publisher = self.create_publisher(WheelVelocityCommand, '/base/wheel_velocity', 1)
        self.twist_publisher = self.create_publisher(TwistStamped, '/base/cmd_vel', 1)
        self.relative_client = self.create_client(MoveWheelBaseRelative, '/base/move_relative')
        self.heartbeat_publisher = self.create_publisher(WheelMotionHeartbeat, '/base/relative_keepalive', 1)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('Service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Service response timeout')
        return future.result()

    def configuration(self):
        response = self.call(self.configuration_client, GetWheelConfiguration.Request())
        self.config = yaml.safe_load(response.configuration_yaml)['robot_wheel_control']
        self.config_session = response.session_id
        print(f'configuration={response.configuration_digest}, source={response.source_path}', flush=True)
        return response

    def state(self):
        response = self.call(self.state_client, GetWheelBaseState.Request())
        if not response.available:
            raise RuntimeError('No state snapshot')
        return response.state

    def wait_result(self, response):
        if not response.accepted:
            raise RuntimeError(response.reason)
        deadline = time.monotonic() + self.config['operator']['management_wait_s']
        request = GetWheelControlResult.Request(session_id=response.session_id, request_id=response.request_id)
        while time.monotonic() < deadline:
            result = self.call(self.result_client, request)
            if result.status == result.SUCCEEDED:
                return result.reason
            if result.status in (result.FAILED, result.CANCELLED):
                raise RuntimeError(result.reason)
            time.sleep(0.03)
        raise RuntimeError('Operation completion timeout; query state before retrying')

    def enable(self):
        state = self.state()
        request = SetWheelBaseEnabled.Request(session_id=state.session_id, request_id=str(uuid.uuid4()), enable=True)
        print(self.wait_result(self.call(self.enable_client, request)), flush=True)

    def stop(self):
        request = StopWheelBase.Request(request_id=str(uuid.uuid4()), disable_after_stop=True)
        print(self.wait_result(self.call(self.stop_client, request)), flush=True)

    def move_relative(self, kind, value, speed, timeout, enable):
        state = self.state()
        print(f'backend={state.backend}, mode={state.operation_mode}, relative_kind={kind}', flush=True)
        if state.operation_mode == 'bench' and kind != 3:
            print('Suspended bench: encoder-equivalent distance/yaw only; chassis travel is not measured.', flush=True)
        request = MoveWheelBaseRelative.Request(session_id=state.session_id, request_id=str(uuid.uuid4()),
            kind=kind, value=value, max_speed=speed, timeout_s=timeout, enable=enable)
        heartbeat = WheelMotionHeartbeat(session_id=state.session_id, request_id=request.request_id)
        def refresh():
            heartbeat.header.stamp = self.get_clock().now().to_msg()
            self.heartbeat_publisher.publish(heartbeat)
        timer = self.create_timer(1 / self.config['operator']['command_publish_rate_hz'], refresh)
        accepted = False
        uncertain = True
        try:
            response = self.call(self.relative_client, request)
            uncertain = False
            if not response.accepted:
                raise RuntimeError(response.reason)
            accepted = True
            print('Relative task accepted: ' + request.request_id, flush=True)
            query = GetWheelControlResult.Request(session_id=state.session_id, request_id=request.request_id)
            # Server computes a finite deadline for timeout=0; never impose a second distance cap.
            deadline = time.monotonic() + timeout + self.config['operator']['management_wait_s'] if timeout else math.inf
            next_print = 0
            while time.monotonic() < deadline:
                result = self.call(self.result_client, query)
                if result.status == result.SUCCEEDED:
                    accepted = False
                    print(result.reason, flush=True)
                    return
                if result.status in (result.FAILED, result.CANCELLED):
                    accepted = False  # 服务器已执行停止；不覆盖任务失败原因。
                    raise RuntimeError(result.reason)
                if time.monotonic() >= next_print:
                    s = self.state()
                    if not timeout and s.relative_timeout_s > 0 and math.isinf(deadline):
                        deadline = time.monotonic() + s.relative_timeout_s + self.config['operator']['management_wait_s']
                    if s.session_id != state.session_id:
                        raise RuntimeError('Session changed during relative task')
                    print(f'target={s.relative_target:.5f}, measured={s.relative_measured:.5f}, '
                          f'error={s.relative_error:.5f} ' + ('m' if kind == 1 else 'rad'), flush=True)
                    next_print = time.monotonic() + 1
                rclpy.spin_once(self, timeout_sec=0.04)
            raise RuntimeError('Relative task result timeout')
        finally:
            self.destroy_timer(timer)
            # 请求结果未知或用户中断时仍尝试停止；进程被强杀则由服务器心跳期限停车。
            if accepted or uncertain:
                self.stop()


def positive(value, name):
    if value is None or not math.isfinite(value) or value <= 0:
        raise RuntimeError(name + ' must be finite and positive')
    return value


def main():
    parser = argparse.ArgumentParser(description='H55 commands use the running node YAML configuration')
    parser.add_argument('command', choices=['status', 'configuration', 'enable', 'stop', 'clear',
                        'forward', 'backward', 'left', 'right', 'move', 'turn', 'wheel-angle'])
    parser.add_argument('--speed', type=float, help='wheel rad/s; limit from running node')
    duration = parser.add_mutually_exclusive_group()
    duration.add_argument('--duration', type=float, help='seconds; default/limit from YAML')
    duration.add_argument('--continuous', action='store_true', help='until stop, interruption or action guard')
    parser.add_argument('--linear-speed', type=float, help='body m/s')
    parser.add_argument('--angular-speed', type=float, help='body rad/s')
    parser.add_argument('--enable', action='store_true')
    parser.add_argument('--distance', type=float, help='signed metres')
    parser.add_argument('--angle', type=float, help='signed degrees')
    parser.add_argument('--timeout', type=float, help='relative deadline seconds; omitted = server computes')
    args = parser.parse_args()
    rclpy.init()
    signal.signal(signal.SIGINT, signal.default_int_handler)
    client = Client()
    try:
        response = client.configuration()
        c, op = client.config, client.config['operator']
        if args.command == 'configuration':
            print(response.configuration_yaml)
            return
        if args.command == 'status':
            print(client.state())
            return
        if args.command == 'stop':
            client.stop()
            return
        if args.command == 'clear':
            state = client.state()
            request = ClearWheelBaseFault.Request(session_id=state.session_id, request_id=str(uuid.uuid4()),
                                                 expected_fault_sequence=state.fault_sequence)
            print(client.wait_result(client.call(client.clear_client, request)))
            return
        if args.command == 'enable':
            client.enable()
            return
        if client.state().session_id != response.session_id:
            raise RuntimeError('Session changed; reload configuration')
        speed = positive(args.speed if args.speed is not None else min(op['default_wheel_speed_rad_s'],
                         c['limits']['max_wheel_speed_rad_s']), 'wheel speed')
        linear = positive(args.linear_speed if args.linear_speed is not None else op['default_linear_speed_m_s'], 'linear speed')
        angular = positive(args.angular_speed if args.angular_speed is not None else op['default_angular_speed_rad_s'], 'angular speed')
        # Explicit excessive values are rejected, not silently accepted as a different command.
        for value, limit, label in ((speed, c['limits']['max_wheel_speed_rad_s'], 'wheel'),
                                    (linear, c['limits']['max_linear_speed_m_s'], 'linear'),
                                    (angular, c['limits']['max_angular_speed_rad_s'], 'angular')):
            if limit is not None and value > limit:
                raise RuntimeError(label + ' speed exceeds effective YAML limit')
        if args.command in ('move', 'turn', 'wheel-angle'):
            if args.continuous or args.duration is not None:
                raise RuntimeError('Relative targets use --timeout, not --duration/--continuous')
            kind = {'move': 1, 'turn': 2, 'wheel-angle': 3}[args.command]
            value = args.distance if kind == 1 else args.angle
            if value is None or not math.isfinite(value) or value == 0 or not args.enable:
                raise RuntimeError('Relative motion needs nonzero --distance/--angle and --enable')
            value = value if kind == 1 else math.radians(value)
            timeout = positive(args.timeout, 'timeout') if args.timeout is not None else 0.0
            client.move_relative(kind, value, {1: linear, 2: angular, 3: speed}[kind], timeout, args.enable)
            return
        if args.timeout is not None or args.distance is not None or args.angle is not None:
            raise RuntimeError('Direction commands accept --duration/--continuous, not relative arguments')
        seconds = positive(args.duration if args.duration is not None else op['default_duration_s'], 'duration')
        if not args.continuous and op['max_duration_s'] is not None and seconds > op['max_duration_s']:
            raise RuntimeError('Duration exceeds YAML limit')
        mode = c['operation_mode']
        signs = {'forward': (1, 1), 'backward': (-1, -1), 'left': (-1, 1), 'right': (1, -1)}[args.command]
        if args.speed is not None and (args.linear_speed is not None or args.angular_speed is not None):
            raise RuntimeError('Choose wheel --speed or body speed, not both')
        if mode == 'base':
            if args.speed is not None:
                raise RuntimeError('base controller takes --linear-speed/--angular-speed')
            message = TwistStamped()
            message.twist.linear.x = {'forward': linear, 'backward': -linear}.get(args.command, 0.0)
            message.twist.angular.z = {'left': angular, 'right': -angular}.get(args.command, 0.0)
            publisher = client.twist_publisher
        else:
            wheels = [speed * sign for sign in signs]
            if args.linear_speed is not None or args.angular_speed is not None:
                if args.command in ('forward', 'backward'):
                    wheels = [signs[i] * linear / c['wheels'][side]['effective_radius_m']
                              for i, side in enumerate(('left', 'right'))]
                else:
                    wheels = [signs[i] * angular * c['geometry']['wheel_separation_m'] / 2 /
                              c['wheels'][side]['effective_radius_m'] for i, side in enumerate(('left', 'right'))]
            if max(map(abs, wheels)) > c['limits']['max_wheel_speed_rad_s']:
                raise RuntimeError('Requested body speed exceeds effective wheel limit')
            message = WheelVelocityCommand(session_id=response.session_id, source_id=str(uuid.uuid4()), left_rad_s=wheels[0], right_rad_s=wheels[1])
            publisher = client.publisher
        # The publish timer continues during status service waits; loss of this process stops refresh.
        if args.enable:
            client.enable()
        state = client.state()
        if state.session_id != response.session_id or not state.motion_authorized:
            raise RuntimeError('Fresh session and explicit enable required')
        def publish():
            message.header.stamp = client.get_clock().now().to_msg()
            publisher.publish(message)
        timer = client.create_timer(1 / op['command_publish_rate_hz'], publish)
        try:
            publish()
            deadline = math.inf if args.continuous else time.monotonic() + seconds
            next_status = time.monotonic()
            while time.monotonic() < deadline:
                rclpy.spin_once(client, timeout_sec=1 / op['command_publish_rate_hz'])
                if time.monotonic() >= next_status:
                    state = client.state()
                    if state.session_id != response.session_id or state.lifecycle_state == state.FAULT:
                        raise RuntimeError(state.reason)
                    if not state.motion_authorized:
                        print('Server stopped motion: ' + state.reason, flush=True)
                        break
                    next_status = time.monotonic() + 0.25
        finally:
            client.destroy_timer(timer)
            client.stop()
    except (RuntimeError, KeyboardInterrupt) as error:
        print('Operation stopped: ' + str(error), flush=True)
        raise SystemExit(1)
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
