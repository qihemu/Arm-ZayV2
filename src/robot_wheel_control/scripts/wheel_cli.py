#!/usr/bin/env python3
"""Bounded suspended wheel commands. Never writes registers or automatically migrates IDs."""
import argparse
import time
import uuid
import math
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from robot_interfaces.msg import WheelVelocityCommand, WheelMotionHeartbeat
from robot_interfaces.srv import GetWheelBaseState, GetWheelControlResult, SetWheelBaseEnabled, StopWheelBase, ClearWheelBaseFault
from robot_interfaces.srv import MoveWheelBaseRelative


class Client(Node):
    def __init__(self):
        super().__init__('wheel_operator')
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

    def state(self):
        response = self.call(self.state_client, GetWheelBaseState.Request())
        if not response.available:
            raise RuntimeError('No state snapshot')
        return response.state

    def wait_result(self, response):
        if not response.accepted:
            raise RuntimeError(response.reason)
        deadline = time.monotonic() + 6
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
        timer = self.create_timer(0.05, refresh)
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
            deadline = time.monotonic() + timeout + 10
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'enable', 'stop', 'clear', 'forward', 'backward', 'left', 'right', 'move', 'turn', 'wheel-angle'])
    parser.add_argument('--speed', type=float, default=0.1, help='logical wheel rad/s, >0 and <=0.2')
    parser.add_argument('--duration', type=float, default=1.0, help='bounded motion seconds, >0 and <=3')
    parser.add_argument('--linear-speed', type=float, default=0.005, help='base mode linear m/s, (0,0.01]')
    parser.add_argument('--angular-speed', type=float, default=0.02, help='base mode yaw rad/s, (0,0.05]')
    parser.add_argument('--enable', action='store_true', help='explicitly request enable before this motion')
    parser.add_argument('--distance', type=float, help='move: signed relative distance in metres')
    parser.add_argument('--angle', type=float, help='turn: signed chassis degrees; wheel-angle: signed logical wheel degrees')
    parser.add_argument('--timeout', type=float, default=120.0, help='relative motion deadline seconds (not commanded travel duration)')
    args = parser.parse_args()
    if not 0 < args.speed <= 0.2 or not 0 < args.duration <= 3:
        parser.error('speed must be (0,0.2], duration (0,3]')
    if not 0 < args.linear_speed <= 0.01 or not 0 < args.angular_speed <= 0.05:
        parser.error('bounded base CLI allows linear <=0.01m/s, angular <=0.05rad/s')
    if args.command in ('move', 'turn', 'wheel-angle'):
        value = args.distance if args.command == 'move' else args.angle
        if value is None or not math.isfinite(value) or value == 0 or not args.enable:
            parser.error('relative motion requires nonzero --distance/--angle and explicit --enable')
        if not math.isfinite(args.timeout) or not 0 < args.timeout <= 300:
            parser.error('relative --timeout must be (0,300] seconds')
    rclpy.init()
    # Ctrl+C先抛出KeyboardInterrupt，让finally可在ROS上下文仍有效时请求停车。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    client = Client()
    try:
        if args.command in ('move', 'turn', 'wheel-angle'):
            kind = {'move': 1, 'turn': 2, 'wheel-angle': 3}[args.command]
            value = args.distance if kind == 1 else math.radians(args.angle)
            speed = args.linear_speed if kind == 1 else (args.angular_speed if kind == 2 else args.speed)
            client.move_relative(kind, value, speed, args.timeout, args.enable)
        elif args.command == 'status':
            print(client.state())
        elif args.command == 'enable':
            client.enable()
        elif args.command == 'stop':
            client.stop()
        elif args.command == 'clear':
            state = client.state()
            request = ClearWheelBaseFault.Request(session_id=state.session_id, request_id=str(uuid.uuid4()), expected_fault_sequence=state.fault_sequence)
            print(client.wait_result(client.call(client.clear_client, request)))
        else:
            if args.enable:
                client.enable()
            state = client.state()
            if not state.motion_authorized:
                raise RuntimeError('Explicit enable required; add --enable or run enable first')
            signs = {'forward': (1, 1), 'backward': (-1, -1), 'left': (-1, 1), 'right': (1, -1)}[args.command]
            if state.operation_mode == 'bench':
                message = WheelVelocityCommand(session_id=state.session_id, left_rad_s=signs[0]*args.speed, right_rad_s=signs[1]*args.speed)
                publisher = client.publisher
            elif state.operation_mode == 'base':
                message = TwistStamped()
                message.twist.linear.x = {'forward': args.linear_speed, 'backward': -args.linear_speed}.get(args.command, 0.0)
                message.twist.angular.z = {'left': args.angular_speed, 'right': -args.angular_speed}.get(args.command, 0.0)
                publisher = client.twist_publisher
            else:
                raise RuntimeError('Unsupported operation mode')
            print(f'backend={state.backend}, mode={state.operation_mode}, command={args.command}', flush=True)
            deadline = time.monotonic() + args.duration
            # 新时间戳20Hz；结束或Ctrl+C均请求停止。节点被kill时由命令期限兜底。
            try:
                while time.monotonic() < deadline:
                    message.header.stamp = client.get_clock().now().to_msg()
                    publisher.publish(message)
                    rclpy.spin_once(client, timeout_sec=0.01)
                    time.sleep(0.04)
                observed = client.state()
                if observed.lifecycle_state == observed.FAULT or not observed.command_fresh:
                    raise RuntimeError('Motion not verified: ' + observed.reason)
            finally:
                client.stop()
    except (RuntimeError, KeyboardInterrupt) as error:
        print('Operation stopped:', error, flush=True)
        raise SystemExit(1)
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
