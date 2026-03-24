#!/usr/bin/env python3
"""
patrol_waypoint_node.py
-----------------------
기능:
  1. YAML 파일에 정의된 웨이포인트를 순서대로 순찰 (loop 옵션 지원)
  2. LaserScan 기반 전방 장애물 감지
  3. 장애물 발견 시 정지 + Nav 목표 취소
  4. 장애물 해소 시 같은 웨이포인트로 재출발
  5. timer 기반 상태머신으로 안정적 동작

파라미터:
  waypoint_file              (str)   : YAML 웨이포인트 파일 경로
  frame_id                   (str)   : 좌표 프레임 (기본값: map)
  obstacle_dist              (float) : 장애물 감지 거리 [m]
  front_obstacle_angle_deg   (float) : 전방 장애물 검사 각도 범위 (정면 기준 ±각도)
  obstacle_resume_delay      (float) : 장애물 해소 후 재출발 전 대기 시간 [s]
  tick_hz                    (float) : 메인 상태머신 주기 [Hz]
"""

import math
import time
from enum import Enum, auto
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PatrolState(Enum):
    IDLE = auto()
    SEND_GOAL = auto()
    WAIT_RESULT = auto()
    WAIT_OBSTACLE_CLEAR = auto()
    FINISHED = auto()


def yaw_to_quaternion(z_yaw: float):
    return math.sin(z_yaw / 2.0), math.cos(z_yaw / 2.0)


def normalize_angle_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


class PatrolWaypointNode(Node):
    def __init__(self):
        super().__init__('patrol_waypoint_node')

        # 파라미터
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('obstacle_dist', 0.5)
        self.declare_parameter('front_obstacle_angle_deg', 30.0)
        self.declare_parameter('obstacle_resume_delay', 0.5)
        self.declare_parameter('tick_hz', 10.0)

        self.waypoint_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.obstacle_dist = self.get_parameter('obstacle_dist').get_parameter_value().double_value
        self.front_obstacle_angle_deg = (
            self.get_parameter('front_obstacle_angle_deg').get_parameter_value().double_value
        )
        self.obstacle_resume_delay = (
            self.get_parameter('obstacle_resume_delay').get_parameter_value().double_value
        )
        self.tick_hz = self.get_parameter('tick_hz').get_parameter_value().double_value

        if not self.waypoint_file:
            raise RuntimeError('waypoint_file parameter is empty')
        if self.tick_hz <= 0.0:
            raise RuntimeError('tick_hz must be > 0')

        # 내부 상태
        self.state = PatrolState.IDLE
        self.running = True
        self.obstacle_stop = False
        self.last_front_min_dist = float('inf')
        self.last_obstacle_clear_time = None
        self.current_waypoint_index = 0
        self.goal_sent = False

        # ROS 인터페이스
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)

        # Nav2
        self.navigator = BasicNavigator()
        self.waypoints_data, self.loop = self._load_waypoints(self.waypoint_file)

        self.get_logger().info('Nav2 활성화 대기 중...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화 완료.')

        if len(self.waypoints_data) == 0:
            self.get_logger().warn('웨이포인트가 비어 있음. 종료 상태로 진입')
            self.state = PatrolState.FINISHED
        else:
            self.state = PatrolState.SEND_GOAL

        self.timer = self.create_timer(1.0 / self.tick_hz, self._tick)

    def _load_waypoints(self, yaml_file: str):
        path = Path(yaml_file)
        if not path.exists():
            raise FileNotFoundError(f'웨이포인트 파일 없음: {yaml_file}')

        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if 'waypoints' not in data or not isinstance(data['waypoints'], list):
            raise RuntimeError('YAML에 "waypoints" 리스트가 없습니다')

        loop = bool(data.get('loop', False))

        cleaned = []
        for i, wp in enumerate(data['waypoints']):
            if 'x' not in wp or 'y' not in wp:
                raise RuntimeError(f'waypoints[{i}]에 x 또는 y가 없습니다')

            cleaned.append({
                'x': float(wp['x']),
                'y': float(wp['y']),
                'yaw': float(wp.get('yaw', 0.0)),
                'wait_sec': float(wp.get('wait_sec', 0.0)),
            })

        return cleaned, loop

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _scan_callback(self, msg: LaserScan):
        valid_front_ranges = []

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = normalize_angle_deg(math.degrees(angle_rad))

            if -self.front_obstacle_angle_deg <= angle_deg <= self.front_obstacle_angle_deg:
                valid_front_ranges.append(r)

        if not valid_front_ranges:
            return

        front_min = min(valid_front_ranges)
        detected = front_min < self.obstacle_dist
        prev = self.obstacle_stop

        self.obstacle_stop = detected
        self.last_front_min_dist = front_min

        if detected and not prev:
            self.get_logger().warn(
                f'[전방 장애물 감지] front_min={front_min:.2f} m < obstacle_dist={self.obstacle_dist:.2f} m'
            )
            self._emergency_stop()

        elif not detected and prev:
            self.last_obstacle_clear_time = time.time()
            self.get_logger().info(
                f'[장애물 해소] front_min={front_min:.2f} m, 경로 재개 준비'
            )

    def _emergency_stop(self):
        self.cmd_vel_pub.publish(Twist())

        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().warn(f'cancelTask 예외 무시: {e}')

    def _tick(self):
        if not self.running or not rclpy.ok():
            self._shutdown_once()
            return

        if self.state == PatrolState.FINISHED:
            return

        if self.obstacle_stop and self.state == PatrolState.WAIT_RESULT:
            self.state = PatrolState.WAIT_OBSTACLE_CLEAR
            return

        if self.state == PatrolState.IDLE:
            self.state = PatrolState.SEND_GOAL

        elif self.state == PatrolState.SEND_GOAL:
            self._send_current_goal()

        elif self.state == PatrolState.WAIT_RESULT:
            self._process_navigation_result()

        elif self.state == PatrolState.WAIT_OBSTACLE_CLEAR:
            self._process_obstacle_wait()

    def _send_current_goal(self):
        if self.current_waypoint_index >= len(self.waypoints_data):
            self._advance_after_full_cycle()
            return

        wp = self.waypoints_data[self.current_waypoint_index]
        x = wp['x']
        y = wp['y']
        yaw = wp['yaw']

        self.get_logger().info(
            f'웨이포인트 {self.current_waypoint_index} 이동 시작: '
            f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
        )

        goal_pose = self._make_pose(x, y, yaw)
        self.navigator.goToPose(goal_pose)
        self.goal_sent = True
        self.state = PatrolState.WAIT_RESULT

    def _process_navigation_result(self):
        if not self.goal_sent:
            self.state = PatrolState.SEND_GOAL
            return

        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'웨이포인트 {self.current_waypoint_index} 진행 중...',
                    throttle_duration_sec=2.0
                )
            return

        result = self.navigator.getResult()
        wp = self.waypoints_data[self.current_waypoint_index]
        wait_sec = wp['wait_sec']

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'웨이포인트 {self.current_waypoint_index} 도달 완료')

            if wait_sec > 0.0:
                self.get_logger().info(
                    f'웨이포인트 {self.current_waypoint_index} 대기: {wait_sec:.1f}초'
                )
                time.sleep(wait_sec)

            self.current_waypoint_index += 1
            self.goal_sent = False

            if self.current_waypoint_index >= len(self.waypoints_data):
                self._advance_after_full_cycle()
            else:
                self.state = PatrolState.SEND_GOAL

        elif result == TaskResult.CANCELED:
            if self.obstacle_stop:
                self.state = PatrolState.WAIT_OBSTACLE_CLEAR
                self.get_logger().info(
                    f'웨이포인트 {self.current_waypoint_index} 일시중단: 장애물 대기'
                )
            else:
                self.get_logger().warn(
                    f'웨이포인트 {self.current_waypoint_index} 취소됨. 같은 목표 재시도'
                )
                self.goal_sent = False
                self.state = PatrolState.SEND_GOAL

        elif result == TaskResult.FAILED:
            self.get_logger().error(
                f'웨이포인트 {self.current_waypoint_index} 실패. 다음 웨이포인트로 넘어감'
            )
            self.current_waypoint_index += 1
            self.goal_sent = False

            if self.current_waypoint_index >= len(self.waypoints_data):
                self._advance_after_full_cycle()
            else:
                self.state = PatrolState.SEND_GOAL

        else:
            self.get_logger().warn(
                f'웨이포인트 {self.current_waypoint_index} 알 수 없는 결과. 재시도'
            )
            self.goal_sent = False
            self.state = PatrolState.SEND_GOAL

    def _process_obstacle_wait(self):
        if self.obstacle_stop:
            self.get_logger().info(
                f'장애물 해소 대기 중... front_min={self.last_front_min_dist:.2f} m',
                throttle_duration_sec=2.0
            )
            return

        if self.last_obstacle_clear_time is not None:
            elapsed = time.time() - self.last_obstacle_clear_time
            if elapsed < self.obstacle_resume_delay:
                return

        self.get_logger().info(
            f'장애물 해소 확인 → 웨이포인트 {self.current_waypoint_index} 재시도'
        )
        self.goal_sent = False
        self.state = PatrolState.SEND_GOAL

    def _advance_after_full_cycle(self):
        if self.loop:
            self.get_logger().info('순찰 루프 완료. 첫 웨이포인트부터 재시작')
            self.current_waypoint_index = 0
            self.goal_sent = False
            self.state = PatrolState.SEND_GOAL
        else:
            self.get_logger().info('순찰 완료. loop=false → 종료 상태')
            self.state = PatrolState.FINISHED
            self.cmd_vel_pub.publish(Twist())

    def _shutdown_once(self):
        if not self.running:
            return
        self.running = False
        self.cmd_vel_pub.publish(Twist())
        try:
            self.navigator.cancelTask()
        except Exception:
            pass

    def destroy_node(self):
        self._shutdown_once()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolWaypointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('[patrol_waypoint_node] KeyboardInterrupt')
    except Exception as e:
        print(f'[patrol_waypoint_node] 시작 실패: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
