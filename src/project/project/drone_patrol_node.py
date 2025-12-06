#!/usr/bin/env python3
import os
import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from msg_interface.action import FindItem
from msg_interface.srv import SetDetectTarget   # 인식 노드에 타겟 전달용 서비스


class DronePatrol(Node):
    def __init__(self):
        super().__init__('drone_patrol_commander')

        # =====================================
        # 1. 퍼블리셔 / 서브스크라이버
        # =====================================
        self.vel_pub = self.create_publisher(Twist, '/X4/cmd_vel', 10)

        self.current_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.create_subscription(
            Odometry,
            '/model/X4_GPS_RGBD/odometry',
            self.pose_callback,
            10
        )

        # =====================================
        # 2. 순회 경로 / 상태머신
        # =====================================
        self.waypoints = [
            {'x': -1.8, 'y': -7.7, 'z': 2.025, 'yaw_after': 0.0},    # 1
            {'x': 4.0,  'y': -7.7, 'z': 2.025, 'yaw_after': 0.0},    # 2
            {'x': 4.0,  'y': -4.3, 'z': 2.025, 'yaw_after': 3.1416}, # 3
            {'x': -1.8, 'y': -4.3, 'z': 2.025, 'yaw_after': 3.1416}, # 4 
            {'x': -1.8, 'y': -1.7, 'z': 2.025, 'yaw_after': 0.0},    # 5
            {'x': 4.0,  'y': -1.7, 'z': 2.025, 'yaw_after': 0.0},    # 6
            {'x': 4.0,  'y': 1.7,  'z': 2.025, 'yaw_after': 3.1416}, # 7
            {'x': -1.8, 'y': 1.7,  'z': 2.025, 'yaw_after': 3.1416}, # 8 
            {'x': -1.8, 'y': 4.3,  'z': 2.025, 'yaw_after': 0.0},    # 9
            {'x': 4.0,  'y': 4.3,  'z': 2.025, 'yaw_after': 0.0},    # 10
            {'x': 4.0,  'y': 7.7,  'z': 2.025, 'yaw_after': 3.1416}, # 11
            {'x': -4.0, 'y': 7.7,  'z': 2.025, 'yaw_after': 0.0},    # 12
            {'x': -4.0, 'y': -7.7, 'z': 2.025, 'yaw_after': 3.1416}  # 13
        ]

        # 이륙 목표 고도(z=3.0). 처음엔 바닥(z≈0)에 있다가,
        # 액션 요청 들어온 뒤 TAKE_OFF 상태에서만 이 값을 향해 올라감.
        self.takeoff_target = {'x': -1.8, 'y': -10.5, 'z': 2.025}

        self.waypoint_index = 0
        self.first_loop_done = False
        self.extra_yaw_start = None

        # 상태: IDLE, TAKE_OFF, MOVE, ALIGN_EXTRA
        self.state = 'IDLE'
        self.is_patrolling = False  # False일 땐 아무 제어 안 함 (바닥에 붙어 있음)
        self.pending_detect_item = None
        self.detect_retry_timer = None

        # =====================================
        # 3. item_db.yaml 로드 (DB 용)
        # ===================================== #001
        yaml_path = os.path.join(
            get_package_share_directory('project_description'),
            'config',
            'item_db.yaml',
        )
        self.item_db = {}

        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                self.item_db = data.get('items', {})
                self.get_logger().info(
                    f'Loaded {len(self.item_db)} items from {yaml_path}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to load item_db.yaml: {e}')
        else:
            self.get_logger().warn(
                f'item_db.yaml not found at {yaml_path}. No items will be available.'
            )

        # =====================================
        # 4. 인식 노드 서비스 클라이언트
        # =====================================
        self.detect_target_cli = self.create_client(SetDetectTarget, 'set_detect_target')

        # =====================================
        # 5. FindItem 액션 서버
        # =====================================
        self.find_item_server = ActionServer(
            self,
            FindItem,
            'find_item',
            execute_callback=self.execute_find_item,
            goal_callback=self.find_item_goal_callback,
        )

        # =====================================
        # 6. 순회 제어 타이머
        # =====================================
        self.patrol_timer = self.create_timer(0.1, self.patrol_loop)

    # =========================================
    # Pose / 거리 / 웨이포인트 관련
    # =========================================

    def pose_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.current_pose['x'] = pose.position.x
        self.current_pose['y'] = pose.position.y
        self.current_pose['z'] = pose.position.z

        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose['yaw'] = yaw

    def get_distance_to_current_wp(self) -> float:
        target = self.waypoints[self.waypoint_index]
        dx = target['x'] - self.current_pose['x']
        dy = target['y'] - self.current_pose['y']
        return math.sqrt(dx * dx + dy * dy)

    def goto_next_waypoint(self):
        if self.waypoint_index == len(self.waypoints) - 1 and not self.first_loop_done:
            self.first_loop_done = True

        self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)

        if self.first_loop_done and self.waypoint_index == 0:
            self.waypoint_index = 1

        self.state = 'MOVE'
        self.get_logger().info(
            f"[PATROL] Next WP: {self.waypoint_index + 1} (index {self.waypoint_index})"
        )

    def stop_drone(self):
        self.vel_pub.publish(Twist())

    # =========================================
    # FindItem 액션 서버: goal / execute
    # =========================================

    def find_item_goal_callback(self, goal_request: FindItem.Goal):
        item_name = goal_request.item_name
        self.get_logger().info(f'[FindItem] Goal received: "{item_name}"')

        # item_db에 없으면 바로 거절
        if item_name not in self.item_db:
            self.get_logger().warn(
                f'[FindItem] item "{item_name}" not found in item_db.yaml. Rejecting.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'[FindItem] item "{item_name}" exists in item_db.yaml. Accepting.'
        )
        return GoalResponse.ACCEPT

    def execute_find_item(self, goal_handle):
        item_name = goal_handle.request.item_name
        self.get_logger().info(f'[FindItem] Execute started for "{item_name}"')

        result = FindItem.Result()
        feedback = FindItem.Feedback()

        # ---------------------------
        # 1) 인식 노드에 타겟 전달 (Service)
        # ---------------------------
        self.pending_detect_item = item_name
        if self.detect_target_cli.wait_for_service(timeout_sec=1.0):
            self._call_set_detect_target(item_name)
        else:
            self.get_logger().warn(
                '[FindItem] set_detect_target service not available. '
                'Will keep retrying in background.'
            )
            self._start_detect_retry_timer()

        # ---------------------------
        # 2) 드론 순회 시작 (TAKE_OFF → MOVE...)
        #    → 이 시점부터 고도 3.0m를 향해 상승 시작
        # ---------------------------
        self.state = 'TAKE_OFF'
        self.is_patrolling = True
        self.waypoint_index = 0
        self.first_loop_done = False
        self.extra_yaw_start = None

        feedback.state = 'TAKE_OFF'
        goal_handle.publish_feedback(feedback)

        # 여기서는 "순회 시작 명령"만 하는 액션이므로
        # 바로 성공 리턴 (패트롤은 타이머에서 계속 동작)
        result.success = True
        result.message = f'Patrol started for item "{item_name}".'

        goal_handle.succeed()
        self.get_logger().info('[FindItem] Patrol started. Action finished.')
        return result

    def _handle_set_detect_target_response(self, future):
        try:
            resp = future.result()
            if resp is None:
                self.get_logger().error('[FindItem] set_detect_target service call failed (no response).')
                return
            if resp.accepted:
                self.get_logger().info(f'[FindItem] set_detect_target accepted: {resp.message}')
                self.pending_detect_item = None
                self._stop_detect_retry_timer()
            else:
                self.get_logger().warn(f'[FindItem] set_detect_target rejected: {resp.message}')
                # Retry if rejected
                if self.pending_detect_item:
                    self._start_detect_retry_timer()
        except Exception as e:
            self.get_logger().error(f'[FindItem] set_detect_target call exception: {e}')
            if self.pending_detect_item:
                self._start_detect_retry_timer()

    def _call_set_detect_target(self, item_name: str):
        req = SetDetectTarget.Request()
        req.item_name = item_name
        future = self.detect_target_cli.call_async(req)
        future.add_done_callback(self._handle_set_detect_target_response)

    def _start_detect_retry_timer(self):
        if self.detect_retry_timer is None:
            self.detect_retry_timer = self.create_timer(2.0, self._retry_set_detect_target)
        else:
            self.detect_retry_timer.reset()

    def _stop_detect_retry_timer(self):
        if self.detect_retry_timer is not None:
            self.detect_retry_timer.cancel()
            self.detect_retry_timer = None

    def _retry_set_detect_target(self):
        if not self.pending_detect_item:
            self._stop_detect_retry_timer()
            return
        if not self.detect_target_cli.service_is_ready():
            return
        self.get_logger().info(f'[FindItem] Retrying set_detect_target for "{self.pending_detect_item}"')
        self._call_set_detect_target(self.pending_detect_item)

    # =========================================
    # 주기적 제어 루프 (순회 제어)
    # =========================================

    def patrol_loop(self):
        cmd = Twist()

        # 아직 미션 시작 안 했으면 아무 제어도 안 함 → 바닥에 붙어 있게 됨
        if not self.is_patrolling:
            self.vel_pub.publish(cmd)
            return

        # 고도 유지 P 제어 (z = 3.0m)
        desired_z = self.takeoff_target['z']
        z_error = desired_z - self.current_pose['z']
        k_z = 1.0
        cmd.linear.z = max(min(k_z * z_error, 1.0), -1.0)

        if self.state == 'IDLE':
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        elif self.state == 'TAKE_OFF':
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            # 고도 어느 정도 맞으면 MOVE로 전환
            if abs(z_error) < 0.1:
                self.state = 'MOVE'
                self.get_logger().info(
                    f"[PATROL] Takeoff complete. Moving to WP {self.waypoint_index + 1}."
                )

        elif self.state == 'MOVE':
            target_wp = self.waypoints[self.waypoint_index]
            target_dist = self.get_distance_to_current_wp()

            if target_dist > 0.5:
                target_x = target_wp['x']
                target_y = target_wp['y']
                current_x = self.current_pose['x']
                current_y = self.current_pose['y']
                yaw = self.current_pose['yaw']

                if target_dist > 0.01:
                    vx_world = (target_x - current_x) / target_dist * 1.0
                    vy_world = (target_y - current_y) / target_dist * 1.0
                else:
                    vx_world, vy_world = 0.0, 0.0

                cmd.linear.x = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
                cmd.linear.y = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)
                cmd.angular.z = 0.0
            else:
                yaw_after = target_wp['yaw_after']
                if abs(yaw_after) > 0.01:
                    self.state = 'ALIGN_EXTRA'
                    self.extra_yaw_start = self.current_pose['yaw']
                    self.get_logger().info(
                        f"[PATROL] Reached WP {self.waypoint_index + 1}. Start extra rotation."
                    )
                else:
                    self.goto_next_waypoint()

        elif self.state == 'ALIGN_EXTRA':
            if self.extra_yaw_start is None:
                self.extra_yaw_start = self.current_pose['yaw']

            target_wp = self.waypoints[self.waypoint_index]
            yaw_after = target_wp['yaw_after']

            target_yaw = self.extra_yaw_start + yaw_after
            target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

            angle_diff = target_yaw - self.current_pose['yaw']
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) > 0.1:
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
            else:
                self.get_logger().info(
                    f"[PATROL] Extra rotation complete at WP {self.waypoint_index + 1}."
                )
                self.extra_yaw_start = None
                self.goto_next_waypoint()

        self.vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DronePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
