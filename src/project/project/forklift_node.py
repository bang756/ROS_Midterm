#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.time import Time

from geometry_msgs.msg import Twist
from msg_interface.action import GoToId

import tf2_ros


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# Gazebo PosePublisher 기준 TF 프레임 (이미 확인한 값)
PARENT_FRAME = 'odom'
ROBOT_FRAME  = 'base_footprint'


class TurtlebotWaypointTF(Node):
    def __init__(self):
        super().__init__('turtlebot_waypoint_tf_commander')

        # 1) cmd_vel 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2) TF 버퍼 / 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3) 제어 파라미터
        self.dist_tol = 0.20        # MOVE 도달 거리
        self.angle_tol = 0.05       # TURN 도달 각도(rad) ≈ 3도
        self.k_v = 0.6
        self.max_lin = 0.5
        self.turn_speed = 0.6       # 회전 속도 (rad/s)

        # 4) 상태 (스텝 기반)
        # steps 예시:
        # [{'type': 'move', 'x': 0.2, 'y': -7.7},
        #  {'type': 'turn', 'yaw_delta': -pi/2}, ...]
        self.active = False
        self.steps = []
        self.step_index = 0
        self.turn_start_yaw = None  # TURN 단계에서 시작각 저장

        # 5) 액션 서버
        self.action_server = ActionServer(
            self,
            GoToId,
            'goto_id',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )

        # 6) 제어 루프 타이머 (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('[GoToId] TurtlebotWaypointTF (step-based) started.')

    # --------------- 액션 콜백들 ---------------
    def goal_callback(self, goal_request: GoToId.Goal):
        self.get_logger().info(f'[GoToId] Goal received: id={goal_request.id}')
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        rid = goal_handle.request.id
        self.get_logger().info(f'[GoToId] Execute started for id={rid}')

        self.steps = []

        if rid == 1:
            # TF(odom) 기준으로 측정한 id1 정지 위치
            self.steps = [
                {'type': 'move', 'x': 2.86705, 'y': 0.0},
            ]

        elif rid == 2:
            # id2: 네가 말한 TF 좌표 + 회전 순서 반영
            # A = (2.86705,  0.0)     : 직진
            # 오른쪽 90도 회전
            # C = (2.33410, -3.65621) : 직진
            # 왼쪽 90도 회전
            # E = (7.84669, -4.97260) : 직진
            # 왼쪽 90도 회전
            # G = (8.66189, -0.83109) : 직진
            self.steps = [
                # 1) A까지 직진
                {'type': 'move', 'x': 2.86705, 'y': 0.0},

                # 2) 오른쪽 90도 회전
                {'type': 'turn', 'yaw_delta': -math.pi / 2},

                # 3) C까지 직진
                {'type': 'move', 'x': 2.33410, 'y': -3.65621},

                # 4) 왼쪽 90도 회전
                {'type': 'turn', 'yaw_delta':  math.pi / 2},

                # 5) E까지 직진
                {'type': 'move', 'x': 8.47550, 'y': -4.97260},

                # 6) 왼쪽 90도 회전
                {'type': 'turn', 'yaw_delta':  math.pi / 2},

                # 7) G까지 직진
                {'type': 'move', 'x': 8.16189, 'y': -0.83109},
                # 8) 오른쪽 90도 회전
                {'type': 'turn', 'yaw_delta': -math.pi / 2},
            ]

        elif rid == 3:
            # 일단 직선 waypoint만 사용 (TURN은 필요하면 여기에 추가)
            self.steps = [
                # 1) A까지 직진
                {'type': 'move', 'x': 2.86705, 'y': 0.0},

                # 2) 오른쪽 90도 회전
                {'type': 'turn', 'yaw_delta': -math.pi / 2},

                # 3) C까지 직진
                {'type': 'move', 'x': 2.33410, 'y': -3.65621},

                # 4) 왼쪽 90도 회전
                {'type': 'turn', 'yaw_delta':  math.pi / 2},

                # 5) E까지 직진
                {'type': 'move', 'x': 14.55810, 'y': -4.43537},

                # 6) 왼쪽 90도 회전
                {'type': 'turn', 'yaw_delta':  math.pi / 2},

                # 7) G까지 직진
                {'type': 'move', 'x': 14.24463, 'y': -0.83326},
                # 8) 오른쪽 90도 회전
                {'type': 'turn', 'yaw_delta': -math.pi / 2},
            ]

        else:
            result = GoToId.Result()
            result.success = False
            result.message = f'Invalid id: {rid}'
            goal_handle.succeed()
            return result

        self.step_index = 0
        self.active = True
        self.turn_start_yaw = None

        result = GoToId.Result()
        result.success = True
        result.message = f'Started TF-based step path for id={rid}'
        goal_handle.succeed()
        return result

    # --------------- 메인 제어 루프 ---------------
    def control_loop(self):
        if not self.active or not self.steps:
            return

        # TF에서 현재 pose 가져오기
        try:
            trans = self.tf_buffer.lookup_transform(
                PARENT_FRAME,
                ROBOT_FRAME,
                Time()
            )
        except Exception:
            # TF 안 들어오면 멈춤
            self.stop_robot()
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # 모든 step 끝났으면 종료
        if self.step_index >= len(self.steps):
            self.stop_robot()
            self.active = False
            self.get_logger().info('[GoToId] All steps completed.')
            return

        step = self.steps[self.step_index]

        # ---------------- MOVE 스텝 ----------------
        if step['type'] == 'move':
            tx, ty = step['x'], step['y']

            dx = tx - x
            dy = ty - y
            dist = math.sqrt(dx * dx + dy * dy)

            # 목표점 도달 → 다음 스텝으로
            if dist < self.dist_tol:
                self.get_logger().info(
                    f'[GoToId] Reached MOVE step {self.step_index+1} at ({tx:.2f},{ty:.2f}).'
                )
                self.step_index += 1
                # TURN 스텝 들어가면 새로 yaw 기준 잡아야 하니 초기화
                self.turn_start_yaw = None
                return

            # MOVE는 항상 "목표점 방향"을 향해서 전진 + 약간 회전
            angle_to_goal = math.atan2(dy, dx)
            yaw_err = norm_angle(angle_to_goal - yaw)

            v = self.k_v * dist
            v = clip(v, 0.1, self.max_lin)
            w = yaw_err * 2.0  # 간단한 비례 제어

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = clip(w, -self.turn_speed, self.turn_speed)
            self.cmd_pub.publish(cmd)

        # ---------------- TURN 스텝 ----------------
        elif step['type'] == 'turn':
            # 처음 턴 시작할 때 현재 yaw 저장
            if self.turn_start_yaw is None:
                self.turn_start_yaw = yaw

            target_yaw = self.turn_start_yaw + step['yaw_delta']
            target_yaw = norm_angle(target_yaw)

            yaw_err = norm_angle(target_yaw - yaw)

            # 각도 오차 충분히 작으면 턴 완료
            if abs(yaw_err) < self.angle_tol:
                self.get_logger().info(
                    f'[GoToId] Completed TURN step {self.step_index+1} (delta={step["yaw_delta"]:.2f} rad).'
                )
                self.step_index += 1
                self.turn_start_yaw = None
                self.stop_robot()
                return

            # 회전만 (전진 0)
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed if yaw_err > 0 else -self.turn_speed
            self.cmd_pub.publish(cmd)

        else:
            # 정의 안 된 타입이면 그냥 넘어감
            self.get_logger().warn(f'Unknown step type: {step["type"]}')
            self.step_index += 1

    # --------------- 정지 ---------------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotWaypointTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
