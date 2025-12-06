#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ros2_aruco_interfaces.msg import ArucoMarkers
from msg_interface.action import GoToId


class ArucoToForklift(Node):
    """Bridge ArUco detections to forklift GoToId actions."""

    def __init__(self):
        super().__init__("aruco_to_forklift")

        # Marker ID → GoToId mapping
        self.marker_to_goal = {
            3: 1,  # 토마토
            4: 2,  # 바나나
            5: 3,  # 사과
        }

        # 한 번 전송 후 추가 명령 차단
        self.sent_goal = False

        self.goto_client = ActionClient(self, GoToId, "goto_id")
        self.goto_connected_logged = False
        self.current_goal_marker = None
        self.current_goal_future = None

        self.create_subscription(
            ArucoMarkers, "aruco_markers", self.marker_callback, 10
        )

        self.get_logger().info(
            "[ArucoToForklift] Ready. Will send GoToId based on markers: "
            + ", ".join(f"{mid}->{gid}" for mid, gid in self.marker_to_goal.items())
        )

    def marker_callback(self, msg: ArucoMarkers):
        # Ensure action server availability (non-blocking check)
        if not self.goto_client.server_is_ready():
            if not self.goto_connected_logged:
                self.get_logger().warn(
                    "[ArucoToForklift] goto_id action server not ready yet."
                )
                self.goto_connected_logged = False
            return
        else:
            self.goto_connected_logged = True

        # Pick the first matching marker in the message
        target_marker = None
        for mid in msg.marker_ids:
            if mid in self.marker_to_goal:
                target_marker = mid
                break

        if target_marker is None:
            return

        # 한 번 전송 후에는 더 이상 명령 전송 안 함
        if self.sent_goal:
            return

        # Avoid resending the same marker while a goal is active
        if (
            self.current_goal_marker == target_marker
            and self.current_goal_future
            and not self.current_goal_future.done()
        ):
            return

        goal_id = self.marker_to_goal[target_marker]
        goal_msg = GoToId.Goal()
        goal_msg.id = goal_id

        self.get_logger().info(
            f"[ArucoToForklift] Sending GoToId {goal_id} for marker {target_marker}"
        )
        self.current_goal_marker = target_marker
        self.current_goal_future = self.goto_client.send_goal_async(goal_msg)
        self.current_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(
                    f"[ArucoToForklift] GoToId goal rejected for marker {self.current_goal_marker}"
                )
                self.current_goal_marker = None
                return

            self.get_logger().info(
                f"[ArucoToForklift] GoToId goal accepted for marker {self.current_goal_marker}"
            )
            # 한 번 성공적으로 전송하면 이후 추가 명령 차단
            self.sent_goal = True
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._result_callback)
        except Exception as e:
            self.get_logger().error(f"[ArucoToForklift] Goal send exception: {e}")
            self.current_goal_marker = None

    def _result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f"[ArucoToForklift] GoToId result: success={result.success}, message={result.message}"
            )
        except Exception as e:
            self.get_logger().error(f"[ArucoToForklift] Result exception: {e}")
        finally:
            # Allow sending the same marker again after completion
            self.current_goal_marker = None
            self.current_goal_future = None


def main(args=None):
    rclpy.init(args=args)
    node = ArucoToForklift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
