#!/usr/bin/env python3
"""Coordinate direct PCT global-path following without ART local planning."""

from copy import deepcopy
import math
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class GlobalPathFollowState:
    WAIT_PATH = 'WAIT_PATH'
    TRACKING = 'TRACKING'
    FINAL_APPROACH = 'FINAL_APPROACH'
    GOAL_REACHED = 'GOAL_REACHED'
    BLOCKED = 'BLOCKED'


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def set_pose_yaw(pose, yaw: float):
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)


def pose_distance_xy(a, b) -> float:
    return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)


class GlobalPathFollowCoordinator(Node):
    """Latch completion and final-heading control for direct global path tests."""

    def __init__(self):
        super().__init__('pct_global_path_follow_coordinator')
        self._declare_parameters()
        self._read_parameters()

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._tracking_path_pub = self.create_publisher(
            Path, self.tracking_path_topic, latched_qos
        )
        self._tracking_segment_pub = self.create_publisher(
            Path, self.tracking_segment_topic, latched_qos
        )
        self._tracking_point_pub = self.create_publisher(
            PoseStamped, self.tracking_point_topic, latched_qos
        )
        self._final_approach_pub = self.create_publisher(
            Bool, self.final_approach_topic, latched_qos
        )
        self._status_pub = self.create_publisher(String, self.status_topic, latched_qos)

        self.create_subscription(
            Path, self.global_path_topic, self._global_path_cb, latched_qos
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._active_path: Optional[Path] = None
        self._active_signature: Optional[Tuple[float, ...]] = None
        self._completed_signature: Optional[Tuple[float, ...]] = None
        self._progress_segment_index: Optional[int] = None
        self._final_approach_active: Optional[bool] = None
        self._last_tracking_publish_time = None
        self._state = ''
        self._state_reason = ''

        self._set_state(GlobalPathFollowState.WAIT_PATH, 'waiting for global path')
        self._publish_final_approach(False)
        self._publish_empty_tracking_path('startup')

        self.create_timer(1.0 / self.update_rate, self._update)
        self.create_timer(
            1.0 / self.tracking_path_republish_rate,
            self._republish_tracking_path,
        )

        self.get_logger().info(
            'Global path follow coordinator ready: '
            f'input={self.global_path_topic}, output={self.tracking_path_topic}, '
            f'goal_distance={self.goal_reached_distance:.3f}, '
            f'goal_yaw={self.goal_yaw_tolerance:.3f}'
        )

    def _declare_parameters(self):
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('global_path_topic', '/pct_path')
        self.declare_parameter('tracking_path_topic', '/global_path_follow/path')
        self.declare_parameter(
            'tracking_segment_topic', '/global_path_follow/tracking_segment'
        )
        self.declare_parameter(
            'tracking_point_topic', '/global_path_follow/tracking_point'
        )
        self.declare_parameter(
            'final_approach_topic', '/pct_scan_navigation/final_approach'
        )
        self.declare_parameter(
            'status_topic', '/pct_global_path_follow/status'
        )
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('tracking_path_republish_rate', 5.0)
        self.declare_parameter('goal_reached_distance', 0.15)
        self.declare_parameter('goal_yaw_tolerance', 0.10)
        self.declare_parameter('goal_reached_z_tolerance', 0.8)
        self.declare_parameter('final_heading_entry_distance', 0.25)
        self.declare_parameter('tf_timeout', 0.1)
        self.declare_parameter('crop_path_to_robot', True)
        self.declare_parameter('tracking_path_length', 6.0)
        self.declare_parameter('tracking_path_min_points', 8)
        self.declare_parameter('tracking_point_lookahead_distance', 0.7)
        self.declare_parameter('use_height_for_progress', True)
        self.declare_parameter('progress_z_weight', 1.0)
        self.declare_parameter('progress_max_z_distance', 1.2)
        self.declare_parameter('progress_backtrack_segments', 20)
        self.declare_parameter('progress_forward_segments', 300)

    def _read_parameters(self):
        def get(name):
            return self.get_parameter(name).value

        self.global_frame = str(get('global_frame'))
        self.robot_frame = str(get('robot_frame'))
        self.global_path_topic = str(get('global_path_topic'))
        self.tracking_path_topic = str(get('tracking_path_topic'))
        self.tracking_segment_topic = str(get('tracking_segment_topic'))
        self.tracking_point_topic = str(get('tracking_point_topic'))
        self.final_approach_topic = str(get('final_approach_topic'))
        self.status_topic = str(get('status_topic'))
        self.update_rate = float(get('update_rate'))
        self.tracking_path_republish_rate = float(get('tracking_path_republish_rate'))
        self.goal_reached_distance = float(get('goal_reached_distance'))
        self.goal_yaw_tolerance = float(get('goal_yaw_tolerance'))
        self.goal_reached_z_tolerance = float(get('goal_reached_z_tolerance'))
        self.final_heading_entry_distance = float(get('final_heading_entry_distance'))
        self.tf_timeout = float(get('tf_timeout'))
        self.crop_path_to_robot = bool(get('crop_path_to_robot'))
        self.tracking_path_length = float(get('tracking_path_length'))
        self.tracking_path_min_points = int(get('tracking_path_min_points'))
        self.tracking_point_lookahead_distance = float(
            get('tracking_point_lookahead_distance')
        )
        self.use_height_for_progress = bool(get('use_height_for_progress'))
        self.progress_z_weight = float(get('progress_z_weight'))
        self.progress_max_z_distance = float(get('progress_max_z_distance'))
        self.progress_backtrack_segments = int(get('progress_backtrack_segments'))
        self.progress_forward_segments = int(get('progress_forward_segments'))

        positive = {
            'update_rate': self.update_rate,
            'tracking_path_republish_rate': self.tracking_path_republish_rate,
            'goal_reached_distance': self.goal_reached_distance,
            'goal_yaw_tolerance': self.goal_yaw_tolerance,
            'final_heading_entry_distance': self.final_heading_entry_distance,
            'tf_timeout': self.tf_timeout,
            'tracking_path_length': self.tracking_path_length,
            'tracking_point_lookahead_distance': (
                self.tracking_point_lookahead_distance
            ),
            'goal_reached_z_tolerance': self.goal_reached_z_tolerance,
            'progress_z_weight': self.progress_z_weight,
            'progress_max_z_distance': self.progress_max_z_distance,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero')

        if self.goal_yaw_tolerance > math.pi:
            raise ValueError('goal_yaw_tolerance must not exceed pi')
        if self.final_heading_entry_distance < self.goal_reached_distance:
            raise ValueError(
                'final_heading_entry_distance must be >= goal_reached_distance'
            )
        if self.tracking_path_min_points < 2:
            raise ValueError('tracking_path_min_points must be at least 2')
        if self.progress_backtrack_segments < 0 or self.progress_forward_segments < 1:
            raise ValueError(
                'progress search windows must satisfy backtrack >= 0 and forward >= 1'
            )

    def _global_path_cb(self, message: Path):
        if not message.poses:
            self._clear_active_path('received empty global path')
            return

        if message.header.frame_id and message.header.frame_id != self.global_frame:
            self._clear_active_path(
                f'global path frame {message.header.frame_id!r} is not '
                f'{self.global_frame!r}',
                state=GlobalPathFollowState.BLOCKED,
            )
            return

        signature = self._path_signature(message)
        if signature is None:
            self._clear_active_path('received invalid global path')
            return

        if (
            self._completed_signature == signature
            and self._active_signature != signature
        ):
            self._publish_empty_tracking_path('completed path ignored')
            return

        path_copy = deepcopy(message)
        path_copy.header.frame_id = self.global_frame
        self._active_path = path_copy
        if signature == self._active_signature:
            self._publish_tracking_path()
            return

        self._active_signature = signature
        self._progress_segment_index = None
        self._set_state(GlobalPathFollowState.TRACKING, 'tracking global path')
        self._publish_final_approach(False)
        self._publish_tracking_path()

    def _update(self):
        if self._active_path is None:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            self._set_state(
                GlobalPathFollowState.BLOCKED,
                f'cannot lookup {self.global_frame}->{self.robot_frame}',
            )
            self._publish_final_approach(False)
            return

        robot_x, robot_y, robot_z, robot_yaw = robot_pose
        goal = self._active_path.poses[-1].pose
        goal_x = goal.position.x
        goal_y = goal.position.y
        goal_z = goal.position.z
        goal_yaw = quaternion_to_yaw(goal.orientation)
        goal_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        goal_z_error = goal_z - robot_z
        yaw_error = normalize_angle(goal_yaw - robot_yaw)
        goal_z_ok = (
            not self.use_height_for_progress
            or abs(goal_z_error) <= self.goal_reached_z_tolerance
        )

        final_approach = (
            goal_distance <= self.final_heading_entry_distance and goal_z_ok
        )
        self._publish_final_approach(final_approach)
        if final_approach:
            self._set_state(
                GlobalPathFollowState.FINAL_APPROACH,
                f'distance={goal_distance:.3f}, z_error={goal_z_error:.3f}, '
                f'yaw_error={yaw_error:.3f}',
            )
        elif self._state != GlobalPathFollowState.TRACKING:
            self._set_state(GlobalPathFollowState.TRACKING, 'tracking global path')

        if (
            goal_distance <= self.goal_reached_distance
            and goal_z_ok
            and abs(yaw_error) <= self.goal_yaw_tolerance
        ):
            self._complete_goal(goal_distance, goal_z_error, yaw_error)
            return

        self._publish_tracking_path(robot_pose)

    def _lookup_robot_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exception:
            self.get_logger().debug(f'TF lookup failed: {exception}')
            return None

        translation = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        return translation.x, translation.y, translation.z, yaw

    def _path_signature(self, message: Path) -> Optional[Tuple[float, ...]]:
        first = message.poses[0].pose
        last = message.poses[-1].pose
        values = (
            first.position.x,
            first.position.y,
            first.position.z,
            last.position.x,
            last.position.y,
            last.position.z,
            quaternion_to_yaw(last.orientation),
            float(len(message.poses)),
        )
        if not all(math.isfinite(value) for value in values):
            return None
        return tuple(round(value, 3) for value in values)

    def _complete_goal(
        self, goal_distance: float, goal_z_error: float, yaw_error: float
    ):
        self._completed_signature = self._active_signature
        self._clear_active_path(
            f'goal reached: distance={goal_distance:.3f}, '
            f'z_error={goal_z_error:.3f}, '
            f'yaw_error={yaw_error:.3f}',
            state=GlobalPathFollowState.GOAL_REACHED,
        )

    def _clear_active_path(self, reason: str, state: str = GlobalPathFollowState.WAIT_PATH):
        self._active_path = None
        self._active_signature = None
        self._progress_segment_index = None
        self._publish_final_approach(False)
        self._publish_empty_tracking_path(reason)
        self._set_state(state, reason)

    def _publish_tracking_path(self, robot_pose=None):
        if self._active_path is None:
            return

        if self.crop_path_to_robot:
            if robot_pose is None:
                robot_pose = self._lookup_robot_pose()
            if robot_pose is None:
                return
            message = self._make_cropped_tracking_path(robot_pose)
            if message is None or not message.poses:
                return
        else:
            message = deepcopy(self._active_path)

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame
        for pose in message.poses:
            pose.header = message.header
        self._tracking_path_pub.publish(message)
        self._tracking_segment_pub.publish(message)
        self._publish_tracking_point(message)
        self._last_tracking_publish_time = self.get_clock().now()

    def _republish_tracking_path(self):
        if self._active_path is not None:
            self._publish_tracking_path()

    def _publish_empty_tracking_path(self, reason: str):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame
        self._tracking_path_pub.publish(message)
        self._tracking_segment_pub.publish(message)
        self._last_tracking_publish_time = self.get_clock().now()
        self.get_logger().info(f'published empty tracking path: {reason}')

    def _make_cropped_tracking_path(self, robot_pose) -> Optional[Path]:
        if self._active_path is None:
            return None
        source = self._active_path
        if len(source.poses) <= 1:
            return deepcopy(source)

        robot_x, robot_y, robot_z, _ = robot_pose
        projection = self._project_robot_to_path(robot_x, robot_y, robot_z)
        if projection is None:
            return None

        segment_index, ratio, proj_x, proj_y, proj_z = projection
        self._progress_segment_index = segment_index

        output = Path()
        output.header.frame_id = self.global_frame

        projected_pose = deepcopy(source.poses[segment_index])
        projected_pose.pose.position.x = proj_x
        projected_pose.pose.position.y = proj_y
        projected_pose.pose.position.z = proj_z
        output.poses.append(projected_pose)

        accumulated = 0.0
        previous_pose = projected_pose.pose
        reached_global_goal = False
        next_index = segment_index + 1
        if ratio >= 0.999:
            next_index = min(segment_index + 2, len(source.poses) - 1)

        for index in range(next_index, len(source.poses)):
            pose = deepcopy(source.poses[index])
            accumulated += pose_distance_xy(previous_pose, pose.pose)
            output.poses.append(pose)
            previous_pose = pose.pose
            reached_global_goal = index == len(source.poses) - 1
            if (
                len(output.poses) >= self.tracking_path_min_points
                and accumulated >= self.tracking_path_length
            ):
                break

        if len(output.poses) == 1:
            output.poses.append(deepcopy(source.poses[-1]))
            reached_global_goal = True

        self._assign_tracking_orientations(output, reached_global_goal)
        return output

    def _project_robot_to_path(self, robot_x: float, robot_y: float, robot_z: float):
        if self._active_path is None or len(self._active_path.poses) < 2:
            return None

        poses = self._active_path.poses
        if self._progress_segment_index is None:
            start_index = 0
            end_index = len(poses) - 2
        else:
            start_index = max(
                0, self._progress_segment_index - self.progress_backtrack_segments
            )
            end_index = min(
                len(poses) - 2,
                self._progress_segment_index + self.progress_forward_segments,
            )

        best_height_valid = None
        best_height_valid_score = float('inf')
        best_any = None
        best_any_score = float('inf')
        for index in range(start_index, end_index + 1):
            start = poses[index].pose.position
            end = poses[index + 1].pose.position
            dx = end.x - start.x
            dy = end.y - start.y
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                ratio = 0.0
            else:
                ratio = ((robot_x - start.x) * dx + (robot_y - start.y) * dy) / length_sq
                ratio = max(0.0, min(1.0, ratio))

            proj_x = start.x + ratio * dx
            proj_y = start.y + ratio * dy
            start_z = start.z if math.isfinite(start.z) else 0.0
            end_z = end.z if math.isfinite(end.z) else start_z
            proj_z = start_z + ratio * (end_z - start_z)
            xy_distance_sq = (robot_x - proj_x) ** 2 + (robot_y - proj_y) ** 2
            z_error = abs(robot_z - proj_z)
            score = xy_distance_sq
            if self.use_height_for_progress:
                score += (self.progress_z_weight * z_error) ** 2

            candidate = (index, ratio, proj_x, proj_y, proj_z)
            if score < best_any_score:
                best_any_score = score
                best_any = candidate

            if (
                not self.use_height_for_progress
                or z_error <= self.progress_max_z_distance
            ):
                if score < best_height_valid_score:
                    best_height_valid_score = score
                    best_height_valid = candidate

        if best_height_valid is not None:
            return best_height_valid
        return best_any

    def _assign_tracking_orientations(self, path: Path, reached_global_goal: bool):
        if not path.poses:
            return
        if len(path.poses) == 1:
            return

        last_tangent_yaw = 0.0
        for index in range(len(path.poses) - 1):
            current = path.poses[index].pose.position
            nxt = path.poses[index + 1].pose.position
            yaw = math.atan2(nxt.y - current.y, nxt.x - current.x)
            if math.isfinite(yaw):
                last_tangent_yaw = yaw
                set_pose_yaw(path.poses[index].pose, yaw)

        if reached_global_goal and self._active_path is not None:
            path.poses[-1].pose.orientation = self._active_path.poses[-1].pose.orientation
        else:
            set_pose_yaw(path.poses[-1].pose, last_tangent_yaw)

    def _publish_tracking_point(self, path: Path):
        point = self._interpolate_pose_on_path(
            path, self.tracking_point_lookahead_distance
        )
        if point is None:
            return
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.global_frame
        self._tracking_point_pub.publish(point)

    def _interpolate_pose_on_path(
        self, path: Path, arc_distance: float
    ) -> Optional[PoseStamped]:
        if not path.poses:
            return None
        if len(path.poses) == 1 or arc_distance <= 0.0:
            return deepcopy(path.poses[0])

        remaining = arc_distance
        for index in range(len(path.poses) - 1):
            start = path.poses[index].pose.position
            end = path.poses[index + 1].pose.position
            segment_length = math.hypot(end.x - start.x, end.y - start.y)
            if segment_length <= 1e-9:
                continue
            if remaining > segment_length:
                remaining -= segment_length
                continue

            ratio = remaining / segment_length
            point = deepcopy(path.poses[index])
            point.pose.position.x = start.x + ratio * (end.x - start.x)
            point.pose.position.y = start.y + ratio * (end.y - start.y)
            start_z = start.z if math.isfinite(start.z) else 0.0
            end_z = end.z if math.isfinite(end.z) else start_z
            point.pose.position.z = start_z + ratio * (end_z - start_z)
            set_pose_yaw(point.pose, math.atan2(end.y - start.y, end.x - start.x))
            return point

        return deepcopy(path.poses[-1])

    def _publish_final_approach(self, active: bool):
        if active == self._final_approach_active:
            return
        self._final_approach_active = active
        message = Bool()
        message.data = active
        self._final_approach_pub.publish(message)

    def _set_state(self, state: str, reason: str):
        if state == self._state and reason == self._state_reason:
            return
        self._state = state
        self._state_reason = reason
        message = String()
        message.data = f'{state}: {reason}'
        self._status_pub.publish(message)
        self.get_logger().info(message.data)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathFollowCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
