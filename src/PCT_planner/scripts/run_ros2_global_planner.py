#!/usr/bin/env python3
"""
PCT Global Planner — standalone ROS2 online node.

Workflow:
  1. Loads an existing tomogram pickle.
  2. Publishes tomogram layers to RViz2.
  3. Waits for a single goal point:
       - /goal_pose   (geometry_msgs/PoseStamped)  — RViz "2D Goal Pose" tool
       - /clicked_point (geometry_msgs/PointStamped) — RViz "Publish Point" tool
  4. Gets robot current pose via TF (map → base_link) as start.
  5. Plans a global path and publishes:
       - /pct_path        (nav_msgs/Path)          — smoothed PCT trajectory
       - /pct_astar_path  (nav_msgs/Path)          — raw A* path (debug)
       - /pct_marker      (visualization_msgs/Marker) — line strip + goal sphere

Usage:
  source /opt/ros/humble/setup.bash
  python3 run_ros2_global_planner.py \
      --ros-args \
      -p tomo_path:=/path/to/tomogram.pickle
"""

import os, sys, argparse, pickle, time, math, threading
import ctypes
import numpy as np


def _find_project_root():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.abspath(os.path.join(script_dir, '..')),
        os.path.abspath(
            os.path.join(script_dir, '..', '..', '..', '..', 'src', 'PCT_planner')
        ),
        os.path.abspath(os.path.join(os.getcwd(), 'src', 'PCT_planner')),
    ]
    for candidate in candidates:
        if os.path.isfile(
            os.path.join(candidate, 'planner', 'lib', 'libmetis-gtsam.so')
        ):
            return candidate
    raise FileNotFoundError(
        'Could not find PCT_planner root containing planner/lib/libmetis-gtsam.so'
    )


ROOT = _find_project_root()
LIB_PATH = os.path.join(ROOT, 'planner', 'lib')


# ── GTSAM + smoothing libs preload (must happen before pybind11 imports) ──
for _lib in [
    os.path.join(LIB_PATH, 'libmetis-gtsam.so'),
    os.path.join(LIB_PATH, 'libgtsam.so.4'),
    os.path.join(LIB_PATH, 'libcommon_smoothing.so'),
]:
    ctypes.CDLL(_lib, mode=ctypes.RTLD_GLOBAL)

sys.path.insert(0, os.path.join(ROOT, 'planner'))

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped
from pct_scan_navigation.srv import LoadTomogram

import importlib.util as _ilu


def _load(path, module_name):
    spec = _ilu.spec_from_file_location(module_name, path)
    mod = _ilu.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


# Planner modules (same loading pattern as the interactive runner)
sys.path.insert(0, os.path.join(ROOT, 'planner', 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'planner'))
_plan_cfg = _load(os.path.join(ROOT, 'planner', 'config', '__init__.py'), 'plan_config')
PlanCfg = _plan_cfg.Config
from planner_wrapper import TomogramPlanner


# ─── helpers ────────────────────────────────────────────────────────────────

def make_pc2(node, points_f32, fields_xyz=True, frame='map'):
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points_f32)
    if fields_xyz:
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
    else:
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
    msg.row_step = msg.point_step * len(points_f32)
    msg.is_bigendian = False
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points_f32).tobytes()
    return msg


def traj_to_path(traj_3d, node, frame='map', goal_yaw=None):
    """
    Convert a PCT (N,3) trajectory to nav_msgs/Path with proper orientation.

    Each waypoint yaw = direction from that point to the next.
    Last waypoint: uses goal_yaw if provided, otherwise inherits previous direction.
    """
    msg = Path()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame

    n = traj_3d.shape[0]
    if n == 0:
        return msg

    for i in range(n):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(traj_3d[i, 0])
        ps.pose.position.y = float(traj_3d[i, 1])
        ps.pose.position.z = float(traj_3d[i, 2])

        # yaw from current point to next
        if i < n - 1:
            dx = float(traj_3d[i + 1, 0]) - float(traj_3d[i, 0])
            dy = float(traj_3d[i + 1, 1]) - float(traj_3d[i, 1])
            yaw = math.atan2(dy, dx)
        elif goal_yaw is not None:
            yaw = goal_yaw
        else:
            # last point, no goal yaw → keep same direction as previous segment
            yaw = math.atan2(
                float(traj_3d[i, 1]) - float(traj_3d[i - 1, 1]),
                float(traj_3d[i, 0]) - float(traj_3d[i - 1, 0]),
            )

        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        msg.poses.append(ps)
    return msg


def traj_to_marker(traj_3d, node, frame='map', marker_id=0,
                   color=None, width=0.1):
    if color is None:
        color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=1.0)
    m = Marker()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = frame
    m.ns = 'pct'
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = width
    m.color = color
    for pt in traj_3d:
        m.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
    return m


def sphere_marker(pos, node, marker_id, color, frame='map', radius=0.5):
    m = Marker()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = frame
    m.ns = 'pct_goal'
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2]) if len(pos) > 2 else 0.0
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = radius
    m.color = color
    return m


def yaw_from_quaternion(q):
    """Extract yaw from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ─── global planner node ────────────────────────────────────────────────────

class PctGlobalPlannerNode(Node):
    """Standalone PCT global planner — outputs /pct_path, no motion control."""

    def __init__(self):
        super().__init__('pct_global_planner')

        # ── parameters ──────────────────────────────────────────────────
        self.declare_parameter('tomo_path',
                               ROOT + '/rsc/tomogram/clinic.pickle')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('path_topic', '/pct_path')
        self.declare_parameter('astar_path_topic', '/pct_astar_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_odom_fallback', False)
        self.declare_parameter('tf_timeout_s', 0.2)
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('publish_tomogram', True)
        self.declare_parameter('tomogram_republish_period_s', 1.0)
        self.declare_parameter('save_trajectory', False)
        self.declare_parameter('goal_z_epsilon', 0.05)
        self.declare_parameter('infer_goal_z_from_tomogram', True)
        self.declare_parameter('goal_z_search_radius_cells', 2)
        self.declare_parameter('allow_new_goal_during_planning', True)

        # Resolve parameter values
        tomo_path_raw = self.get_parameter('tomo_path').value
        self.tomo_path = os.path.abspath(tomo_path_raw)
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.clicked_point_topic = self.get_parameter('clicked_point_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.astar_path_topic = self.get_parameter('astar_path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_odom_fallback = self.get_parameter('use_odom_fallback').value
        self.tf_timeout_s = self.get_parameter('tf_timeout_s').value
        self.publish_viz = self.get_parameter('publish_visualization').value
        self.publish_tomo = self.get_parameter('publish_tomogram').value
        self.tomo_period = self.get_parameter('tomogram_republish_period_s').value
        self.save_traj = self.get_parameter('save_trajectory').value
        self.goal_z_epsilon = self.get_parameter('goal_z_epsilon').value
        self.infer_goal_z_from_tomogram = (
            self.get_parameter('infer_goal_z_from_tomogram').value
        )
        self.goal_z_search_radius_cells = int(
            self.get_parameter('goal_z_search_radius_cells').value
        )
        self.allow_new_during = self.get_parameter('allow_new_goal_during_planning').value

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── odom fallback ───────────────────────────────────────────────
        self._latest_odom = None
        if self.use_odom_fallback:
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._odom_cb, 10)

        # ── publishers ──────────────────────────────────────────────────
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tomo_pub = self.create_publisher(PointCloud2, '/tomogram', 1)
        self.path_pub = self.create_publisher(Path, self.path_topic, path_qos)
        self.astar_path_pub = self.create_publisher(Path, self.astar_path_topic, path_qos)
        self.marker_pub = self.create_publisher(Marker, '/pct_marker', 1)
        self.load_tomo_srv = self.create_service(
            LoadTomogram, '~/load_tomogram', self._on_load_tomogram)

        # ── concurrent goal state ───────────────────────────────────────
        self._planning = False
        self._pending_goal = None   # (goal_x, goal_y, goal_z, goal_yaw)

        # ── tomo cache ──────────────────────────────────────────────────
        self._planner_lock = threading.RLock()
        self._tomo_data = None
        self._tomo_msg = None
        self._tomo_points_count = 0

        # ── Step 1: load planner ────────────────────────────────────────
        self.get_logger().info(f'Loading tomogram: {self.tomo_path}')
        self.planner = self._create_planner_for_tomogram(self.tomo_path)
        self.get_logger().info('Planner ready.')

        # ── Step 2: publish visualization ───────────────────────────────
        if self.publish_tomo:
            self._publish_tomo()
            self.tomo_timer = self.create_timer(self.tomo_period, self._republish_tomo)
            self.get_logger().info(
                f'Republishing /tomogram every {self.tomo_period}s for RViz.')

        # ── Step 3: subscribe to goal inputs ────────────────────────────
        self.create_subscription(
            PoseStamped, self.goal_pose_topic, self._on_goal_pose, 10)
        self.create_subscription(
            PointStamped, self.clicked_point_topic, self._on_clicked_point, 10)

        self.get_logger().info(
            '\n'
            '══════════════════════════════════════════════\n'
            ' PCT Global Planner — Online Mode\n'
            '──────────────────────────────────────────────\n'
            ' Send a goal via:\n'
            f'   RViz "2D Goal Pose" → {self.goal_pose_topic}\n'
            f'   RViz "Publish Point" → {self.clicked_point_topic}\n'
            f' Robot start pose: TF {self.global_frame} → {self.robot_frame}\n'
            f' Output path: {self.path_topic}\n'
            '══════════════════════════════════════════════'
        )
        self._marker_counter = 0

    # ── odom fallback ───────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

    def _create_planner_for_tomogram(self, tomo_path):
        tomo_path = os.path.abspath(tomo_path)
        tomo_dir = os.path.dirname(tomo_path) + os.sep
        tomo_name = os.path.splitext(os.path.basename(tomo_path))[0]
        planner = TomogramPlanner(PlanCfg())
        planner.tomo_dir = tomo_dir
        planner.loadTomogram(tomo_name)
        return planner

    def _replace_tomogram(self, tomo_path, planner):
        with self._planner_lock:
            self.planner = planner
            self.tomo_path = os.path.abspath(tomo_path)
            self._tomo_data = None
            self._tomo_msg = None
            self._tomo_points_count = 0

    def _on_load_tomogram(self, request, response):
        if not request.tomo_path:
            response.success = False
            response.message = 'tomo_path is empty'
            return response
        if not os.path.exists(request.tomo_path):
            response.success = False
            response.message = f'tomo_path does not exist: {request.tomo_path}'
            return response

        try:
            self.get_logger().info(
                f'Reloading tomogram for map={request.map_name}: {request.tomo_path}')
            new_planner = self._create_planner_for_tomogram(request.tomo_path)
            self._replace_tomogram(request.tomo_path, new_planner)
            if self.publish_tomo:
                self._publish_tomo(log=True)
            self._publish_empty_path('tomogram reloaded')
        except Exception as exc:
            response.success = False
            response.message = f'load tomogram failed: {exc}'
            return response

        response.success = True
        response.message = f'loaded tomogram for map: {request.map_name}'
        return response

    # ── goal callbacks ──────────────────────────────────────────────────────

    def _on_goal_pose(self, msg: PoseStamped):
        # Transform to global_frame if needed
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.global_frame, msg.header.frame_id,
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=self.tf_timeout_s))
                msg = do_transform_pose_stamped(msg, t)
            except TransformException as e:
                self.get_logger().error(
                    f'Cannot transform goal_pose from {msg.header.frame_id} '
                    f'to {self.global_frame}: {e}')
                self._publish_empty_path('goal_pose TF transform failed')
                return

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.position.z
        gyaw = yaw_from_quaternion(msg.pose.orientation)

        self._handle_goal(gx, gy, gz, gyaw, source='goal_pose')

    def _on_clicked_point(self, msg: PointStamped):
        # Transform to global_frame if needed
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.global_frame, msg.header.frame_id,
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=self.tf_timeout_s))
                # Simple transform for point
                px = msg.point.x + t.transform.translation.x
                py = msg.point.y + t.transform.translation.y
                pz = msg.point.z + t.transform.translation.z
                gx, gy, gz = px, py, pz
            except TransformException as e:
                self.get_logger().error(
                    f'Cannot transform clicked_point from {msg.header.frame_id} '
                    f'to {self.global_frame}: {e}')
                self._publish_empty_path('clicked_point TF transform failed')
                return
        else:
            gx = msg.point.x
            gy = msg.point.y
            gz = msg.point.z

        self._handle_goal(gx, gy, gz, None, source='clicked_point')

    def _handle_goal(self, gx, gy, gz, gyaw, source):
        """Queue a goal and start planning (or defer if busy)."""
        goal = (gx, gy, gz, gyaw, source)

        if self._planning:
            if self.allow_new_during:
                self._pending_goal = goal
                self.get_logger().info(
                    f'Planning in progress — queued new {source} goal as pending.')
            else:
                self.get_logger().warn(
                    f'Planning in progress — ignoring {source} goal '
                    '(allow_new_goal_during_planning=false).')
            return

        self._start_planning(goal)

    # ── planning ────────────────────────────────────────────────────────────

    def _start_planning(self, goal):
        gx, gy, gz, gyaw, source = goal
        self._planning = True
        self._pending_goal = None

        self.get_logger().info(
            f'Planning: goal=({gx:.2f}, {gy:.2f}, z={gz:.2f}) via {source}')

        # ── get robot start pose ────────────────────────────────────────
        start_xy, start_z = self._get_robot_pose()
        if start_xy is None:
            self.get_logger().error(
                'Cannot get robot pose (TF and odom fallback both failed).')
            self._publish_empty_path('no robot pose available')
            self._finish_planning()
            return

        sx, sy = start_xy
        self.get_logger().info(f'  Start: ({sx:.2f}, {sy:.2f}, z={start_z:.2f})')

        with self._planner_lock:
            # ── compute slices ───────────────────────────────────────────
            # RViz 2D goals often arrive with z=0 even when the target floor is
            # represented in the tomogram. Prefer the tomogram height at goal XY
            # so descending to a floor near z=0 does not get collapsed to start_z.
            effective_gz = gz
            if abs(gz) < self.goal_z_epsilon:
                inferred_gz = None
                if self.infer_goal_z_from_tomogram:
                    inferred_gz = self._infer_goal_z_from_tomogram(gx, gy, gz)
                if inferred_gz is not None:
                    effective_gz = inferred_gz
                    self.get_logger().info(
                        f'  Goal z ≈ 0, inferred tomogram z={effective_gz:.2f} '
                        'for slice lookup.')
                else:
                    effective_gz = start_z
                    self.get_logger().info(
                        f'  Goal z ≈ 0, using start z={start_z:.2f} '
                        'for slice lookup.')

            start_pos = np.array([sx, sy, start_z], dtype=np.float32)
            end_pos = np.array([gx, gy, effective_gz], dtype=np.float32)
            try:
                start_slice = self.planner.pos2layer(start_pos)
            except Exception:
                start_slice = 0
            try:
                end_slice = self.planner.pos2layer(end_pos)
            except Exception:
                end_slice = start_slice

            self.get_logger().info(f'  Slices: start={start_slice}, goal={end_slice}')

            # ── bounds check ─────────────────────────────────────────────
            if not self._point_in_bounds(sx, sy) or not self._point_in_bounds(gx, gy):
                self.get_logger().error(
                    f'Point out of tomogram bounds: start=({sx:.1f},{sy:.1f}), '
                    f'goal=({gx:.1f},{gy:.1f})')
                self._publish_empty_path('point out of tomogram bounds')
                self._finish_planning()
                return

            # ── run planner ──────────────────────────────────────────────
            t0 = time.time()
            traj = self.planner.plan(start_pos, end_pos)
            elapsed = time.time() - t0

            if traj is None:
                self.get_logger().error(
                    f'PCT found no path. Tried {elapsed:.1f}s. '
                    'Check start/goal positions and tomogram coverage.')
                self._publish_empty_path('PCT no path found')
                self._finish_planning()
                return

            astar_path = self.planner.getLastAstarPath()

        self.get_logger().info(
            f'Path found: {traj.shape[0]} waypoints in {elapsed:.1f}s')

        # ── publish A* raw path ──────────────────────────────────────────
        if astar_path is not None and len(astar_path) > 0:
            self.astar_path_pub.publish(
                traj_to_path(astar_path, self, frame=self.global_frame))
            if self.publish_viz:
                self.marker_pub.publish(
                    traj_to_marker(
                        astar_path, self, frame=self.global_frame,
                        marker_id=self._next_marker_id(),
                        color=ColorRGBA(r=0.1, g=0.45, b=1.0, a=1.0),
                        width=0.07))
            self.get_logger().info(
                f'Raw A* path: {astar_path.shape[0]} waypoints')
            self.get_logger().info(
                'Publishing raw A* path on /pct_astar_path for debug.')

        # ── publish path ────────────────────────────────────────────────
        self.path_pub.publish(
            traj_to_path(traj, self, frame=self.global_frame, goal_yaw=gyaw))
        if self.publish_viz:
            self.marker_pub.publish(
                traj_to_marker(traj, self, frame=self.global_frame,
                               marker_id=self._next_marker_id()))
            # Goal sphere
            self.marker_pub.publish(
                sphere_marker([gx, gy, effective_gz], self,
                              marker_id=self._next_marker_id(),
                              color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)))

        # ── optional save ────────────────────────────────────────────────
        if self.save_traj:
            stem = os.path.splitext(os.path.basename(self.tomo_path))[0]
            out = ROOT + f'/rsc/{stem}_traj_{int(time.time())}.npy'
            np.save(out, traj)
            self.get_logger().info(f'Saved trajectory to {out}')

        self._finish_planning()

    def _finish_planning(self):
        """Mark planning done; start pending goal if any."""
        self._planning = False
        if self._pending_goal is not None:
            pending = self._pending_goal
            self._pending_goal = None
            self.get_logger().info('Starting pending goal…')
            self._start_planning(pending)

    # ── robot pose ──────────────────────────────────────────────────────────

    def _get_robot_pose(self):
        """
        Get robot (x, y, z) in global_frame via TF.
        Returns ((x, y), z) or (None, None) on failure.
        Falls back to odom if configured.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self.tf_timeout_s))
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            rz = t.transform.translation.z
            return (rx, ry), rz
        except TransformException as e:
            self.get_logger().debug(f'TF lookup failed: {e}')

        # odom fallback
        if self.use_odom_fallback and self._latest_odom is not None:
            odom = self._latest_odom
            self.get_logger().warn('Using odom fallback for robot pose.')
            return (
                (odom.pose.pose.position.x, odom.pose.pose.position.y),
                odom.pose.pose.position.z,
            )

        return None, None

    # ── empty path for failure ──────────────────────────────────────────────

    def _publish_empty_path(self, reason=''):
        """Publish an empty path to signal planning failure."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        self.path_pub.publish(msg)
        if reason:
            self.get_logger().warn(f'Published empty /pct_path ({reason}).')

    # ── slice mapping ───────────────────────────────────────────────────────

    def _z_to_slice(self, x, y, z):
        """Map a 3D world point to the best-matching tomogram slice index."""
        if self._tomo_data is None:
            with open(self.tomo_path, 'rb') as f:
                self._tomo_data = pickle.load(f)

        d = self._tomo_data
        elev_g = np.asarray(d['data'][3], dtype=np.float32)  # (n_slice, dim_x, dim_y)
        res = float(d['resolution'])
        ctr = np.asarray(d['center'], dtype=np.float32)
        n_slice, dim_x, dim_y = elev_g.shape
        ox, oy = dim_x // 2, dim_y // 2

        ix = int(round((x - float(ctr[0])) / res)) + ox
        iy = int(round((y - float(ctr[1])) / res)) + oy
        ix = int(np.clip(ix, 0, dim_x - 1))
        iy = int(np.clip(iy, 0, dim_y - 1))

        heights = elev_g[:, ix, iy]

        best_slice = 0
        best_diff = float('inf')
        for s in range(n_slice):
            h = float(heights[s])
            if h < -50:
                continue
            if h <= z + 0.3:
                diff = z - h
                if diff < best_diff:
                    best_diff = diff
                    best_slice = s

        return best_slice

    def _infer_goal_z_from_tomogram(self, x, y, reference_z):
        """Infer a target height from nearby tomogram elevation cells."""
        if self.planner.elev_g is None:
            return None

        try:
            idx = self.planner.pos2array_idx([x, y])
        except Exception as exc:
            self.get_logger().debug(f'Cannot index tomogram goal XY: {exc}')
            return None

        if (
            idx[0] < 0 or idx[0] >= self.planner.map_dim[0] or
            idx[1] < 0 or idx[1] >= self.planner.map_dim[1]
        ):
            return None

        radius = max(0, self.goal_z_search_radius_cells)
        x0 = max(0, idx[0] - radius)
        x1 = min(self.planner.map_dim[0], idx[0] + radius + 1)
        y0 = max(0, idx[1] - radius)
        y1 = min(self.planner.map_dim[1], idx[1] + radius + 1)

        local_elev = self.planner.elev_g[:, x0:x1, y0:y1]
        finite = np.isfinite(local_elev)
        if not np.any(finite):
            return None

        scores = np.abs(local_elev - reference_z)
        scores[~finite] = np.inf
        return float(local_elev[np.unravel_index(np.argmin(scores), scores.shape)])

    def _point_in_bounds(self, x, y):
        """Check whether (x, y) falls within the tomogram grid."""
        if self._tomo_data is None:
            with open(self.tomo_path, 'rb') as f:
                self._tomo_data = pickle.load(f)
        d = self._tomo_data
        res = float(d['resolution'])
        ctr = np.asarray(d['center'], dtype=np.float32)
        _, dim_x, dim_y = np.asarray(d['data'][3], dtype=np.float32).shape
        ox, oy = dim_x // 2, dim_y // 2
        ix = int(round((x - float(ctr[0])) / res)) + ox
        iy = int(round((y - float(ctr[1])) / res)) + oy
        return 0 <= ix < dim_x and 0 <= iy < dim_y

    # ── visualization publishers ────────────────────────────────────────────

    def _build_tomo_msg(self):
        with open(self.tomo_path, 'rb') as f:
            self._tomo_data = pickle.load(f)

        d = self._tomo_data
        tomo = np.asarray(d['data'], dtype=np.float32)
        trav = tomo[0].copy()
        elev = tomo[3].copy()
        res = float(d['resolution'])
        ctr = np.asarray(d['center'], dtype=np.float32)
        slice_dh = float(d['slice_dh'])
        n_sl, dim_x, dim_y = trav.shape
        ox, oy = dim_x // 2, dim_y // 2

        # Match the original ROS1 tomography visualization: hide lower
        # overlapping slices and carry their cost onto the visible layer.
        for s in range(n_sl - 1):
            hidden = (elev[s + 1] - elev[s]) < slice_dh
            elev[s, hidden] = np.nan
            trav[s + 1, hidden] = np.minimum(trav[s, hidden], trav[s + 1, hidden])

        all_pts = []
        for s in range(n_sl):
            g = elev[s]
            t = trav[s]
            mask = np.isfinite(g)
            ix, iy = np.where(mask)
            if len(ix) == 0:
                continue
            wx = (ix - ox) * res + ctr[0]
            wy = (iy - oy) * res + ctr[1]
            wz = g[mask]
            wt = t[mask]
            layer = np.stack([wx, wy, wz, wt], axis=1).astype(np.float32)
            all_pts.append(layer)

        if not all_pts:
            raise RuntimeError('Tomogram contains no valid elevation cells.')

        pts4 = np.concatenate(all_pts, 0)
        self._tomo_points_count = len(pts4)
        self._tomo_msg = make_pc2(self, pts4, fields_xyz=False, frame=self.global_frame)

    def _publish_tomo(self, log=True):
        with self._planner_lock:
            if self._tomo_msg is None:
                self._build_tomo_msg()
            self._tomo_msg.header.stamp = self.get_clock().now().to_msg()
            self.tomo_pub.publish(self._tomo_msg)
            if log:
                self.get_logger().info(f'Published {self._tomo_points_count} tomogram points')

    def _republish_tomo(self):
        self._publish_tomo(log=False)

    def _next_marker_id(self):
        self._marker_counter += 1
        return self._marker_counter


# ─── entry point ─────────────────────────────────────────────────────────────

def main():
    rclpy.init(args=sys.argv)

    node = None
    try:
        node = PctGlobalPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.get_logger().info('Stopped by user.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
