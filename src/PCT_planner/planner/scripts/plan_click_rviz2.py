#!/usr/bin/env python3
"""Interactive RViz2 runner for planning from two Publish Point clicks."""
import argparse
import ctypes
import os
import pickle
import subprocess
import sys

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, '../..'))

LIB_PATH = os.path.join(PLANNER_ROOT, 'lib')
for library in (
    'libmetis-gtsam.so',
    'libgtsam.so.4',
    'libcommon_smoothing.so',
):
    ctypes.CDLL(os.path.join(LIB_PATH, library), mode=ctypes.RTLD_GLOBAL)

sys.path.insert(0, PLANNER_ROOT)
sys.path.insert(0, os.path.join(PLANNER_ROOT, 'lib'))

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from config import Config
from planner_wrapper import TomogramPlanner


def resolve_tomo_path(tomo_arg):
    if tomo_arg.endswith('.pickle') or os.path.sep in tomo_arg:
        path = os.path.abspath(tomo_arg)
    else:
        path = os.path.join(PACKAGE_ROOT, 'rsc/tomogram', tomo_arg + '.pickle')

    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return path


def make_pointcloud2(node, points, has_intensity=False, frame_id='map'):
    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.is_dense = True

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    if has_intensity:
        msg.fields.append(
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        )
        msg.point_step = 16

    msg.row_step = msg.point_step * len(points)
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


def traj_to_path(node, traj, frame_id='map'):
    msg = Path()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    for pt in traj:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(pt[0])
        pose.pose.position.y = float(pt[1])
        pose.pose.position.z = float(pt[2])
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def sphere_marker(node, marker_id, xyz, rgba, frame_id='map'):
    marker = Marker()
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.header.frame_id = frame_id
    marker.ns = 'pct_clicks'
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(xyz[0])
    marker.pose.position.y = float(xyz[1])
    marker.pose.position.z = float(xyz[2])
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.55
    marker.scale.y = 0.55
    marker.scale.z = 0.55
    marker.color = rgba
    return marker


def path_marker(
    node,
    traj,
    frame_id='map',
    ns='pct_path',
    marker_id=1,
    rgba=None,
    width=0.12,
):
    if rgba is None:
        rgba = ColorRGBA(r=0.0, g=1.0, b=0.25, a=1.0)
    marker = Marker()
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.color = rgba
    for pt in traj:
        marker.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
    return marker


def delete_marker(node, ns, marker_id, frame_id='map'):
    marker = Marker()
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.action = Marker.DELETE
    return marker


class ClickPlannerNode(Node):
    def __init__(self, tomo_path, frame_id='map', publish_period=1.0):
        super().__init__('pct_click_planner')
        self.frame_id = frame_id
        self.tomo_path = tomo_path
        self.tomo_name = os.path.splitext(os.path.basename(tomo_path))[0]
        self.clicks = []
        self.click_z_epsilon = 0.05
        self.z_search_radius_cells = 2

        self.tomo_pub = self.create_publisher(PointCloud2, '/tomogram', 1)
        self.path_pub = self.create_publisher(Path, '/pct_path', 1)
        self.astar_path_pub = self.create_publisher(Path, '/pct_astar_path', 1)
        self.marker_pub = self.create_publisher(Marker, '/pct_marker', 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)

        self.get_logger().info(f'Loading tomogram: {self.tomo_path}')
        self.tomo_data = self.load_tomo_data(self.tomo_path)
        self.tomo_msg = self.build_tomo_cloud()

        cfg = Config()
        self.planner = TomogramPlanner(cfg)
        self.planner.tomo_dir = os.path.dirname(self.tomo_path) + os.sep
        self.planner.loadTomogram(self.tomo_name)

        self.publish_tomo(log=True)
        self.create_timer(publish_period, self.publish_tomo)

        self.get_logger().info(
            'RViz ready: select "Publish Point", click start once, then click goal.'
        )

    def load_tomo_data(self, tomo_path):
        with open(tomo_path, 'rb') as handle:
            data = pickle.load(handle)
        data['data'] = np.asarray(data['data'], dtype=np.float32)
        data['resolution'] = float(data['resolution'])
        data['center'] = np.asarray(data['center'], dtype=np.float32)
        return data

    def build_tomo_cloud(self):
        tomogram = self.tomo_data['data']
        traversability = tomogram[0].copy()
        elevation = tomogram[3].copy()
        resolution = self.tomo_data['resolution']
        center = self.tomo_data['center']
        slice_dh = float(self.tomo_data['slice_dh'])

        n_slice, dim_x, dim_y = traversability.shape
        offset_x = dim_x // 2
        offset_y = dim_y // 2
        clouds = []

        for slice_idx in range(n_slice - 1):
            hidden = (elevation[slice_idx + 1] - elevation[slice_idx]) < slice_dh
            elevation[slice_idx, hidden] = np.nan
            traversability[slice_idx + 1, hidden] = np.minimum(
                traversability[slice_idx, hidden],
                traversability[slice_idx + 1, hidden],
            )

        for slice_idx in range(n_slice):
            height_layer = elevation[slice_idx]
            cost_layer = traversability[slice_idx]
            valid = np.isfinite(height_layer)
            grid_x, grid_y = np.where(valid)
            if len(grid_x) == 0:
                continue

            world_x = (grid_x - offset_x) * resolution + center[0]
            world_y = (grid_y - offset_y) * resolution + center[1]
            world_z = height_layer[valid]
            intensity = cost_layer[valid]
            clouds.append(np.stack([world_x, world_y, world_z, intensity], axis=1))

        if not clouds:
            raise RuntimeError('Tomogram contains no valid elevation cells.')

        points = np.concatenate(clouds, axis=0).astype(np.float32)
        self.get_logger().info(f'Tomogram cloud points: {len(points)}')
        return make_pointcloud2(self, points, has_intensity=True, frame_id=self.frame_id)

    def publish_tomo(self, log=False):
        self.tomo_msg.header.stamp = self.get_clock().now().to_msg()
        self.tomo_pub.publish(self.tomo_msg)
        if log:
            self.get_logger().info('Published /tomogram')

    def on_clicked_point(self, msg):
        xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float32)

        if len(self.clicks) == 0:
            self.marker_pub.publish(delete_marker(self, 'pct_path', 1, self.frame_id))
            self.marker_pub.publish(delete_marker(self, 'pct_astar_path', 2, self.frame_id))
            self.clicks.append(xyz)
            self.marker_pub.publish(
                sphere_marker(
                    self, 100, xyz, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), self.frame_id
                )
            )
            self.get_logger().info(
                f'Start selected: x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}'
            )
            return

        self.clicks.append(xyz)
        self.marker_pub.publish(
            sphere_marker(
                self, 101, xyz, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), self.frame_id
            )
        )
        self.get_logger().info(
            f'Goal selected: x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}'
        )
        self.plan_from_clicks()
        self.clicks.clear()

    def plan_from_clicks(self):
        start_xyz = np.array(self.clicks[0], dtype=np.float32, copy=True)
        goal_xyz = self.infer_goal_height(self.clicks[1])

        start_layer = self.find_slice(start_xyz)
        goal_layer = self.find_slice(goal_xyz)
        self.get_logger().info(
            f'Planning: start_slice={start_layer}, goal_slice={goal_layer}'
        )
        self.get_logger().info(
            f'  Start: ({start_xyz[0]:.3f}, {start_xyz[1]:.3f}, '
            f'z={start_xyz[2]:.3f})'
        )
        self.get_logger().info(
            f'  Goal: ({goal_xyz[0]:.3f}, {goal_xyz[1]:.3f}, '
            f'z={goal_xyz[2]:.3f})'
        )

        traj = self.planner.plan(start_xyz, goal_xyz)
        if traj is None:
            self.get_logger().warn('No path found. Click another start/goal pair.')
            return

        astar_path = self.get_astar_path()
        if astar_path is not None and len(astar_path) > 0:
            self.astar_path_pub.publish(traj_to_path(self, astar_path, self.frame_id))
            self.marker_pub.publish(
                path_marker(
                    self,
                    astar_path,
                    self.frame_id,
                    ns='pct_astar_path',
                    marker_id=2,
                    rgba=ColorRGBA(r=0.1, g=0.45, b=1.0, a=1.0),
                    width=0.07,
                )
            )
            self.get_logger().info(f'Raw A* path published: {astar_path.shape[0]} waypoints')

        self.path_pub.publish(traj_to_path(self, traj, self.frame_id))
        self.marker_pub.publish(path_marker(self, traj, self.frame_id))

        out = os.path.join(PACKAGE_ROOT, 'rsc', self.tomo_name + '_click_traj.npy')
        np.save(out, traj)
        self.get_logger().info(
            f'Path published: {traj.shape[0]} waypoints, '
            f'z_span={self.z_span(traj):.3f}, saved to {out}'
        )

    def infer_goal_height(self, xyz):
        effective_xyz = np.array(xyz, dtype=np.float32, copy=True)
        if abs(float(effective_xyz[2])) >= self.click_z_epsilon:
            return effective_xyz

        inferred_z = self.infer_goal_z_from_tomogram(
            float(effective_xyz[0]),
            float(effective_xyz[1]),
            float(effective_xyz[2]),
        )
        if inferred_z is None:
            return effective_xyz

        effective_xyz[2] = inferred_z
        self.get_logger().info(
            f'Goal z ~= 0, inferred tomogram z={inferred_z:.3f} '
            'for slice lookup.'
        )
        return effective_xyz

    def infer_goal_z_from_tomogram(self, x, y, reference_z):
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

        radius = self.z_search_radius_cells
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

    @staticmethod
    def z_span(traj):
        if traj is None or len(traj) == 0:
            return 0.0
        z = np.asarray(traj)[:, 2]
        finite = np.isfinite(z)
        if not np.any(finite):
            return 0.0
        return float(z[finite].max() - z[finite].min())

    def get_astar_path(self):
        if hasattr(self.planner, 'getLastAstarPath'):
            return self.planner.getLastAstarPath()
        return getattr(self.planner, 'last_astar_traj', None)

    def find_slice(self, xyz):
        elevation = self.tomo_data['data'][3]
        resolution = self.tomo_data['resolution']
        center = self.tomo_data['center']
        n_slice, dim_x, dim_y = elevation.shape
        offset_x = dim_x // 2
        offset_y = dim_y // 2

        ix = int(round((float(xyz[0]) - float(center[0])) / resolution)) + offset_x
        iy = int(round((float(xyz[1]) - float(center[1])) / resolution)) + offset_y
        if ix < 0 or ix >= dim_x or iy < 0 or iy >= dim_y:
            return self.z_to_slice_layer(float(xyz[2]))

        search_radius = 2
        x0 = max(0, ix - search_radius)
        x1 = min(dim_x, ix + search_radius + 1)
        y0 = max(0, iy - search_radius)
        y1 = min(dim_y, iy + search_radius + 1)

        local_elev = elevation[:, x0:x1, y0:y1]
        finite = np.isfinite(local_elev)
        if not np.any(finite):
            return self.z_to_slice_layer(float(xyz[2]))

        scores = np.abs(local_elev - float(xyz[2]))
        scores[~finite] = np.inf
        return int(np.unravel_index(np.argmin(scores), scores.shape)[0])

    def z_to_slice_layer(self, z):
        layer = int(round((z - float(self.tomo_data['slice_h0'])) / float(self.tomo_data['slice_dh'])))
        return int(np.clip(layer, 0, self.tomo_data['data'][3].shape[0] - 1))


def launch_rviz(rviz_config):
    try:
        return subprocess.Popen(['rviz2', '-d', rviz_config])
    except FileNotFoundError:
        print('[WARN] rviz2 not found. Start RViz manually and load:', rviz_config)
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--tomo',
        type=str,
        default='global_ground_map_floor2',
        help='Tomogram stem in rsc/tomogram/, or an absolute/relative .pickle path.',
    )
    parser.add_argument('--frame-id', type=str, default='map')
    parser.add_argument('--no-rviz', action='store_true', help='Do not start RViz2.')
    parser.add_argument(
        '--rviz-config',
        type=str,
        default=os.path.join(PACKAGE_ROOT, 'rsc/rviz/pct_ros2.rviz'),
    )
    args = parser.parse_args()

    tomo_path = resolve_tomo_path(args.tomo)
    rviz_proc = None

    rclpy.init()
    node = ClickPlannerNode(tomo_path, frame_id=args.frame_id)

    if not args.no_rviz:
        rviz_proc = launch_rviz(os.path.abspath(args.rviz_config))

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if rviz_proc is not None:
            rviz_proc.terminate()


if __name__ == '__main__':
    main()
