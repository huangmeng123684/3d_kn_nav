import argparse
import os
import pickle
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from planner_wrapper import TomogramPlanner
from utils import traj2ros

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, '..'))
from config import Config


SCENES = {
    'Spiral': 'spiral0.3_2',
    'Building': 'global_ground_map_floor2',
    'Plaza': 'plaza3_10',
}

POINT_FIELDS_XYZI = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('intensity', 12, PointField.FLOAT32, 1),
]


class RvizPlannerNode(object):
    def __init__(self, scene, tomo_file):
        self.lock = threading.Lock()
        self.start_pos = None
        self.goal_pos = None
        self.waiting_for_start = True

        self.path_pub = rospy.Publisher('/pct_path', Path, latch=True, queue_size=1)
        self.astar_path_pub = rospy.Publisher(
            '/pct_astar_path', Path, latch=True, queue_size=1
        )
        self.tomogram_pub = rospy.Publisher('/tomogram', PointCloud2, latch=True, queue_size=1)
        self.marker_pub = rospy.Publisher(
            '/pct_start_goal_markers', MarkerArray, latch=True, queue_size=1
        )

        cfg = Config()
        self.planner = TomogramPlanner(cfg)
        rospy.loginfo('Loading tomogram: %s', tomo_file)
        self.planner.loadTomogram(tomo_file)
        rospy.loginfo('Tomogram loaded for scene: %s', scene)
        self.publish_tomogram(tomo_file)

        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_cb, queue_size=1)

    def publish_tomogram(self, tomo_file):
        tomo_path = os.path.join(self.planner.tomo_dir, tomo_file + '.pickle')
        with open(tomo_path, 'rb') as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict['data'], dtype=np.float32)
        layers_t = tomogram[0].copy()
        layers_g = tomogram[3].copy()
        resolution = float(data_dict['resolution'])
        center = np.asarray(data_dict['center'], dtype=np.float32)
        slice_dh = float(data_dict['slice_dh'])
        n_slice, dim_x, dim_y = layers_g.shape

        index_proto, point_proto = self.grid_points_xyzi(resolution, dim_x, dim_y)
        point_proto[:, :2] += center

        global_points = []
        for i in range(n_slice - 1):
            mask_h = (layers_g[i + 1] - layers_g[i]) < slice_dh
            layers_g[i, mask_h] = np.nan
            layers_t[i + 1, mask_h] = np.minimum(layers_t[i, mask_h], layers_t[i + 1, mask_h])
            global_points.append(self.layer_to_points(point_proto, index_proto, layers_g[i], layers_t[i]))

        global_points.append(self.layer_to_points(point_proto, index_proto, layers_g[-1], layers_t[-1]))
        global_points = [points for points in global_points if points.size > 0]
        if not global_points:
            rospy.logwarn('Tomogram visualization has no valid points')
            return

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, np.concatenate(global_points, axis=0))
        self.tomogram_pub.publish(points_msg)
        rospy.loginfo('Tomogram published on /tomogram')

    @staticmethod
    def grid_points_xyzi(resolution, dim_x, dim_y):
        index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
        lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
        ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
        ix, iy = np.meshgrid(lx, ly)
        index_proto[:, 0] = ix.flatten()
        index_proto[:, 1] = iy.flatten()

        point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
        point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
        point_proto[:, 0] -= 0.5 * dim_x
        point_proto[:, 1] -= 0.5 * dim_y
        point_proto[:, :2] *= resolution
        point_proto[:, 3] = 1.0

        return index_proto, point_proto

    @staticmethod
    def layer_to_points(point_proto, index_proto, height_layer, cost_layer):
        layer_points = point_proto.copy()
        layer_points[:, 2] = height_layer[index_proto[:, 0], index_proto[:, 1]]
        layer_points[:, 3] = cost_layer[index_proto[:, 0], index_proto[:, 1]]
        return layer_points[~np.isnan(layer_points).any(axis=-1)]

    def clicked_point_cb(self, msg):
        pos = msg.point
        with self.lock:
            if self.waiting_for_start:
                self.start_pos = np.array([pos.x, pos.y, pos.z], dtype=np.float32)
                self.goal_pos = None
                self.waiting_for_start = False
                self.clear_path()
                rospy.loginfo(
                    'Start set from /clicked_point: x=%.3f, y=%.3f, z=%.3f',
                    pos.x, pos.y, pos.z
                )
                self.publish_markers()
                return

            self.goal_pos = np.array([pos.x, pos.y, pos.z], dtype=np.float32)
            self.waiting_for_start = True
            rospy.loginfo(
                'Goal set from /clicked_point: x=%.3f, y=%.3f, z=%.3f',
                pos.x, pos.y, pos.z
            )
            self.publish_markers()
            self.plan_if_ready()

    def clear_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(path_msg)
        self.astar_path_pub.publish(path_msg)

    def plan_if_ready(self):
        if self.start_pos is None or self.goal_pos is None:
            rospy.loginfo('Waiting for both start and goal from RViz')
            return

        rospy.loginfo('Planning from %s to %s', self.start_pos, self.goal_pos)
        traj_3d = self.planner.plan(self.start_pos, self.goal_pos)
        if traj_3d is None:
            rospy.logwarn('Planning failed: no path found')
            return

        if self.planner.last_astar_traj is not None:
            astar_path_msg = traj2ros(self.planner.last_astar_traj)
            astar_path_msg.header.stamp = rospy.Time.now()
            for pose in astar_path_msg.poses:
                pose.header.stamp = astar_path_msg.header.stamp
            self.astar_path_pub.publish(astar_path_msg)
            rospy.loginfo(
                'Raw A* path published on /pct_astar_path with %d poses',
                len(astar_path_msg.poses)
            )

        path_msg = traj2ros(traj_3d)
        path_msg.header.stamp = rospy.Time.now()
        for pose in path_msg.poses:
            pose.header.stamp = path_msg.header.stamp
        self.path_pub.publish(path_msg)
        rospy.loginfo('Trajectory published on /pct_path with %d poses', len(path_msg.poses))

    def publish_markers(self):
        markers = MarkerArray()
        now = rospy.Time.now()
        markers.markers.append(self.make_delete_marker(0, now))
        markers.markers.append(self.make_delete_marker(1, now))

        if self.start_pos is not None:
            markers.markers.append(
                self.make_sphere_marker(0, now, self.start_pos, 'start', (0.1, 0.7, 0.2))
            )
        if self.goal_pos is not None:
            markers.markers.append(
                self.make_sphere_marker(1, now, self.goal_pos, 'goal', (0.9, 0.1, 0.1))
            )
        self.marker_pub.publish(markers)

    @staticmethod
    def make_delete_marker(marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'pct_start_goal'
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def make_sphere_marker(marker_id, stamp, pos, name, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'pct_start_goal'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2]) if len(pos) > 2 else 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = name
        return marker


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scene',
        type=str,
        default='Building',
        choices=sorted(SCENES.keys()),
        help='Name of the scene.',
    )
    parser.add_argument(
        '--tomo-file',
        type=str,
        default=None,
        help='Tomogram pickle name without .pickle. Overrides --scene.',
    )
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    rospy.init_node('pct_rviz_planner', anonymous=True)
    tomo_file = args.tomo_file if args.tomo_file else SCENES[args.scene]
    RvizPlannerNode(args.scene, tomo_file)
    rospy.loginfo('Use Publish Point: odd clicks set start, even clicks set goal.')
    rospy.spin()
