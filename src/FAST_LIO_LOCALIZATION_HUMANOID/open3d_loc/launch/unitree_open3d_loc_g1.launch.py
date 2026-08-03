from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import glob
import os
import time


LOG_KEEP_DAYS = 7
LOG_MAX_FILES = 50


def clean_open3d_logs(log_dir):
    if not os.path.isdir(log_dir):
        return

    now = time.time()
    keep_seconds = LOG_KEEP_DAYS * 24 * 60 * 60
    log_files = []

    for name in os.listdir(log_dir):
        path = os.path.join(log_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        if now - mtime > keep_seconds:
            try:
                os.remove(path)
            except OSError:
                pass
            continue
        log_files.append((mtime, path))

    log_files.sort(reverse=True)
    for _, path in log_files[LOG_MAX_FILES:]:
        try:
            os.remove(path)
        except OSError:
            pass


def generate_launch_description():
    # 获取包路径
    open3d_loc_share = get_package_share_directory('open3d_loc')
    workspace_root = open3d_loc_share.split('/install/')[0] if '/install/' in open3d_loc_share else ''
    source_candidates = glob.glob(os.path.join(workspace_root, 'src', '**', 'open3d_loc'), recursive=True)
    open3d_loc_dir = source_candidates[0] if source_candidates else open3d_loc_share
    log_dir = os.path.join(open3d_loc_dir, 'log')
    os.makedirs(log_dir, exist_ok=True)
    clean_open3d_logs(log_dir)

    ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    # 配置文件路径
    config_file = os.path.join(open3d_loc_share, 'config', 'loc_param_g1_unitree.yaml')

    # 全局定位节点
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='both',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    localization_service_node = Node(
        package='open3d_loc',
        executable='localization_service_node',
        name='localization_service_node',
        output='both',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    return LaunchDescription([
        ros_log_dir,
        use_sim_time_arg,
        global_localization_node,
        localization_service_node,
    ])
