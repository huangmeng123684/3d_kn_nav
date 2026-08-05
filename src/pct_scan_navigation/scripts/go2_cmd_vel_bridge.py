#!/usr/bin/env python3
# coding=utf-8
"""Go2 cmd_vel bridge（ROS2 / Humble，Python + WebRTC）。

替代被跳过的 C++ go2_cmd_vel_bridge（pure_pursuit_planner）。
ROS 接口与 Go2SafetyController 语义与 C++ 版完全一致，仅传输层从 DDS 换成
unitree_webrtc_connect（LocalSTA 直连狗）：

  订阅 /cmd_vel                    Twist      上游速度指令
  订阅 /Odometry_open3d            Odometry   心跳（只做 isfinite 校验）
  服务 /go2_cmd_vel_bridge/enable  SetBool    默认禁用，需手动 enable
  发布 /go2_cmd_vel_bridge/armed   Bool       使能状态（transient_local）
  发布 /go2_cmd_vel_bridge/safe_cmd_vel  Twist 安全输出（监控用）
  20Hz 定时器                               control tick

webrtc_sport_client.py 保持与 pure_pursuit_planner 原文件逐字节一致，不改动；
假 rospy 注入、sportmodestate 心跳订阅、0.5s 运动 RPC 超时都落在这里。
"""

import math
import os
import sys
import time

# ── rospy 兼容（test_new.py 同款手法，见 scripts/test_new.py）──
# webrtc_sport_client.py 里 `import rospy` 是硬依赖；kn_nav 容器是纯 ROS2 无
# rospy。这里：容器里有真 rospy 就用真，没有就注入一个只带日志的假模块，
# 并写进 sys.modules 让后续 `import rospy` 命中。
try:
    import rospy  # noqa: F401  真 rospy（若容器里有）
except ImportError:
    import types

    _rospy_shim = types.ModuleType('rospy')

    def _make_logger(level):
        def _log(msg, *args, **kwargs):
            print(f'[{level}] {msg}')

        return _log

    _rospy_shim.loginfo = _make_logger('INFO')
    _rospy_shim.logwarn = _make_logger('WARN')
    _rospy_shim.logerr = _make_logger('ERR')
    sys.modules['rospy'] = _rospy_shim
    rospy = _rospy_shim

# 同目录脚本互相 import（与 nav_manager_node.py 同一约定）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from unitree_webrtc_connect.constants import RTC_TOPIC  # noqa: E402

from go2_safety_controller import (  # noqa: E402
    Go2SafetyConfig,
    Go2SafetyController,
    Go2VelocityCommand,
)
from webrtc_sport_client import WebRTCSportClient  # noqa: E402


class FastWebRTCSportClient(WebRTCSportClient):
    """收紧运动 RPC 超时到 0.5s。

    webrtc_sport_client.py 保持原样不改（其 _call 用 bridge.run 默认 10s）。
    这里子类重写 _call：狗断连时 Move/StopMove 0.5s 内返回失败，安全控制器
    才能 0.5s 级 disarm（对齐 C++ sdk_timeout=0.5）。Init() 的 connect() 不
    经过 _call，仍用 10s，不受影响。
    """

    def _call(self, api_id, parameter=None):
        try:
            self.bridge.run(self._sport(api_id, parameter), timeout=0.5)
            return 0
        except Exception as e:
            rospy.logerr(f'WebRTC 命令失败 api_id={api_id}: {e}')
            return -1


class Go2CmdVelBridge(Node):
    def __init__(self):
        super().__init__('go2_cmd_vel_bridge')

        self._throttle_marks = {}

        # ── 参数（照抄 go2_bridge.yaml，robot_ip 替代 C++ 的 network_interface）──
        # use_sim_time 不手动声明：rclpy 的 TimeSource 在节点构造时已自动声明
        # （attach_node 内 has_parameter 检查后 declare_parameter('use_sim_time', False)），
        # 再声明一次会抛 ParameterAlreadyDeclaredException。
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('min_vx', 0.0)
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('max_abs_vy', 0.0)
        self.declare_parameter('max_abs_vyaw', 0.5)
        self.declare_parameter('max_linear_acceleration', 0.25)
        self.declare_parameter('max_yaw_acceleration', 0.5)
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('odometry_timeout', 0.3)
        self.declare_parameter('sport_state_timeout', 0.5)

        robot_ip = self.get_parameter('robot_ip').value
        if not robot_ip:
            raise ValueError('robot_ip is required (Go2 STA IP, e.g. 192.168.123.161)')

        control_rate = self.get_parameter('control_rate').value
        if not (control_rate > 0.0):
            raise ValueError('control_rate must be positive')

        config = Go2SafetyConfig(
            min_vx=self.get_parameter('min_vx').value,
            max_vx=self.get_parameter('max_vx').value,
            max_abs_vy=self.get_parameter('max_abs_vy').value,
            max_abs_vyaw=self.get_parameter('max_abs_vyaw').value,
            max_linear_acceleration=self.get_parameter(
                'max_linear_acceleration').value,
            max_yaw_acceleration=self.get_parameter('max_yaw_acceleration').value,
            command_timeout=self.get_parameter('command_timeout').value,
            odometry_timeout=self.get_parameter('odometry_timeout').value,
            sport_state_timeout=self.get_parameter('sport_state_timeout').value,
        )
        if not (config.min_vx <= config.max_vx):
            raise ValueError('min_vx must not exceed max_vx')

        # ── WebRTC 客户端。Init() 是阻塞的：连不上会在超时后抛异常，
        #    构造失败 → main() 记 FATAL 退出（对齐 C++ 构造失败即崩）。──
        self._client = None
        self._controller = None
        try:
            self._client = FastWebRTCSportClient(ip=robot_ip)
            self._client.Init()
        except Exception:
            if self._client is not None:
                try:
                    self._client.cleanup()
                except Exception:
                    pass
            raise

        self._controller = Go2SafetyController(self._client, config)

        # ── sportmodestate 心跳（替代 C++ 的 DDS rt/sportmodestate）。
        #    回调在 asyncio 后台线程触发，controller 内部有锁，线程安全。──
        async def _subscribe_sport_state():
            self._client.conn.datachannel.pub_sub.subscribe(
                RTC_TOPIC['LF_SPORT_MOD_STATE'],
                lambda message: self._controller.updateSportStateHeartbeat(
                    time.monotonic()
                ),
            )

        self._client.bridge.run(_subscribe_sport_state())

        # ── ROS 接口（与 C++ 版完全一致）──
        armed_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_armed = self.create_publisher(
            Bool, '/go2_cmd_vel_bridge/armed', armed_qos)
        self._pub_safe_cmd = self.create_publisher(
            Twist, '/go2_cmd_vel_bridge/safe_cmd_vel', 10)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._command_cb, 10)
        self._sub_odom = self.create_subscription(
            Odometry, '/Odometry_open3d', self._odometry_cb, 10)
        self._srv_enable = self.create_service(
            SetBool, '/go2_cmd_vel_bridge/enable', self._enable_cb)

        period = 1.0 / control_rate
        self._timer = self.create_timer(period, self._control_tick)

        self._publish_armed(False)
        self._publish_safe_command(Go2VelocityCommand())
        self.get_logger().info(
            f"Go2 cmd_vel bridge initialized via WebRTC to '{robot_ip}'; "
            'bridge is DISABLED and will not change posture')

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _command_cb(self, msg):
        command = Go2VelocityCommand(msg.linear.x, msg.linear.y, msg.angular.z)
        was_armed = self._controller.armed()
        _, reason = self._controller.acceptCommand(command, time.monotonic())
        is_armed = self._controller.armed()
        command = self._controller.lastOutput()

        if reason and was_armed:
            self.get_logger().error(f'{reason}')
        if was_armed != is_armed:
            self._publish_armed(is_armed)
        self._publish_safe_command(command)

    def _odometry_cb(self, msg):
        if not self._finite_odometry(msg):
            if self._throttled('odom_nonfinite', 2.0):
                self.get_logger().error(
                    'Ignoring /Odometry_open3d containing non-finite pose values')
            return
        self._controller.updateOdometryHeartbeat(time.monotonic())

    def _enable_cb(self, request, response):
        if request.data:
            success, reason = self._controller.enable(time.monotonic())
            response.success = success
            response.message = reason
        else:
            self._controller.disable('disabled by enable service')
            response.success = True
            reason = 'bridge disabled and StopMove sent'
            response.message = reason

        self._publish_safe_command(self._controller.lastOutput())
        self._publish_armed(self._controller.armed())

        if response.success:
            self.get_logger().info(f'{response.message}')
        else:
            self.get_logger().warn(f'{response.message}')

    def _control_tick(self):
        was_armed = self._controller.armed()
        self._controller.tick(time.monotonic())
        is_armed = self._controller.armed()
        fault = self._controller.lastFault()
        output = self._controller.lastOutput()

        self._publish_safe_command(output)
        if was_armed != is_armed:
            self._publish_armed(is_armed)
            if not is_armed:
                self.get_logger().error(
                    f'Safety fault: {fault}; bridge is now DISABLED')

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _finite_odometry(msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        return all(math.isfinite(x) for x in (p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def _throttled(self, key, seconds):
        now = self.get_clock().now().nanoseconds
        last = self._throttle_marks.get(key, 0)
        if now - last >= int(seconds * 1e9):
            self._throttle_marks[key] = now
            return True
        return False

    def _publish_armed(self, armed):
        msg = Bool()
        msg.data = bool(armed)
        self._pub_armed.publish(msg)

    def _publish_safe_command(self, command):
        msg = Twist()
        msg.linear.x = command.vx
        msg.linear.y = command.vy
        msg.angular.z = command.vyaw
        self._pub_safe_cmd.publish(msg)

    def _shutdown_bridge(self):
        try:
            self._controller.shutdown()
        except Exception as exc:
            self.get_logger().warn(f'safety controller shutdown failed: {exc}')
        try:
            self._client.cleanup()
        except Exception as exc:
            self.get_logger().warn(f'webrtc client cleanup failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Go2CmdVelBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('go2_cmd_vel_bridge').fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    finally:
        if node is not None:
            try:
                node._shutdown_bridge()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
