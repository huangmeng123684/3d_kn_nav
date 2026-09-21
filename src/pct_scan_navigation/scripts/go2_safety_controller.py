#!/usr/bin/env python3
# coding=utf-8
"""Python 移植版 Go2 安全控制器（1:1 复刻 C++ go2_safety_controller.cpp/hpp）。

对外契约与 C++ 版一致。sport_client 只需提供两个方法：
    Move(vx, vy, vyaw) -> int   # 0 成功，非 0 失败
    StopMove() -> int           # 0 成功，非 0 失败
（WebRTCSportClient 已满足，见同目录 webrtc_sport_client.py）

线程安全：所有公共方法内部持锁，对应 C++ 的 controller_mutex_。
"""

import math
import threading


class Go2VelocityCommand:
    __slots__ = ('vx', 'vy', 'vyaw')

    def __init__(self, vx=0.0, vy=0.0, vyaw=0.0):
        self.vx = float(vx)
        self.vy = float(vy)
        self.vyaw = float(vyaw)

    def __eq__(self, other):
        if not isinstance(other, Go2VelocityCommand):
            return NotImplemented
        return (
            self.vx == other.vx
            and self.vy == other.vy
            and self.vyaw == other.vyaw
        )

    def __repr__(self):
        return (
            f'Go2VelocityCommand(vx={self.vx}, vy={self.vy}, vyaw={self.vyaw})'
        )


class Go2SafetyConfig:
    """对应 C++ 的 Go2SafetyConfig 结构体，字段名与 go2_bridge.yaml 一致。"""

    def __init__(
        self,
        min_vx=0.0,
        max_vx=0.25,
        max_abs_vy=0.0,
        max_abs_vyaw=0.5,
        max_linear_acceleration=0.25,
        max_yaw_acceleration=0.5,
        command_timeout=0.3,
        odometry_timeout=0.3,
        sport_state_timeout=0.5,
    ):
        self.min_vx = float(min_vx)
        self.max_vx = float(max_vx)
        self.max_abs_vy = float(max_abs_vy)
        self.max_abs_vyaw = float(max_abs_vyaw)
        self.max_linear_acceleration = float(max_linear_acceleration)
        self.max_yaw_acceleration = float(max_yaw_acceleration)
        self.command_timeout = float(command_timeout)
        self.odometry_timeout = float(odometry_timeout)
        self.sport_state_timeout = float(sport_state_timeout)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _approach(current, target, maximum_delta):
    """对应 C++ 的 approach()：朝目标斜坡逼近，步长不超过 maximum_delta。"""
    return current + _clamp(target - current, -maximum_delta, maximum_delta)


class Go2SafetyController:
    """1:1 复刻 C++ Go2SafetyController 的状态机与规则，一条语义都不省。"""

    def __init__(self, sport_client, config):
        self._sport_client = sport_client
        self._config = config
        self._lock = threading.RLock()

        self._armed = False
        self._stopped = True          # 是否已处于停止态（零速去抖用）
        self._waiting_for_command = True
        self._odometry_received = False
        self._sport_state_received = False
        self._command_received = False
        self._shutdown = False

        self._last_odometry_time = 0.0
        self._last_sport_state_time = 0.0
        self._last_command_time = 0.0
        self._last_tick_time = 0.0

        self._target_command = Go2VelocityCommand()
        self._last_output = Go2VelocityCommand()
        self._last_fault = ''

    # ------------------------------------------------------------------
    # heartbeats
    # ------------------------------------------------------------------
    def updateOdometryHeartbeat(self, now):
        with self._lock:
            self._odometry_received = True
            self._last_odometry_time = now

    def updateSportStateHeartbeat(self, now):
        with self._lock:
            self._sport_state_received = True
            self._last_sport_state_time = now

    # ------------------------------------------------------------------
    # state transitions
    # ------------------------------------------------------------------
    def enable(self, now):
        """返回 (success, reason)。对应 C++ enable(now, reason)。"""
        with self._lock:
            if self._armed:
                return True, 'bridge is already enabled'
            if not self._heartbeat_fresh(
                self._odometry_received,
                self._last_odometry_time,
                now,
                self._config.odometry_timeout,
            ):
                return False, 'cannot enable: odometry heartbeat is missing or stale'
            if not self._heartbeat_fresh(
                self._sport_state_received,
                self._last_sport_state_time,
                now,
                self._config.sport_state_timeout,
            ):
                return False, 'cannot enable: sport state heartbeat is missing or stale'
            if not self._send_stop():
                self._last_fault = 'cannot enable: StopMove failed'
                return False, self._last_fault

            self._target_command = Go2VelocityCommand()
            self._last_output = Go2VelocityCommand()
            self._stopped = True
            self._command_received = False
            self._waiting_for_command = True
            self._last_tick_time = now
            self._last_fault = ''
            self._armed = True
            return True, 'bridge enabled; waiting for a new cmd_vel'

    def disable(self, reason):
        with self._lock:
            self._send_stop()
            self._armed = False
            self._stopped = True
            self._waiting_for_command = True
            self._command_received = False
            self._target_command = Go2VelocityCommand()
            self._last_output = Go2VelocityCommand()
            self._last_fault = reason

    def acceptCommand(self, command, now):
        """返回 (accepted, reason)。对应 C++ acceptCommand(command, now, reason)。"""
        with self._lock:
            if not self._armed:
                return False, 'ignored cmd_vel while bridge is disabled'
            if not self._command_is_finite(command):
                reason = 'cmd_vel contains a non-finite value'
                self._fault_and_disarm(reason)
                return False, reason

            self._target_command = self._clamp_command(command)
            self._last_command_time = now
            self._command_received = True
            self._waiting_for_command = False

            if self._command_is_zero(self._target_command):
                # 去抖：仅在“由动转停”时下发一次 StopMove。上游零速命令可能
                # 以高频率到达（闭环控制器 100Hz 发零速），若每条零速都
                # StopMove，会反复打断狗的 SportMode，造成顿挫/漂移。
                if not self._stopped:
                    if not self._send_stop():
                        reason = 'failed to stop for zero cmd_vel'
                        self._fault_and_disarm(reason)
                        return False, reason
                    self._stopped = True
                self._last_tick_time = now
            else:
                self._stopped = False

            return True, ''

    def tick(self, now):
        with self._lock:
            if not self._armed:
                return

            if not self._heartbeat_fresh(
                self._odometry_received,
                self._last_odometry_time,
                now,
                self._config.odometry_timeout,
            ):
                self._fault_and_disarm('odometry heartbeat timed out')
                return
            if not self._heartbeat_fresh(
                self._sport_state_received,
                self._last_sport_state_time,
                now,
                self._config.sport_state_timeout,
            ):
                self._fault_and_disarm('sport state heartbeat timed out')
                return

            if self._waiting_for_command:
                self._last_tick_time = now
                return
            if not self._heartbeat_fresh(
                True, self._last_command_time, now, self._config.command_timeout
            ):
                self._fault_and_disarm('cmd_vel timed out')
                return
            if self._command_is_zero(self._target_command):
                self._last_tick_time = now
                return

            elapsed = max(0.0, now - self._last_tick_time)
            self._last_tick_time = now
            linear_delta = self._config.max_linear_acceleration * elapsed
            yaw_delta = self._config.max_yaw_acceleration * elapsed

            nxt = Go2VelocityCommand()
            nxt.vx = _approach(self._last_output.vx, self._target_command.vx, linear_delta)
            nxt.vy = _approach(self._last_output.vy, self._target_command.vy, linear_delta)
            nxt.vyaw = _approach(
                self._last_output.vyaw, self._target_command.vyaw, yaw_delta
            )

            result = self._sport_client.Move(nxt.vx, nxt.vy, nxt.vyaw)
            if result != 0:
                self._fault_and_disarm(f'SportClient::Move failed with code {result}')
                return
            self._stopped = False
            self._last_output = nxt

    def shutdown(self):
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self.disable('bridge shutdown')

    # ------------------------------------------------------------------
    # getters
    # ------------------------------------------------------------------
    def armed(self):
        with self._lock:
            return self._armed

    def waitingForCommand(self):
        with self._lock:
            return self._waiting_for_command

    def lastOutput(self):
        with self._lock:
            return Go2VelocityCommand(
                self._last_output.vx, self._last_output.vy, self._last_output.vyaw
            )

    def lastFault(self):
        with self._lock:
            return self._last_fault

    # ------------------------------------------------------------------
    # internals（必须在持锁状态下调用）
    # ------------------------------------------------------------------
    def _heartbeat_fresh(self, received, stamp, now, timeout):
        return received and now >= stamp and now - stamp <= timeout

    def _command_is_finite(self, command):
        return all(math.isfinite(x) for x in (command.vx, command.vy, command.vyaw))

    def _command_is_zero(self, command):
        eps = 1e-6
        return (
            abs(command.vx) <= eps
            and abs(command.vy) <= eps
            and abs(command.vyaw) <= eps
        )

    def _clamp_command(self, command):
        cfg = self._config
        result = Go2VelocityCommand()
        result.vx = _clamp(command.vx, cfg.min_vx, cfg.max_vx)
        result.vy = _clamp(command.vy, -cfg.max_abs_vy, cfg.max_abs_vy)
        result.vyaw = _clamp(command.vyaw, -cfg.max_abs_vyaw, cfg.max_abs_vyaw)
        return result

    def _fault_and_disarm(self, reason):
        self._send_stop()
        self._armed = False
        self._stopped = True
        self._waiting_for_command = True
        self._command_received = False
        self._target_command = Go2VelocityCommand()
        self._last_output = Go2VelocityCommand()
        self._last_fault = reason

    def _send_stop(self):
        # 只发 StopMove，不再附带 Move(0,0,0)：上游零速命令可能以高频率到达，
        # 每次 Move(0,0,0)+StopMove 会反复打断狗的 SportMode，导致顿挫/漂移。
        stop_result = self._sport_client.StopMove()
        self._last_output = Go2VelocityCommand()
        return stop_result == 0
