#!/usr/bin/env python3
# coding=utf-8

import asyncio
import json
import os
import threading
import time

import rospy
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

ROBOT_IP = os.environ.get("UNITREE_ROBOT_IP", "192.168.123.161")

_GAIT_MAP = {
    "freewalk": "FreeWalk",
    "economic": "EconomicGait",
}


class AsyncBridge:
    def __init__(self):
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def run(self, coro, timeout=10.0):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout=timeout)

    def stop(self):
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)


class WebRTCSportClient:
    """对外接口模仿 unitree_sdk2py 的 SportClient，内部走 WebRTC"""

    def __init__(self, ip=ROBOT_IP):
        self.ip = ip
        self.bridge = AsyncBridge()
        self.conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA, ip=self.ip
        )
        self._ready = False

    # 兼容旧代码的空方法
    def SetTimeout(self, timeout):
        pass

    def Init(self):
        rospy.loginfo(f"WebRTC 连接机器狗: {self.ip}")
        self.bridge.run(self.conn.connect(), timeout=30)
        self.bridge.run(self._ensure_normal_mode())
        self.bridge.run(self._sport(SPORT_CMD["FreeWalk"], {}))
        self._ready = True
        rospy.loginfo("WebRTC SportClient 初始化成功")

    async def _ensure_normal_mode(self):
        resp = await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}
        )
        if resp["data"]["header"]["status"]["code"] != 0:
            return
        mode = json.loads(resp["data"]["data"])["name"]
        if mode != "normal":
            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": "normal"}},
            )
            await asyncio.sleep(5)

    async def _sport(self, api_id, parameter=None):
        payload = {"api_id": api_id}
        if parameter is not None:
            payload["parameter"] = parameter
        await self.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], payload
        )

    def _call(self, api_id, parameter=None):
        try:
            self.bridge.run(self._sport(api_id, parameter))
            return 0
        except Exception as e:
            rospy.logerr(f"WebRTC 命令失败 api_id={api_id}: {e}")
            return -1

    def Move(self, vx, vy, vyaw):
        return self._call(
            SPORT_CMD["Move"],
            {"x": vx, "y": vy, "z": vyaw},
        )

    def StopMove(self):
        return self._call(SPORT_CMD["StopMove"], {})

    def StandUp(self):
        ret = self._call(SPORT_CMD["StandUp"])
        if ret == 0:
            time.sleep(2)  # 等站立动作完成
            return self._call(SPORT_CMD["BalanceStand"])
        return ret

    def StandDown(self):
        return self._call(SPORT_CMD["StandDown"], {})

    def Sit(self):
        return self._call(SPORT_CMD["Sit"], {})

    def Hello(self):
        return self._call(SPORT_CMD["Hello"], {})

    def Stretch(self):
        return self._call(SPORT_CMD["Stretch"], {})

    def FreeWalk(self):
        return self._call(SPORT_CMD["FreeWalk"], {})

    def BalanceStand(self):
        return self._call(SPORT_CMD["BalanceStand"])

    def SwitchGait(self, gait_name):
        """go2_control.py 传 'freewalk' / 'economic' 字符串"""
        key = _GAIT_MAP.get(str(gait_name).lower())
        if not key:
            rospy.logwarn(f"未知步态: {gait_name}")
            return -1
        return self._call(SPORT_CMD[key], {})

    def cleanup(self):
        try:
            self.StopMove()
            self.bridge.run(self.conn.disconnect())
        except Exception as e:
            rospy.logwarn(f"WebRTC 断开失败: {e}")
        finally:
            self.bridge.stop()
