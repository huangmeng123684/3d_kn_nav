#!/usr/bin/env python3
# coding=utf-8
"""
绕过 go2_control，直接用 WebRTCSportClient 做键盘测试。

用法:
  export UNITREE_ROBOT_IP=192.168.123.161   # Linux
  set UNITREE_ROBOT_IP=192.168.123.161        # Windows
  python keyboard_test_sport.py

判断:
  - 这里能动、go2_control 不能动 → 问题在 go2_control 循环
  - 这里也不能动 → 问题在 webrtc_sport_client / 机器人状态 / 模式
"""

import json
import os
import sys
import time
import types
import asyncio

# ── 无 ROS 环境也能跑：在 import webrtc_sport_client 前注入假 rospy ──
def _make_fake_rospy():
    m = types.ModuleType("rospy")

    def _log(fn):
        return lambda msg, *a, **k: print(f"[{fn}] {msg}")

    m.loginfo = _log("INFO")
    m.logwarn = _log("WARN")
    m.logerr = _log("ERR")
    return m


sys.modules.setdefault("rospy", _make_fake_rospy())

# 保证能 import 同目录的 webrtc_sport_client.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from webrtc_sport_client import WebRTCSportClient  # noqa: E402

# ── 运动参数 ──
LINEAR_SPEED = 0.35   # 前后 / 左右平移 (m/s 量级，可按狗反应调大)
TURN_SPEED = 0.45     # 转向角速度
MOVE_HZ = 50
MOVE_INTERVAL = 1.0 / MOVE_HZ
BURST_SEC = 0.4       # 每按一次键，持续发 Move 的时长

HELP = """
======== Go2 键盘直连测试（绕过 go2_control）========
移动（按住或连按）:
  W / ↑  前进        S / ↓  后退
  A / ←  左移        D / →  右移
  J      左转        L      右转
  Space  停止

姿态:
  U  起立 StandUp      N  趴下 StandDown
  M  坐下 Sit          B  BalanceStand（平衡站立，可选）

其他:
    F  切换 FreeWalk 开/关   E  切换 EconomicGait 开/关   H  Hello
  I  查询 GetState（运动状态/步态/高度等，按一次查一次）

运动模式切换 (MOTION_SWITCHER):
  G  查询当前模式
  1  切 normal          2  切 ai          3  切 mcf
  Q  退出
=====================================================
"""

GET_STATE_KEYS = [
    "state",
    "bodyHeight",
    "footRaiseHeight",
    "speedLevel",
    "gait",
    "joystick",
    "dance",
    "continuousGait",
    "economicGait",
]

# MCF 示例里实际使用的字段子集（全量字段在 mcf 下可能返回空 data）
GET_STATE_KEYS_MCF = [
    "state",
    "bodyHeight",
    "speedLevel",
    "gait",
    "continuousGait",
    "economicGait",
]

MODE_NAMES = {
    0: "idle",
    1: "balanceStand",
    2: "pose",
    3: "locomotion",
    4: "reserve",
    5: "lieDown",
    6: "jointLock",
    7: "damping",
    8: "recoveryStand",
    9: "reserve",
    10: "sit",
    11: "frontFlip",
    12: "frontJump",
    13: "frontPounce",
}

GAIT_NAMES = {
    0: "idle",
    1: "trot",
    2: "run",
    3: "climb stair",
    4: "forwardDownStair",
    9: "adjust",
}


def move_burst(client, vx, vy, vyaw, duration=BURST_SEC):
    """在 deadline 内连续高频发 Move，不自动 StopMove（用 Space 停止）"""
    print(f"  -> Move burst: x={vx:.2f}, y={vy:.2f}, z={vyaw:.2f}, {duration}s @ {MOVE_HZ}Hz")
    deadline = time.time() + duration
    n = 0
    while time.time() < deadline:
        ret = client.Move(vx, vy, vyaw)
        if ret != 0:
            print(f"  !! Move 返回 {ret}")
        n += 1
        time.sleep(MOVE_INTERVAL)
    print(f"  -> burst 结束 (共发 {n} 次 Move)，按 Space 停止")


def move_single(client, vx, vy, vyaw, hold=2.0):
    """只发一次 Move，不自动 StopMove（用 Space 停止）"""
    print(f"  -> Move single: x={vx:.2f}, y={vy:.2f}, z={vyaw:.2f}")
    ret = client.Move(vx, vy, vyaw)
    print(f"  -> Move 返回 {ret}")
    if hold > 0:
        time.sleep(hold)
    print("  -> 按 Space 停止")

# ── 跨平台读键 ──
def _get_key_windows():
    import msvcrt

    if not msvcrt.kbhit():
        return None
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):
        ch2 = msvcrt.getch()
        arrow = {72: "up", 80: "down", 75: "left", 77: "right"}.get(ch2[0])
        return arrow
    try:
        return ch.decode("utf-8", errors="ignore").lower()
    except Exception:
        return None


def get_key():
    """获取键盘输入（非阻塞），逻辑同 test_yqh.py"""
    if sys.platform == "win32":
        return _get_key_windows()

    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    rest = sys.stdin.read(2)
                    arrow_map = {"[A": "up", "[B": "down", "[D": "left", "[C": "right"}
                    return arrow_map.get(rest, None)
                return None
            return ch.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

# 当前运动模式，由 query_motion_mode 更新；FreeWalk/EconomicGait 据此选对应编号
CURRENT_MODE = None
# E/F 键切换开关的本地状态（每次按键在 on/off 间切换）
ECONOMIC_GAIT_ENABLED = False
FREEWALK_ENABLED = False


def query_motion_mode(client):
    """查询当前 MOTION_SWITCHER 模式，返回模式名字符串或 None"""
    from unitree_webrtc_connect.constants import RTC_TOPIC

    global CURRENT_MODE
    try:
        resp = client.bridge.run(
            client.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}
            )
        )
    except Exception as e:
        print(f"  !! 查询模式失败: {e}")
        return None

    if resp["data"]["header"]["status"]["code"] != 0:
        print("  !! 查询模式返回非 0")
        return None
    mode = json.loads(resp["data"]["data"])["name"]
    CURRENT_MODE = mode
    print(f"  -> 当前模式: {mode}")
    return mode


def _sport_rpc_call(client, api_id, parameter=None):
    """发 sport RPC 并返回机器狗真实 status.code（不是 _call 的假 0）。"""
    from unitree_webrtc_connect.constants import RTC_TOPIC

    payload = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter
    resp = client.bridge.run(
        client.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"], payload
        )
    )
    code = resp.get("data", {}).get("header", {}).get("status", {}).get("code", -1)
    raw = _extract_rpc_payload(resp)
    return code, raw


def _unwrap_state_value(v):
    """新固件 GetState 常为 {"data": value}，展开成实际值。"""
    if isinstance(v, str):
        s = v.strip()
        if s.startswith("{"):
            try:
                v = json.loads(s)
            except json.JSONDecodeError:
                return v
    if isinstance(v, dict) and "data" in v:
        return _unwrap_state_value(v["data"])
    return v


def do_freewalk(client):
    """切换 FreeWalk。normal 上常走 1045+{"data":bool}；mcf 走 2045。"""
    from unitree_webrtc_connect.constants import SPORT_CMD, SPORT_CMD_MCF

    global FREEWALK_ENABLED
    FREEWALK_ENABLED = not FREEWALK_ENABLED
    enable = FREEWALK_ENABLED
    param = {"data": enable}

    if CURRENT_MODE == "mcf":
        attempts = [
            ("2045", SPORT_CMD_MCF["FreeWalk"], param),
        ]
    else:
        attempts = [
            ("1045+toggle", SPORT_CMD["FreeWalk"], param),
            ("2045+toggle", SPORT_CMD_MCF["FreeWalk"], param),
            ("1045+legacy", SPORT_CMD["FreeWalk"], {}),
        ]

    print(f"  -> FreeWalk {'开' if enable else '关'} (mode={CURRENT_MODE})")
    last_code = -1
    for label, api_id, p in attempts:
        code, _ = _sport_rpc_call(client, api_id, p)
        print(f"  -> try {label} api_id={api_id} param={p} -> code={code}")
        last_code = code
        if code == 0:
            print(f"  -> FreeWalk 机器狗接受 ({label})")
            print("  -> 提示: 按 I 查看 state/gait 是否变化")
            return code

    FREEWALK_ENABLED = not enable
    print(f"  !! FreeWalk 未生效，最后 code={last_code}")
    return last_code


def do_economicgait(client):
    """切换 EconomicGait。新固件优先 1063+{"data":bool}，旧固件再试 1035。"""
    from unitree_webrtc_connect.constants import SPORT_CMD, SPORT_CMD_MCF

    global ECONOMIC_GAIT_ENABLED
    ECONOMIC_GAIT_ENABLED = not ECONOMIC_GAIT_ENABLED
    enable = ECONOMIC_GAIT_ENABLED
    param = {"data": enable}

    if CURRENT_MODE == "mcf":
        attempts = [
            ("1063", SPORT_CMD_MCF["EconomicGait"], param),
        ]
    else:
        # normal：你这台狗实测 1035+toggle 有效，1063 返回 3203
        attempts = [
            ("1035+toggle", SPORT_CMD["EconomicGait"], param),
            ("1063", SPORT_CMD_MCF["EconomicGait"], param),
            ("1035+legacy", SPORT_CMD["EconomicGait"], {} if enable else param),
        ]

    print(f"  -> EconomicGait {'开' if enable else '关'} (mode={CURRENT_MODE})")
    last_code = -1
    for label, api_id, p in attempts:
        code, _ = _sport_rpc_call(client, api_id, p)
        print(f"  -> try {label} api_id={api_id} param={p} -> code={code}")
        last_code = code
        if code == 0:
            print(f"  -> EconomicGait 机器狗接受 ({label})")
            print("  -> 提示: 按 I 查看 economicGait 是否已变为 1")
            return code

    # 全部失败则回滚本地开关
    ECONOMIC_GAIT_ENABLED = not enable
    print(f"  !! EconomicGait 未生效，最后 code={last_code}")
    return last_code


def _print_get_state_map(parsed):
    print("  State:")
    for k, v in parsed.items():
        v = _unwrap_state_value(v)
        hint = ""
        if k in ("state", "mode"):
            if isinstance(v, str):
                hint = ""
            else:
                try:
                    hint = f" ({MODE_NAMES.get(int(v), '?')})"
                except (TypeError, ValueError):
                    pass
        elif k in ("gait", "gait_type"):
            if isinstance(v, str):
                hint = ""
            else:
                try:
                    hint = f" ({GAIT_NAMES.get(int(v), '?')})"
                except (TypeError, ValueError):
                    pass
        elif k in ("economicGait", "continuousGait", "dance", "joystick"):
            if v in (1, True, "1"):
                hint = " (开)"
            elif v in (0, False, "0"):
                hint = " (关)"
        print(f"    {k}: {v}{hint}")


def _extract_rpc_payload(resp):
    """从 WebRTC 响应里取出 JSON 字符串（兼容 data / parameter 字段）。"""
    body = resp.get("data", {}) if isinstance(resp, dict) else {}
    for key in ("data", "parameter"):
        value = body.get(key, "")
        if value:
            return value
    return ""


def _call_get_state_rpc(client, keys):
    from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

    resp = client.bridge.run(
        client.conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["GetState"], "parameter": keys},
        )
    )
    code = resp.get("data", {}).get("header", {}).get("status", {}).get("code", -1)
    raw = _extract_rpc_payload(resp)
    return code, raw


async def _fetch_sport_state_once(conn, timeout=3.0):
    """GetState RPC 无数据时，订阅一次 sportmodestate 快照。"""
    from unitree_webrtc_connect.constants import RTC_TOPIC

    result = {"data": None}

    def on_message(message):
        result["data"] = message.get("data")
        conn.datachannel.pub_sub.unsubscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"])

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_message)

    deadline = asyncio.get_event_loop().time() + timeout
    while result["data"] is None and asyncio.get_event_loop().time() < deadline:
        await asyncio.sleep(0.05)

    if result["data"] is None:
        conn.datachannel.pub_sub.unsubscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"])
    return result["data"]


def _print_sport_mode_snapshot(msg):
    """打印 sportmodestate 流里的字段（与 GetState 字段名略有不同）。"""
    print("  State (sportmodestate 快照):")
    if CURRENT_MODE:
        print(f"    motion_switcher: {CURRENT_MODE}")
    fields = [
        ("mode", msg.get("mode")),
        ("gait_type", msg.get("gait_type")),
        ("body_height", msg.get("body_height")),
        ("foot_raise_height", msg.get("foot_raise_height")),
        ("velocity", msg.get("velocity")),
        ("yaw_speed", msg.get("yaw_speed")),
        ("progress", msg.get("progress")),
    ]
    for k, v in fields:
        hint = ""
        if k == "mode":
            try:
                hint = f" ({MODE_NAMES.get(int(v), '?')})"
            except (TypeError, ValueError):
                pass
        elif k == "gait_type":
            try:
                hint = f" ({GAIT_NAMES.get(int(v), '?')})"
            except (TypeError, ValueError):
                pass
        print(f"    {k}: {v}{hint}")


def do_get_state(client):
    """通过 WebRTC 查询状态：先 GetState RPC，空则回退 sportmodestate 快照。"""
    print("  -> GetState")
    try:
        key_sets = [GET_STATE_KEYS]
        if CURRENT_MODE == "mcf":
            key_sets.insert(0, GET_STATE_KEYS_MCF)

        raw = ""
        code = -1
        for keys in key_sets:
            code, raw = _call_get_state_rpc(client, keys)
            if code != 0:
                print(f"  !! GetState 返回 code={code}")
                return
            if raw:
                break

        if raw:
            parsed = json.loads(raw) if isinstance(raw, str) else raw
            _print_get_state_map(parsed)
            return

        print("  !! GetState RPC 成功但 data 为空（MCF 固件上较常见）")
        print("  -> 改用 sportmodestate 状态流快照...")
        snapshot = client.bridge.run(_fetch_sport_state_once(client.conn))
        if snapshot:
            _print_sport_mode_snapshot(snapshot)
        else:
            print("  !! 未收到 sportmodestate 数据，可运行 sportmodestate.py 持续查看")
    except Exception as e:
        print(f"  !! GetState 失败: {e}")


def switch_motion_mode(client, target, wait=5.0, safe_posture=True):
    """切换 MOTION_SWITCHER 到 target (normal / ai / mcf)。

    切换运动框架相当于重启底层控制器，机器狗必须处于安全姿态才允许切换，
    否则请求会被拒绝（模式切不过去）。因此这里默认先趴下(StandDown)再切，
    切换成功后再起立(StandUp)。
    """
    from unitree_webrtc_connect.constants import RTC_TOPIC

    current = query_motion_mode(client)
    if current == target:
        print(f"  -> 已经是 {target} 模式，无需切换")
        return

    if safe_posture:
        print("  -> 切换前先趴下到安全姿态 (StandDown) ...")
        client.StandDown()
        time.sleep(3)

    print(f"  -> 切换模式 {current} -> {target} ...")
    try:
        resp = client.bridge.run(
            client.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"],
                {"api_id": 1002, "parameter": {"name": target}},
            )
        )
        code = resp["data"]["header"]["status"]["code"]
        data = resp["data"].get("data", "")
        print(f"  -> 切换请求返回 code={code}, data={data}")
    except Exception as e:
        print(f"  !! 切换模式失败: {e}")
        return

    if wait > 0:
        print(f"  -> 等待 {wait}s 让机器狗完成切换...")
        time.sleep(wait)

    new_mode = query_motion_mode(client)
    if new_mode == target:
        print(f"  -> 切换成功，当前 {new_mode}")
        if safe_posture:
            print("  -> 重新起立 (StandUp) ...")
            client.StandUp()
    else:
        print(f"  !! 切换未生效，仍为 {new_mode}（可能被机器狗拒绝，看上面的 code/data）")


def dispatch_key(client, key, single_move=False):
    if key in ("q", "\x03"):
        return False

    if key == " ":
        client.StopMove()
        print("  -> StopMove")
        return True

    moves = {
        "w": (LINEAR_SPEED, 0, 0),
        "up": (LINEAR_SPEED, 0, 0),
        "s": (-LINEAR_SPEED, 0, 0),
        "down": (-LINEAR_SPEED, 0, 0),
        "a": (0, LINEAR_SPEED, 0),
        "left": (0, LINEAR_SPEED, 0),
        "d": (0, -LINEAR_SPEED, 0),
        "right": (0, -LINEAR_SPEED, 0),
        "j": (0, 0, TURN_SPEED),
        "l": (0, 0, -TURN_SPEED),
    }
    if key in moves:
        vx, vy, vyaw = moves[key]
        if single_move:
            move_single(client, vx, vy, vyaw)
        else:
            move_burst(client, vx, vy, vyaw)
        return True

    actions = {
        "u": ("StandUp", client.StandUp),
        "n": ("StandDown", client.StandDown),
        "m": ("Sit", client.Sit),
        "h": ("Hello", client.Hello),
    }
    if key in actions:
        name, fn = actions[key]
        print(f"  -> {name}")
        ret = fn()
        print(f"  -> {name} 返回 {ret}")
        return True

    if key == "f":
        do_freewalk(client)
        return True

    if key == "e":
        do_economicgait(client)
        return True

    if key == "b":
        from unitree_webrtc_connect.constants import SPORT_CMD

        print("  -> BalanceStand")
        ret = client._call(SPORT_CMD["BalanceStand"])
        print(f"  -> BalanceStand 返回 {ret}")
        return True

    if key == "g":
        query_motion_mode(client)
        return True

    if key == "i":
        do_get_state(client)
        return True

    mode_keys = {"1": "normal", "2": "ai", "3": "mcf"}
    if key in mode_keys:
        switch_motion_mode(client, mode_keys[key])
        return True

    return True


def main():
    single_move = "--single" in sys.argv

    print(HELP)
    if single_move:
        print("【模式】--single：每次只发 1 次 Move（对比用）")
    else:
        print("【模式】默认：50Hz 连续发 Move（推荐）")

    ip = os.environ.get("UNITREE_ROBOT_IP", "192.168.123.161")
    print(f"连接 IP: {ip}")

    client = WebRTCSportClient(ip=ip)
    try:
        client.Init()
        # 启动时查一次当前模式，让 FreeWalk/EconomicGait 用对编号
        query_motion_mode(client)
        print("初始化完成。请先按 U 起立，再用 WASD 测试移动。\n")

        while True:
            key = get_key()
            if key:
                if not dispatch_key(client, key, single_move=single_move):
                    break
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        print("清理中...")
        client.cleanup()
        print("已退出")


if __name__ == "__main__":
    main()