#!/usr/bin/env python3
"""Expose the KN navigation ROS 2 services through a FastAPI HTTP API.

Run this script from a shell where ROS 2 and this workspace have been sourced::

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/tools/ros2_service_api.py --host 127.0.0.1 --port 8000

FastAPI's interactive API documentation is then available at ``/docs``.
Set ``KN_NAV_API_KEY`` to require the same value in the ``X-API-Key`` HTTP
header.  Authentication is strongly recommended when binding to a non-loopback
address.
"""

import argparse
import asyncio
import copy
from contextlib import asynccontextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time
from typing import Any, Dict, Optional, Tuple
import uuid

try:
    import uvicorn
    from fastapi import APIRouter, Depends, FastAPI, Header, HTTPException
    from pydantic import BaseModel, Field
except ImportError as error:  # pragma: no cover - depends on the deployment host
    raise SystemExit(
        "FastAPI dependencies are missing. Install them with: "
        "python3 -m pip install fastapi uvicorn"
    ) from error


@dataclass(frozen=True)
class Settings:
    host: str = "127.0.0.1"
    port: int = 8000
    service_timeout: float = 30.0
    service_wait_timeout: float = 1.0
    queue_poll_interval: float = 0.5
    queue_distance_tolerance: float = 0.15
    queue_yaw_tolerance: float = 0.10
    odometry_topic: str = "/Odometry_open3d"
    points_file: str = ""
    points_directory: str = str(Path(__file__).parent)
    map_node: str = "/global_localization_node"
    map_parameter: str = "path_map"
    map_wait_timeout: float = 30.0
    api_key: str = ""

    @classmethod
    def from_environment(cls) -> "Settings":
        return cls(
            host=os.environ.get("KN_NAV_API_HOST", "127.0.0.1"),
            port=int(os.environ.get("KN_NAV_API_PORT", "8000")),
            service_timeout=float(os.environ.get("KN_NAV_SERVICE_TIMEOUT", "30.0")),
            service_wait_timeout=float(
                os.environ.get("KN_NAV_SERVICE_WAIT_TIMEOUT", "1.0")
            ),
            queue_poll_interval=float(
                os.environ.get("KN_NAV_QUEUE_POLL_INTERVAL", "0.5")
            ),
            queue_distance_tolerance=float(
                os.environ.get("KN_NAV_QUEUE_DISTANCE_TOLERANCE", "0.15")
            ),
            queue_yaw_tolerance=float(
                os.environ.get("KN_NAV_QUEUE_YAW_TOLERANCE", "0.10")
            ),
            odometry_topic=os.environ.get(
                "KN_NAV_ODOMETRY_TOPIC", "/Odometry_open3d"
            ),
            points_file=os.environ.get(
                "KN_NAV_POINTS_FILE", ""
            ),
            points_directory=os.environ.get(
                "KN_NAV_POINTS_DIRECTORY", str(Path(__file__).parent)
            ),
            map_node=os.environ.get(
                "KN_NAV_MAP_NODE", "/global_localization_node"
            ),
            map_parameter=os.environ.get("KN_NAV_MAP_PARAMETER", "path_map"),
            map_wait_timeout=float(
                os.environ.get("KN_NAV_MAP_WAIT_TIMEOUT", "30.0")
            ),
            api_key=os.environ.get("KN_NAV_API_KEY", ""),
        )


class QuaternionPose(BaseModel):
    """A map-frame pose represented by position and quaternion."""

    x: float = Field(..., description="Map-frame X position in metres")
    y: float = Field(..., description="Map-frame Y position in metres")
    z: float = Field(..., description="Map-frame Z position in metres")
    qx: float = Field(..., description="Quaternion X")
    qy: float = Field(..., description="Quaternion Y")
    qz: float = Field(..., description="Quaternion Z")
    qw: float = Field(..., description="Quaternion W")


class SavePointRequest(BaseModel):
    name: str = Field(..., description="Unique navigation point name")
    overwrite: bool = Field(False, description="Replace an existing point")


class RenamePointRequest(BaseModel):
    old_name: str = Field(..., description="Existing navigation point name")
    new_name: str = Field(..., description="New navigation point name")


class NavigationGoalRequest(BaseModel):
    """A saved point name, or a direct planar map-frame goal."""

    name: Optional[str] = Field(None, description="Saved navigation point name")
    x: Optional[float] = Field(None, description="Map-frame X position in metres")
    y: Optional[float] = Field(None, description="Map-frame Y position in metres")
    z: float = Field(0.0, description="Map-frame Z position in metres")
    yaw: float = Field(0.0, description="Target yaw in radians")


class BridgeEnableRequest(BaseModel):
    data: bool = Field(..., description="True to arm, false to disarm the bridge")


class SwitchMapRequest(BaseModel):
    map_name: str = Field(..., description="Map profile name from map_profiles.yaml")


class RestartNavigationRequest(BaseModel):
    mode: int = Field(
        0,
        ge=0,
        le=1,
        description="0 performs a soft reset; 1 requests a configured full restart",
    )


def _ensure_finite(values: Dict[str, float]) -> None:
    invalid = [name for name, value in values.items() if not math.isfinite(value)]
    if invalid:
        raise HTTPException(
            status_code=422,
            detail=f"Non-finite numeric fields are not allowed: {', '.join(invalid)}",
        )


def _fill_pose_request(request: Any, pose: QuaternionPose) -> None:
    values = {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }
    _ensure_finite(values)
    for name, value in values.items():
        setattr(request, name, float(value))


def _normalise_quaternion_pose(
    pose: QuaternionPose,
) -> Tuple[QuaternionPose, float]:
    values = {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }
    _ensure_finite(values)
    norm = math.sqrt(
        pose.qx * pose.qx
        + pose.qy * pose.qy
        + pose.qz * pose.qz
        + pose.qw * pose.qw
    )
    if norm < 1e-6:
        raise HTTPException(status_code=422, detail="Quaternion norm is too small")
    return (
        QuaternionPose(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            qx=pose.qx / norm,
            qy=pose.qy / norm,
            qz=pose.qz / norm,
            qw=pose.qw / norm,
        ),
        norm,
    )


def _quaternion_pose_to_dict(pose: QuaternionPose) -> Dict[str, float]:
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }


def _quaternion_to_rpy(
    qx: float, qy: float, qz: float, qw: float
) -> Dict[str, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return {
        "roll": roll,
        "pitch": pitch,
        "yaw": math.atan2(siny_cosp, cosy_cosp),
    }


def _normalise_point_name(name: str) -> str:
    normalised = name.strip()
    if not normalised:
        raise HTTPException(status_code=422, detail="Point name must not be empty")
    if len(normalised) > 128:
        raise HTTPException(
            status_code=422, detail="Point name must not exceed 128 characters"
        )
    if any(ord(character) < 32 for character in normalised):
        raise HTTPException(
            status_code=422, detail="Point name must not contain control characters"
        )
    return normalised


def _normalise_map_name(name: str) -> str:
    normalised = name.strip()
    if not normalised:
        raise HTTPException(status_code=422, detail="Map name must not be empty")
    if len(normalised) > 128:
        raise HTTPException(
            status_code=422, detail="Map name must not exceed 128 characters"
        )
    if any(ord(character) < 32 for character in normalised):
        raise HTTPException(
            status_code=422, detail="Map name must not contain control characters"
        )
    return normalised


def _map_name_from_path(map_path: str) -> str:
    map_file_name = Path(map_path.strip()).name
    map_name = Path(map_file_name).stem.strip()
    if not map_name:
        raise RuntimeError(f"Unable to determine map name from path: {map_path!r}")
    safe_name = "".join(
        character if character.isalnum() or character in ("-", "_", ".") else "_"
        for character in map_name
    ).strip("._")
    if not safe_name:
        raise RuntimeError(f"Offline map name is invalid: {map_name!r}")
    return safe_name


def _points_file_for_map(points_directory: str, map_path: str) -> Path:
    directory = Path(points_directory).expanduser().resolve()
    return directory / f"{_map_name_from_path(map_path)}.json"


class NavigationPointStore:
    """Thread-safe JSON storage for named navigation points."""

    def __init__(self, file_path: str, map_path: str = ""):
        self.path = Path(file_path).expanduser().resolve()
        self.map_path = map_path
        self.map_name = (
            _map_name_from_path(map_path) if map_path else self.path.stem
        )
        self._lock = threading.Lock()
        self._initialise_file()
        self._ensure_unique_codes()

    def _initialise_file(self) -> None:
        """Create a missing store and repair an empty store as an empty object."""
        with self._lock:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                try:
                    with self.path.open("x", encoding="utf-8") as handle:
                        handle.write("{}\n")
                    return
                except FileExistsError:
                    pass

                with self.path.open("r", encoding="utf-8") as handle:
                    content = handle.read()
                if not content.strip():
                    with self.path.open("w", encoding="utf-8") as handle:
                        handle.write("{}\n")
            except OSError as error:
                raise RuntimeError(
                    f"Unable to initialise navigation points file {self.path}: {error}"
                ) from error

    def _load_unlocked(self) -> Dict[str, Dict[str, Any]]:
        if not self.path.exists():
            return {}
        try:
            with self.path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError) as error:
            raise HTTPException(
                status_code=500,
                detail=f"Unable to read navigation points file: {error}",
            ) from error
        if not isinstance(data, dict):
            raise HTTPException(
                status_code=500,
                detail="Navigation points file must contain a JSON object",
            )
        invalid = [name for name, point in data.items() if not isinstance(point, dict)]
        if invalid:
            raise HTTPException(
                status_code=500,
                detail=(
                    "Navigation points must be JSON objects; invalid entries: "
                    + ", ".join(str(name) for name in invalid)
                ),
            )
        return data

    def _write_unlocked(self, points: Dict[str, Dict[str, Any]]) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            with temporary.open("w", encoding="utf-8") as handle:
                json.dump(points, handle, indent=2, sort_keys=True)
                handle.write("\n")
            temporary.replace(self.path)
        except OSError as error:
            raise HTTPException(
                status_code=500,
                detail=f"Unable to write navigation points file: {error}",
            ) from error

    @staticmethod
    def _valid_code(code: Any) -> bool:
        return isinstance(code, str) and len(code) == 6 and code.isdigit()

    @staticmethod
    def _generate_unique_code(used_codes: set) -> str:
        if len(used_codes) >= 900000:
            raise HTTPException(
                status_code=500,
                detail="All six-digit navigation point codes are in use",
            )
        while True:
            code = str(secrets.randbelow(900000) + 100000)
            if code not in used_codes:
                return code

    def _ensure_unique_codes(self) -> None:
        """Add stable unique codes to old records and repair duplicate codes."""
        with self._lock:
            points = self._load_unlocked()
            used_codes = set()
            changed = False
            for name in sorted(points):
                point = points[name]
                code = point.get("code")
                if not self._valid_code(code) or code in used_codes:
                    code = self._generate_unique_code(used_codes)
                    point["code"] = code
                    changed = True
                used_codes.add(code)
            if changed:
                self._write_unlocked(points)

    def list_points(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            points = self._load_unlocked()
            return {name: dict(points[name]) for name in sorted(points)}

    def get_point(self, name: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            point = self._load_unlocked().get(name)
            return dict(point) if isinstance(point, dict) else None

    def save_point(
        self, name: str, point: Dict[str, Any], overwrite: bool
    ) -> Dict[str, Any]:
        with self._lock:
            points = self._load_unlocked()
            if name in points and not overwrite:
                raise HTTPException(
                    status_code=409,
                    detail=f"Navigation point already exists: {name}",
                )
            used_codes = {
                existing.get("code")
                for existing_name, existing in points.items()
                if existing_name != name and self._valid_code(existing.get("code"))
            }
            existing_code = points.get(name, {}).get("code")
            if not self._valid_code(existing_code) or existing_code in used_codes:
                existing_code = self._generate_unique_code(used_codes)

            stored_point = dict(point)
            stored_point["code"] = existing_code
            points[name] = stored_point
            self._write_unlocked(points)
            return dict(stored_point)

    def rename_point(self, old_name: str, new_name: str) -> Dict[str, Any]:
        with self._lock:
            points = self._load_unlocked()
            if old_name not in points:
                raise HTTPException(
                    status_code=404,
                    detail=f"Navigation point not found: {old_name}",
                )
            if old_name == new_name:
                return dict(points[old_name])
            if new_name in points:
                raise HTTPException(
                    status_code=409,
                    detail=f"Navigation point already exists: {new_name}",
                )

            point = points.pop(old_name)
            points[new_name] = point
            self._write_unlocked(points)
            return dict(point)


@dataclass(frozen=True)
class ResolvedNavigationGoal:
    """A validated queue goal whose saved-point values are already resolved."""

    source: str
    x: float
    y: float
    z: float
    yaw: float
    name: Optional[str] = None

    def pose(self) -> QuaternionPose:
        half_yaw = 0.5 * self.yaw
        return QuaternionPose(
            x=self.x,
            y=self.y,
            z=self.z,
            qx=0.0,
            qy=0.0,
            qz=math.sin(half_yaw),
            qw=math.cos(half_yaw),
        )

    def as_dict(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {
            "source": self.source,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
        }
        if self.name is not None:
            result["name"] = self.name
        return result


class RosServiceGateway:
    """Own a ROS node and spin it on a background thread."""

    SERVICE_NAMES = {
        "relocalize": "/open3d_loc/relocalize",
        "get_pose": "/open3d_loc/get_pose",
        "publish_goal": "/open3d_loc/publish_goal",
        "pose_deviation": "/open3d_loc/pose_deviation",
        "bridge_enable": "/go2_cmd_vel_bridge/enable",
        "switch_map": "/switch_map",
        "restart_navigation": "/restart_navigation",
    }

    STATUS_TOPICS = {
        "current_map": "/current_map",
        "localization_status": "/localization_status",
        "navigation_status": "/navigation_status",
    }

    MAP_STATE_NAMES = {
        0: "UNLOADED",
        1: "LOADING",
        2: "LOADED",
        3: "FAILED",
    }
    LOCALIZATION_STATE_NAMES = {
        0: "UNINITIALIZED",
        1: "INITIALIZING",
        2: "INIT_SUCCESS",
        3: "TRACKING",
        4: "TRACKING_WARN",
        5: "TRACKING_LOST",
        6: "MAP_SWITCHING",
    }
    NAVIGATION_STATE_NAMES = {
        0: "IDLE",
        1: "WAITING_GOAL",
        2: "PLANNING_GLOBAL",
        3: "GLOBAL_READY",
        4: "PLANNING_LOCAL",
        5: "NAVIGATING",
        6: "AVOIDING",
        7: "BLOCKED",
        8: "GOAL_REACHED",
        9: "CANCELED",
        10: "FAILED",
        11: "LOCALIZATION_LOST",
        12: "MAP_SWITCHING",
    }

    def __init__(self, settings: Settings):
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from open3d_loc.srv import GetPose, PoseDeviation, PublishGoal, Relocalize
            from pct_scan_navigation.msg import (
                LocalizationStatus,
                MapStatus,
                NavigationStatus,
            )
            from pct_scan_navigation.srv import RestartNavigation, SwitchMap
            from rcl_interfaces.srv import GetParameters
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from std_srvs.srv import SetBool
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 Python interfaces are unavailable. Source /opt/ros/humble/"
                "setup.bash and the workspace install/setup.bash before starting "
                "this API."
            ) from error

        self.settings = settings
        self._rclpy = rclpy
        self._context = Context()
        rclpy.init(args=None, context=self._context)
        self._node = Node(
            f"kn_nav_fastapi_gateway_{os.getpid()}", context=self._context
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name="kn-nav-ros2-executor",
            daemon=True,
        )

        map_node = "/" + settings.map_node.strip("/")
        self.service_names = dict(self.SERVICE_NAMES)
        self.service_names["get_map_parameters"] = (
            f"{map_node}/get_parameters"
        )
        self.service_types = {
            "relocalize": Relocalize,
            "get_pose": GetPose,
            "publish_goal": PublishGoal,
            "pose_deviation": PoseDeviation,
            "bridge_enable": SetBool,
            "switch_map": SwitchMap,
            "restart_navigation": RestartNavigation,
            "get_map_parameters": GetParameters,
        }
        self.clients = {
            key: self._node.create_client(self.service_types[key], service_name)
            for key, service_name in self.service_names.items()
        }
        self._robot_status_lock = threading.Lock()
        self._latest_robot_status: Optional[Dict[str, Any]] = None
        self._odometry_subscription = self._node.create_subscription(
            Odometry,
            settings.odometry_topic,
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self._topic_status_lock = threading.Lock()
        self._latest_topic_status: Dict[str, Dict[str, Any]] = {}

        current_map_qos = QoSProfile(depth=1)
        current_map_qos.reliability = ReliabilityPolicy.RELIABLE
        current_map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        periodic_status_qos = QoSProfile(depth=10)
        self._current_map_subscription = self._node.create_subscription(
            MapStatus,
            self.STATUS_TOPICS["current_map"],
            self._current_map_callback,
            current_map_qos,
        )
        self._localization_status_subscription = self._node.create_subscription(
            LocalizationStatus,
            self.STATUS_TOPICS["localization_status"],
            self._localization_status_callback,
            periodic_status_qos,
        )
        self._navigation_status_subscription = self._node.create_subscription(
            NavigationStatus,
            self.STATUS_TOPICS["navigation_status"],
            self._navigation_status_callback,
            periodic_status_qos,
        )

    def start(self) -> None:
        self._spin_thread.start()
        self._node.get_logger().info("KN navigation FastAPI ROS gateway started")

    def stop(self) -> None:
        self._executor.shutdown(timeout_sec=2.0)
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._context.ok():
            self._context.shutdown()

    def status(self) -> Dict[str, Any]:
        services = {
            self.service_names[key]: client.service_is_ready()
            for key, client in self.clients.items()
        }
        with self._topic_status_lock:
            topics = {
                topic: key in self._latest_topic_status
                for key, topic in self.STATUS_TOPICS.items()
            }
        return {
            "success": self._context.ok() and self._spin_thread.is_alive(),
            "node": self._node.get_name(),
            "services": services,
            "topics": topics,
        }

    async def call(
        self,
        key: str,
        request: Any,
        timeout: Optional[float] = None,
        wait_timeout: Optional[float] = None,
    ) -> Any:
        client = self.clients[key]
        service_name = self.service_names[key]
        loop = asyncio.get_running_loop()

        ready_deadline = loop.time() + (
            self.settings.service_wait_timeout
            if wait_timeout is None
            else wait_timeout
        )
        while not client.service_is_ready():
            if loop.time() >= ready_deadline:
                raise HTTPException(
                    status_code=503,
                    detail=f"ROS 2 service is unavailable: {service_name}",
                )
            await asyncio.sleep(0.05)

        try:
            future = client.call_async(request)
        except Exception as error:
            raise HTTPException(
                status_code=502,
                detail=f"Failed to call ROS 2 service {service_name}: {error}",
            ) from error

        deadline = loop.time() + (
            self.settings.service_timeout if timeout is None else timeout
        )
        while not future.done():
            if loop.time() >= deadline:
                future.cancel()
                raise HTTPException(
                    status_code=504,
                    detail=f"Timed out waiting for ROS 2 service: {service_name}",
                )
            await asyncio.sleep(0.01)

        try:
            response = future.result()
        except Exception as error:
            raise HTTPException(
                status_code=502,
                detail=f"ROS 2 service call failed for {service_name}: {error}",
            ) from error
        if response is None:
            raise HTTPException(
                status_code=502,
                detail=f"ROS 2 service returned no response: {service_name}",
            )
        return response

    def request(self, key: str) -> Any:
        return self.service_types[key].Request()

    def log_error(self, message: str) -> None:
        self._node.get_logger().error(message)

    def log_info(self, message: str) -> None:
        self._node.get_logger().info(message)

    async def current_map_path(self) -> str:
        request = self.request("get_map_parameters")
        request.names = [self.settings.map_parameter]
        try:
            response = await self.call(
                "get_map_parameters",
                request,
                wait_timeout=self.settings.map_wait_timeout,
            )
        except HTTPException as error:
            raise RuntimeError(
                "Unable to read the active offline map from "
                f"{self.service_names['get_map_parameters']}: {error.detail}"
            ) from error
        if not response.values:
            raise RuntimeError(
                f"Map parameter was not returned: {self.settings.map_parameter}"
            )
        map_path = response.values[0].string_value.strip()
        if not map_path:
            raise RuntimeError(
                "Active offline map parameter is empty: "
                f"{self.settings.map_node}.{self.settings.map_parameter}"
            )
        return map_path

    @staticmethod
    def _message_timestamp(message: Any) -> Dict[str, Any]:
        stamp = message.header.stamp
        return {
            "sec": int(stamp.sec),
            "nanosec": int(stamp.nanosec),
            "seconds": float(stamp.sec) + float(stamp.nanosec) / 1e9,
            "received_at": datetime.now(timezone.utc).isoformat(),
        }

    @staticmethod
    def _finite_status_value(value: Any) -> Optional[float]:
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None

    def _cache_topic_status(self, key: str, status: Dict[str, Any]) -> None:
        status["_received_monotonic"] = time.monotonic()
        with self._topic_status_lock:
            self._latest_topic_status[key] = status

    def _base_topic_status(
        self,
        key: str,
        message: Any,
        state_names: Dict[int, str],
    ) -> Dict[str, Any]:
        state = int(message.state)
        return {
            "success": True,
            "topic": self.STATUS_TOPICS[key],
            "frame_id": message.header.frame_id,
            "state": state,
            "state_name": state_names.get(state, f"UNKNOWN_{state}"),
            "map_name": message.map_name,
            "reason": message.reason,
            "timestamp": self._message_timestamp(message),
        }

    def _current_map_callback(self, message: Any) -> None:
        status = self._base_topic_status(
            "current_map", message, self.MAP_STATE_NAMES
        )
        self._cache_topic_status("current_map", status)

    def _localization_status_callback(self, message: Any) -> None:
        status = self._base_topic_status(
            "localization_status", message, self.LOCALIZATION_STATE_NAMES
        )
        status["fitness"] = self._finite_status_value(message.fitness)
        self._cache_topic_status("localization_status", status)

    def _navigation_status_callback(self, message: Any) -> None:
        status = self._base_topic_status(
            "navigation_status", message, self.NAVIGATION_STATE_NAMES
        )
        status.update(
            goal_active=bool(message.goal_active),
            distance_to_goal=self._finite_status_value(message.distance_to_goal),
            remaining_waypoints=int(message.remaining_waypoints),
        )
        self._cache_topic_status("navigation_status", status)

    def topic_status(self, key: str) -> Dict[str, Any]:
        if key not in self.STATUS_TOPICS:
            raise KeyError(f"Unknown ROS status topic key: {key}")
        with self._topic_status_lock:
            source = self._latest_topic_status.get(key)
            if source is None:
                raise HTTPException(
                    status_code=503,
                    detail=(
                        "ROS status is unavailable: no message received from "
                        + self.STATUS_TOPICS[key]
                    ),
                )
            result = copy.deepcopy(source)

        received_monotonic = float(result.pop("_received_monotonic"))
        age_seconds = max(0.0, time.monotonic() - received_monotonic)
        result["timestamp"]["age_seconds"] = age_seconds
        if key in ("localization_status", "navigation_status"):
            result["stale_after_seconds"] = 1.0
            result["stale"] = age_seconds > 1.0
        return result

    def _odometry_callback(self, message: Any) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        values = {
            "x": position.x,
            "y": position.y,
            "z": position.z,
            "qx": orientation.x,
            "qy": orientation.y,
            "qz": orientation.z,
            "qw": orientation.w,
            "linear_x": linear.x,
            "linear_y": linear.y,
            "linear_z": linear.z,
            "angular_x": angular.x,
            "angular_y": angular.y,
            "angular_z": angular.z,
        }
        if not all(math.isfinite(float(value)) for value in values.values()):
            self._node.get_logger().warning(
                f"Ignoring non-finite odometry from {self.settings.odometry_topic}"
            )
            return

        linear_speed = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        angular_speed = math.sqrt(
            angular.x * angular.x + angular.y * angular.y + angular.z * angular.z
        )
        motion_state = (
            "moving"
            if linear_speed > 0.01 or angular_speed > 0.01
            else "stopped"
        )
        stamp = message.header.stamp
        status: Dict[str, Any] = {
            "success": True,
            "state": motion_state,
            "topic": self.settings.odometry_topic,
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "position": {
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
            },
            "orientation": {
                "qx": float(orientation.x),
                "qy": float(orientation.y),
                "qz": float(orientation.z),
                "qw": float(orientation.w),
            },
            "velocity": {
                "linear": {
                    "x": float(linear.x),
                    "y": float(linear.y),
                    "z": float(linear.z),
                    "speed": float(linear_speed),
                },
                "angular": {
                    "x": float(angular.x),
                    "y": float(angular.y),
                    "z": float(angular.z),
                    "speed": float(angular_speed),
                },
            },
            "timestamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
                "seconds": float(stamp.sec) + float(stamp.nanosec) / 1e9,
                "received_at": datetime.now(timezone.utc).isoformat(),
            },
            "_received_monotonic": time.monotonic(),
        }
        with self._robot_status_lock:
            self._latest_robot_status = status

    def robot_status(self) -> Dict[str, Any]:
        with self._robot_status_lock:
            if self._latest_robot_status is None:
                raise HTTPException(
                    status_code=503,
                    detail=(
                        "Robot status is unavailable: no odometry received from "
                        + self.settings.odometry_topic
                    ),
                )
            source = self._latest_robot_status
            result = dict(source)
            result["position"] = dict(source["position"])
            result["orientation"] = dict(source["orientation"])
            result["velocity"] = {
                "linear": dict(source["velocity"]["linear"]),
                "angular": dict(source["velocity"]["angular"]),
            }
            result["timestamp"] = dict(source["timestamp"])
            received_monotonic = float(source["_received_monotonic"])

        result.pop("_received_monotonic", None)
        result["timestamp"]["age_seconds"] = max(
            0.0, time.monotonic() - received_monotonic
        )
        return result


class NavigationQueueManager:
    """Run accepted navigation goals sequentially without changing direct-goal APIs."""

    def __init__(
        self,
        ros: RosServiceGateway,
        store: NavigationPointStore,
        poll_interval: float,
        distance_tolerance: float,
        yaw_tolerance: float,
    ):
        self._ros = ros
        self._store = store
        self._poll_interval = poll_interval
        self._distance_tolerance = distance_tolerance
        self._yaw_tolerance = yaw_tolerance
        self._queue: asyncio.Queue = asyncio.Queue()
        self._records: Dict[str, Dict[str, Any]] = {}
        self._sequence = 0
        self._active_task_id: Optional[str] = None
        self._lock = asyncio.Lock()
        self._worker: Optional[asyncio.Task] = None

    def start(self) -> None:
        if self._worker is None or self._worker.done():
            self._worker = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._worker is None:
            return
        self._worker.cancel()
        try:
            await self._worker
        except asyncio.CancelledError:
            pass
        self._worker = None

    def _resolve_goal(self, goal: NavigationGoalRequest) -> ResolvedNavigationGoal:
        if goal.name is not None:
            if goal.x is not None or goal.y is not None:
                raise HTTPException(
                    status_code=422,
                    detail="Provide either name or coordinates, not both",
                )
            point_name = _normalise_point_name(goal.name)
            point = self._store.get_point(point_name)
            if point is None:
                raise HTTPException(
                    status_code=404,
                    detail=f"Navigation point not found: {point_name}",
                )
            try:
                x = float(point["x"])
                y = float(point["y"])
                z = float(point.get("z", 0.0))
                yaw = float(point["yaw"])
            except (KeyError, TypeError, ValueError) as error:
                raise HTTPException(
                    status_code=500,
                    detail=f"Saved navigation point is invalid: {point_name}",
                ) from error
            source = "saved_point"
        else:
            if goal.x is None or goal.y is None:
                raise HTTPException(
                    status_code=422,
                    detail="x and y are required when name is not provided",
                )
            point_name = None
            source = "coordinates"
            x, y, z, yaw = goal.x, goal.y, goal.z, goal.yaw

        _ensure_finite({"x": x, "y": y, "z": z, "yaw": yaw})
        return ResolvedNavigationGoal(
            source=source,
            x=float(x),
            y=float(y),
            z=float(z),
            yaw=float(yaw),
            name=point_name,
        )

    @staticmethod
    def _copy_record(record: Dict[str, Any]) -> Dict[str, Any]:
        result = dict(record)
        if isinstance(result.get("goal"), dict):
            result["goal"] = dict(result["goal"])
        if isinstance(result.get("current_pose"), dict):
            result["current_pose"] = dict(result["current_pose"])
        return result

    async def enqueue(self, goal: NavigationGoalRequest) -> Dict[str, Any]:
        resolved = self._resolve_goal(goal)
        task_id = uuid.uuid4().hex
        async with self._lock:
            self._sequence += 1
            record: Dict[str, Any] = {
                "task_id": task_id,
                "sequence": self._sequence,
                "status": "queued",
                "message": "waiting in navigation queue",
                "created_at": datetime.now(timezone.utc).isoformat(),
                "started_at": None,
                "finished_at": None,
                "goal": resolved.as_dict(),
            }
            self._records[task_id] = record
            self._queue.put_nowait((task_id, resolved))
            queued_ahead = max(self._queue.qsize() - 1, 0)
            result = self._copy_record(record)
            result["queue_position"] = queued_ahead + 1
            return result

    async def status(self) -> Dict[str, Any]:
        async with self._lock:
            active = None
            if self._active_task_id is not None:
                active_record = self._records.get(self._active_task_id)
                if active_record is not None:
                    active = self._copy_record(active_record)

            queued_records = sorted(
                (
                    record
                    for record in self._records.values()
                    if record["status"] == "queued"
                ),
                key=lambda record: record["sequence"],
            )
            queued = []
            for position, record in enumerate(queued_records, start=1):
                item = self._copy_record(record)
                item["queue_position"] = position
                queued.append(item)

            history = [
                self._copy_record(record)
                for record in sorted(
                    self._records.values(),
                    key=lambda record: record["sequence"],
                )
                if record["status"] in ("completed", "failed")
            ]
            return {
                "success": True,
                "active": active,
                "queued_count": len(queued),
                "queued": queued,
                "history": history,
                "distance_tolerance": self._distance_tolerance,
                "yaw_tolerance": self._yaw_tolerance,
            }

    async def _update_record(self, task_id: str, **changes: Any) -> None:
        async with self._lock:
            record = self._records.get(task_id)
            if record is not None:
                record.update(changes)

    async def _finish_task(
        self, task_id: str, status: str, message: str, **changes: Any
    ) -> None:
        changes.update(
            status=status,
            message=message,
            finished_at=datetime.now(timezone.utc).isoformat(),
        )
        async with self._lock:
            record = self._records.get(task_id)
            if record is not None:
                record.update(changes)
            if self._active_task_id == task_id:
                self._active_task_id = None

    async def _publish_goal(self, goal: ResolvedNavigationGoal) -> Any:
        request = self._ros.request("publish_goal")
        _fill_pose_request(request, goal.pose())
        return await self._ros.call("publish_goal", request)

    async def _publish_when_ready(
        self, task_id: str, goal: ResolvedNavigationGoal
    ) -> Any:
        while True:
            try:
                return await self._publish_goal(goal)
            except HTTPException as error:
                await self._update_record(
                    task_id,
                    status="publishing",
                    message=f"waiting for publish goal service: {error.detail}",
                )
                await asyncio.sleep(self._poll_interval)

    async def _wait_until_reached(
        self, task_id: str, goal: ResolvedNavigationGoal
    ) -> None:
        pose = goal.pose()
        while True:
            request = self._ros.request("pose_deviation")
            _fill_pose_request(request, pose)
            try:
                response = await self._ros.call("pose_deviation", request)
            except HTTPException as error:
                await self._update_record(
                    task_id,
                    message=f"waiting for pose deviation service: {error.detail}",
                )
                await asyncio.sleep(self._poll_interval)
                continue

            if not response.success:
                await self._update_record(
                    task_id,
                    message=f"waiting for current pose: {response.message}",
                )
                await asyncio.sleep(self._poll_interval)
                continue

            distance = float(response.distance_xy)
            yaw_error = abs(float(response.yaw_error_rad))
            current = response.current_pose
            current_pose = {
                "x": float(current.position.x),
                "y": float(current.position.y),
                "z": float(current.position.z),
                "qx": float(current.orientation.x),
                "qy": float(current.orientation.y),
                "qz": float(current.orientation.z),
                "qw": float(current.orientation.w),
            }
            await self._update_record(
                task_id,
                message="navigation goal is active",
                distance_xy=distance,
                yaw_error_rad=yaw_error,
                current_pose=current_pose,
            )
            if (
                distance <= self._distance_tolerance
                and yaw_error <= self._yaw_tolerance
            ):
                await self._finish_task(
                    task_id,
                    "completed",
                    "navigation goal reached",
                    distance_xy=distance,
                    yaw_error_rad=yaw_error,
                    current_pose=current_pose,
                )
                return
            await asyncio.sleep(self._poll_interval)

    async def _run(self) -> None:
        while True:
            task_id, goal = await self._queue.get()
            try:
                async with self._lock:
                    self._active_task_id = task_id
                    record = self._records[task_id]
                    record.update(
                        status="publishing",
                        message="publishing navigation goal",
                        started_at=datetime.now(timezone.utc).isoformat(),
                    )

                response = await self._publish_when_ready(task_id, goal)
                if not response.success:
                    await self._finish_task(
                        task_id,
                        "failed",
                        f"failed to publish navigation goal: {response.message}",
                    )
                    continue

                await self._update_record(
                    task_id,
                    status="navigating",
                    message=response.message,
                )
                await self._wait_until_reached(task_id, goal)
            except asyncio.CancelledError:
                raise
            except Exception as error:
                await self._finish_task(
                    task_id,
                    "failed",
                    f"navigation queue task failed: {error}",
                )
                self._ros.log_error(
                    f"Navigation queue task {task_id} failed: {error}"
                )
            finally:
                self._queue.task_done()


def create_app(settings: Settings) -> FastAPI:
    @asynccontextmanager
    async def lifespan(application: FastAPI):
        gateway = RosServiceGateway(settings)
        queue_manager: Optional[NavigationQueueManager] = None
        gateway.start()
        try:
            if settings.points_file.strip():
                map_path = ""
                points_file = Path(settings.points_file).expanduser().resolve()
            else:
                map_path = await gateway.current_map_path()
                points_file = _points_file_for_map(
                    settings.points_directory, map_path
                )

            point_store_instance = NavigationPointStore(
                str(points_file), map_path=map_path
            )
            queue_manager = NavigationQueueManager(
                gateway,
                point_store_instance,
                poll_interval=settings.queue_poll_interval,
                distance_tolerance=settings.queue_distance_tolerance,
                yaw_tolerance=settings.queue_yaw_tolerance,
            )
            application.state.point_store = point_store_instance
            application.state.relocalize_lock = asyncio.Lock()
            application.state.ros_gateway = gateway
            application.state.navigation_queue = queue_manager
            application.state.map_path = map_path
            gateway.log_info(
                "Navigation points store ready: map=%s file=%s"
                % (point_store_instance.map_name, point_store_instance.path)
            )
            queue_manager.start()
            yield
        finally:
            if queue_manager is not None:
                await queue_manager.stop()
            gateway.stop()

    application = FastAPI(
        title="KN Navigation ROS 2 Service API",
        version="2.5.0",
        description=(
            "Five core APIs: relocalize, save a navigation point, publish a goal, "
            "enable the velocity bridge, and list saved navigation points. "
            "A sequential navigation-goal queue, navigation status monitoring, map "
            "switching, and navigation reset are also available. All poses use the "
            "map frame and yaw values use radians."
        ),
        lifespan=lifespan,
    )

    def gateway() -> RosServiceGateway:
        return application.state.ros_gateway

    def point_store() -> NavigationPointStore:
        return application.state.point_store

    def relocalize_lock() -> Any:
        return application.state.relocalize_lock

    def navigation_queue() -> NavigationQueueManager:
        return application.state.navigation_queue

    def require_api_key(x_api_key: Optional[str] = Header(default=None)) -> None:
        if settings.api_key and (
            x_api_key is None
            or not hmac.compare_digest(x_api_key, settings.api_key)
        ):
            raise HTTPException(status_code=401, detail="Invalid or missing X-API-Key")

    router = APIRouter(prefix="/api", dependencies=[Depends(require_api_key)])

    @application.get("/", tags=["system"])
    async def root() -> Dict[str, Any]:
        return {
            "name": "KN Navigation ROS 2 Service API",
            "docs": "/docs",
            "health": "/api/health",
            "functions": [
                "relocalize",
                "save_navigation_point",
                "publish_navigation_goal",
                "enable_velocity_bridge",
                "list_navigation_points",
                "rename_navigation_point",
                "queue_navigation_goal",
                "get_navigation_queue",
                "get_robot_status",
                "get_current_map",
                "get_localization_status",
                "get_navigation_status",
                "switch_map",
                "restart_navigation",
            ],
        }

    @router.get("/health", tags=["system"], summary="Check ROS gateway health")
    async def health(ros: RosServiceGateway = Depends(gateway)) -> Dict[str, Any]:
        return ros.status()

    @router.get(
        "/services", tags=["system"], summary="List mapped ROS services and readiness"
    )
    async def services(ros: RosServiceGateway = Depends(gateway)) -> Dict[str, Any]:
        status = ros.status()
        return {"success": status["success"], "services": status["services"]}

    @router.get(
        "/robot/status",
        tags=["robot"],
        summary="Return the latest robot pose, velocity, and timestamp",
    )
    async def get_robot_status(
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        return ros.robot_status()

    @router.get(
        "/current_map",
        tags=["status"],
        summary="Return the latest current-map status",
    )
    async def get_current_map(
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        return ros.topic_status("current_map")

    @router.get(
        "/localization_status",
        tags=["status"],
        summary="Return the latest Open3D localization status",
    )
    async def get_localization_status(
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        return ros.topic_status("localization_status")

    @router.get(
        "/navigation_status",
        tags=["status"],
        summary="Return the latest navigation execution status",
    )
    async def get_navigation_status(
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        return ros.topic_status("navigation_status")

    @router.post(
        "/switch_map",
        tags=["navigation-control"],
        summary="Switch the localization and planning maps through nav_manager",
    )
    async def switch_map(
        command: SwitchMapRequest,
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        map_name = _normalise_map_name(command.map_name)
        request = ros.request("switch_map")
        request.map_name = map_name
        response = await ros.call("switch_map", request)
        return {
            "success": bool(response.success),
            "message": response.message,
            "map_name": map_name,
        }

    @router.post(
        "/restart_navigation",
        tags=["navigation-control"],
        summary="Soft-reset navigation or request a configured full restart",
    )
    async def restart_navigation(
        command: RestartNavigationRequest,
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        request = ros.request("restart_navigation")
        request.mode = command.mode
        response = await ros.call("restart_navigation", request)
        return {
            "success": bool(response.accepted),
            "accepted": bool(response.accepted),
            "message": response.message,
            "mode": command.mode,
            "mode_name": "SOFT_RESET" if command.mode == 0 else "FULL_RESTART",
        }

    @router.post(
        "/open3d_loc/relocalize",
        tags=["localization"],
        summary="1. Relocalize and return success or failure",
    )
    async def relocalize(
        pose: QuaternionPose,
        ros: RosServiceGateway = Depends(gateway),
        lock: Any = Depends(relocalize_lock),
    ) -> Dict[str, Any]:
        normalised_pose, original_norm = _normalise_quaternion_pose(pose)
        sent_pose = _quaternion_pose_to_dict(normalised_pose)
        if lock.locked():
            return {
                "success": False,
                "message": "another relocalize request is running",
                "pose": sent_pose,
            }

        request = ros.request("relocalize")
        _fill_pose_request(request, normalised_pose)
        async with lock:
            response = await ros.call("relocalize", request)
        return {
            "success": response.success,
            "message": response.message,
            "pose": sent_pose,
            "input_quaternion_norm": original_norm,
            "quaternion_normalized": not math.isclose(
                original_norm, 1.0, rel_tol=1e-9, abs_tol=1e-9
            ),
        }

    @router.post(
        "/navigation/points",
        tags=["navigation"],
        summary="2. Save the current localization pose as a named JSON point",
    )
    async def save_navigation_point(
        command: SavePointRequest,
        ros: RosServiceGateway = Depends(gateway),
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        name = _normalise_point_name(command.name)
        response = await ros.call("get_pose", ros.request("get_pose"))
        if not response.success:
            return {"success": False, "message": response.message}

        values = {
            "x": response.x,
            "y": response.y,
            "z": response.z,
            "qx": response.qx,
            "qy": response.qy,
            "qz": response.qz,
            "qw": response.qw,
        }
        _ensure_finite(values)
        rpy = _quaternion_to_rpy(
            response.qx, response.qy, response.qz, response.qw
        )
        point: Dict[str, Any] = {
            "frame_id": "map",
            "child_frame_id": "base_link",
            **values,
            **rpy,
        }
        saved_point = store.save_point(name, point, command.overwrite)
        return {
            "success": True,
            "message": f"Navigation point saved: {name}",
            "map_name": store.map_name,
            "point": {"name": name, **saved_point},
        }

    @router.get(
        "/navigation/points",
        tags=["navigation"],
        summary="5. Return all saved navigation points",
    )
    async def list_navigation_points(
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        stored_points = store.list_points()
        points = []
        for name, point in stored_points.items():
            item = dict(point)
            item["name"] = name
            points.append(item)
        return {
            "success": True,
            "map_name": store.map_name,
            "map_path": store.map_path or None,
            "count": len(points),
            "points": points,
        }

    @router.patch(
        "/navigation/points/rename",
        tags=["navigation"],
        summary="Rename a saved navigation point without changing its pose",
    )
    async def rename_navigation_point(
        command: RenamePointRequest,
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        old_name = _normalise_point_name(command.old_name)
        new_name = _normalise_point_name(command.new_name)
        point = store.rename_point(old_name, new_name)
        return {
            "success": True,
            "message": f"Navigation point renamed: {old_name} -> {new_name}",
            "old_name": old_name,
            "point": {"name": new_name, **point},
        }

    @router.post(
        "/navigation/goal",
        tags=["navigation"],
        summary="3. Publish a saved point or direct XYZ/yaw navigation goal",
    )
    async def publish_navigation_goal(
        goal: NavigationGoalRequest,
        ros: RosServiceGateway = Depends(gateway),
        store: NavigationPointStore = Depends(point_store),
    ) -> Dict[str, Any]:
        point_name: Optional[str] = None
        source = "coordinates"
        if goal.name is not None:
            if goal.x is not None or goal.y is not None:
                raise HTTPException(
                    status_code=422,
                    detail="Provide either name or coordinates, not both",
                )
            point_name = _normalise_point_name(goal.name)
            point = store.get_point(point_name)
            if point is None:
                raise HTTPException(
                    status_code=404,
                    detail=f"Navigation point not found: {point_name}",
                )
            try:
                x = float(point["x"])
                y = float(point["y"])
                z = float(point.get("z", 0.0))
                yaw = float(point["yaw"])
            except (KeyError, TypeError, ValueError) as error:
                raise HTTPException(
                    status_code=500,
                    detail=f"Saved navigation point is invalid: {point_name}",
                ) from error
            source = "saved_point"
        else:
            if goal.x is None or goal.y is None:
                raise HTTPException(
                    status_code=422,
                    detail="x and y are required when name is not provided",
                )
            x, y, z, yaw = goal.x, goal.y, goal.z, goal.yaw

        _ensure_finite({"x": x, "y": y, "z": z, "yaw": yaw})
        half_yaw = 0.5 * yaw
        pose = QuaternionPose(
            x=x,
            y=y,
            z=z,
            qx=0.0,
            qy=0.0,
            qz=math.sin(half_yaw),
            qw=math.cos(half_yaw),
        )
        request = ros.request("publish_goal")
        _fill_pose_request(request, pose)
        response = await ros.call("publish_goal", request)
        goal_result: Dict[str, Any] = {
            "source": source,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        }
        if point_name is not None:
            goal_result["name"] = point_name
        return {
            "success": response.success,
            "message": response.message,
            "goal": goal_result,
        }

    @router.post(
        "/navigation/queue",
        tags=["navigation"],
        summary="Queue a saved point or direct goal for sequential navigation",
    )
    async def queue_navigation_goal(
        goal: NavigationGoalRequest,
        queue: NavigationQueueManager = Depends(navigation_queue),
    ) -> Dict[str, Any]:
        task = await queue.enqueue(goal)
        return {
            "success": True,
            "message": "navigation goal added to queue",
            "task": task,
        }

    @router.get(
        "/navigation/queue",
        tags=["navigation"],
        summary="Return the active goal, waiting queue, and completed task history",
    )
    async def get_navigation_queue(
        queue: NavigationQueueManager = Depends(navigation_queue),
    ) -> Dict[str, Any]:
        return await queue.status()

    @router.post(
        "/go2_cmd_vel_bridge/enable",
        tags=["robot"],
        summary="4. Enable or disable robot motion through the velocity bridge",
    )
    async def bridge_enable(
        command: BridgeEnableRequest,
        ros: RosServiceGateway = Depends(gateway),
    ) -> Dict[str, Any]:
        request = ros.request("bridge_enable")
        request.data = command.data
        response = await ros.call("bridge_enable", request)
        return {"success": response.success, "message": response.message}

    application.include_router(router)
    return application


def _parse_args() -> argparse.Namespace:
    defaults = Settings.from_environment()
    parser = argparse.ArgumentParser(
        description="Expose KN navigation ROS 2 services through FastAPI"
    )
    parser.add_argument("--host", default=defaults.host)
    parser.add_argument("--port", type=int, default=defaults.port)
    parser.add_argument(
        "--service-timeout", type=float, default=defaults.service_timeout
    )
    parser.add_argument(
        "--service-wait-timeout", type=float, default=defaults.service_wait_timeout
    )
    parser.add_argument(
        "--queue-poll-interval",
        type=float,
        default=defaults.queue_poll_interval,
        help="Seconds between navigation queue completion checks",
    )
    parser.add_argument(
        "--queue-distance-tolerance",
        type=float,
        default=defaults.queue_distance_tolerance,
        help="XY distance in metres used to mark a queued goal complete",
    )
    parser.add_argument(
        "--queue-yaw-tolerance",
        type=float,
        default=defaults.queue_yaw_tolerance,
        help="Absolute yaw error in radians used to mark a queued goal complete",
    )
    parser.add_argument(
        "--odometry-topic",
        default=defaults.odometry_topic,
        help="nav_msgs/Odometry topic used by the robot status API",
    )
    parser.add_argument(
        "--points-file",
        default=defaults.points_file,
        help=(
            "Explicit navigation points JSON file; when omitted, the file name "
            "is derived from the active offline map"
        ),
    )
    parser.add_argument(
        "--points-directory",
        default=defaults.points_directory,
        help="Directory for automatically named per-map navigation point files",
    )
    parser.add_argument(
        "--map-node",
        default=defaults.map_node,
        help="ROS 2 node that owns the active offline map path parameter",
    )
    parser.add_argument(
        "--map-parameter",
        default=defaults.map_parameter,
        help="Parameter on --map-node containing the active offline map path",
    )
    parser.add_argument(
        "--map-wait-timeout",
        type=float,
        default=defaults.map_wait_timeout,
        help="Seconds to wait for the map node parameter service at API startup",
    )
    parser.add_argument("--log-level", default="info")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    positive_values = {
        "--service-timeout": args.service_timeout,
        "--service-wait-timeout": args.service_wait_timeout,
        "--queue-poll-interval": args.queue_poll_interval,
        "--queue-distance-tolerance": args.queue_distance_tolerance,
        "--queue-yaw-tolerance": args.queue_yaw_tolerance,
        "--map-wait-timeout": args.map_wait_timeout,
    }
    invalid_values = [
        name
        for name, value in positive_values.items()
        if not math.isfinite(value) or value <= 0.0
    ]
    if invalid_values:
        raise SystemExit(
            "These options must be finite and greater than zero: "
            + ", ".join(invalid_values)
        )
    if not args.odometry_topic.strip():
        raise SystemExit("--odometry-topic must not be empty")
    if not args.points_file.strip() and not args.points_directory.strip():
        raise SystemExit("--points-directory must not be empty")
    if not args.points_file.strip() and not args.map_node.strip("/").strip():
        raise SystemExit("--map-node must not be empty")
    if not args.points_file.strip() and not args.map_parameter.strip():
        raise SystemExit("--map-parameter must not be empty")

    environment = Settings.from_environment()
    settings = Settings(
        host=args.host,
        port=args.port,
        service_timeout=args.service_timeout,
        service_wait_timeout=args.service_wait_timeout,
        queue_poll_interval=args.queue_poll_interval,
        queue_distance_tolerance=args.queue_distance_tolerance,
        queue_yaw_tolerance=args.queue_yaw_tolerance,
        odometry_topic=args.odometry_topic,
        points_file=args.points_file,
        points_directory=args.points_directory,
        map_node=args.map_node,
        map_parameter=args.map_parameter,
        map_wait_timeout=args.map_wait_timeout,
        api_key=environment.api_key,
    )
    if settings.host not in ("127.0.0.1", "localhost", "::1") and not settings.api_key:
        print(
            "WARNING: API is exposed beyond loopback without KN_NAV_API_KEY set.",
            flush=True,
        )

    uvicorn.run(
        create_app(settings),
        host=settings.host,
        port=settings.port,
        log_level=args.log_level,
    )


# This supports `uvicorn src.tools.ros2_service_api:app` when the repository is
# imported as a Python namespace, while direct execution remains the usual path.
app = create_app(Settings.from_environment())


if __name__ == "__main__":
    main()
