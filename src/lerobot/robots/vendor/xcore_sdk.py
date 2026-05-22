#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import importlib
import platform
import socket
import sys
from pathlib import Path
from typing import Any


_SDK_MODULE: Any | None = None
_SDK_ROOT = Path(__file__).resolve().parent / "xcoresdk_python-v0.7.0.ar_3"


class XCoreSDKError(RuntimeError):
    pass


def _configure_xcore_sdk_paths() -> None:
    release_directory = _SDK_ROOT / "Release"
    path_candidates = [
        _SDK_ROOT,
        release_directory,
        release_directory / "windows",
        release_directory / "linux",
        release_directory / "windows" / "xCoreSDK_python",
        release_directory / "linux" / "xCoreSDK_python",
    ]
    for candidate in path_candidates:
        candidate_str = str(candidate)
        if candidate_str not in sys.path:
            sys.path.append(candidate_str)


def get_xcore_sdk_python() -> Any:
    global _SDK_MODULE

    if _SDK_MODULE is not None:
        return _SDK_MODULE

    _configure_xcore_sdk_paths()
    system = platform.system()
    if system == "Windows":
        module_name = "Release.windows.xCoreSDK_python"
    elif system == "Linux":
        module_name = "Release.linux.xCoreSDK_python"
    else:
        raise ImportError(f"Unsupported operating system: {system}")

    _SDK_MODULE = importlib.import_module(module_name)
    return _SDK_MODULE


def _assert_success(ec: dict[str, Any], action: str) -> None:
    if ec.get("ec", 0) != 0:
        raise XCoreSDKError(f"{action} failed: {ec}")


class XCoreZionnerP1Client:
    def __init__(
        self,
        ip_address: str,
        local_ip_address: str | None = None,
        command_speed: float = 100.0,
        blend_tolerance: float = 10.0,
        use_realtime: bool = False,
        rt_move_duration: float = 0.05,
        rt_network_tolerance: int = 100,
    ):
        self.ip_address = ip_address
        self.local_ip_address = local_ip_address
        self.command_speed = command_speed
        self.blend_tolerance = blend_tolerance
        self.use_realtime = use_realtime
        self.rt_move_duration = rt_move_duration
        self.rt_network_tolerance = rt_network_tolerance
        self._sdk: Any | None = None
        self._robot: Any | None = None
        self._rt_controller: Any | None = None
        self._rt_configured = False
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        if self._is_connected:
            return

        self._sdk = get_xcore_sdk_python()
        if self.use_realtime:
            local_ip_address = self.local_ip_address
            if local_ip_address is None:
                local_ip_address = _infer_local_ip_address(self.ip_address)
            self.local_ip_address = local_ip_address
            self._robot = self._sdk.ArRobot(self.ip_address, local_ip_address)
        else:
            self._robot = self._sdk.ArRobot(self.ip_address)
        ec: dict[str, Any] = {}
        self._robot.connectToRobot(ec)
        _assert_success(ec, f"connectToRobot({self.ip_address})")
        self._is_connected = True
        if self.use_realtime:
            self.configure_realtime_joint_control()

    def configure_realtime_joint_control(self) -> None:
        if self._rt_configured and self._rt_controller is not None:
            return

        robot = self._require_robot()
        ec: dict[str, Any] = {}
        robot.setOperateMode(self._sdk.OperateMode.automatic, ec)
        _assert_success(ec, "setOperateMode")
        robot.setRtNetworkTolerance(self.rt_network_tolerance, ec)
        _assert_success(ec, "setRtNetworkTolerance")
        robot.setMotionControlMode(
            self._sdk.MotionControlMode.RtCommandMode,
            ec,
        )
        _assert_success(ec, "setMotionControlMode")
        robot.setPowerState(True, ec)
        _assert_success(ec, "setPowerState")
        self._rt_controller = robot.getRtMotionController()
        self._rt_configured = True

    def configure_joint_control(self, force: bool = False) -> None:
        if self.use_realtime:
            self.configure_realtime_joint_control()
            return

        robot = self._require_robot()
        ec: dict[str, Any] = {}
        robot.setMotionControlMode(
            self._sdk.MotionControlMode.NrtCommandMode,
            ec,
        )
        _assert_success(ec, "setMotionControlMode")

        if force or robot.operateMode(ec) != self._sdk.OperateMode.automatic:
            _assert_success(ec, "operateMode")
            robot.setOperateMode(self._sdk.OperateMode.automatic, ec)
            _assert_success(ec, "setOperateMode")

        if force or robot.powerState(ec) != self._sdk.PowerState.on:
            _assert_success(ec, "powerState")
            robot.setPowerState(True, ec)
            _assert_success(ec, "setPowerState")

    def operation_state(self) -> Any:
        robot = self._require_robot()
        ec: dict[str, Any] = {}
        state = robot.operationState(ec)
        _assert_success(ec, "operationState")
        return state

    def is_idle_for_command(self) -> bool:
        state = self.operation_state()
        return state in {
            self._sdk.OperationState.idle,
            self._sdk.OperationState.unknown,
        }

    def read_joint_positions(self) -> list[float]:
        robot = self._require_robot()
        ec: dict[str, Any] = {}
        joint_positions = list(robot.jointPos(ec))
        _assert_success(ec, "jointPos")
        if len(joint_positions) < 7:
            raise XCoreSDKError(
                f"Expected at least 7 joints, got {len(joint_positions)}"
            )
        return joint_positions[:7]

    def send_joint_positions(
        self,
        joint_positions: list[float],
    ) -> list[float]:
        if len(joint_positions) != 7:
            raise ValueError(
                f"Expected 7 joint values, got {len(joint_positions)}"
            )

        if self.use_realtime:
            self.configure_realtime_joint_control()
            current_joint_positions = self.read_joint_positions()
            self._require_rt_controller().MoveJ(
                self.rt_move_duration,
                current_joint_positions,
                joint_positions,
            )
            return joint_positions

        robot = self._require_robot()

        if not self.is_idle_for_command():
            return joint_positions

        self.configure_joint_control()
        ec: dict[str, Any] = {}
        robot.moveReset(ec)
        _assert_success(ec, "moveReset")
        command_id = self._sdk.PyString()
        command = self._sdk.MoveAbsJCommand(
            joint_positions,
            self.command_speed,
            self.blend_tolerance,
        )
        robot.moveAppend([command], command_id, ec)
        _assert_success(ec, "moveAppend")
        robot.moveStart(ec)
        _assert_success(ec, "moveStart")
        return joint_positions

    def disconnect(self) -> None:
        if not self._is_connected or self._robot is None:
            return

        ec: dict[str, Any] = {}
        if self.use_realtime:
            self._disconnect_realtime(ec)
        self._robot.disconnectFromRobot(ec)
        _assert_success(ec, "disconnectFromRobot")
        self._is_connected = False
        self._robot = None
        self._rt_controller = None
        self._rt_configured = False

    def _disconnect_realtime(self, ec: dict[str, Any]) -> None:
        robot = self._require_robot()
        try:
            robot.setMotionControlMode(
                self._sdk.MotionControlMode.NrtCommandMode,
                ec,
            )
            _assert_success(ec, "setMotionControlMode")
        except XCoreSDKError:
            pass

    def _require_robot(self) -> Any:
        if not self._is_connected or self._robot is None:
            raise XCoreSDKError("xCore robot is not connected")
        return self._robot

    def _require_rt_controller(self) -> Any:
        if self._rt_controller is None:
            raise XCoreSDKError("xCore realtime controller is not configured")
        return self._rt_controller


def _infer_local_ip_address(robot_ip_address: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe_socket:
        probe_socket.connect((robot_ip_address, 1))
        return str(probe_socket.getsockname()[0])
