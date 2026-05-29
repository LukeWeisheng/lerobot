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

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.gemini335l import Gemini335LCamera
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import (
    DeviceAlreadyConnectedError,
    DeviceNotConnectedError,
)

from ..robot import Robot
from ..vendor import XCoreSDKError, XCoreZionnerP1Client
from .config_zionnerP1_follower import ZionnerP1FollowerConfig

logger = logging.getLogger(__name__)

ZIONNER_P1_JOINTS = tuple(f"joint_{index}.pos" for index in range(1, 8))


class ZionnerP1Follower(Robot):
    config_class = ZionnerP1FollowerConfig
    name = "zionnerP1_follower"

    def __init__(self, config: ZionnerP1FollowerConfig):
        super().__init__(config)
        self.config = config
        self.robot_type = config.type
        self.client = XCoreZionnerP1Client(
            ip_address=config.ip_address,
            local_ip_address=config.local_ip_address,
            command_speed=config.command_speed,
            blend_tolerance=config.blend_tolerance,
            use_realtime=config.use_realtime,
            rt_move_duration=config.rt_move_duration,
            rt_network_tolerance=config.rt_network_tolerance,
            rt_joint_step=config.rt_joint_step,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return dict.fromkeys(ZIONNER_P1_JOINTS, float)

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        camera_features: dict[str, tuple] = {}
        for cam_name, cam_config in self.config.cameras.items():
            camera_features[cam_name] = (
                cam_config.height,
                cam_config.width,
                3,
            )
            if getattr(cam_config, "use_depth", False):
                camera_features[f"{cam_name}_depth"] = (
                    getattr(cam_config, "depth_height"),
                    getattr(cam_config, "depth_width"),
                    3,
                )
        return camera_features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.client.is_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    def connect(self, calibrate: bool = False) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            self.client.connect()
            self.client.configure_joint_control()
        except XCoreSDKError as error:
            raise ConnectionError(str(error)) from error

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("%s does not require an explicit calibration step.", self)

    def configure(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        joint_positions = self.client.read_joint_positions()
        observation = {
            joint_name: value
            for joint_name, value in zip(
                ZIONNER_P1_JOINTS,
                joint_positions,
                strict=True,
            )
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        for cam_key, cam in self.cameras.items():
            if isinstance(cam, Gemini335LCamera) and cam.use_depth:
                frame_bundle = cam.read_frame_bundle(timeout_ms=500)
                observation[cam_key] = frame_bundle["color"]
                depth_map = frame_bundle["depth"]
                if depth_map is not None:
                    observation[f"{cam_key}_depth"] = (
                        Gemini335LCamera.pack_depth_for_storage(depth_map)
                    )
            else:
                observation[cam_key] = cam.async_read()

        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        missing_keys = [
            joint_name
            for joint_name in ZIONNER_P1_JOINTS
            if joint_name not in action
        ]
        if missing_keys:
            raise ValueError(f"Missing joint targets for {missing_keys}")

        joint_positions = [
            float(action[joint_name]) for joint_name in ZIONNER_P1_JOINTS
        ]
        self.client.send_joint_positions(joint_positions)
        return dict(zip(ZIONNER_P1_JOINTS, joint_positions, strict=True))

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.client.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
