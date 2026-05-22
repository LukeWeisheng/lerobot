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

from lerobot.utils.errors import (
    DeviceAlreadyConnectedError,
    DeviceNotConnectedError,
)

from lerobot.robots.vendor import XCoreSDKError, XCoreZionnerP1Client

from ..teleoperator import Teleoperator
from .config_zionnerP1_leader import ZionnerP1LeaderConfig

logger = logging.getLogger(__name__)

ZIONNER_P1_JOINTS = tuple(f"joint_{index}.pos" for index in range(1, 8))


class ZionnerP1Leader(Teleoperator):
    config_class = ZionnerP1LeaderConfig
    name = "zionnerP1_leader"

    def __init__(self, config: ZionnerP1LeaderConfig):
        super().__init__(config)
        self.config = config
        self.client = XCoreZionnerP1Client(
            ip_address=config.ip_address,
            command_speed=config.command_speed,
            blend_tolerance=config.blend_tolerance,
        )

    @cached_property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(ZIONNER_P1_JOINTS, float)

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.client.is_connected

    def connect(self, calibrate: bool = False) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            self.client.connect()
            self.client.configure_joint_control()
        except XCoreSDKError as error:
            raise ConnectionError(str(error)) from error

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("%s does not require an explicit calibration step.", self)

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        joint_positions = self.client.read_joint_positions()
        action = {
            joint_name: value
            for joint_name, value in zip(
                ZIONNER_P1_JOINTS,
                joint_positions,
                strict=True,
            )
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        return None

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.client.disconnect()
        logger.info(f"{self} disconnected.")
