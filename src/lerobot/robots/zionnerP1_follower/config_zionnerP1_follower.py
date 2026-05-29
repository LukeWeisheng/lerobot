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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.gemini335l import Gemini335LCameraConfig
from lerobot.cameras.configs import ColorMode

from ..config import RobotConfig


@RobotConfig.register_subclass("zionnerP1_follower")
@dataclass
class ZionnerP1FollowerConfig(RobotConfig):
    ip_address: str = "169.254.160.182"
    local_ip_address: str | None = None
    command_speed: float = 100.0
    blend_tolerance: float = 10.0
    use_realtime: bool = True
    rt_move_duration: float = 0.05
    rt_network_tolerance: int = 100
    rt_joint_step: float = 0.0005
    enable_default_cameras: bool = True
    arm_camera_serial_number: str = "CP2N1630006X"
    head_camera_serial_number: str = "CP26363000BJ"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.enable_default_cameras and not self.cameras:
            self.cameras = {
                "arm_camera": Gemini335LCameraConfig(
                    serial_number_or_name=self.arm_camera_serial_number,
                    width=640,
                    height=480,
                    fps=30,
                    color_mode=ColorMode.RGB,
                    use_depth=True,
                    depth_width=640,
                    depth_height=480,
                    depth_fps=30,
                    enable_frame_sync=True,
                ),
                "head_camera": Gemini335LCameraConfig(
                    serial_number_or_name=self.head_camera_serial_number,
                    width=640,
                    height=480,
                    fps=30,
                    color_mode=ColorMode.RGB,
                    use_depth=True,
                    depth_width=640,
                    depth_height=480,
                    depth_fps=30,
                    enable_frame_sync=True,
                ),
            }

        super().__post_init__()
